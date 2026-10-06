#!/usr/bin/env python3
"""Queue CLI skills; execute sequentially when /action/command receives action."""

import json
import math
import re
import shlex
import signal
import subprocess
import sys
import time
import uuid
from collections import deque
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from planner_state import WorldState, canonical, compare_state, validate_plan


def parse_skills(text):
    commands = []
    for part in re.split(r'[;\n]', text):
        if not part.strip():
            continue
        tokens = shlex.split(part)
        if not tokens or tokens[0] not in ('home', 'place', 'pick'):
            raise ValueError('Use home, place OBJECT, or pick OBJECT ZONE')
        if len(tokens) != {'home': 1, 'place': 2, 'pick': 3}[tokens[0]]:
            raise ValueError('Incorrect number of skill parameters')
        if any(not token or token.startswith('-') for token in tokens[1:]):
            raise ValueError('Invalid skill parameter syntax')
        commands.append(tokens)
    if not commands:
        raise ValueError('Empty skill command')
    return commands


class Action(Node):
    def __init__(self):
        super().__init__('action')
        self.skill_timeout = float(self.declare_parameter('skill_timeout', 300.).value)
        if not math.isfinite(self.skill_timeout) or self.skill_timeout <= 0:
            raise ValueError('skill_timeout must be positive')
        self.queue = deque()
        self.executing = False
        self.done = False
        self.exit_code = 0
        self.process = None
        self.active = None
        self.validated = None
        self.exited_at = None
        self.deadline = None
        self.guard = None
        self.world = WorldState(self)
        self.robot_script = Path(__file__).with_name('robot_skill.py')
        self.task_result = self.create_publisher(
            String, '/task_validation',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.plan_status = self.create_publisher(
            String, '/action/plan_status',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.validator = self.create_subscription(
            String, '/task_validator/events', self.on_validation, 10)
        self.create_subscription(String, '/action/command', self.on_command, 10)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Queue skills on /action/command; send action to execute')

    def on_command(self, msg):
        if self.executing or self.done:
            # Give another planner explicit rejection instead of a silent timeout.
            if msg.data.lstrip().startswith('{'):
                try:
                    incoming = json.loads(msg.data)
                    if isinstance(incoming, dict) and incoming.get('plan_id'):
                        self.plan_status.publish(String(data=json.dumps({
                            'plan_id': incoming['plan_id'], 'status': 'rejected',
                            'detail': 'action is already executing or finishing a task'})))
                except ValueError:
                    pass
            return
        if msg.data.strip() == 'action':
            if not self.queue:
                self.get_logger().error('Empty queue: send skills before action')
                return
            self.executing = True
            self.deadline = time.monotonic() + 10.
            if getattr(self, 'guard', None):
                self.guard['after'] = time.monotonic()
                self.plan_status.publish(String(data=json.dumps({
                    'plan_id': self.guard['plan_id'], 'status': 'executing'})))
            self.get_logger().info(f'Starting task with {len(self.queue)} skills')
            return
        plan = None
        try:
            if msg.data.lstrip().startswith('{'):
                plan = json.loads(msg.data)
                if (not isinstance(plan, dict) or plan.get('type') != 'validated_plan'
                        or self.queue or not plan.get('steps') or not isinstance(plan.get('plan_id'), str)
                        or not plan['plan_id']):
                    raise ValueError('Validated plan requires an empty action queue')
                commands = [canonical(c) for c in plan['steps']]
                goals = [canonical(c) for c in plan['goals']]
                if not goals:
                    raise ValueError('Missing task goal')
                validate_plan(plan['initial_state'], commands, goals)
                self.guard = {'initial': plan['initial_state'], 'goals': goals,
                              'started': False, 'predicted': None, 'plan_id': plan['plan_id']}
                self.queue.extend(commands)
                self.plan_status.publish(String(data=json.dumps({
                    'plan_id': plan['plan_id'], 'status': 'queued'})))
                self.get_logger().info(f'Queued {len(commands)} checked LLM skills; send action to execute')
                return
            if getattr(self, 'guard', None):
                raise ValueError('A checked plan is already queued; execute it first')
            commands = parse_skills(msg.data)
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(str(exc))
            if isinstance(plan, dict) and plan.get('plan_id'):
                self.plan_status.publish(String(data=json.dumps({
                    'plan_id': plan['plan_id'], 'status': 'rejected', 'detail': str(exc)})))
            return
        self.queue.extend(commands)
        self.get_logger().info(f'Queued {len(commands)} skills; total: {len(self.queue)}')

    def on_validation(self, msg):
        if self.done or self.active is None:
            return
        try:
            event = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(event, dict) or event.get('request_id') != self.active['request_id']:
            return
        if (event.get('skill') != self.active['command'][0]
                or event.get('parameters') != self.active['command'][1:]):
            self.finish(False, 'Validator result does not match the active skill')
            return
        if event.get('status') == 'failed' or event.get('error_code'):
            self.finish(False, event.get('message', 'Skill failed'))
        elif event.get('status') == 'success':
            self.validated = True

    def tick(self):
        if not self.executing or self.done:
            return
        now = time.monotonic()
        guard = getattr(self, 'guard', None)
        if guard and not guard['started']:
            try:
                actual = self.world.snapshot(after=guard['after'])
            except ValueError as exc:
                if now >= self.deadline:
                    self.finish(False, str(exc))
                return
            try:
                compare_state(guard['initial'], actual)
                guard['predicted'] = validate_plan(actual, list(self.queue), guard['goals'])
            except ValueError as exc:
                self.finish(False, 'Preflight rejected: ' + str(exc))
                return
            guard['started'] = True
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                if self.exited_at is None:
                    self.exited_at = now
                if code == 0 and self.validated:
                    if guard:
                        try:
                            actual = self.world.snapshot(after=self.exited_at)
                        except ValueError as exc:
                            if now - self.exited_at >= 10.:
                                self.finish(False, str(exc))
                            return
                        try:
                            compare_state(guard['predicted'][0], actual,
                                          check_home=self.active['command'][0] == 'home')
                            guard['predicted'] = validate_plan(actual, list(self.queue), guard['goals'])
                        except ValueError as exc:
                            self.finish(False, 'Post-skill state check failed: ' + str(exc))
                            return
                        self.get_logger().info('Actual state verified after ' + ' '.join(self.active['command']))
                    self.process = None
                    self.active = None
                    self.validated = None
                    self.exited_at = None
                    self.deadline = now + 10.
                elif now - self.exited_at >= 3.:
                    self.finish(False, 'Skill exited without a successful validator result')
                    return
                else:
                    return
            elif now >= self.deadline:
                self.finish(False, 'Skill execution timed out')
                return
            else:
                return
        if not self.queue:
            self.finish(True)
            return
        if self.count_publishers('/task_validator/events') == 0:
            if now >= self.deadline:
                self.finish(False, 'Start task_validator.py before executing the task')
            return
        command = self.queue.popleft()
        request_id = str(uuid.uuid4())
        self.active = {'request_id': request_id, 'command': command}
        self.deadline = now + self.skill_timeout
        self.get_logger().info('Executing ' + ' '.join(command))
        try:
            self.process = subprocess.Popen([
                sys.executable, str(self.robot_script), *command,
                '--ros-args', '-p', f'request_id:={request_id}'])
        except OSError as exc:
            self.finish(False, f'Could not start robot_skill: {exc}')

    def finish(self, success, detail=None):
        if self.done:
            return
        self.done = True
        self.exit_code = 0 if success else 1
        self.queue.clear()
        message = 'task success' if success else 'task failed'
        if detail:
            self.get_logger().error(str(detail))
        self.task_result.publish(String(data=message))
        if getattr(self, 'guard', None):
            self.plan_status.publish(String(data=json.dumps({
                'plan_id': self.guard['plan_id'], 'status': 'success' if success else 'failed',
                'detail': str(detail or '')})))
            self.plan_status.wait_for_all_acked(timeout=Duration(seconds=2.))
        self.task_result.wait_for_all_acked(timeout=Duration(seconds=2.))
        self.get_logger().info(message)

    def stop_child(self):
        if self.process is None or self.process.poll() is not None:
            return
        # A validator failure often arrives while the CLI is acknowledging
        # its final result. Let that completed invocation exit cleanly first.
        try:
            self.process.wait(timeout=3.)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            self.process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            self.process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def main(args=None):
    rclpy.init(args=args)
    node = Action()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.finish(False, 'Task interrupted')
        node.exit_code = 1
    except Exception as exc:
        if rclpy.ok():
            node.finish(False, str(exc))
        node.exit_code = 1
    finally:
        node.stop_child()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
