#!/usr/bin/env python3
"""9Router plans; deterministic checks admit plans; action verifies execution."""

import getpass
import json
import math
import select
import signal
import subprocess
import sys
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from action import parse_skills
from planner_state import WorldState, canonical, parse_goal, validate_plan


def build_messages(template, camera, request, state=None, goals=None):
    context = template.strip() + '\n\nCAMERA_CHECK\n' + json.dumps(camera, ensure_ascii=False)
    if state is not None:
        context += '\n\nWORLD_STATE\n' + json.dumps(state, ensure_ascii=False)
        context += '\n\nGOALS\n' + json.dumps(goals, ensure_ascii=False)
    return [{'role': 'system', 'content': context}, {'role': 'user', 'content': request}]


def request_plan(base_url, key, model, messages, timeout):
    request = Request(base_url.rstrip('/') + '/chat/completions',
                      data=json.dumps({'model': model, 'messages': messages, 'stream': False}).encode(),
                      headers={'Authorization': 'Bearer ' + key, 'Content-Type': 'application/json'},
                      method='POST')
    try:
        with urlopen(request, timeout=timeout) as response:
            data = json.load(response)
        text = data['choices'][0]['message']['content']
        if not isinstance(text, str) or not text.strip():
            raise ValueError('Empty response')
        return text.strip()
    except HTTPError as exc:
        # Do not expose provider error bodies, headers or credentials.
        raise ValueError(f'9Router trả lỗi HTTP {exc.code}') from None
    except (URLError, OSError, TimeoutError):
        raise ValueError('Không kết nối được 9Router hoặc yêu cầu hết thời gian') from None
    except (ValueError, KeyError, IndexError, TypeError):
        raise ValueError('9Router trả dữ liệu không hợp lệ') from None


def normalize_plan(text):
    """Legacy CLI syntax validation, retained for callers; not plan admission."""
    commands = parse_skills(text)
    for command in commands:
        canonical(command)
    return '; '.join(' '.join(c) for c in commands)


def decode_plan(text):
    try:
        data = json.loads(text)
    except (ValueError, TypeError):
        raise ValueError('LLM phải trả JSON {"steps": [...]}') from None
    if not isinstance(data, dict) or set(data) != {'steps'} or not isinstance(data['steps'], list):
        raise ValueError('LLM response must contain only a steps array')
    if len(data['steps']) > 20:
        raise ValueError('Too many plan steps')
    commands = []
    for step in data['steps']:
        if not isinstance(step, dict) or step.get('skill') not in ('home', 'place', 'pick'):
            raise ValueError('Unknown skill in LLM JSON')
        fields = {'home': {'skill'}, 'place': {'skill', 'object'},
                  'pick': {'skill', 'object', 'zone'}}[step['skill']]
        if set(step) != fields:
            raise ValueError('Invalid JSON skill parameters')
        command = [step['skill']]
        if 'object' in fields:
            command.append(step['object'])
        if 'zone' in fields:
            command.append(step['zone'])
        commands.append(canonical(command))
    return commands


class LLMControl(Node):
    def __init__(self):
        super().__init__('llm_control')
        self.base_url = self.declare_parameter('base_url', 'http://localhost:20128/v1').value
        self.model = self.declare_parameter('model', 'ur3e').value
        self.timeout = float(self.declare_parameter('timeout', 60.).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('timeout must be positive')
        share = Path(get_package_share_directory('ur3e_llm_control'))
        self.template = (share / 'config/promote_config').read_text()
        self.world = WorldState(self)
        self.camera = None
        self.camera_received = 0.
        self.plan_id = None
        self.status = None
        self.child = None
        self.command = self.create_publisher(String, '/action/command', 10)
        self.create_subscription(String, '/camera_check/zones', self.on_camera, 10)
        self.create_subscription(String, '/action/plan_status', self.on_status, 10)

    def on_camera(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.camera = data
                self.camera_received = time.monotonic()
        except ValueError:
            pass

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and data.get('plan_id') == self.plan_id:
                self.status = data
        except ValueError:
            pass

    def wait(self, predicate, timeout, error):
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise ValueError(error)
            rclpy.spin_once(self, timeout_sec=.1)
        raise KeyboardInterrupt

    def gather_context(self):
        state = None
        last_error = ''
        def ready():
            nonlocal state, last_error
            try:
                state = self.world.snapshot()
                if (not self.camera or not self.camera.get('valid')
                        or time.monotonic() - self.camera_received > 3.):
                    raise ValueError('Cần dữ liệu camera_check mới và hợp lệ')
                return True
            except ValueError as exc:
                last_error = str(exc)
                return False
        try:
            self.wait(ready, 15., 'Chưa có trạng thái đầy đủ')
        except ValueError:
            raise ValueError('Chưa có trạng thái đầy đủ: ' + last_error) from None
        return state, self.camera

    def plan(self, key, request, pool):
        goals = parse_goal(request)
        state, camera = self.gather_context()
        messages = build_messages(self.template, camera, request, state, goals)
        for attempt in range(2):
            print('Đang chờ LLM lập kế hoạch...' if attempt == 0 else
                  'Yêu cầu LLM sửa kế hoạch một lần...', flush=True)
            future = pool.submit(request_plan, self.base_url, key, self.model, messages, self.timeout)
            self.wait(future.done, self.timeout + 2., '9Router hết thời gian')
            response = future.result()
            state, camera = self.gather_context()  # Recheck using current, not earlier state.
            try:
                commands = decode_plan(response)
                validate_plan(state, commands, goals)
                return commands, goals, state
            except ValueError as exc:
                print('Kế hoạch bị từ chối: ' + str(exc), flush=True)
                if attempt == 1:
                    raise ValueError('LLM vẫn lập kế hoạch sai. Không gửi lệnh tới robot.') from None
                messages = build_messages(self.template, camera, request, state, goals)
                messages.extend([{'role': 'assistant', 'content': response},
                                 {'role': 'user', 'content': 'VALIDATION_ERROR: ' + str(exc)
                                  + '\nCorrect the FULL plan. Keep the original GOALS.'}])
        raise AssertionError('Unreachable')

    def execute(self, commands, goals, state):
        if not commands:
            print('Mục tiêu đã đạt; không cần di chuyển robot.', flush=True)
            return
        if self.count_publishers('/task_validator/events') == 0:
            raise ValueError('Hãy chạy task_validator.py trước')
        if self.command.get_subscription_count() == 0:
            self.child = subprocess.Popen([sys.executable, str(Path(__file__).with_name('action.py'))])
        self.wait(lambda: self.command.get_subscription_count() > 0, 10., 'Không kết nối được action.py')
        if self.command.get_subscription_count() != 1:
            raise ValueError('Chỉ được chạy một node action')
        self.plan_id = str(uuid.uuid4())
        self.status = None
        envelope = {'type': 'validated_plan', 'plan_id': self.plan_id, 'steps': commands,
                    'goals': goals, 'initial_state': state}
        self.command.publish(String(data=json.dumps(envelope)))
        self.wait(lambda: self.status is not None, 10., 'action.py không xác nhận kế hoạch')
        if self.status.get('status') != 'queued':
            raise ValueError('action.py từ chối: ' + self.status.get('detail', 'queue unavailable'))
        print('\nChuỗi skill đã kiểm tra:\n' + '; '.join(' '.join(c) for c in commands), flush=True)
        self.command.publish(String(data='action'))
        print('Đang thực thi; chờ hoàn tất trước khi nhập yêu cầu tiếp theo.', flush=True)
        # action enforces individual skill deadlines and exits after the task.
        self.wait(lambda: self.status.get('status') in ('success', 'failed')
                  or (self.child is not None and self.child.poll() is not None),
                  20 * 310., 'Không nhận được kết quả action')
        if self.status.get('status') not in ('success', 'failed'):
            raise ValueError('action.py đã thoát mà không trả kết quả')
        print('task ' + self.status['status'], flush=True)
        if self.status.get('detail'):
            print(self.status['detail'], flush=True)
        # Wait for this action to leave discovery before accepting a new task.
        self.wait(lambda: self.command.get_subscription_count() == 0, 10., 'action.py chưa kết thúc')
        if self.child is not None:
            self.child.wait(timeout=3.)
            self.child = None
        self.plan_id = None

    def stop_child(self):
        if self.child is None or self.child.poll() is not None:
            return
        self.child.send_signal(signal.SIGINT)
        try:
            self.child.wait(timeout=12.)
        except subprocess.TimeoutExpired:
            self.child.terminate()
            self.child.wait(timeout=3.)
        self.child = None


def main(args=None):
    if not sys.stdin.isatty():
        print('Chạy node trong terminal để nhập API key ẩn.', file=sys.stderr)
        return 1
    rclpy.init(args=args)
    node = None
    pool = ThreadPoolExecutor(max_workers=1)
    try:
        key = getpass.getpass('Nhập API key 9Router (ẩn khi nhập): ').strip()
        if not key:
            raise ValueError('API key không được để trống')
        node = LLMControl()
        print('Nhập mục tiêu, ví dụ: pick blue zone_3_yellow; home. Nhập exit để thoát.\n'
              'Node tự chạy action sau khi kế hoạch vượt qua kiểm tra.', flush=True)
        while rclpy.ok():
            print('\nBạn: ', end='', flush=True)
            while not select.select([sys.stdin], [], [], .1)[0]:
                rclpy.spin_once(node, timeout_sec=.05)
            line = sys.stdin.readline()
            if not line or line.strip().lower() == 'exit':
                break
            if not line.strip():
                continue
            try:
                commands, goals, state = node.plan(key, line.strip(), pool)
                node.execute(commands, goals, state)
            except (ValueError, OSError, subprocess.TimeoutExpired) as exc:
                print('Lỗi: ' + str(exc), flush=True)
                node.stop_child()
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print('Lỗi: ' + str(exc), flush=True)
        return 1
    finally:
        if node is not None:
            node.stop_child()
            node.destroy_node()
        pool.shutdown(wait=False, cancel_futures=True)
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
