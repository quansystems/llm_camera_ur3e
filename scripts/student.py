#!/usr/bin/env python3
"""Convert an MSSV to a checked skill sequence; queue it for manual action."""

import json
import select
import sys
import time
import uuid
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from planner_state import WorldState, apply_skill, check_goal, validate_plan


ASSIGNMENTS = (
    ('red', 'yellow', 'blue'), ('red', 'blue', 'yellow'),
    ('yellow', 'red', 'blue'), ('yellow', 'blue', 'red'),
    ('blue', 'red', 'yellow'), ('blue', 'yellow', 'red'),
)
MAIN_ZONES = ('zone_1_red', 'zone_2_green', 'zone_3_yellow')


def student_goals(student_id):
    student_id = student_id.strip()
    if len(student_id) < 2 or not student_id.isascii() or not student_id.isdigit():
        raise ValueError('MSSV phải là chuỗi chữ số, có ít nhất hai chữ số.')
    xx = int(student_id[-2:])
    p = xx % 6
    return xx, p, [['pick', colour, zone] for colour, zone in zip(ASSIGNMENTS[p], MAIN_ZONES)]


def make_plan(initial, goals):
    """Shortest legal transfer sequence, including temporary moves for cycles."""
    def key(state):
        return (state['held'], tuple((colour, tuple(obj['position']))
                                    for colour, obj in sorted(state['objects'].items())))

    queue = deque([(initial, [])])
    visited = {key(initial)}
    while queue:
        state, commands = queue.popleft()
        try:
            check_goal(state, goals)
        except ValueError:
            pass
        else:
            validate_plan(initial, commands, goals)
            return commands
        if len(commands) >= 20:
            continue
        for colour in sorted(state['objects']):
            for zone in ('zone_Intermediate', *MAIN_ZONES):
                command = ['pick', colour, zone]
                try:
                    following = apply_skill(state, command)
                except ValueError:
                    continue
                signature = key(following)
                if signature not in visited:
                    visited.add(signature)
                    queue.append((following, commands + [command]))
    raise ValueError('Không tìm được chuỗi skill hợp lệ. Kiểm tra zone trống, vật đang giữ và vị trí vật.')


class Student(Node):
    def __init__(self):
        super().__init__('student')
        self.world = WorldState(self)
        self.command = self.create_publisher(String, '/action/command', 10)
        self.plan_id = None
        self.status = None
        self.create_subscription(String, '/action/plan_status', self.on_status, 10)

    def on_status(self, msg):
        try:
            event = json.loads(msg.data)
            if isinstance(event, dict) and event.get('plan_id') == self.plan_id:
                self.status = event
        except ValueError:
            pass

    def wait(self, predicate, timeout, detail):
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise ValueError(detail)
            rclpy.spin_once(self, timeout_sec=.1)
        raise KeyboardInterrupt

    def submit(self, student_id):
        xx, p, goals = student_goals(student_id)
        print(f'XX = {xx:02d}; P = {p}', flush=True)
        for index, goal in enumerate(goals, 1):
            print(f'Zone {index} ({goal[2]}) = {goal[1]}', flush=True)
        state = None
        def ready():
            nonlocal state
            try:
                state = self.world.snapshot()
                return True
            except ValueError:
                return False
        self.wait(ready, 15., 'Chưa có trạng thái Gazebo/MoveIt mới. Hãy chạy workcell.')
        commands = make_plan(state, goals)
        if not commands:
            print('Các vật đã đúng quy luật; không cần gửi skill.', flush=True)
            return
        print('Chuỗi skill:\n' + '; '.join(' '.join(command) for command in commands), flush=True)
        self.wait(lambda: self.command.get_subscription_count() > 0, 10.,
                  'Hãy chạy action.py rồi nhập lại MSSV.')
        if self.command.get_subscription_count() != 1:
            raise ValueError('Chỉ chạy một node action.py.')
        self.plan_id = str(uuid.uuid4())
        self.status = None
        self.command.publish(String(data=json.dumps({
            'type': 'validated_plan', 'plan_id': self.plan_id, 'steps': commands,
            'goals': goals, 'initial_state': state})))
        self.wait(lambda: self.status is not None, 10., 'action.py chưa xác nhận kế hoạch.')
        if self.status.get('status') != 'queued':
            raise ValueError('action.py từ chối: ' + self.status.get('detail', 'queue unavailable'))
        print('Đã đẩy chuỗi vào action. Gửi tín hiệu action để bắt đầu thực thi.', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = Student()
    try:
        print('Nhập MSSV; nhập exit để thoát.', flush=True)
        while rclpy.ok():
            print('MSSV: ', end='', flush=True)
            while not select.select([sys.stdin], [], [], .1)[0]:
                rclpy.spin_once(node, timeout_sec=.05)
            line = sys.stdin.readline()
            if not line or line.strip().lower() == 'exit':
                break
            try:
                node.submit(line.strip())
            except ValueError as exc:
                print('Lỗi: ' + str(exc), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
