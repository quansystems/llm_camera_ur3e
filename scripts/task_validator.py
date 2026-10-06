#!/usr/bin/env python3
"""Report the final result received from a robot_skill CLI invocation."""

import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def result_message(event):
    if not isinstance(event, dict):
        raise ValueError('Expected a JSON object')
    skill, status = event.get('skill'), event.get('status')
    if skill not in ('home', 'place', 'pick') or status not in ('success', 'failed'):
        raise ValueError('Invalid skill or execution status')
    parameters = event.get('parameters', [])
    if (not isinstance(parameters, list)
            or len(parameters) != {'home': 0, 'place': 1, 'pick': 2}[skill]
            or any(not isinstance(value, str) or not value for value in parameters)):
        raise ValueError('Invalid skill parameters')
    command = ' '.join([skill, *parameters])
    code = event.get('error_code')
    if status == 'success' and code:
        status = 'failed'
    if status == 'failed':
        return f'{command} failed' + (f' {code}' if isinstance(code, str) and code else '')
    return f'{command} success'


class TaskValidator(Node):
    def __init__(self):
        super().__init__('task_validator')
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.results = self.create_publisher(
            String, '/task_validator/result',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.events = self.create_publisher(String, '/task_validator/events', 10)
        self.create_subscription(String, '/robot_skill/events', self.on_result, qos)
        self.seen = set()
        self.order = deque()
        self.get_logger().info('Waiting for /robot_skill/events; output: /task_validator/result')

    def on_result(self, msg):
        try:
            event = json.loads(msg.data)
            message = result_message(event)
        except (ValueError, TypeError):
            self.get_logger().warning('Ignoring invalid robot_skill result')
            return
        request_id = event.get('request_id')
        if not isinstance(request_id, str) or not request_id:
            self.get_logger().warning('Ignoring result without request_id')
            return
        if request_id in self.seen:
            return
        self.seen.add(request_id)
        self.order.append(request_id)
        if len(self.order) > 1024:
            self.seen.remove(self.order.popleft())
        self.results.publish(String(data=message))
        self.events.publish(String(data=json.dumps({**event, 'message': message})))
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = TaskValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
