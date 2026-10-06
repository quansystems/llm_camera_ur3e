import json
import sys
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from action import Action, parse_skills


class ActionQueue(unittest.TestCase):
    def make_node(self):
        node = SimpleNamespace(
            queue=deque(), executing=False, done=False, exit_code=0,
            process=None, active=None, validated=None, exited_at=None,
            deadline=None, skill_timeout=300., robot_script=Path('/tmp/robot_skill.py'),
            task_result=Mock(), count_publishers=lambda topic: 1, get_logger=lambda: Mock())
        node.finish = lambda *args: Action.finish(node, *args)
        return node

    def test_single_and_sequence_syntax(self):
        self.assertEqual(parse_skills('place green'), [['place', 'green']])
        self.assertEqual(parse_skills('home; place green\npick green gray'),
                         [['home'], ['place', 'green'], ['pick', 'green', 'gray']])
        for command in ('', 'home; invalid', 'pick red', 'place --ros-args'):
            with self.assertRaises(ValueError):
                parse_skills(command)

    def test_trigger_freezes_queue(self):
        node = self.make_node()
        Action.on_command(node, String(data='home; place green'))
        Action.on_command(node, String(data='action'))
        Action.on_command(node, String(data='pick green gray'))
        self.assertTrue(node.executing)
        self.assertEqual(list(node.queue), [['home'], ['place', 'green']])

    def event(self, node, status, request_id=None):
        return String(data=json.dumps({
            'request_id': request_id or node.active['request_id'],
            'skill': node.active['command'][0],
            'parameters': node.active['command'][1:], 'status': status,
            'error_code': 'INVALID_OBJECT' if status == 'failed' else None}))

    def test_sequential_execution_waits_for_process_and_validator(self):
        node = self.make_node()
        Action.on_command(node, String(data='home; home'))
        Action.on_command(node, String(data='action'))
        first, second = Mock(), Mock()
        first.poll.return_value = None
        second.poll.return_value = None
        with patch('action.subprocess.Popen', side_effect=[first, second]) as launch:
            Action.tick(node)
            first_id = node.active['request_id']
            Action.on_validation(node, self.event(node, 'success'))
            Action.tick(node)
            self.assertEqual(launch.call_count, 1)
            first.poll.return_value = 0
            Action.tick(node)
            self.assertEqual(launch.call_count, 2)
            self.assertNotEqual(first_id, node.active['request_id'])
            Action.on_validation(node, self.event(node, 'success', first_id))
            self.assertIsNone(node.validated)
            second.poll.return_value = 0
            Action.tick(node)
            self.assertFalse(node.done)
            Action.on_validation(node, self.event(node, 'success'))
            Action.tick(node)
        self.assertTrue(node.done)
        self.assertEqual(node.exit_code, 0)
        node.task_result.publish.assert_called_once_with(String(data='task success'))

    def test_validator_failure_stops_remaining_skills(self):
        node = self.make_node()
        Action.on_command(node, String(data='place pink; home'))
        Action.on_command(node, String(data='action'))
        with patch('action.subprocess.Popen', return_value=Mock()) as launch:
            Action.tick(node)
            Action.on_validation(node, self.event(node, 'failed'))
            Action.tick(node)
        self.assertEqual(launch.call_count, 1)
        self.assertTrue(node.done)
        self.assertFalse(node.queue)
        self.assertEqual(node.exit_code, 1)
        node.task_result.publish.assert_called_once_with(String(data='task failed'))

    def test_process_exit_without_validator_is_failure(self):
        node = self.make_node()
        Action.on_command(node, String(data='home'))
        Action.on_command(node, String(data='action'))
        process = Mock()
        process.poll.return_value = 0
        with patch('action.subprocess.Popen', return_value=process):
            Action.tick(node)
        with patch('action.time.monotonic', side_effect=[100., 104.]):
            Action.tick(node)
            Action.tick(node)
        self.assertEqual(node.exit_code, 1)
        node.task_result.publish.assert_called_once_with(String(data='task failed'))


if __name__ == '__main__':
    unittest.main()
