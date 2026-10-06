import copy
import json
import sys
import unittest
from collections import deque
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from action import Action
from llm_control import LLMControl, decode_plan
from planner_state import (apply_skill, compare_state, parse_goal,
                           refresh_zones, validate_plan)


def world():
    positions = {'red': [.144721, -.389443, .735], 'yellow': [.170711, -.220711, .735],
                 'green': [.25, .24, .735], 'blue': [.170711, .220711, .735],
                 'purple': [.144721, .389443, .735]}
    return refresh_zones({
        'table_z': .7, 'home': True, 'held': None,
        'objects': {c: {'id': c + '_box', 'position': p, 'size': [.05, .05, .07],
                        'on_table': True, 'upright': True} for c, p in positions.items()},
        'zones': {z: {'id': z, 'color': c, 'position': [.25, y, .7025], 'size': [.1, .1, .005]}
                  for z, c, y in [('zone_Intermediate', 'gray', -.12), ('zone_1_red', 'red', 0.),
                                  ('zone_2_green', 'green', .12), ('zone_3_yellow', 'yellow', .24)]}})


class PlanningChecks(unittest.TestCase):
    def test_original_failure_is_blocked_and_clearing_plan_succeeds(self):
        state = world()
        goals = [['pick', 'blue', 'zone_3_yellow']]
        with self.assertRaisesRegex(ValueError, 'blocked by green'):
            validate_plan(state, goals, goals)
        states = validate_plan(state, [['pick', 'green', 'zone_Intermediate'], *goals], goals)
        self.assertEqual(states[-1]['zones']['zone_3_yellow']['objects'], ['blue'])
        self.assertEqual(states[-1]['zones']['zone_Intermediate']['objects'], ['green'])
        self.assertEqual(state, world(), 'Checking must not mutate real input state')

    def test_physical_footprint_can_block_with_centre_outside(self):
        state = world()
        state['objects']['green']['position'] = [.25, 0., .735]
        state['objects']['blue']['position'] = [.19, .24, .735]
        refresh_zones(state)
        zone = state['zones']['zone_3_yellow']
        self.assertFalse(zone['objects'])
        self.assertEqual(zone['blocked_by'], ['blue'])
        with self.assertRaisesRegex(ValueError, 'blocked by blue'):
            apply_skill(state, ['pick', 'red', 'yellow'])

    def test_place_means_hold_and_home_preserves_hold(self):
        state = apply_skill(world(), ['place', 'green'])
        self.assertEqual(state['held'], 'green')
        self.assertFalse(state['zones']['zone_3_yellow']['blocked_by'])
        self.assertEqual(apply_skill(state, ['home'])['held'], 'green')
        with self.assertRaisesRegex(ValueError, 'Already holding green'):
            apply_skill(state, ['place', 'blue'])
        released = apply_skill(state, ['pick', 'green', 'yellow'])
        self.assertIsNone(released['held'])

    def test_wrong_goal_unknown_parameter_and_ungraspable_object_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Final goal'):
            validate_plan(world(), [['home']], [['pick', 'blue', 'yellow']])
        with self.assertRaisesRegex(ValueError, 'INVALID_OBJECT'):
            apply_skill(world(), ['pick', 'pink', 'yellow'])
        state = world()
        state['objects']['blue']['upright'] = False
        with self.assertRaisesRegex(ValueError, 'upright'):
            apply_skill(state, ['place', 'blue'])

    def test_already_satisfied_goal_can_be_empty_but_not_self_pick(self):
        validate_plan(world(), [], [['pick', 'green', 'yellow']])
        with self.assertRaisesRegex(ValueError, 'blocked by green'):
            apply_skill(world(), ['pick', 'green', 'yellow'])

    def test_strict_json_and_independent_goal(self):
        self.assertEqual(decode_plan('{"steps":[{"skill":"pick","object":"blue","zone":"yellow"}]}'),
                         [['pick', 'blue', 'zone_3_yellow']])
        for response in ('pick blue yellow', '{"steps": [], "goal": "home"}',
                         '{"steps":[{"skill":"home","object":"blue"}]}',
                         '{"steps":[{"skill":"place","object":"--ros-args"}]}'):
            with self.subTest(response=response), self.assertRaises(ValueError):
                decode_plan(response)
        for text in ('put blue object in yellow zone', 'pick blue object in zone_3_yellow',
                     'Đặt vật xanh dương vào vùng vàng', 'pick blue zone_3_yellow'):
            self.assertEqual(parse_goal(text), [['pick', 'blue', 'zone_3_yellow']])
        self.assertEqual(parse_goal('pick blue yellow; home'), [['pick', 'blue', 'zone_3_yellow'], ['home']])
        for text in ('lại', 'đặt vật xanh vào vùng xanh', "don't put blue object in yellow zone",
                     'put blue object in yellow zone and move red'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_goal(text)

    def test_llm_receives_feedback_once_without_python_inserting_steps(self):
        bad = '{"steps":[{"skill":"pick","object":"blue","zone":"yellow"}]}'
        good = '{"steps":[{"skill":"pick","object":"green","zone":"gray"},' \
               '{"skill":"pick","object":"blue","zone":"yellow"}]}'
        node = SimpleNamespace(template='Prompt', base_url='http://localhost/v1', model='ur3e', timeout=5.,
                               gather_context=lambda: (world(), {'valid': True}),
                               wait=lambda ready, timeout, error: self.assertTrue(ready()))
        def plan_with(responses):
            pool = Mock()
            def submit(*args):
                future = Future()
                future.set_result(next(responses))
                return future
            pool.submit.side_effect = submit
            return pool
        pool = plan_with(iter([bad, good]))
        commands, _, _ = LLMControl.plan(node, 'test-key', 'pick blue yellow', pool)
        self.assertEqual(commands, [['pick', 'green', 'zone_Intermediate'], ['pick', 'blue', 'zone_3_yellow']])
        self.assertEqual(pool.submit.call_count, 2)
        self.assertIn('blocked by green', pool.submit.call_args.args[4][-1]['content'])
        pool = plan_with(iter([bad, bad]))
        with self.assertRaisesRegex(ValueError, 'Không gửi lệnh'):
            LLMControl.plan(node, 'test-key', 'pick blue yellow', pool)
        self.assertEqual(pool.submit.call_count, 2)


class GuardedExecution(unittest.TestCase):
    def node(self):
        node = SimpleNamespace(
            queue=deque(), executing=False, done=False, exit_code=0, process=None,
            active=None, validated=None, exited_at=None, deadline=None, guard=None,
            skill_timeout=300., robot_script=Path('/tmp/robot_skill.py'), plan_status=Mock(),
            world=Mock(), task_result=Mock(), count_publishers=lambda topic: 1,
            get_logger=lambda: Mock())
        node.finish = lambda *args: Action.finish(node, *args)
        return node

    def queue(self, node):
        Action.on_command(node, String(data=json.dumps({
            'type': 'validated_plan', 'plan_id': 'test-plan', 'initial_state': world(),
            'steps': [['pick', 'green', 'zone_Intermediate'], ['pick', 'blue', 'zone_3_yellow']],
            'goals': [['pick', 'blue', 'zone_3_yellow']]})))
        Action.on_command(node, String(data='action'))

    def test_changed_world_before_execution_never_starts_child(self):
        node = self.node()
        self.queue(node)
        changed = world()
        changed['objects']['red']['position'][0] += .1
        node.world.snapshot.return_value = changed
        with patch('action.subprocess.Popen') as launch:
            Action.tick(node)
        launch.assert_not_called()
        self.assertEqual(node.exit_code, 1)

    def test_mismatch_after_skill_stops_following_step(self):
        node = self.node()
        self.queue(node)
        node.world.snapshot.return_value = world()
        process = Mock()
        process.poll.return_value = None
        with patch('action.subprocess.Popen', return_value=process) as launch:
            Action.tick(node)
            node.validated = True
            process.poll.return_value = 0
            # A success event alone cannot allow the second skill: object did not move.
            Action.tick(node)
        self.assertEqual(launch.call_count, 1)
        self.assertEqual(node.exit_code, 1)

    def test_empty_action_does_not_report_false_success(self):
        node = self.node()
        Action.on_command(node, String(data='action'))
        self.assertFalse(node.executing)
        node.task_result.publish.assert_not_called()

    def test_busy_action_rejects_new_plan_without_touching_active_task(self):
        node = self.node()
        self.queue(node)
        original = list(node.queue)
        node.plan_status.reset_mock()
        Action.on_command(node, String(data='{"plan_id":"another-plan","type":"validated_plan"}'))
        self.assertEqual(list(node.queue), original)
        status = json.loads(node.plan_status.publish.call_args.args[0].data)
        self.assertEqual(status['plan_id'], 'another-plan')
        self.assertEqual(status['status'], 'rejected')

    def test_tolerable_pose_noise_accepted_but_displacement_rejected(self):
        expected = world()
        actual = copy.deepcopy(expected)
        actual['objects']['blue']['position'][0] += .003
        compare_state(expected, actual)
        actual['objects']['blue']['position'][0] += .03
        with self.assertRaisesRegex(ValueError, 'blue'):
            compare_state(expected, actual)


if __name__ == '__main__':
    unittest.main()
