import itertools
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from student import ASSIGNMENTS, MAIN_ZONES, make_plan, student_goals
from planner_state import refresh_zones, validate_plan
from test_planner_state import world


class StudentPlanning(unittest.TestCase):
    def test_all_six_assignments_and_leading_zeroes(self):
        expected = [('red', 'yellow', 'blue'), ('red', 'blue', 'yellow'),
                    ('yellow', 'red', 'blue'), ('yellow', 'blue', 'red'),
                    ('blue', 'red', 'yellow'), ('blue', 'yellow', 'red')]
        self.assertEqual(list(ASSIGNMENTS), expected)
        for xx in range(100):
            number, p, goals = student_goals('202400' + f'{xx:02d}')
            self.assertEqual(number, xx)
            self.assertEqual(p, xx % 6)
            self.assertEqual([g[1] for g in goals], list(expected[p]))
            self.assertEqual([g[2] for g in goals], list(MAIN_ZONES))
        for value in ('', '1', '123a', '-12', '１２', '12 34'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                student_goals(value)

    def test_initial_objects_and_occupied_zone_all_assignments(self):
        for p in range(6):
            state = world()
            goals = student_goals(f'{p:02d}')[2]
            commands = make_plan(state, goals)
            validate_plan(state, commands, goals)
            self.assertLessEqual(len(commands), 20)

    def test_all_permutations_can_be_rearranged_using_gray(self):
        for current in itertools.permutations(('red', 'yellow', 'blue')):
            for p in range(6):
                state = world()
                state['objects']['green']['position'] = [.1, 0., .735]
                for colour, zone in zip(current, MAIN_ZONES):
                    state['objects'][colour]['position'] = [.25, state['zones'][zone]['position'][1], .735]
                refresh_zones(state)
                goals = student_goals(f'{p:02d}')[2]
                commands = make_plan(state, goals)
                validate_plan(state, commands, goals)
                if current == ASSIGNMENTS[p]:
                    self.assertEqual(commands, [])
                else:
                    self.assertTrue(any(c[2] == 'zone_Intermediate' for c in commands))

    def test_held_object_and_impossible_full_zones(self):
        state = world()
        state['held'] = 'red'
        state['objects']['red']['on_table'] = False
        goals = student_goals('00')[2]
        commands = make_plan(state, goals)
        self.assertEqual(commands[0][1], 'red')
        validate_plan(state, commands, goals)
        state = world()
        for colour, zone in zip(('green', 'red', 'blue', 'yellow'), state['zones']):
            state['objects'][colour]['position'] = [.25, state['zones'][zone]['position'][1], .735]
        refresh_zones(state)
        with self.assertRaisesRegex(ValueError, 'Không tìm được'):
            make_plan(state, student_goals('00')[2])


if __name__ == '__main__':
    unittest.main()
