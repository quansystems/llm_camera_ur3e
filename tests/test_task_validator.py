import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from robot_skill import RobotSkill
from robot_action import SkillError
from task_validator import result_message

class ValidatorContracts(unittest.TestCase):
    def test_required_result_format(self):
        cases = [
            ('home', [], 'success', None, 'home success'),
            ('place', ['green'], 'success', None, 'place green success'),
            ('pick', ['green_box', 'gray'], 'success', None, 'pick green_box gray success'),
            ('place', ['pink'], 'failed', 'INVALID_OBJECT', 'place pink failed INVALID_OBJECT'),
            ('pick', ['red', 'missing_zone'], 'failed', 'INVALID_OBJECT',
             'pick red missing_zone failed INVALID_OBJECT'),
            ('pick', ['red', 'zone_1_red'], 'failed', 'PLANNING_FAILED',
             'pick red zone_1_red failed PLANNING_FAILED'),
        ]
        published = []
        node = SimpleNamespace(result_signal=SimpleNamespace(
            get_subscription_count=lambda: 1,
            publish=lambda msg: published.append(json.loads(msg.data)),
            wait_for_all_acked=lambda **kwargs: True))
        for skill, parameters, status, code, expected in cases:
            RobotSkill.publish_result(node, skill, status, code, parameters=parameters)
            self.assertEqual(published[-1]['parameters'], parameters)
            self.assertEqual(result_message(published[-1]), expected)

    def test_error_cannot_be_success(self):
        self.assertEqual(result_message({'skill':'pick','parameters':['red','red'],
                                        'status':'success','error_code':'PLANNING_FAILED'}),
                         'pick red red failed PLANNING_FAILED')
        with self.assertRaises(ValueError):result_message({'skill':'home','status':'started'})
        with self.assertRaises(ValueError):result_message({'skill':'pick','status':'success'})

    def test_occupied_zone_blocks_before_grasp(self):
        def grasp(*args):raise AssertionError('Grasp attempted for occupied destination')
        node=SimpleNamespace(held_object=None,
            zones={'zone_1_red':{'position':[.25,0,.7025],'size':[.1,.1,.005]}},
            object_snapshot=lambda:[{'id':'green_box','live':True,'position':[.25,0,.735],'size':[.05,.05,.07]}],
            place=grasp)
        node.zone_empty=lambda zone:RobotSkill.zone_empty(node,zone)
        with self.assertRaises(SkillError) as exc:RobotSkill.pick(node,'red_box','zone_1_red')
        self.assertEqual(exc.exception.code,'PLANNING_FAILED')

if __name__=='__main__':unittest.main()
