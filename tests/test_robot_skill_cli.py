import contextlib
import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import threading
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from robot_skill import RobotSkill, argument_parser
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class SkillCLI(unittest.TestCase):
    def test_exact_skill_arguments(self):
        parser=argument_parser()
        self.assertEqual(parser.parse_args(['home']).skill,'home')
        self.assertEqual(parser.parse_args(['place','green']).object,'green')
        args=parser.parse_args(['pick','green','gray'])
        self.assertEqual((args.object,args.zone),('green','gray'))
        for argv in [[],['home','green'],['place'],['pick','green'],['unknown']]:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                parser.parse_args(argv)
            self.assertEqual(exc.exception.code,2)

    def test_current_object_array_updates_from_gazebo(self):
        obj={'id':'green_box','color':'green','position':[.25,0,.735]}
        node=SimpleNamespace(lock=threading.RLock(),object_map={'green_box':obj},objects=[obj])
        t=TransformStamped();t.child_frame_id='green_box';t.transform.translation.x=.3
        t.transform.translation.y=.12;t.transform.translation.z=.735;t.transform.rotation.w=1.
        RobotSkill.on_poses(node,TFMessage(transforms=[t]))
        self.assertEqual(node.objects[0]['position'],[.3,.12,.735])
        self.assertEqual(node.objects[0]['color'],'green')

    def test_gripper_waits_for_subscription_discovery(self):
        gripper = Mock()
        gripper.get_subscription_count.side_effect = [0, 0, 1]
        stopping = Mock()
        stopping.wait.return_value = False
        node = SimpleNamespace(gripper=gripper, stopping=stopping, wait_sim=Mock(),
                               settings={'service_timeout': 10., 'gripper_force': 5.,
                                         'grip_wait': 1., 'release_wait': .5})
        RobotSkill.drive_gripper(node, True)
        self.assertEqual(stopping.wait.call_count, 2)
        self.assertTrue(gripper.publish.called)
        self.assertEqual(list(gripper.publish.call_args.args[0].data), [5., 5.])
        node.wait_sim.assert_called_once_with(1.)

if __name__=='__main__':unittest.main()
