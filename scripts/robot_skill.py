#!/usr/bin/env python3
"""Execute home, place(object)=grasp/hold, pick(object,zone)=transfer skills."""

import copy
import argparse
import json
import uuid
import math
import threading
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.duration import Duration as RosDuration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage

from robot_action import RobotAction, SkillError, pose_at, quaternion_matrix


ARM_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


class RobotSkill(Node):
    def __init__(self):
        super().__init__('robot_skill', parameter_overrides=[Parameter('use_sim_time', value=True)])
        share = Path(get_package_share_directory('ur3e_llm_control'))
        defaults = yaml.safe_load((share / 'config/robot_skill.yaml').read_text())['robot_skill']['ros__parameters']
        defaults.pop('use_sim_time', None)
        defaults['world_file'] = str(share / 'description/workcell.world')
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.request_id = self.declare_parameter('request_id', '').value
        for name in ('grasp_offset', 'table_clearance', 'gripper_force', 'grip_wait',
                     'release_wait', 'hold_check_seconds', 'pose_timeout',
                     'service_timeout', 'execution_timeout', 'cartesian_step', 'cartesian_speed',
                     'max_cartesian_joint_speed', 'placement_tolerance', 'lift_tolerance'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.settings['gripper_force'] > 5:
            raise ValueError('gripper_force exceeds URDF limit (5 N per finger)')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            if not 0 < self.settings[name] <= 1:
                raise ValueError(f'{name} must be in (0,1]')
        self.home_joints = yaml.safe_load((share / 'config/initial_positions.yaml').read_text())
        self.lock = threading.RLock()
        self.stopping = threading.Event()
        self.held_object = None
        self.held_offset = None
        self.joints = None
        self.joints_received = 0.
        self.object_map, self.zones, self.table_collisions = self.load_world(self.settings['world_file'])
        self.objects = list(self.object_map.values())  # Array of coloured objects/current poses.
        self.object_aliases = {alias: name for name, obj in self.object_map.items() for alias in (name, obj['color'])}
        self.zone_aliases = {alias: name for name, zone in self.zones.items() for alias in (name, zone['color'])}
        self.gripper = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 1)
        self.result_signal = self.create_publisher(
            String, '/robot_skill/events',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(TFMessage, self.settings['object_pose_topic'], self.on_poses, qos_profile_sensor_data)
        self.action = RobotAction(self)

    @staticmethod
    def load_world(path):
        world = ET.parse(path).getroot().find('world')
        objects, zones, tables = {}, {}, []
        for model in world.findall('model'):
            name = model.get('name')
            if name.endswith('_box'):
                values = list(map(float, model.findtext('pose').split()))
                obj_pose = pose_at(*values[:3], downward=False)
                if any(abs(x) > 1e-8 for x in values[3:]):
                    raise ValueError('Initial workcell boxes must be upright')
                dimensions = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
                objects[name] = {'id': name, 'color': name.removesuffix('_box'), 'size': dimensions,
                                 'pose': obj_pose, 'position': values[:3], 'received': 0.}
            elif name.startswith('zone_'):
                values = list(map(float, model.findtext('pose').split()))
                dimensions = list(map(float, model.findtext('link/visual/geometry/box/size').split()))
                colour = 'gray' if name == 'zone_Intermediate' else name.split('_')[-1]
                zones[name] = {'id': name, 'color': colour, 'position': values[:3], 'size': dimensions}
            elif name == 'work_table':
                for collision in model.findall('link/collision'):
                    dimensions = list(map(float, collision.findtext('geometry/box/size').split()))
                    values = list(map(float, collision.findtext('pose').split()))
                    tables.append(RobotAction.collision('skill_table_' + collision.get('name'),
                                                        dimensions, pose_at(*values[:3], downward=False)))
        return objects, zones, tables

    def on_joints(self, msg):
        with self.lock:
            self.joints, self.joints_received = msg, time.monotonic()

    def on_poses(self, msg):
        now = time.monotonic()
        with self.lock:
            for transform in msg.transforms:
                if transform.child_frame_id not in self.object_map:
                    continue
                t, q = transform.transform.translation, transform.transform.rotation
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
                pose.orientation = copy.deepcopy(q)
                self.object_map[transform.child_frame_id]['pose'] = pose
                self.object_map[transform.child_frame_id]['position'] = [t.x, t.y, t.z]
                self.object_map[transform.child_frame_id]['received'] = now

    def joint_snapshot(self):
        with self.lock:
            if self.joints is None or time.monotonic()-self.joints_received > self.settings['pose_timeout']:
                raise SkillError('SENSOR_UNAVAILABLE', 'No fresh joint states')
            if not all(name in self.joints.name for name in ARM_JOINTS):
                raise SkillError('SENSOR_UNAVAILABLE', 'Incomplete arm joint states')
            return copy.deepcopy(self.joints)

    def object_pose(self, object_id):
        with self.lock:
            obj = self.object_map[object_id]
            if time.monotonic()-obj['received'] > self.settings['pose_timeout']:
                raise SkillError('SENSOR_UNAVAILABLE', f'No fresh Gazebo pose for {object_id}; restart workcell with pose bridge')
            return copy.deepcopy(obj['pose'])

    def object_snapshot(self):
        with self.lock:
            return [{'id': name, 'color': obj['color'], 'size': list(obj['size']),
                     'position': [obj['pose'].position.x, obj['pose'].position.y, obj['pose'].position.z],
                     'live': time.monotonic()-obj['received'] <= self.settings['pose_timeout']}
                    for name, obj in self.object_map.items()]

    def ready(self):
        self.joint_snapshot()
        for name in self.object_map:
            self.object_pose(name)
        if self.get_clock().now().nanoseconds <= 0:
            raise SkillError('SENSOR_UNAVAILABLE', 'No simulation clock')
        self.action.update_scene()
        self.action.mount_table_exemption()

    def wait_sim(self, duration):
        start = self.get_clock().now().nanoseconds
        deadline = time.monotonic() + self.settings['execution_timeout']
        while (self.get_clock().now().nanoseconds-start)/1e9 < duration:
            if self.stopping.wait(.02):
                raise SkillError('EXECUTION_FAILED', 'Node stopping')
            if time.monotonic() > deadline:
                raise SkillError('EXECUTION_FAILED', 'Simulation paused or too slow')

    def drive_gripper(self, close):
        deadline = time.monotonic() + self.settings['service_timeout']
        while self.gripper.get_subscription_count() == 0:
            if self.stopping.wait(.05) or time.monotonic() >= deadline:
                raise SkillError('EXECUTION_FAILED', 'Gripper controller unavailable')
        drive = self.settings['gripper_force'] * (1 if close else -1)
        message = Float64MultiArray(data=[drive, drive])
        self.gripper.publish(message)
        self.wait_sim(self.settings['grip_wait'] if close else self.settings['release_wait'])
        self.gripper.publish(message)

    def zone_empty(self, zone_id):
        target = self.zones[zone_id]
        for obj in self.object_snapshot():
            if obj['id'] == self.held_object:
                continue
            if not obj['live']:
                raise SkillError('SENSOR_UNAVAILABLE', f"Object pose stale: {obj['id']}")
            distance = np.abs(np.array(obj['position'][:2]) - np.array(target['position'][:2]))
            footprint = (np.array(obj['size'][:2]) + np.array(target['size'][:2])) / 2
            if np.all(distance < footprint):
                raise SkillError('PLANNING_FAILED', f"Destination {zone_id} occupied by {obj['id']}")

    def publish_result(self, skill, status, error_code=None, detail=None, parameters=None):
        event = {'request_id': getattr(self, 'request_id', '') or str(uuid.uuid4()), 'skill': skill,
                 'parameters': list(parameters or []),
                 'status': status, 'error_code': error_code, 'detail': detail}
        # A CLI process exits after one skill. Give an already-starting validator
        # time to connect, then acknowledge delivery before destroying the writer.
        deadline = time.monotonic() + 1.
        while self.result_signal.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
        self.result_signal.publish(String(data=json.dumps(event)))
        self.result_signal.wait_for_all_acked(timeout=RosDuration(seconds=2.))

    def verify_held(self, wait=True):
        if self.held_object is None:
            raise SkillError('GRASP_FAILED', 'No object is held')
        # Verify the real Gazebo box follows the tool; attachment is planning-only.
        tool = self.action.tool_pose()
        q = [tool.orientation.x, tool.orientation.y, tool.orientation.z, tool.orientation.w]
        expected = np.array([tool.position.x, tool.position.y, tool.position.z]) + quaternion_matrix(q) @ self.held_offset
        checks = 5 if wait else 1
        for _ in range(checks):
            actual = self.object_pose(self.held_object)
            point = np.array([actual.position.x, actual.position.y, actual.position.z])
            if np.linalg.norm(point-expected) > self.settings['lift_tolerance']:
                raise SkillError('GRASP_FAILED', 'Real box did not follow gripper or slipped')
            if wait:
                self.wait_sim(self.settings['hold_check_seconds']/checks)

    def lift_clear(self):
        tool = self.action.tool_pose()
        min_z = self.table_z + max(obj['size'][2] for obj in self.object_map.values()) + .12 + .03
        if tool.position.z < min_z:
            tool.position.z = min_z
            self.action.cartesian(tool)
            if self.held_object:
                self.verify_held()

    def home(self):
        self.lift_clear()
        self.action.home()
        joints = self.joint_snapshot()
        actual = dict(zip(joints.name, joints.position))
        if any(abs(actual[name]-value) > .02 for name, value in self.home_joints.items()):
            raise SkillError('EXECUTION_FAILED', 'Home joint verification failed')
        if self.held_object:
            self.verify_held()

    @property
    def table_z(self):
        top = next(c for c in self.table_collisions if c.id == 'skill_table_top_collision')
        return top.primitive_poses[0].position.z + top.primitives[0].dimensions[2]/2

    def place(self, object_id):
        if self.held_object:
            if self.held_object == object_id:
                self.verify_held()
                return
            raise SkillError('INVALID_OBJECT', f'Already holding {self.held_object}')
        obj = self.object_pose(object_id)
        q = [obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w]
        if quaternion_matrix(q)[2, 2] < .95:
            raise SkillError('PLANNING_FAILED', 'Only upright boxes are supported')
        if abs(obj.position.z-(self.table_z+self.object_map[object_id]['size'][2]/2)) > .015:
            raise SkillError('PLANNING_FAILED', 'Object is not resting on the table')
        grasp = pose_at(obj.position.x, obj.position.y, obj.position.z+self.settings['grasp_offset'])
        hover = copy.deepcopy(grasp)
        hover.position.z += self.settings['table_clearance']
        self.lift_clear()
        self.drive_gripper(False)
        self.action.allow_target(object_id, True)
        self.action.move_pose(hover, descend_to=grasp)
        self.action.cartesian(grasp)
        self.drive_gripper(True)
        self.held_offset = self.action.attach(object_id)
        self.held_object = object_id
        self.action.cartesian(hover)
        self.verify_held()
        actual = self.object_pose(object_id)
        if actual.position.z - obj.position.z < self.settings['table_clearance']-self.settings['lift_tolerance']:
            raise SkillError('GRASP_FAILED', 'Box did not rise sufficiently')

    def pick(self, object_id, zone_id):
        self.zone_empty(zone_id)
        self.place(object_id)
        self.verify_held()
        self.zone_empty(zone_id)
        target = self.zones[zone_id]
        release = pose_at(target['position'][0], target['position'][1],
                          self.table_z+self.object_map[object_id]['size'][2]/2+self.settings['grasp_offset']+.002)
        hover = copy.deepcopy(release)
        hover.position.z += self.settings['table_clearance']
        self.action.move_pose(hover, descend_to=release)
        self.verify_held()
        self.action.cartesian(release)
        self.verify_held(wait=False)
        self.drive_gripper(False)
        self.held_object, self.held_offset = None, None
        self.action.detach(object_id)
        self.action.cartesian(hover)
        self.action.allow_target(object_id, False)
        self.wait_sim(.5)
        actual = self.object_pose(object_id)
        delta = np.linalg.norm(np.array([actual.position.x, actual.position.y])-np.array(target['position'][:2]))
        resting_z = self.table_z+self.object_map[object_id]['size'][2]/2
        if delta > self.settings['placement_tolerance'] or abs(actual.position.z-resting_z) > .015:
            raise SkillError('PLACEMENT_FAILED', 'Released object is outside destination or did not settle on table')

    def recover_held_object(self):
        # A CLI invocation ends after each skill. MoveIt's attachment retains
        # the held object's geometry/offset for the next invocation.
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene = self.action.call(self.action.get_scene, req).scene
        for attached in scene.robot_state.attached_collision_objects:
            name = attached.object.id.removeprefix('skill_object_')
            if name not in self.object_map:
                continue
            if self.held_object is not None:
                raise SkillError('EXECUTION_FAILED', 'Multiple held objects in planning scene')
            tool = self.action.tool_pose()
            actual = self.object_pose(name)
            q = [tool.orientation.x, tool.orientation.y, tool.orientation.z, tool.orientation.w]
            delta = np.array([actual.position.x-tool.position.x,
                              actual.position.y-tool.position.y,
                              actual.position.z-tool.position.z])
            offset = quaternion_matrix(q).T @ delta
            if np.linalg.norm(offset[:2]) > self.settings['lift_tolerance'] or abs(offset[2]-self.settings['grasp_offset']) > self.settings['lift_tolerance']:
                self.action.detach(name)
                raise SkillError('GRASP_FAILED', 'Previously held object has slipped')
            self.held_object = name
            self.held_offset = offset
        if self.held_object:
            try:
                self.verify_held(wait=False)
            except SkillError:
                dropped = self.held_object
                self.held_object, self.held_offset = None, None
                self.action.detach(dropped)
                raise

    def run_skill(self, skill, object_name=None, zone_name=None):
        if object_name is not None and object_name not in self.object_aliases:
            raise SkillError('INVALID_OBJECT', f'Unknown object: {object_name}')
        if zone_name is not None and zone_name not in self.zone_aliases:
            raise SkillError('INVALID_OBJECT', f'Unknown zone: {zone_name}')
        deadline = time.monotonic() + 15.
        while True:
            try:
                self.joint_snapshot()
                for name in self.object_map:
                    self.object_pose(name)
                break
            except SkillError:
                if time.monotonic() >= deadline or self.stopping.wait(.1):
                    raise SkillError('SENSOR_UNAVAILABLE', 'Start ur3e_workcell.launch.py and unpause Gazebo')
        self.recover_held_object()
        try:
            self.ready()
            if skill == 'home':
                self.home()
            elif skill == 'place':
                self.place(self.object_aliases[object_name])
            else:
                self.pick(self.object_aliases[object_name], self.zone_aliases[zone_name])
        except SkillError as exc:
            if exc.code == 'GRASP_FAILED' and self.held_object:
                dropped = self.held_object
                self.held_object, self.held_offset = None, None
                self.action.detach(dropped)
            raise
        finally:
            if self.action.target_contacts and not self.held_object and not self.stopping.is_set():
                self.action.allow_target(self.action.target_contacts, False)


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    skills = parser.add_subparsers(dest='skill', required=True)
    skills.add_parser('home', help='Return arm to its launch-time joint posture')
    place = skills.add_parser('place', help='Grasp and hold an object')
    place.add_argument('object', help='Colour or model ID, e.g. green or green_box')
    pick = skills.add_parser('pick', help='Transfer an object to a zone')
    pick.add_argument('object', help='Colour or model ID')
    pick.add_argument('zone', help='Zone ID or colour, e.g. zone_Intermediate or gray')
    return parser


def main(args=None):
    import sys
    argv = sys.argv if args is None else ['robot_skill.py', *args]
    command = argument_parser().parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
    rclpy.init(args=args)
    node = None
    pool = ThreadPoolExecutor(max_workers=1)
    result_code = 0
    failure_code = None
    detail = None
    try:
        node = RobotSkill()
        job = pool.submit(node.run_skill, command.skill,
                          getattr(command, 'object', None), getattr(command, 'zone', None))
        while rclpy.ok() and not job.done():
            rclpy.spin_once(node, timeout_sec=.1)
        job.result()
        node.get_logger().info(f'{command.skill}: completed')
    except SkillError as exc:
        failure_code, detail = exc.code, exc.detail
        if node:
            node.get_logger().error(f'{command.skill}: {exc.detail}')
        result_code = 1
    except KeyboardInterrupt:
        result_code = 130
        failure_code, detail = 'EXECUTION_FAILED', 'Interrupted'
    except Exception as exc:
        result_code = 1
        failure_code, detail = 'EXECUTION_FAILED', str(exc)
        if node:
            node.get_logger().error(f'{command.skill}: {detail}')
    finally:
        if node:
            node.stopping.set()
            if node.action.active_goal:
                node.action.active_goal.cancel_goal_async()
        pool.shutdown(wait=True, cancel_futures=True)
        if node:
            if rclpy.ok():
                node.publish_result(command.skill, 'success' if result_code == 0 else 'failed',
                                    failure_code, detail,
                                    parameters=[value for value in
                                                (getattr(command, 'object', None),
                                                 getattr(command, 'zone', None))
                                                if value is not None])
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())
