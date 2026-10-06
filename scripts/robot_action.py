"""MoveIt execution helpers for robot_skill (not a separate ROS node)."""

import copy
import math
import threading
import time

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                             CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes,
                             PlanningScene, PlanningSceneComponents,
                             RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan,
                             GetPlanningScene, GetPositionFK, GetPositionIK)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class SkillError(Exception):
    def __init__(self, code, detail):
        super().__init__(detail)
        self.code = code
        self.detail = detail


def pose_at(x, y, z, downward=True):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    if downward:
        pose.orientation.x = 1.0  # Rx(pi): tool +Z down, fingers close along world Y.
        pose.orientation.w = 0.0
    else:
        pose.orientation.w = 1.0
    return pose


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion_product(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return [w*u+x*s+y*t-z*v, w*v-x*t+y*s+z*u,
            w*t+x*v-y*u+z*s, w*s-x*u-y*v-z*t]


class RobotAction:
    def __init__(self, node):
        self.node = node
        self.p = node.settings
        self.plan = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cart = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.active_goal = None
        self.target_contacts = None

    def wait_future(self, future, timeout):
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        deadline = time.monotonic() + timeout
        while not future.done():
            if self.node.stopping.is_set():
                raise SkillError('EXECUTION_FAILED', 'Node stopping')
            if time.monotonic() >= deadline:
                raise SkillError('EXECUTION_FAILED', 'ROS response timeout')
            ready.wait(.05)
        try:
            result = future.result()
        except Exception as exc:
            raise SkillError('EXECUTION_FAILED', str(exc)) from exc
        if result is None:
            raise SkillError('EXECUTION_FAILED', 'Empty ROS response')
        return result

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.):
            raise SkillError('PLANNING_FAILED', f'Service unavailable: {client.srv_name}')
        return self.wait_future(client.call_async(request), self.p['service_timeout'])

    def state(self):
        state = RobotState()
        state.joint_state = self.node.joint_snapshot()
        state.is_diff = True
        return state

    def tool_pose(self):
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = [self.p['tool_link']]
        req.robot_state = self.state()
        res = self.call(self.fk, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS or not res.pose_stamped:
            raise SkillError('PLANNING_FAILED', f'FK failed: {res.error_code.val}')
        return res.pose_stamped[0].pose

    def apply(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        req = ApplyPlanningScene.Request(scene=scene)
        if not self.call(self.scene, req).success:
            raise SkillError('PLANNING_FAILED', 'Planning scene update rejected')

    @staticmethod
    def collision(name, dimensions, pose, frame='world'):
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = name
        obj.pose.orientation.w = 1.0
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dimensions))]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def update_scene(self):
        scene = PlanningScene()
        scene.world.collision_objects = copy.deepcopy(self.node.table_collisions)
        for obj in self.node.object_snapshot():
            name = 'skill_object_' + obj['id']
            if obj['id'] == self.node.held_object:
                continue
            scene.world.collision_objects.append(self.collision(
                name, obj['size'], self.node.object_pose(obj['id'])))
        self.apply(scene)

    def collision_pairs(self, pairs):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.call(self.get_scene, req).scene.allowed_collision_matrix
        for a, b, allowed in pairs:
            for name in (a, b):
                if name not in acm.entry_names:
                    for row in acm.entry_values:
                        row.enabled.append(False)
                    acm.entry_names.append(name)
                    acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
            i, j = acm.entry_names.index(a), acm.entry_names.index(b)
            acm.entry_values[i].enabled[j] = bool(allowed)
            acm.entry_values[j].enabled[i] = bool(allowed)
        scene = PlanningScene()
        scene.allowed_collision_matrix = acm
        self.apply(scene)

    def allow_target(self, object_id, allowed):
        self.collision_pairs([('skill_object_' + object_id, link, allowed)
                              for link in ('left_finger_link', 'right_finger_link')])
        self.target_contacts = object_id if allowed else None

    def mount_table_exemption(self):
        self.collision_pairs([('skill_table_top_collision', link, True)
                              for link in ('base_link', 'base_link_inertia')])

    def attach(self, object_id):
        # Planning attachment only. Gazebo still holds the real box by friction.
        actual = self.node.object_pose(object_id)
        tool = self.tool_pose()
        tq = [tool.orientation.x, tool.orientation.y, tool.orientation.z, tool.orientation.w]
        oq = [actual.orientation.x, actual.orientation.y, actual.orientation.z, actual.orientation.w]
        inverse = [-tq[0], -tq[1], -tq[2], tq[3]]
        delta = np.array([actual.position.x-tool.position.x,
                          actual.position.y-tool.position.y,
                          actual.position.z-tool.position.z])
        offset = quaternion_matrix(tq).T @ delta
        relative = pose_at(*offset, downward=False)
        q = quaternion_product(inverse, oq)
        relative.orientation.x, relative.orientation.y, relative.orientation.z, relative.orientation.w = q
        attached = AttachedCollisionObject()
        attached.link_name = self.p['tool_link']
        attached.touch_links = [self.p['tool_link'], 'gripper_base_link',
                                'left_finger_link', 'right_finger_link']
        attached.object = self.collision('skill_object_' + object_id,
                                         self.node.object_map[object_id]['size'], relative,
                                         frame=self.p['tool_link'])
        scene = PlanningScene()
        # MoveIt removes the existing world object when attaching it. An extra
        # REMOVE in the same diff would fail after that automatic transfer.
        scene.robot_state.attached_collision_objects = [attached]
        self.apply(scene)
        return offset

    def detach(self, object_id):
        remove = AttachedCollisionObject()
        remove.link_name = self.p['tool_link']
        remove.object.id = 'skill_object_' + object_id
        remove.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [remove]
        scene.world.collision_objects = [self.collision(
            remove.object.id, self.node.object_map[object_id]['size'], self.node.object_pose(object_id))]
        self.apply(scene)

    def execute_trajectory(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            raise SkillError('PLANNING_FAILED', 'Empty trajectory')
        names = trajectory.joint_trajectory.joint_names
        base_index = names.index('shoulder_pan_joint')
        if any(abs(point.positions[base_index]) > 3*math.pi/4 + 1e-5 for point in points):
            raise SkillError('PLANNING_FAILED', 'Trajectory violates 270-degree base range')
        if not self.execute.wait_for_server(timeout_sec=5.):
            raise SkillError('EXECUTION_FAILED', 'ExecuteTrajectory action unavailable')
        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        handle = self.wait_future(self.execute.send_goal_async(goal), self.p['service_timeout'])
        if not handle.accepted:
            raise SkillError('EXECUTION_FAILED', 'Trajectory rejected')
        self.active_goal = handle
        try:
            result = self.wait_future(handle.get_result_async(), self.p['execution_timeout'])
            if result.status != 4 or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise SkillError('EXECUTION_FAILED', f'Execution error {result.result.error_code.val}')
        except SkillError:
            self.wait_future(handle.cancel_goal_async(), 5.)
            raise
        finally:
            self.active_goal = None

    def motion(self, constraints):
        self.update_scene()
        req = GetMotionPlan.Request()
        motion = req.motion_plan_request
        motion.group_name = self.p['planning_group']
        motion.pipeline_id = 'ompl'
        motion.planner_id = 'RRTConnect'
        motion.start_state = self.state()
        motion.goal_constraints = [constraints]
        motion.num_planning_attempts = 4
        motion.allowed_planning_time = 5.
        motion.max_velocity_scaling_factor = self.p['velocity_scaling']
        motion.max_acceleration_scaling_factor = self.p['acceleration_scaling']
        res = self.call(self.plan, req).motion_plan_response
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise SkillError('PLANNING_FAILED', f'MoveIt planning error {res.error_code.val}')
        self.execute_trajectory(res.trajectory)

    def home(self):
        goal = Constraints(name='home')
        goal.joint_constraints = [JointConstraint(joint_name=name, position=float(value),
                                                  tolerance_above=.005, tolerance_below=.005, weight=1.)
                                  for name, value in self.node.home_joints.items()]
        self.motion(goal)

    def move_pose(self, pose, descend_to=None):
        # Seed IK from current/home and favour short joint changes. Check that
        # the chosen elbow branch can complete the subsequent vertical descent.
        self.update_scene()
        current_state = self.state()
        current = dict(zip(current_state.joint_state.name, current_state.joint_state.position))
        bearing = math.atan2(pose.position.y, pose.position.x + .05)
        seeds = [current, {**current, **self.node.home_joints}]
        for elbow in (1.5708, -1.5708):
            seeds.append({**current, **self.node.home_joints,
                          'shoulder_pan_joint': max(-2.2, min(2.2, bearing-.4)),
                          'elbow_joint': elbow})
        candidates = []
        for seed in seeds:
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.p['planning_group']
            req.ik_request.ik_link_name = self.p['tool_link']
            req.ik_request.pose_stamped.header.frame_id = 'world'
            req.ik_request.pose_stamped.pose = pose
            req.ik_request.avoid_collisions = True
            req.ik_request.robot_state = copy.deepcopy(current_state)
            req.ik_request.robot_state.joint_state.position = [seed[name] for name in current_state.joint_state.name]
            req.ik_request.timeout = Duration(sec=0, nanosec=300000000)
            result = self.call(self.ik, req)
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            state = result.solution
            state.is_diff = True
            values = dict(zip(state.joint_state.name, state.joint_state.position))
            for name in self.node.home_joints:
                if name == 'shoulder_pan_joint':
                    continue
                limit = math.pi if name == 'elbow_joint' else 2*math.pi
                equivalent = [values[name]+turn*2*math.pi for turn in (-1,0,1)]
                valid = [v for v in equivalent if -limit <= v <= limit]
                if valid:
                    values[name] = min(valid, key=lambda v: abs(v-current[name]))
            if abs(values['shoulder_pan_joint']) > 3*math.pi/4:
                continue
            state.joint_state.position = [values[name] for name in state.joint_state.name]
            cost = sum((values[name]-current[name])**2 for name in self.node.home_joints)
            candidates.append((cost, state, values))
        for _, state, values in sorted(candidates, key=lambda candidate: candidate[0]):
            if descend_to is not None:
                check = self.compute_cartesian(descend_to, state)
                if check.error_code.val != MoveItErrorCodes.SUCCESS or check.fraction < .999:
                    continue
            goal = Constraints(name='nearest_valid_ik')
            goal.joint_constraints = [JointConstraint(joint_name=name, position=values[name],
                                                      tolerance_above=.001, tolerance_below=.001, weight=1.)
                                      for name in self.node.home_joints]
            try:
                self.motion(goal)
                return
            except SkillError as exc:
                if exc.code != 'PLANNING_FAILED':
                    raise
        raise SkillError('PLANNING_FAILED', 'No reachable IK branch with a complete collision-free approach')

    def compute_cartesian(self, pose, start_state=None):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state = start_state if start_state is not None else self.state()
        req.group_name = self.p['planning_group']
        req.link_name = self.p['tool_link']
        req.waypoints = [pose]
        req.max_step = self.p['cartesian_step']
        # Relative jump filtering depends on the path's average step and can
        # truncate smooth near-boundary paths. Keep an absolute joint-step cap.
        req.jump_threshold = 0.
        req.revolute_jump_threshold = .25
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = self.p['velocity_scaling']
        req.max_acceleration_scaling_factor = self.p['acceleration_scaling']
        return self.call(self.cart, req)

    def cartesian(self, pose):
        self.update_scene()
        start_state = self.state()
        res = self.compute_cartesian(pose, start_state)
        if res.error_code.val != MoveItErrorCodes.SUCCESS or res.fraction < .999:
            raise SkillError('PLANNING_FAILED', f'Cartesian path incomplete ({res.fraction:.3f}), error {res.error_code.val}')
        # Retain the collision-checked path, with conservative explicit timing.
        points = res.solution.joint_trajectory.points
        current = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
        previous = np.array([current[n] for n in res.solution.joint_trajectory.joint_names])
        elapsed = .05
        for i, point in enumerate(points):
            delta = float(np.max(np.abs(np.array(point.positions) - previous)))
            elapsed += max(.05, delta / self.p['max_cartesian_joint_speed'],
                           self.p['cartesian_step'] / self.p['cartesian_speed'] if i else .05)
            point.time_from_start = Duration(sec=int(elapsed), nanosec=int((elapsed % 1) * 1e9))
            point.velocities = [0.] * len(point.positions)
            point.accelerations = [0.] * len(point.positions)
            previous = np.array(point.positions)
        self.execute_trajectory(res.solution)
