"""Shared, deterministic world model and skill precondition checker.

The LLM chooses steps. This module only checks those steps; it never adds any.
"""

import copy
import math
import re
import time
import unicodedata
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


COLOURS = ('red', 'yellow', 'green', 'blue', 'purple')
ZONE_ALIASES = {'gray': 'zone_Intermediate', 'grey': 'zone_Intermediate',
                'red': 'zone_1_red', 'green': 'zone_2_green', 'yellow': 'zone_3_yellow'}


def canonical(command):
    if not isinstance(command, list) or not command or command[0] not in ('home', 'place', 'pick'):
        raise ValueError('Unknown skill')
    if len(command) != {'home': 1, 'place': 2, 'pick': 3}[command[0]]:
        raise ValueError('Incorrect skill parameters')
    result = list(command)
    if len(result) > 1:
        if not isinstance(result[1], str):
            raise ValueError('INVALID_OBJECT')
        result[1] = result[1].removesuffix('_box')
        if result[1] not in COLOURS:
            raise ValueError('INVALID_OBJECT: ' + result[1])
    if len(result) > 2:
        if not isinstance(result[2], str):
            raise ValueError('INVALID_OBJECT: zone')
        result[2] = ZONE_ALIASES.get(result[2], result[2])
        if result[2] not in ZONE_ALIASES.values():
            raise ValueError('INVALID_OBJECT: ' + result[2])
    return result


def refresh_zones(state):
    for zone_id, zone in state['zones'].items():
        zone['objects'], zone['blocked_by'] = [], []
        for colour, obj in state['objects'].items():
            if colour == state['held']:
                continue
            distance = [abs(obj['position'][i] - zone['position'][i]) for i in (0, 1)]
            if all(distance[i] <= zone['size'][i] / 2 for i in (0, 1)):
                zone['objects'].append(colour)
            # Match robot_skill.zone_empty exactly: even footprint overlap blocks.
            if all(distance[i] < (obj['size'][i] + zone['size'][i]) / 2 for i in (0, 1)):
                zone['blocked_by'].append(colour)
        zone['state'] = 'occupied' if zone['objects'] else 'empty'
    for colour, obj in state['objects'].items():
        obj['location'] = ('held' if colour == state['held'] else
                           next((z for z, v in state['zones'].items()
                                 if colour in v['objects']), 'outside_zones'))
    return state


def apply_skill(state, command):
    command = canonical(command)
    state = refresh_zones(copy.deepcopy(state))
    skill = command[0]
    if skill == 'home':
        state['home'] = True
        return state
    colour = command[1]
    if colour not in state['objects']:
        raise ValueError('INVALID_OBJECT: ' + colour)
    if state['held'] not in (None, colour):
        raise ValueError('Already holding ' + state['held'])
    obj = state['objects'][colour]
    if state['held'] != colour and (not obj['on_table'] or not obj['upright']):
        raise ValueError(colour + ' is not upright and resting on the table')
    if skill == 'place':
        if state['held'] == colour:
            return state  # robot_skill only verifies the existing grip here.
        state['held'] = colour
    else:
        zone_id = command[2]
        if zone_id not in state['zones']:
            raise ValueError('INVALID_OBJECT: ' + zone_id)
        zone = state['zones'][zone_id]
        if zone['blocked_by']:
            raise ValueError('PLANNING_FAILED: ' + zone_id + ' blocked by '
                             + ', '.join(zone['blocked_by'])
                             + '; move the blocking object to a free zone first')
        obj['position'] = [*zone['position'][:2], state['table_z'] + obj['size'][2] / 2]
        obj['on_table'], obj['upright'] = True, True
        state['held'] = None
    state['home'] = False
    return refresh_zones(state)


def check_goal(state, goals):
    for goal in goals:
        command = canonical(goal)
        if command[0] == 'home':
            if not state['home']:
                raise ValueError('Final goal not reached: home')
        elif command[0] == 'place':
            if state['held'] != command[1]:
                raise ValueError('Final goal not reached: holding ' + command[1])
        else:
            obj, zone = state['objects'][command[1]], state['zones'][command[2]]
            if (state['held'] == command[1] or not obj['on_table']
                    or math.dist(obj['position'][:2], zone['position'][:2]) > .015):
                raise ValueError('Final goal not reached: ' + ' '.join(command))


def validate_plan(state, commands, goals):
    if not isinstance(commands, list) or len(commands) > 20:
        raise ValueError('Plan must contain at most 20 steps')
    predicted = copy.deepcopy(state)
    states = []
    for index, command in enumerate(commands):
        try:
            predicted = apply_skill(predicted, command)
        except ValueError as exc:
            raise ValueError(f'Step {index + 1}: {exc}') from exc
        states.append(predicted)
    check_goal(predicted, goals)
    return states


def compare_state(expected, actual, check_home=False):
    if expected['held'] != actual['held']:
        raise ValueError('Actual held object differs from the plan')
    if check_home and not actual['home']:
        raise ValueError('Robot did not reach home')
    for colour, obj in expected['objects'].items():
        if colour == expected['held']:
            continue  # robot_skill verifies real grip relative to tool pose.
        observed = actual['objects'][colour]
        if (math.dist(obj['position'], observed['position']) > .015
                or obj['on_table'] != observed['on_table']
                or obj['upright'] != observed['upright']):
            raise ValueError('Actual position/orientation differs from plan: ' + colour)


def parse_goal(text):
    """Extract the user's goal independently of the LLM's proposed plan.

    Accept CLI skills and simple English/Vietnamese placement requests. Reject
    ambiguous requests rather than letting the planning model redefine a goal.
    """
    text = text.strip()
    goals = []
    for part in text.split(';'):
        tokens = part.strip().split()
        if tokens and tokens[0] in ('home', 'place', 'pick'):
            try:
                goals.append(canonical(tokens))
                continue
            except ValueError:
                pass
        normalized = ''.join(c for c in unicodedata.normalize('NFD', part.lower())
                             if unicodedata.category(c) != 'Mn').replace('đ', 'd')
        for phrase, replacement in (('xanh duong', 'blue'), ('xanh la cay', 'green'),
                                    ('xanh la', 'green'), ('mau do', 'red'),
                                    ('do', 'red'), ('vang', 'yellow'),
                                    ('tim', 'purple'), ('xam', 'gray')):
            normalized = re.sub(r'\b' + phrase + r'\b', replacement, normalized)
        if normalized.strip() in ('home', 've home', 'go home', 'return home'):
            goals.append(['home'])
            continue
        # A bounded grammar avoids silently accepting negation or extra tasks.
        colour = r'(red|yellow|green|blue|purple)(?:_box)?'
        zone = r'(zone_intermediate|zone_1_red|zone_2_green|zone_3_yellow|gray|grey|red|green|yellow)'
        match = re.fullmatch(
            r'\s*(?:put|move|pick|place|dat|chuyen)\s+(?:the\s+|vat\s+|hop\s+)?'
            + colour + r'(?:\s+(?:object|box))?\s+(?:into|in|to|vao)\s+'
            + r'(?:the\s+)?(?:vung\s+|zone\s+)?' + zone + r'(?:\s+zone)?[.!]?\s*', normalized)
        if match:
            destination = 'zone_Intermediate' if match[2] == 'zone_intermediate' else match[2]
            goals.append(canonical(['pick', match[1], destination]))
            continue
        raise ValueError('Yêu cầu chưa rõ. Dùng: pick blue zone_3_yellow, place purple, '
                         'home; hoặc put blue object in yellow zone. Ngăn nhiều mục tiêu bằng dấu ;')
    if not goals:
        raise ValueError('Empty goal')
    return goals


class WorldState:
    """Fresh Gazebo poses + joints + MoveIt attachment; no initial-pose fallback."""

    def __init__(self, node):
        # Use the same world loader/settings as robot_skill to prevent drift.
        from robot_skill import RobotSkill
        share = Path(get_package_share_directory('ur3e_llm_control'))
        settings = yaml.safe_load((share / 'config/robot_skill.yaml').read_text())['robot_skill']['ros__parameters']
        self.objects, self.zones, tables = RobotSkill.load_world(share / 'description/workcell.world')
        top = next(c for c in tables if c.id == 'skill_table_top_collision')
        self.table_z = top.primitive_poses[0].position.z + top.primitives[0].dimensions[2] / 2
        self.home = yaml.safe_load((share / 'config/initial_positions.yaml').read_text())
        self.timeout = settings['pose_timeout']
        self.joints = None
        self.joints_received = self.scene_received = self.scene_requested = 0.
        self.held = None
        self.scene_error = None
        self.pending = None
        self.node = node
        self.client = node.create_client(GetPlanningScene, '/get_planning_scene')
        node.create_subscription(TFMessage, settings['object_pose_topic'], self.on_poses, qos_profile_sensor_data)
        node.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        node.create_timer(.25, self.poll_scene)

    def on_poses(self, msg):
        for transform in msg.transforms:
            obj = self.objects.get(transform.child_frame_id)
            if obj is not None:
                t, q = transform.transform.translation, transform.transform.rotation
                if not all(math.isfinite(value) for value in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
                    obj['received'] = 0.
                    continue
                obj['position'] = [t.x, t.y, t.z]
                obj['upright'] = 1 - 2 * (q.x * q.x + q.y * q.y) >= .95
                obj['received'] = time.monotonic()

    def on_joints(self, msg):
        if not all(math.isfinite(value) for value in msg.position):
            self.joints_received = 0.
            return
        self.joints = dict(zip(msg.name, msg.position))
        self.joints_received = time.monotonic()

    def poll_scene(self):
        if self.pending is not None:
            if not self.pending.done():
                if time.monotonic() - self.scene_requested > self.timeout:
                    self.client.remove_pending_request(self.pending)
                    self.pending.cancel()
                    self.pending = None
                return
            try:
                held = [a.object.id.removeprefix('skill_object_')
                        for a in self.pending.result().scene.robot_state.attached_collision_objects]
                if any(name not in self.objects for name in held) or len(held) > 1:
                    raise ValueError('Unexpected attachment in MoveIt scene')
                self.held = held[0].removesuffix('_box') if held else None
                self.scene_error = None
                self.scene_received = self.scene_requested
            except Exception:
                self.scene_error = 'Cannot read held object from MoveIt'
            self.pending = None
        if self.client.service_is_ready():
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            self.scene_requested = time.monotonic()
            self.pending = self.client.call_async(request)

    def snapshot(self, after=0.):
        now = time.monotonic()
        if (self.scene_error or self.scene_received <= after
                or now - self.scene_received > self.timeout):
            raise ValueError(self.scene_error or 'Waiting for fresh MoveIt attachment state')
        if self.joints is None or self.joints_received <= after or now - self.joints_received > self.timeout:
            raise ValueError('Waiting for fresh robot joint states')
        objects = {}
        for obj in self.objects.values():
            if obj['received'] <= after or now - obj['received'] > self.timeout:
                raise ValueError('Waiting for fresh Gazebo pose: ' + obj['id'])
            objects[obj['color']] = {
                'id': obj['id'], 'size': list(obj['size']), 'position': list(obj['position']),
                'upright': obj['upright'],
                'on_table': abs(obj['position'][2] - self.table_z - obj['size'][2] / 2) <= .015}
        if not all(name in self.joints for name in self.home):
            raise ValueError('Incomplete joint states')
        return refresh_zones({'objects': objects, 'zones': copy.deepcopy(self.zones),
                              'table_z': self.table_z, 'held': self.held,
                              'home': all(abs(self.joints[k] - v) <= .02 for k, v in self.home.items())})
