#!/usr/bin/env python3
"""Report visible coloured boxes in workcell zones from RGB camera images."""

import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


# OpenCV hue uses 0..179. Orange gripper fingers are deliberately excluded.
COLOUR_HUES = {
    'red': [(0, 10), (170, 179)],
    'green': [(40, 85)],
    'yellow': [(20, 38)],
    'blue': [(90, 130)],
    'purple': [(131, 169)],
}
ZONE_COLOURS = {
    'zone_Intermediate': 'gray',
    'zone_1_red': 'red',
    'zone_2_green': 'green',
    'zone_3_yellow': 'yellow',
}


def rotation_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class WorkcellVision:
    """Geometry/calibration plus RGB segmentation; never uses box model poses."""

    def __init__(self, world_file, object_height, object_width, saturation_min,
                 value_min, min_area_pixels):
        world = ET.parse(world_file).getroot().find('world')
        camera = world.find("model[@name='workcell_camera']")
        pose = np.array(list(map(float, camera.findtext('pose').split())))
        self.camera_position = pose[:3]
        # Gazebo camera: X forward, Y left, Z up; optical: X right, Y down, Z forward.
        gz_to_optical = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        self.world_to_camera = gz_to_optical @ rotation_rpy(*pose[3:]).T
        self.zones = {}
        for name, colour in ZONE_COLOURS.items():
            model = world.find(f"model[@name='{name}']")
            if model is None:
                raise ValueError(f'Missing zone {name} in {world_file}')
            pos = np.array(list(map(float, model.findtext('pose').split())))
            if not np.allclose(pos[3:], 0):
                raise ValueError('Zone rotation must be zero for this workcell detector')
            size = np.array(list(map(float, model.findtext(
                'link/visual/geometry/box/size').split())))
            lo, hi = pos[:2] - size[:2] / 2, pos[:2] + size[:2] / 2
            self.zones[name] = dict(colour=colour, lo=lo, hi=hi,
                                    marker_z=pos[2] + size[2] / 2)
        table = world.find("model[@name='work_table']/link")
        collision = next(c for c in table.findall('collision')
                         if c.find('geometry/box') is not None)
        table_pose = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
        table_size = list(map(float, collision.findtext('geometry/box/size').split()))
        self.object_top_z = table_pose[2] + table_size[2] / 2 + object_height
        self.object_width = object_width
        self.saturation_min = saturation_min
        self.value_min = value_min
        self.min_area_pixels = min_area_pixels

    def project(self, points, k, d):
        camera = (self.world_to_camera @ (np.asarray(points) - self.camera_position).T).T
        if np.any(camera[:, 2] <= 0):
            raise ValueError('A zone is behind the camera')
        pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, d)
        return pixels.reshape(-1, 2)

    def unproject(self, pixels, z, k, d):
        normal = cv2.undistortPoints(np.asarray(pixels, np.float64).reshape(-1, 1, 2), k, d)
        rays = np.column_stack((normal.reshape(-1, 2), np.ones(len(normal))))
        rays = (self.world_to_camera.T @ rays.T).T
        scale = (z - self.camera_position[2]) / rays[:, 2]
        return self.camera_position + rays * scale[:, None]

    def analyse(self, frame, k, d):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        objects = []
        kernel = np.ones((3, 3), np.uint8)
        for colour, hue_ranges in COLOUR_HUES.items():
            mask = np.zeros(frame.shape[:2], np.uint8)
            for lo, hi in hue_ranges:
                mask |= cv2.inRange(hsv, (lo, self.saturation_min, self.value_min),
                                    (hi, 255, 255))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) < self.min_area_pixels:
                    continue
                component = np.zeros_like(mask)
                cv2.drawContours(component, [contour], -1, 255, -1)
                # Bright upper faces avoid the lateral-face perspective offset.
                values = hsv[:, :, 2][(component > 0) & (mask > 0)]
                cutoff = max(self.value_min, float(np.percentile(values, 90)) - 12)
                top = (component > 0) & (mask > 0) & (hsv[:, :, 2] >= cutoff)
                ys, xs = np.nonzero(top)
                if len(xs) < self.min_area_pixels:
                    continue
                top_world = self.unproject(np.column_stack((xs, ys)), self.object_top_z, k, d)
                xy_lo, xy_hi = np.percentile(top_world[:, :2], [2, 98], axis=0)
                extent = xy_hi - xy_lo
                # Reject large zone backgrounds and tiny colourful fragments.
                if np.any(extent < self.object_width * .45) or np.any(extent > self.object_width * 1.6):
                    continue
                centre = (xy_lo + xy_hi) / 2
                objects.append(dict(colour=colour, centre=centre,
                                    contour=contour, pixel=np.array([np.mean(xs), np.mean(ys)])))

        zones = {}
        debug = frame.copy()
        height, width = frame.shape[:2]
        for name, zone in self.zones.items():
            lo, hi, z = zone['lo'], zone['hi'], zone['marker_z']
            polygon = self.project([[lo[0], lo[1], z], [lo[0], hi[1], z],
                                    [hi[0], hi[1], z], [hi[0], lo[1], z]], k, d)
            in_frame = bool(np.all((polygon[:, 0] >= 0) & (polygon[:, 0] < width)
                                   & (polygon[:, 1] >= 0) & (polygon[:, 1] < height)))
            matches = []
            for obj in objects:
                # Classify the box by whether its estimated centre is inside the zone.
                if np.all(obj['centre'] >= lo) and np.all(obj['centre'] <= hi):
                    matches.append({'color': obj['colour'],
                                    'position_xy_m': [round(float(x), 4) for x in obj['centre']]})
            colours = sorted(set(obj['color'] for obj in matches))
            occupied = bool(matches) if in_frame else None
            zones[name] = {
                'zone_color': zone['colour'],
                'role': 'auxiliary' if name == 'zone_Intermediate' else 'main',
                'state': ('occupied' if occupied else 'empty') if in_frame else 'unknown',
                'has_object': occupied,
                'object_color': colours[0] if len(colours) == 1 else None,
                'object_colors': colours,
                'objects': matches,
            }
            cv2.polylines(debug, [np.rint(polygon).astype(np.int32)], True,
                          (0, 255, 255) if occupied else (255, 255, 255), 2)
            label = f"{zone['colour']}: {','.join(colours) or zones[name]['state']}"
            anchor = np.rint(polygon[np.argmax(polygon[:, 1])]).astype(int)
            cv2.putText(debug, label, (int(anchor[0]), min(height - 5, int(anchor[1]) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
        for obj in objects:
            cv2.drawContours(debug, [obj['contour']], -1, (255, 255, 255), 1)
        return zones, debug


class CameraCheck(Node):
    def __init__(self):
        super().__init__('camera_check')
        defaults = {
            'world_file': str(Path(get_package_share_directory('ur3e_llm_control')) / 'description/workcell.world'),
            'image_topic': '/workcell_camera/image_raw',
            'camera_info_topic': '/workcell_camera/camera_info',
            'output_topic': '/camera_check/zones',
            'debug_image_topic': '/camera_check/image_debug',
            'publish_rate': 2.0,
            'image_timeout': 3.0,
            'object_height': .07,
            'object_width': .05,
            'saturation_min': 175,
            'value_min': 90,
            'min_area_pixels': 80,
        }
        self.parameters = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        p = self.parameters
        for name in ('publish_rate', 'image_timeout', 'object_height', 'object_width'):
            if not math.isfinite(p[name]) or p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('saturation_min', 'value_min'):
            if not 0 <= p[name] <= 255:
                raise ValueError(f'{name} must be in 0..255')
        if p['min_area_pixels'] <= 0:
            raise ValueError('min_area_pixels must be positive')
        self.vision = WorkcellVision(p['world_file'], p['object_height'], p['object_width'],
                                    p['saturation_min'], p['value_min'], p['min_area_pixels'])
        self.bridge = CvBridge()
        self.image = None
        self.info = None
        self.image_received_at = 0.
        self.last_log = None
        self.output = self.create_publisher(String, p['output_topic'], 10)
        self.debug = self.create_publisher(Image, p['debug_image_topic'], 1)
        self.create_subscription(Image, p['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_timer(1 / p['publish_rate'], self.check)
        self.get_logger().info(f"Waiting for {p['image_topic']} and {p['camera_info_topic']}; output: {p['output_topic']}")

    def on_image(self, msg):
        self.image = msg
        self.image_received_at = time.monotonic()

    def on_info(self, msg):
        self.info = msg

    def check(self):
        reason = None
        if self.image is None or self.info is None:
            reason = 'waiting_for_image_and_camera_info'
        elif time.monotonic() - self.image_received_at > self.parameters['image_timeout']:
            reason = 'camera_image_timeout'
        elif self.image.width != self.info.width or self.image.height != self.info.height:
            reason = 'image_camera_info_size_mismatch'
        if reason:
            self.publish_unknown(reason)
            return
        try:
            k = np.array(self.info.k, np.float64).reshape(3, 3)
            if not np.all(np.isfinite(k)) or min(k[0, 0], k[1, 1]) <= 0:
                raise ValueError('Invalid CameraInfo intrinsics')
            d = np.array(self.info.d, np.float64) if self.info.d else np.zeros(5)
            frame = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
            zones, debug = self.vision.analyse(frame, k, d)
            payload = {
                'stamp': {'sec': self.image.header.stamp.sec, 'nanosec': self.image.header.stamp.nanosec},
                'frame_id': self.image.header.frame_id,
                'valid': all(z['state'] != 'unknown' for z in zones.values()),
                'zones': zones,
            }
            self.output.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            signature = [(name, z['state'], z['object_colors']) for name, z in zones.items()]
            if signature != self.last_log:
                self.get_logger().info(' | '.join(f"{name}: {z['state']} {','.join(z['object_colors'])}" for name, z in zones.items()))
                self.last_log = signature
            if self.debug.get_subscription_count():
                msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                msg.header = self.image.header
                self.debug.publish(msg)
        except (ValueError, cv2.error, CvBridgeError, RuntimeError) as exc:
            self.publish_unknown(str(exc))

    def publish_unknown(self, reason):
        zones = {name: {'zone_color': colour,
                        'role': 'auxiliary' if name == 'zone_Intermediate' else 'main',
                        'state': 'unknown', 'has_object': None, 'object_color': None,
                        'object_colors': [], 'objects': []}
                 for name, colour in ZONE_COLOURS.items()}
        self.output.publish(String(data=json.dumps({'valid': False, 'reason': reason, 'zones': zones})))
        if self.last_log != reason:
            self.get_logger().warning(reason)
            self.last_log = reason


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraCheck()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
