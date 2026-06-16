#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan


class APFAvoidance:
    def __init__(
        self,
        influence_radius=3.0,
        max_offset=0.9,
        gain=0.7,
        sample_step=3,
        emergency_radius=0.55,
        tangent_gain=0.75,
        path_sector_angle=0.65,
    ):
        self.influence_radius = influence_radius
        self.max_offset = max_offset
        self.gain = gain
        self.sample_step = max(1, sample_step)
        self.emergency_radius = emergency_radius
        self.tangent_gain = tangent_gain
        self.path_sector_angle = path_sector_angle
        self.locked_turn_side = None
        self.lock_release_distance = self.influence_radius * 0.9

    def compute_offset(self, scan, yaw=0.0):
        offset_x, offset_y, _, _, _ = self.compute_avoidance(scan, yaw, (1.0, 0.0))
        return offset_x, offset_y

    def compute_avoidance(self, scan, yaw=0.0, goal_body_vector=None):
        if scan is None:
            return 0.0, 0.0, False, float("inf"), "none"

        goal_x, goal_y = self.normalize_body_vector(goal_body_vector)
        goal_angle = math.atan2(goal_y, goal_x)

        body_x = 0.0
        body_y = 0.0
        min_distance = float("inf")
        path_min_distance = float("inf")
        obstacle_detected = False
        path_blocked = False

        left_clearance = 0.0
        right_clearance = 0.0
        left_count = 0
        right_count = 0

        for index in range(0, len(scan.ranges), self.sample_step):
            distance = scan.ranges[index]
            if not self.is_valid_distance(distance, scan):
                continue

            angle = scan.angle_min + index * scan.angle_increment
            relative_angle = self.angle_diff(angle, goal_angle)
            clearance = min(distance, self.influence_radius)

            if 0.0 < relative_angle <= math.pi / 2.0:
                left_clearance += clearance
                left_count += 1
            elif -math.pi / 2.0 <= relative_angle < 0.0:
                right_clearance += clearance
                right_count += 1

            min_distance = min(min_distance, distance)
            if distance > self.influence_radius:
                continue

            obstacle_detected = True
            direction_x = -math.cos(angle)
            direction_y = -math.sin(angle)
            strength = self.gain * (1.0 / distance - 1.0 / self.influence_radius) / (distance * distance)
            body_x += direction_x * strength
            body_y += direction_y * strength

            if abs(relative_angle) <= self.path_sector_angle:
                path_blocked = True
                path_min_distance = min(path_min_distance, distance)

        if not obstacle_detected:
            self.locked_turn_side = None
            return 0.0, 0.0, False, min_distance, "none"

        left_score = left_clearance / max(left_count, 1)
        right_score = right_clearance / max(right_count, 1)
        measured_turn_side = "left" if left_score >= right_score else "right"
        turn_side = measured_turn_side

        if path_blocked:
            if self.locked_turn_side is None or min_distance >= self.lock_release_distance:
                self.locked_turn_side = measured_turn_side
            turn_side = self.locked_turn_side

            if path_min_distance == float("inf"):
                path_min_distance = min_distance
            tangent_scale = self.scale_by_distance(path_min_distance)
            if turn_side == "left":
                tangent_x = -goal_y
                tangent_y = goal_x
            else:
                tangent_x = goal_y
                tangent_y = -goal_x
            body_x += tangent_x * self.tangent_gain * tangent_scale
            body_y += tangent_y * self.tangent_gain * tangent_scale
        else:
            self.locked_turn_side = None
            turn_side = "repulse"

        if min_distance <= self.emergency_radius:
            emergency_scale = 1.0 + (self.emergency_radius - min_distance) / max(self.emergency_radius, 0.01)
            body_x *= emergency_scale
            body_y *= emergency_scale

        body_x, body_y = self.limit_vector(body_x, body_y, self.max_offset)
        map_x, map_y = self.rotate_body_to_map(body_x, body_y, yaw)
        return map_x, map_y, True, min_distance, turn_side

    def is_valid_distance(self, distance, scan):
        if math.isinf(distance) or math.isnan(distance):
            return False
        if distance < scan.range_min or distance > scan.range_max:
            return False
        return True

    def normalize_body_vector(self, vector):
        if vector is None:
            return 1.0, 0.0
        x, y = vector
        length = math.hypot(x, y)
        if length == 0.0:
            return 1.0, 0.0
        return x / length, y / length

    def scale_by_distance(self, distance):
        usable_range = max(self.influence_radius - self.emergency_radius, 0.01)
        scale = (self.influence_radius - distance) / usable_range
        return min(max(scale, 0.0), 1.0)

    def angle_diff(self, angle, reference):
        return (angle - reference + math.pi) % (2.0 * math.pi) - math.pi

    def limit_vector(self, x, y, limit):
        length = math.hypot(x, y)
        if length <= limit or length == 0.0:
            return x, y
        scale = limit / length
        return x * scale, y * scale

    def rotate_body_to_map(self, body_x, body_y, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = body_x * cos_yaw - body_y * sin_yaw
        map_y = body_x * sin_yaw + body_y * cos_yaw
        return map_x, map_y


class APFAvoidanceDebugNode:
    def __init__(self):
        rospy.init_node("apf_avoidance_debug")
        self.avoidance = APFAvoidance(
            influence_radius=rospy.get_param("~apf_influence_radius", 3.0),
            max_offset=rospy.get_param("~apf_max_offset", 0.9),
            gain=rospy.get_param("~apf_gain", 0.7),
            emergency_radius=rospy.get_param("~apf_emergency_radius", 0.55),
            tangent_gain=rospy.get_param("~apf_tangent_gain", 0.75),
            path_sector_angle=rospy.get_param("~apf_path_sector_angle", 0.65),
        )
        self.offset_pub = rospy.Publisher("/avoidance/offset", Point, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        offset_x, offset_y, active, min_distance, turn_side = self.avoidance.compute_avoidance(
            msg,
            0.0,
            (1.0, 0.0),
        )
        self.offset_pub.publish(Point(x=offset_x, y=offset_y, z=0.0))
        rospy.loginfo_throttle(
            1.0,
            "APF 회피 offset: x %.3f m, y %.3f m, 활성 %s, 최소 거리 %.2f m, 우회 %s",
            offset_x,
            offset_y,
            active,
            min_distance,
            turn_side,
        )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        APFAvoidanceDebugNode().run()
    except rospy.ROSInterruptException:
        pass