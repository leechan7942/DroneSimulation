#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan


class VFHObstacleAvoidance:
    def __init__(
        self,
        sector_angle_deg=10.0,
        influence_radius=3.0,
        safe_distance=1.1,
        goal_weight=1.0,
        obstacle_weight=3.0,
        smooth_weight=0.8,
        reverse_penalty=8.0,
        sample_step=1,
        inflation_sectors=2,
    ):
        self.sector_count = max(8, int(round(360.0 / max(sector_angle_deg, 1.0))))
        self.sector_angle = 2.0 * math.pi / self.sector_count
        self.influence_radius = influence_radius
        self.safe_distance = safe_distance
        self.goal_weight = goal_weight
        self.obstacle_weight = obstacle_weight
        self.smooth_weight = smooth_weight
        self.reverse_penalty = reverse_penalty
        self.sample_step = max(1, sample_step)
        self.inflation_sectors = max(0, inflation_sectors)
        self.previous_heading_body = 0.0

    def compute_offset(self, scan, yaw=0.0):
        offset_x, offset_y, _, _, _, _, _ = self.compute_avoidance(
            scan,
            yaw,
            (1.0, 0.0),
            0.3,
        )
        return offset_x, offset_y

    def compute_avoidance(self, scan, yaw=0.0, goal_body_vector=None, step_distance=0.3):
        selected_heading, active, min_distance, goal_diff, selected_distance, status, speed_scale = self.select_heading(
            scan,
            goal_body_vector,
        )
        move_distance = max(step_distance, 0.0) * speed_scale
        body_x = math.cos(selected_heading) * move_distance
        body_y = math.sin(selected_heading) * move_distance
        map_x, map_y = self.rotate_body_to_map(body_x, body_y, yaw)
        return (
            map_x,
            map_y,
            active,
            min_distance,
            status,
            math.degrees(selected_heading),
            math.degrees(goal_diff),
        )

    def select_heading(self, scan, goal_body_vector=None):
        goal_x, goal_y = self.normalize_body_vector(goal_body_vector)
        goal_heading = math.atan2(goal_y, goal_x)

        if scan is None:
            self.previous_heading_body = goal_heading
            return goal_heading, False, float("inf"), 0.0, self.influence_radius, "scan 없음", 1.0

        sector_distances, min_distance = self.build_histogram(scan)
        active = min_distance <= self.influence_radius

        if not active:
            self.previous_heading_body = goal_heading
            return goal_heading, False, min_distance, 0.0, self.influence_radius, "clear", 1.0

        candidates = []
        for sector_index, distance in enumerate(sector_distances):
            heading = self.sector_center(sector_index)
            goal_diff = abs(self.angle_diff(heading, goal_heading))
            smooth_diff = abs(self.angle_diff(heading, self.previous_heading_body))
            obstacle_cost = self.obstacle_cost(distance)
            reverse_cost = self.reverse_cost(goal_diff)
            cost = (
                self.goal_weight * goal_diff / math.pi
                + self.obstacle_weight * obstacle_cost
                + self.smooth_weight * smooth_diff / math.pi
                + reverse_cost
            )
            candidates.append(
                {
                    "heading": heading,
                    "distance": distance,
                    "goal_diff": goal_diff,
                    "cost": cost,
                    "safe": distance >= self.safe_distance,
                    "forward": goal_diff <= math.pi / 2.0,
                }
            )

        safe_forward_candidates = [
            candidate for candidate in candidates if candidate["safe"] and candidate["forward"]
        ]
        safe_candidates = [candidate for candidate in candidates if candidate["safe"]]
        forward_candidates = [candidate for candidate in candidates if candidate["forward"]]

        if safe_forward_candidates:
            usable_candidates = safe_forward_candidates
            status = "avoid"
        elif safe_candidates:
            usable_candidates = safe_candidates
            status = "avoid_side"
        else:
            usable_candidates = forward_candidates if forward_candidates else candidates
            status = "tight"

        selected = min(usable_candidates, key=lambda candidate: candidate["cost"])
        selected_heading = self.clamp_to_forward_arc(selected["heading"], goal_heading)
        selected_goal_diff = abs(self.angle_diff(selected_heading, goal_heading))
        speed_scale = self.speed_scale_by_distance(min_distance)
        self.previous_heading_body = selected_heading
        return (
            selected_heading,
            True,
            min_distance,
            selected_goal_diff,
            selected["distance"],
            status,
            speed_scale,
        )

    def build_histogram(self, scan):
        sector_distances = [self.influence_radius for _ in range(self.sector_count)]
        min_distance = float("inf")

        for index in range(0, len(scan.ranges), self.sample_step):
            distance = scan.ranges[index]
            if not self.is_valid_distance(distance, scan):
                continue

            min_distance = min(min_distance, distance)
            if distance > self.influence_radius:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            sector_index = self.angle_to_sector(angle)
            for offset in range(-self.inflation_sectors, self.inflation_sectors + 1):
                inflated_index = (sector_index + offset) % self.sector_count
                sector_distances[inflated_index] = min(sector_distances[inflated_index], distance)

        return sector_distances, min_distance

    def obstacle_cost(self, distance):
        if distance >= self.influence_radius:
            return 0.0
        if distance <= self.safe_distance:
            return 1.0 + (self.safe_distance - distance) / max(self.safe_distance, 0.01) * 4.0
        return (self.influence_radius - distance) / max(self.influence_radius - self.safe_distance, 0.01)

    def speed_scale_by_distance(self, distance):
        if distance == float("inf"):
            return 1.0
        if distance <= self.safe_distance * 0.5:
            return 0.25
        if distance <= self.safe_distance:
            return 0.35
        if distance >= self.influence_radius:
            return 1.0
        ratio = (distance - self.safe_distance) / max(self.influence_radius - self.safe_distance, 0.01)
        return 0.55 + 0.45 * min(max(ratio, 0.0), 1.0)

    def reverse_cost(self, goal_diff):
        if goal_diff <= math.pi / 2.0:
            return 0.0
        reverse_ratio = (goal_diff - math.pi / 2.0) / (math.pi / 2.0)
        return self.reverse_penalty * (1.0 + reverse_ratio * reverse_ratio)

    def angle_to_sector(self, angle):
        normalized = (angle + math.pi) % (2.0 * math.pi)
        return int(normalized / self.sector_angle) % self.sector_count

    def sector_center(self, sector_index):
        return -math.pi + (sector_index + 0.5) * self.sector_angle

    def clamp_to_forward_arc(self, heading, goal_heading):
        diff = self.angle_diff(heading, goal_heading)
        if abs(diff) <= math.pi / 2.0:
            return heading
        limited_diff = math.copysign(math.pi / 2.0, diff)
        return self.angle_diff(goal_heading + limited_diff, 0.0)

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

    def angle_diff(self, angle, reference):
        return (angle - reference + math.pi) % (2.0 * math.pi) - math.pi

    def rotate_body_to_map(self, body_x, body_y, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = body_x * cos_yaw - body_y * sin_yaw
        map_y = body_x * sin_yaw + body_y * cos_yaw
        return map_x, map_y


APFAvoidance = VFHObstacleAvoidance


class VFHObstacleAvoidanceDebugNode:
    def __init__(self):
        rospy.init_node("vfh_avoidance_debug")
        self.local_setpoint_distance = rospy.get_param("~local_setpoint_distance", 0.3)
        self.avoidance = VFHObstacleAvoidance(
            sector_angle_deg=rospy.get_param("~vfh_sector_angle_deg", 10.0),
            influence_radius=rospy.get_param("~vfh_influence_radius", 3.0),
            safe_distance=rospy.get_param("~vfh_safe_distance", 1.1),
            goal_weight=rospy.get_param("~vfh_goal_weight", 1.0),
            obstacle_weight=rospy.get_param("~vfh_obstacle_weight", 3.0),
            smooth_weight=rospy.get_param("~vfh_smooth_weight", 0.8),
            reverse_penalty=rospy.get_param("~vfh_reverse_penalty", 8.0),
            inflation_sectors=rospy.get_param("~vfh_inflation_sectors", 2),
        )
        self.offset_pub = rospy.Publisher("/avoidance/offset", Point, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        offset_x, offset_y, active, min_distance, status, selected_heading, goal_diff = self.avoidance.compute_avoidance(
            msg,
            0.0,
            (1.0, 0.0),
            self.local_setpoint_distance,
        )
        self.offset_pub.publish(Point(x=offset_x, y=offset_y, z=0.0))
        rospy.loginfo_throttle(
            1.0,
            "VFH 회피 offset: x %.3f m, y %.3f m, 활성 %s, 상태 %s, 최소 거리 %.2f m, 선택 heading %.1f deg, 목표 차이 %.1f deg",
            offset_x,
            offset_y,
            active,
            status,
            min_distance,
            selected_heading,
            goal_diff,
        )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        VFHObstacleAvoidanceDebugNode().run()
    except rospy.ROSInterruptException:
        pass