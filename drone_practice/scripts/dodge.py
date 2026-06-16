#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan


class APFAvoidance:
    def __init__(self, influence_radius=2.0, max_offset=0.7, gain=0.8, sample_step=3):
        self.influence_radius = influence_radius
        self.max_offset = max_offset
        self.gain = gain
        self.sample_step = max(1, sample_step)

    def compute_offset(self, scan, yaw=0.0):
        if scan is None:
            return 0.0, 0.0

        body_x = 0.0
        body_y = 0.0

        for index in range(0, len(scan.ranges), self.sample_step):
            distance = scan.ranges[index]
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            if distance > self.influence_radius:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            direction_x = -math.cos(angle)
            direction_y = -math.sin(angle)
            strength = self.gain * (1.0 / distance - 1.0 / self.influence_radius) / (distance * distance)

            body_x += direction_x * strength
            body_y += direction_y * strength

        body_x, body_y = self.limit_vector(body_x, body_y, self.max_offset)
        return self.rotate_body_to_map(body_x, body_y, yaw)

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
            influence_radius=rospy.get_param("~apf_influence_radius", 2.0),
            max_offset=rospy.get_param("~apf_max_offset", 0.7),
            gain=rospy.get_param("~apf_gain", 0.8),
        )
        self.offset_pub = rospy.Publisher("/avoidance/offset", Point, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        offset_x, offset_y = self.avoidance.compute_offset(msg, 0.0)
        self.offset_pub.publish(Point(x=offset_x, y=offset_y, z=0.0))
        rospy.loginfo_throttle(
            1.0,
            "APF 회피 offset: x %.3f m, y %.3f m",
            offset_x,
            offset_y,
        )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        APFAvoidanceDebugNode().run()
    except rospy.ROSInterruptException:
        pass
