#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node("obstacle_avoidance")

        self.flight_altitude = rospy.get_param("~flight_altitude", 2.5)
        self.goal_x = rospy.get_param("~goal_x", 12.0)
        self.goal_y = rospy.get_param("~goal_y", 5.0)
        self.step_size = rospy.get_param("~step_size", 0.25)
        self.safe_distance = rospy.get_param("~safe_distance", 1.2)
        self.side_distance = rospy.get_param("~side_distance", 0.8)

        self.current_pose = None
        self.front_dist = float("inf")
        self.left_dist = float("inf")
        self.right_dist = float("inf")

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )

    def pose_callback(self, msg):
        self.current_pose = msg

    def scan_callback(self, msg):
        self.front_dist = self.get_sector_min(msg, -20, 20)
        self.left_dist = self.get_sector_min(msg, 30, 90)
        self.right_dist = self.get_sector_min(msg, -90, -30)

    def get_sector_min(self, scan, min_deg, max_deg):
        values = []

        for index, distance in enumerate(scan.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            angle_deg = math.degrees(angle)

            if min_deg <= angle_deg <= max_deg:
                values.append(distance)

        if not values:
            return float("inf")
        return min(values)

    def make_setpoint(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        if self.current_pose is None:
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = self.flight_altitude
            return pose

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        goal_dx = self.goal_x - current_x
        goal_dy = self.goal_y - current_y
        goal_dist = math.hypot(goal_dx, goal_dy)

        if goal_dist > 0.05:
            move_x = goal_dx / goal_dist
            move_y = goal_dy / goal_dist
        else:
            move_x = 0.0
            move_y = 0.0

        if self.front_dist < self.safe_distance:
            move_x = -0.2
            if self.left_dist > self.right_dist:
                move_y = 1.0
            else:
                move_y = -1.0
            rospy.loginfo_throttle(1.0, "전방 장애물 감지: 우회 중")
        elif self.left_dist < self.side_distance:
            move_y -= 0.7
            rospy.loginfo_throttle(1.0, "좌측 장애물 감지: 오른쪽으로 보정")
        elif self.right_dist < self.side_distance:
            move_y += 0.7
            rospy.loginfo_throttle(1.0, "우측 장애물 감지: 왼쪽으로 보정")

        length = math.hypot(move_x, move_y)
        if length > 0.0:
            move_x /= length
            move_y /= length

        pose.pose.position.x = current_x + move_x * self.step_size
        pose.pose.position.y = current_y + move_y * self.step_size
        pose.pose.position.z = self.flight_altitude
        return pose

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.setpoint_pub.publish(self.make_setpoint())
            rate.sleep()


if __name__ == "__main__":
    try:
        ObstacleAvoidanceNode().run()
    except rospy.ROSInterruptException:
        pass
