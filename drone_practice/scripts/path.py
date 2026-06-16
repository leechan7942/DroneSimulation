#!/usr/bin/env python3

import csv
import math
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped


class PathFollower:
    def __init__(self, csv_path=None, reach_threshold=0.3):
        self.reach_threshold = reach_threshold
        self.path = []
        self.index = 0
        self.finished = False

        if csv_path is None:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path("drone_practice")
            csv_path = pkg_path + "/mission/practice_path.csv"

        self.csv_path = csv_path
        self.load_path()

    def load_path(self):
        self.path = []

        with open(self.csv_path, "r") as f:
            reader = csv.DictReader(f)

            for row in reader:
                pose = PoseStamped()
                pose.pose.position.x = float(row["x"])
                pose.pose.position.y = float(row["y"])
                pose.pose.position.z = float(row["z"])
                self.path.append(pose)

        if len(self.path) == 0:
            raise RuntimeError("Path CSV is empty")

        rospy.loginfo("[PathFollower] Loaded %d waypoints", len(self.path))

    def distance(self, current_pose, target_pose):
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        dz = current_pose.pose.position.z - target_pose.pose.position.z

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def get_current_target(self):
        return self.path[self.index]

    def update(self, current_pose):
        if self.finished:
            return self.path[-1]

        target = self.path[self.index]

        if self.distance(current_pose, target) < self.reach_threshold:
            rospy.loginfo("[PathFollower] Reached waypoint %d", self.index)

            if self.index < len(self.path) - 1:
                self.index += 1
                target = self.path[self.index]
            else:
                self.finished = True
                rospy.loginfo("[PathFollower] Path complete")
                target = self.path[-1]

        return target

    def is_finished(self):
        return self.finished

    def reset(self):
        self.index = 0
        self.finished = False
        rospy.loginfo("[PathFollower] Reset path follower")


if __name__ == "__main__":
    rospy.init_node("path_follower_test")

    follower = PathFollower()

    rospy.loginfo("PathFollower test loaded successfully")
    rospy.loginfo("First target: x=%.2f y=%.2f z=%.2f",
                  follower.get_current_target().pose.position.x,
                  follower.get_current_target().pose.position.y,
                  follower.get_current_target().pose.position.z)
