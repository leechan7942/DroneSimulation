#!/usr/bin/env python3

import csv
import math

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped


class PurePursuitPathFollower:
    def __init__(self, csv_path=None, lookahead_distance=1.0, reach_threshold=0.3):
        self.lookahead_distance = lookahead_distance
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

        with open(self.csv_path, "r") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(row["x"])
                pose.pose.position.y = float(row["y"])
                pose.pose.position.z = float(row["z"])
                self.path.append(pose)

        if not self.path:
            raise RuntimeError("경로 CSV가 비어 있습니다")

        rospy.loginfo("Pure Pursuit 경로 로드 완료: %d개 점", len(self.path))

    def copy_pose(self, pose):
        copied = PoseStamped()
        copied.header.stamp = rospy.Time.now()
        copied.header.frame_id = "map"
        copied.pose.position.x = pose.pose.position.x
        copied.pose.position.y = pose.pose.position.y
        copied.pose.position.z = pose.pose.position.z
        copied.pose.orientation = pose.pose.orientation
        return copied

    def distance_2d(self, pose_a, pose_b):
        dx = pose_a.pose.position.x - pose_b.pose.position.x
        dy = pose_a.pose.position.y - pose_b.pose.position.y
        return math.hypot(dx, dy)

    def distance_3d(self, pose_a, pose_b):
        dx = pose_a.pose.position.x - pose_b.pose.position.x
        dy = pose_a.pose.position.y - pose_b.pose.position.y
        dz = pose_a.pose.position.z - pose_b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def find_nearest_index(self, current_pose):
        best_index = self.index
        best_distance = float("inf")

        for index in range(self.index, len(self.path)):
            distance = self.distance_2d(current_pose, self.path[index])
            if distance < best_distance:
                best_distance = distance
                best_index = index

        self.index = max(self.index, best_index)
        return self.index

    def find_lookahead_index(self, current_pose):
        start_index = self.find_nearest_index(current_pose)

        for index in range(start_index, len(self.path)):
            if self.distance_2d(current_pose, self.path[index]) >= self.lookahead_distance:
                return index

        return len(self.path) - 1

    def update(self, current_pose):
        if self.finished:
            return self.get_final_pose()

        target_index = self.find_lookahead_index(current_pose)
        self.index = max(self.index, target_index)

        final_pose = self.path[-1]
        final_index = len(self.path) - 1
        final_reached = (
            self.index >= final_index
            and self.distance_3d(current_pose, final_pose) <= self.reach_threshold
        )
        if final_reached:
            self.finished = True
            self.index = final_index
            rospy.loginfo("Pure Pursuit 경로 추종 완료")
            return self.get_final_pose()

        return self.copy_pose(self.path[target_index])

    def get_start_pose(self):
        return self.copy_pose(self.path[0])

    def get_final_pose(self):
        return self.copy_pose(self.path[-1])

    def is_finished(self):
        return self.finished

    def reset(self):
        self.index = 0
        self.finished = False
        rospy.loginfo("Pure Pursuit 경로 추종 초기화")


PathFollower = PurePursuitPathFollower


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_path_test")
    follower = PurePursuitPathFollower()
    start = follower.get_start_pose().pose.position
    end = follower.get_final_pose().pose.position
    rospy.loginfo(
        "경로 테스트 로드 완료: 시작점 %.2f %.2f %.2f, 끝점 %.2f %.2f %.2f",
        start.x,
        start.y,
        start.z,
        end.x,
        end.y,
        end.z,
    )