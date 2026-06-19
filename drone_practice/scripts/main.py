#!/usr/bin/env python3

import os
import sys
import csv
import math

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

import rospy
import rospkg
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from local_grid_planner import LocalGridPlanner


class PurePursuitPathFollower:
    def __init__(
        self,
        csv_path=None,
        lookahead_distance=1.0,
        reach_threshold=0.3,
        nearest_search_window=2,
        waypoint_pass_threshold=2.5,
        waypoint_pass_progress=0.9,
    ):
        self.lookahead_distance = lookahead_distance
        self.reach_threshold = reach_threshold
        self.nearest_search_window = max(1, nearest_search_window)
        self.waypoint_pass_threshold = waypoint_pass_threshold
        self.waypoint_pass_progress = min(max(waypoint_pass_progress, 0.0), 1.0)
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

    def find_nearest_forward_index(self, current_pose, max_search_ahead=120):
        """
        현재 드론 위치에서 가까운 CSV 경로점을 현재 index 이후 구간에서 찾는다.
        장애물 회피 후 기존 index가 뒤에 남아 있는 경우, 경로 복귀를 위해 사용한다.
        """
        if not self.path:
            return self.index, float("inf")

        start_index = max(0, min(self.index, len(self.path) - 1))
        end_index = min(len(self.path), start_index + max_search_ahead)

        current = current_pose.pose.position
        best_index = start_index
        best_distance = float("inf")

        for idx in range(start_index, end_index):
            path_point = self.path[idx].pose.position
            dx = current.x - path_point.x
            dy = current.y - path_point.y
            distance = math.hypot(dx, dy)

            if distance < best_distance:
                best_distance = distance
                best_index = idx

        return best_index, best_distance

    def has_passed_waypoint(self, current_pose, waypoint_index):
        if waypoint_index <= 0:
            return False

        previous = self.path[waypoint_index - 1].pose.position
        target = self.path[waypoint_index].pose.position
        current = current_pose.pose.position

        segment_x = target.x - previous.x
        segment_y = target.y - previous.y
        segment_length_sq = segment_x * segment_x + segment_y * segment_y
        if segment_length_sq <= 0.01:
            return False

        current_x = current.x - previous.x
        current_y = current.y - previous.y
        projection = (current_x * segment_x + current_y * segment_y) / segment_length_sq
        if projection < self.waypoint_pass_progress:
            return False

        segment_length = math.sqrt(segment_length_sq)
        cross_track = abs(current_x * segment_y - current_y * segment_x) / segment_length
        return cross_track <= self.waypoint_pass_threshold

    def advance_waypoint_if_needed(self, current_pose):
        while not self.finished:
            target = self.path[self.index]
            is_final_waypoint = self.index >= len(self.path) - 1
            reached = self.distance_3d(current_pose, target) <= self.reach_threshold
            passed = (
                not is_final_waypoint
                and self.has_passed_waypoint(current_pose, self.index)
            )
            if not reached and not passed:
                nearest_index, nearest_distance = self.find_nearest_forward_index(
                    current_pose
                )

                # 장애물 회피 후 기존 index가 뒤에 남아 있는 경우,
                # 현재 위치와 가까운 앞쪽 CSV 경로점으로 index를 보정한다.
                if nearest_index > self.index and nearest_distance <= self.waypoint_pass_threshold:
                    rospy.loginfo_throttle(
                        1.0,
                        "CSV 경로 복귀 인덱스 보정: %d -> %d, 경로 거리 %.2f m",
                        self.index + 1,
                        nearest_index + 1,
                        nearest_distance,
                    )
                    self.index = nearest_index
                    continue

                break

            if is_final_waypoint:
                self.finished = True
                self.index = len(self.path) - 1
                rospy.loginfo("Pure Pursuit 경로 추종 완료")
                break

            rospy.loginfo(
                "Waypoint %d/%d 통과: 다음 waypoint로 전환",
                self.index + 1,
                len(self.path),
            )
            self.index += 1

    def update(self, current_pose):
        if self.finished:
            return self.get_final_pose()

        self.advance_waypoint_if_needed(current_pose)
        if self.finished:
            return self.get_final_pose()

        return self.copy_pose(self.path[self.index])

    def get_start_pose(self):
        return self.copy_pose(self.path[0])

    def get_final_pose(self):
        return self.copy_pose(self.path[-1])

    def get_current_index(self):
        return self.index

    def get_path_count(self):
        return len(self.path)

    def is_finished(self):
        return self.finished



class MissionController:
    def __init__(self):
        rospy.init_node("mission_controller")

        self.rate_hz = rospy.get_param("~rate_hz", 20.0)
        self.takeoff_tolerance = rospy.get_param("~takeoff_tolerance", 0.25)
        self.landing_precision_radius = rospy.get_param("~landing_precision_radius", 0.15)
        self.landing_descent_rate = rospy.get_param("~landing_descent_rate", 0.2)
        self.landing_min_altitude = rospy.get_param("~landing_min_altitude", 0.35)
        self.mode_retry_interval = rospy.Duration(rospy.get_param("~mode_retry_interval", 5.0))
        self.initial_setpoint_count = rospy.get_param("~initial_setpoint_count", 100)
        self.local_setpoint_distance = max(rospy.get_param("~local_setpoint_distance", 0.30), 0.15)
        self.nearest_search_window = rospy.get_param("~nearest_search_window", 2)

        self.current_state = State()
        self.current_pose = None
        self.latest_scan = None
        self.landing_error = Point()
        self.landing_visible = False
        self.phase = "TAKEOFF"
        self.land_requested = False
        self.landing_target_z = None
        self.last_landing_time = None
        self.landing_active = False

        self.path_follower = PurePursuitPathFollower(
            lookahead_distance=rospy.get_param("~lookahead_distance", 1.0),
            reach_threshold=rospy.get_param("~reach_threshold", 0.3),
            nearest_search_window=self.nearest_search_window,
            waypoint_pass_threshold=rospy.get_param("~waypoint_pass_threshold", 2.5),
            waypoint_pass_progress=rospy.get_param("~waypoint_pass_progress", 0.9),
        )
        self.avoidance = LocalGridPlanner(
            resolution=rospy.get_param("~grid_resolution", 0.30),
            forward_range=rospy.get_param("~grid_forward_range", 7.0),
            side_range=rospy.get_param("~grid_side_range", 6.0),
            backward_range=rospy.get_param("~grid_backward_range", 2.0),
            obstacle_inflation=rospy.get_param("~grid_obstacle_inflation", 0.40),
            clearance_radius=rospy.get_param("~grid_clearance_radius", 0.75),
            local_goal_distance=rospy.get_param("~local_goal_distance", 5.0),
            path_lookahead=rospy.get_param("~local_path_lookahead", 0.8),
            min_target_distance=rospy.get_param("~local_min_target_distance", 0.5),
            backward_penalty=rospy.get_param("~grid_backward_penalty", 1.5),
            fallback_max_angle_deg=rospy.get_param("~fallback_max_angle_deg", 180.0),
            fallback_angle_step_deg=rospy.get_param(
                "~fallback_angle_step_deg",
                15.0,
            ),
            emergency_escape_distance=rospy.get_param(
                "~emergency_escape_distance",
                0.25,
            ),
        )

        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/landing/xy_error", Point, self.landing_error_callback)
        rospy.Subscriber("/landing/target_visible", Bool, self.landing_visible_callback)

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.landing_active_pub = rospy.Publisher(
            "/mission/landing_active", Bool, queue_size=1, latch=True
        )
        self.landing_active_pub.publish(Bool(data=False))

        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.last_mode_request = rospy.Time.now()

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

    def landing_error_callback(self, msg):
        self.landing_error = msg

    def landing_visible_callback(self, msg):
        self.landing_visible = msg.data

    def publish_landing_active(self, active):
        if self.landing_active == active:
            return
        self.landing_active = active
        self.landing_active_pub.publish(Bool(data=active))
        if active:
            rospy.loginfo("랜딩패드 탐지 활성화")
        else:
            rospy.loginfo("랜딩패드 탐지 비활성화")

    def make_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

    def get_yaw(self):
        if self.current_pose is None:
            return 0.0

        q = self.current_pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    def rotate_map_to_body(self, map_x, map_y):
        yaw = self.get_yaw()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = map_x * cos_yaw + map_y * sin_yaw
        body_y = -map_x * sin_yaw + map_y * cos_yaw
        return body_x, body_y

    def rotate_body_to_map(self, body_x, body_y):
        yaw = self.get_yaw()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = body_x * cos_yaw - body_y * sin_yaw
        map_y = body_x * sin_yaw + body_y * cos_yaw
        return map_x, map_y

    def get_initial_setpoint(self):
        target = self.path_follower.get_start_pose()
        return self.make_pose(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )

    def build_takeoff_setpoint(self):
        target = self.path_follower.get_start_pose()
        if self.current_pose is None:
            return self.get_initial_setpoint()

        altitude_error = abs(self.current_pose.pose.position.z - target.pose.position.z)
        if altitude_error <= self.takeoff_tolerance:
            self.phase = "PATH_FOLLOW"
            self.publish_landing_active(False)
            rospy.loginfo("이륙 완료: 경로 추종 단계로 전환")

        return self.make_pose(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )

    def build_path_follow_setpoint(self):
        target = self.path_follower.update(self.current_pose)
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        target_x = target.pose.position.x
        target_y = target.pose.position.y

        goal_dx = target_x - current_x
        goal_dy = target_y - current_y
        goal_distance = math.hypot(goal_dx, goal_dy)
        target_index = self.path_follower.get_current_index()
        path_count = self.path_follower.get_path_count()

        if goal_distance > 0.05:
            step_distance = min(self.local_setpoint_distance, goal_distance)
            goal_body_vector = self.rotate_map_to_body(goal_dx, goal_dy)
            desired_dx, desired_dy, replanning, min_distance, status, selected_heading, goal_diff = self.avoidance.compute_avoidance(
                self.latest_scan,
                self.get_yaw(),
                goal_body_vector,
                step_distance,
            )
        else:
            desired_dx = 0.0
            desired_dy = 0.0
            replanning = False
            min_distance = float("inf")
            status = "target"
            selected_heading = 0.0
            goal_diff = 0.0

        if replanning:
            rospy.loginfo_throttle(
                0.5,
                "Local A* 회피 경로 추종: WP %d/%d (%.1f, %.1f), 상태 %s, 최소 거리 %.2f m, 선택 heading %.1f deg, 목표 차이 %.1f deg, setpoint offset x %.2f m, y %.2f m",
                target_index + 1,
                path_count,
                target_x,
                target_y,
                status,
                min_distance,
                selected_heading,
                goal_diff,
                desired_dx,
                desired_dy,
            )
        else:
            rospy.loginfo_throttle(
                3.0,
                "경로 추종 중: WP %d/%d (%.1f, %.1f), 상태 %s, 로컬 setpoint offset x %.2f m, y %.2f m",
                target_index + 1,
                path_count,
                target_x,
                target_y,
                status,
                desired_dx,
                desired_dy,
            )

        pose = self.make_pose(
            current_x + desired_dx,
            current_y + desired_dy,
            target.pose.position.z,
        )

        if self.path_follower.is_finished():
            self.phase = "LANDING_ALIGN"
            self.landing_target_z = self.current_pose.pose.position.z
            self.last_landing_time = rospy.Time.now()
            self.publish_landing_active(True)
            rospy.loginfo("경로 추종 완료: 랜딩 정렬 단계로 전환")

        return pose

    def get_landing_dt(self):
        now = rospy.Time.now()
        if self.last_landing_time is None:
            self.last_landing_time = now
            return 1.0 / self.rate_hz

        dt = (now - self.last_landing_time).to_sec()
        self.last_landing_time = now
        return min(max(dt, 0.0), 0.2)

    def build_landing_setpoint(self):
        final_pose = self.path_follower.get_final_pose()

        if self.current_pose is None:
            return self.make_pose(
                final_pose.pose.position.x,
                final_pose.pose.position.y,
                final_pose.pose.position.z,
            )

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_z = self.current_pose.pose.position.z

        if self.landing_target_z is None:
            self.landing_target_z = current_z
            self.last_landing_time = rospy.Time.now()

        if not self.landing_visible:
            rospy.loginfo_throttle(
                1.0,
                "랜딩패드 미검출: 현재 위치와 목표 고도 %.2f m 유지",
                self.landing_target_z,
            )
            return self.make_pose(current_x, current_y, self.landing_target_z)

        error_norm = math.hypot(self.landing_error.x, self.landing_error.y)
        map_error_x, map_error_y = self.rotate_body_to_map(
            self.landing_error.x, self.landing_error.y
        )

        if error_norm <= self.landing_precision_radius:
            dt = self.get_landing_dt()
            self.landing_target_z = max(
                self.landing_target_z - self.landing_descent_rate * dt,
                self.landing_min_altitude,
            )
            self.phase = "LANDING_DESCEND"
            rospy.loginfo_throttle(
                0.5,
                "랜딩패드 정렬 완료: 하강 중, 현재 z %.2f m, 목표 z %.2f m, 수평 오차 %.3f m",
                current_z,
                self.landing_target_z,
                error_norm,
            )
        else:
            self.phase = "LANDING_ALIGN"
            self.last_landing_time = rospy.Time.now()
            rospy.loginfo_throttle(
                1.0,
                "랜딩패드 정렬 중: 현재 z %.2f m, 목표 z %.2f m, 수평 오차 %.3f m",
                current_z,
                self.landing_target_z,
                error_norm,
            )

        if (current_z <= self.landing_min_altitude or self.landing_target_z <= self.landing_min_altitude) and not self.land_requested:
            self.request_auto_land()

        return self.make_pose(current_x + map_error_x, current_y + map_error_y, self.landing_target_z)

    def build_setpoint(self):
        if self.current_pose is None:
            return self.get_initial_setpoint()

        if self.phase == "TAKEOFF":
            return self.build_takeoff_setpoint()

        if self.phase == "PATH_FOLLOW":
            return self.build_path_follow_setpoint()

        if self.phase in ("LANDING_ALIGN", "LANDING_DESCEND"):
            return self.build_landing_setpoint()

        return self.make_pose(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z,
        )

    def request_offboard_and_arm(self):
        if self.land_requested:
            return

        now = rospy.Time.now()
        if now - self.last_mode_request <= self.mode_retry_interval:
            return

        if self.current_state.mode != "OFFBOARD":
            mode_request = SetModeRequest()
            mode_request.custom_mode = "OFFBOARD"
            if self.set_mode_client.call(mode_request).mode_sent:
                rospy.loginfo("OFFBOARD 모드 전환 요청 성공")
            self.last_mode_request = now
            return

        if not self.current_state.armed:
            arm_request = CommandBoolRequest()
            arm_request.value = True
            if self.arming_client.call(arm_request).success:
                rospy.loginfo("기체 ARM 요청 성공")
            self.last_mode_request = now

    def request_auto_land(self):
        mode_request = SetModeRequest()
        mode_request.custom_mode = "AUTO.LAND"
        if self.set_mode_client.call(mode_request).mode_sent:
            rospy.loginfo("AUTO.LAND 모드 전환 요청 성공")
        else:
            rospy.logwarn("AUTO.LAND 모드 전환 요청 실패")
        self.land_requested = True
        self.phase = "LANDING_DONE"

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo_throttle(1.0, "비행 컨트롤러 연결 대기 중")
            rate.sleep()

        for _ in range(self.initial_setpoint_count):
            if rospy.is_shutdown():
                return
            self.setpoint_pub.publish(self.build_setpoint())
            rate.sleep()

        while not rospy.is_shutdown():
            self.request_offboard_and_arm()
            self.setpoint_pub.publish(self.build_setpoint())
            rate.sleep()


if __name__ == "__main__":
    try:
        MissionController().run()
    except rospy.ROSInterruptException:
        pass
