#!/usr/bin/env python3

import csv
import math

import rospy
import rospkg
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class PurePursuitPathFollower:
    def __init__(
        self,
        csv_path=None,
        lookahead_distance=1.0,
        reach_threshold=0.3,
        nearest_search_window=2,
    ):
        self.lookahead_distance = lookahead_distance
        self.reach_threshold = reach_threshold
        self.nearest_search_window = max(1, nearest_search_window)
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
        search_end = min(len(self.path), self.index + self.nearest_search_window + 1)

        for index in range(self.index, search_end):
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


class APFAvoidance:
    def __init__(
        self,
        influence_radius=5.0,
        max_offset=1.5,
        gain=0.8,
        sample_step=3,
        emergency_radius=0.8,
        tangent_gain=1.0,
        path_sector_angle=0.75,
    ):
        self.influence_radius = influence_radius
        self.max_offset = max_offset
        self.gain = gain
        self.sample_step = max(1, sample_step)
        self.emergency_radius = emergency_radius
        self.tangent_gain = tangent_gain
        self.path_sector_angle = path_sector_angle

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
            return 0.0, 0.0, False, min_distance, "none"

        left_score = left_clearance / max(left_count, 1)
        right_score = right_clearance / max(right_count, 1)
        turn_side = "left" if left_score >= right_score else "right"

        if path_blocked:
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
        self.local_setpoint_distance = max(rospy.get_param("~local_setpoint_distance", 1.0), 0.2)
        self.path_attraction_weight = min(max(rospy.get_param("~path_attraction_weight", 0.7), 0.0), 1.0)
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
        )
        self.avoidance = APFAvoidance(
            influence_radius=rospy.get_param("~apf_influence_radius", 5.0),
            max_offset=rospy.get_param("~apf_max_offset", 1.5),
            gain=rospy.get_param("~apf_gain", 0.8),
            emergency_radius=rospy.get_param("~apf_emergency_radius", 0.8),
            tangent_gain=rospy.get_param("~apf_tangent_gain", 1.0),
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

    def rotate_body_to_map(self, body_x, body_y):
        yaw = self.get_yaw()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = body_x * cos_yaw - body_y * sin_yaw
        map_y = body_x * sin_yaw + body_y * cos_yaw
        return map_x, map_y

    def rotate_map_to_body(self, map_x, map_y):
        yaw = self.get_yaw()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = map_x * cos_yaw + map_y * sin_yaw
        body_y = -map_x * sin_yaw + map_y * cos_yaw
        return body_x, body_y

    def limit_vector(self, x, y, limit):
        length = math.hypot(x, y)
        if length <= limit or length == 0.0:
            return x, y
        scale = limit / length
        return x * scale, y * scale

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

        goal_dx = target.pose.position.x - current_x
        goal_dy = target.pose.position.y - current_y
        goal_distance = math.hypot(goal_dx, goal_dy)

        if goal_distance > 0.05:
            step_distance = min(self.local_setpoint_distance, goal_distance)
            path_dx = goal_dx / goal_distance * step_distance
            path_dy = goal_dy / goal_distance * step_distance
        else:
            path_dx = 0.0
            path_dy = 0.0

        goal_body_vector = self.rotate_map_to_body(goal_dx, goal_dy)
        avoid_x, avoid_y, avoiding, min_distance, turn_side = self.avoidance.compute_avoidance(
            self.latest_scan,
            self.get_yaw(),
            goal_body_vector,
        )

        if avoiding:
            if min_distance <= self.avoidance.emergency_radius:
                desired_dx = avoid_x
                desired_dy = avoid_y
                mode_text = "긴급 회피"
            else:
                desired_dx = path_dx * self.path_attraction_weight + avoid_x
                desired_dy = path_dy * self.path_attraction_weight + avoid_y
                mode_text = "회피 경로 추종"
            max_step = max(self.local_setpoint_distance, self.avoidance.max_offset)
            rospy.loginfo_throttle(
                0.5,
                "APF %s: 최소 거리 %.2f m, 우회 %s, setpoint offset x %.2f m, y %.2f m",
                mode_text,
                min_distance,
                turn_side,
                desired_dx,
                desired_dy,
            )
        else:
            desired_dx = path_dx
            desired_dy = path_dy
            max_step = self.local_setpoint_distance
            rospy.loginfo_throttle(
                3.0,
                "경로 추종 중: 로컬 setpoint offset x %.2f m, y %.2f m",
                desired_dx,
                desired_dy,
            )

        desired_dx, desired_dy = self.limit_vector(desired_dx, desired_dy, max_step)
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