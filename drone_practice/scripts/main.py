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


class VFHObstacleAvoidance:
    def __init__(
        self,
        sector_angle_deg=10.0,
        influence_radius=2.2,
        safe_distance=0.65,
        goal_weight=1.0,
        obstacle_weight=2.0,
        smooth_weight=0.4,
        reverse_penalty=4.0,
        sample_step=1,
        inflation_sectors=1,
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
            0.4,
        )
        return offset_x, offset_y

    def compute_avoidance(self, scan, yaw=0.0, goal_body_vector=None, step_distance=0.4):
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

        safe_candidates = [candidate for candidate in candidates if candidate["safe"]]
        if safe_candidates:
            usable_candidates = safe_candidates
            status = "avoid"
            speed_scale = 1.0
        else:
            forward_candidates = [candidate for candidate in candidates if candidate["forward"]]
            usable_candidates = forward_candidates if forward_candidates else candidates
            status = "tight"
            speed_scale = 0.35 if min_distance > self.safe_distance * 0.5 else 0.0

        selected = min(usable_candidates, key=lambda candidate: candidate["cost"])
        selected_heading = self.clamp_to_forward_arc(selected["heading"], goal_heading)
        selected_goal_diff = abs(self.angle_diff(selected_heading, goal_heading))
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
            return 1.0 + (self.safe_distance - distance) / max(self.safe_distance, 0.01) * 3.0
        return (self.influence_radius - distance) / max(self.influence_radius - self.safe_distance, 0.01)

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
        self.local_setpoint_distance = max(rospy.get_param("~local_setpoint_distance", 0.4), 0.2)
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
        self.avoidance = VFHObstacleAvoidance(
            sector_angle_deg=rospy.get_param("~vfh_sector_angle_deg", 10.0),
            influence_radius=rospy.get_param("~vfh_influence_radius", 2.2),
            safe_distance=rospy.get_param("~vfh_safe_distance", 0.65),
            goal_weight=rospy.get_param("~vfh_goal_weight", 1.0),
            obstacle_weight=rospy.get_param("~vfh_obstacle_weight", 2.0),
            smooth_weight=rospy.get_param("~vfh_smooth_weight", 0.4),
            reverse_penalty=rospy.get_param("~vfh_reverse_penalty", 4.0),
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

        goal_dx = target.pose.position.x - current_x
        goal_dy = target.pose.position.y - current_y
        goal_distance = math.hypot(goal_dx, goal_dy)

        if goal_distance > 0.05:
            step_distance = min(self.local_setpoint_distance, goal_distance)
            goal_body_vector = self.rotate_map_to_body(goal_dx, goal_dy)
            desired_dx, desired_dy, avoiding, min_distance, status, selected_heading, goal_diff = self.avoidance.compute_avoidance(
                self.latest_scan,
                self.get_yaw(),
                goal_body_vector,
                step_distance,
            )
        else:
            desired_dx = 0.0
            desired_dy = 0.0
            avoiding = False
            min_distance = float("inf")
            status = "target"
            selected_heading = 0.0
            goal_diff = 0.0

        if avoiding:
            rospy.loginfo_throttle(
                0.5,
                "VFH 회피 경로 추종: 상태 %s, 최소 거리 %.2f m, 선택 heading %.1f deg, 목표 차이 %.1f deg, setpoint offset x %.2f m, y %.2f m",
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
                "경로 추종 중: 로컬 setpoint offset x %.2f m, y %.2f m",
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