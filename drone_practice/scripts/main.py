#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from dodge import APFAvoidance
from path import PurePursuitPathFollower


class MissionController:
    def __init__(self):
        rospy.init_node("mission_controller")

        self.rate_hz = rospy.get_param("~rate_hz", 20.0)
        self.landing_precision_radius = rospy.get_param("~landing_precision_radius", 0.15)
        self.landing_descent_rate = rospy.get_param("~landing_descent_rate", 0.2)
        self.landing_min_altitude = rospy.get_param("~landing_min_altitude", 0.35)
        self.mode_retry_interval = rospy.Duration(rospy.get_param("~mode_retry_interval", 5.0))
        self.initial_setpoint_count = rospy.get_param("~initial_setpoint_count", 100)

        self.current_state = State()
        self.current_pose = None
        self.latest_scan = None
        self.landing_error = Point()
        self.landing_visible = False
        self.phase = "MISSION"
        self.land_requested = False

        self.path_follower = PurePursuitPathFollower(
            lookahead_distance=rospy.get_param("~lookahead_distance", 1.0),
            reach_threshold=rospy.get_param("~reach_threshold", 0.3),
        )
        self.avoidance = APFAvoidance(
            influence_radius=rospy.get_param("~apf_influence_radius", 2.0),
            max_offset=rospy.get_param("~apf_max_offset", 0.7),
            gain=rospy.get_param("~apf_gain", 0.8),
        )

        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/landing/xy_error", Point, self.landing_error_callback)
        rospy.Subscriber("/landing/target_visible", Bool, self.landing_visible_callback)

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )

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

    def get_initial_setpoint(self):
        target = self.path_follower.get_start_pose()
        return self.make_pose(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )

    def build_mission_setpoint(self):
        target = self.path_follower.update(self.current_pose)
        avoid_x, avoid_y = self.avoidance.compute_offset(self.latest_scan, self.get_yaw())

        pose = self.make_pose(
            target.pose.position.x + avoid_x,
            target.pose.position.y + avoid_y,
            target.pose.position.z,
        )

        if self.path_follower.is_finished():
            self.phase = "LANDING_ALIGN"
            rospy.loginfo("경로 추종 완료: 랜딩 정렬 단계로 전환")

        return pose

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

        if not self.landing_visible:
            rospy.loginfo_throttle(1.0, "랜딩패드 미검출: 마지막 경로점 상공에서 대기")
            return self.make_pose(
                final_pose.pose.position.x,
                final_pose.pose.position.y,
                final_pose.pose.position.z,
            )

        error_norm = math.hypot(self.landing_error.x, self.landing_error.y)
        map_error_x, map_error_y = self.rotate_body_to_map(
            self.landing_error.x, self.landing_error.y
        )

        target_z = current_z
        if error_norm <= self.landing_precision_radius:
            target_z = max(
                current_z - self.landing_descent_rate / self.rate_hz,
                self.landing_min_altitude,
            )
            rospy.loginfo_throttle(1.0, "랜딩패드 정렬 완료: 하강 중")
        else:
            rospy.loginfo_throttle(
                1.0,
                "랜딩패드 정렬 중: 수평 오차 %.3f m",
                error_norm,
            )

        if current_z <= self.landing_min_altitude and not self.land_requested:
            self.request_auto_land()

        return self.make_pose(current_x + map_error_x, current_y + map_error_y, target_z)

    def build_setpoint(self):
        if self.current_pose is None:
            return self.get_initial_setpoint()

        if self.phase == "MISSION":
            return self.build_mission_setpoint()

        if self.phase == "LANDING_ALIGN":
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
