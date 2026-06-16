#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class LandingDetectorNode:
    def __init__(self):
        rospy.init_node("landing_detector")

        self.image_topic = rospy.get_param("~image_topic", "/downward_camera/image_raw")
        self.horizontal_fov = rospy.get_param("~horizontal_fov", 1.3962634)
        self.default_altitude = rospy.get_param("~default_altitude", 2.5)
        self.min_area = rospy.get_param("~min_area", 300.0)
        self.offset_gain = rospy.get_param("~offset_gain", 1.0)

        self.bridge = CvBridge()
        self.current_altitude = self.default_altitude

        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

        self.error_pub = rospy.Publisher("/landing/xy_error", Point, queue_size=10)
        self.visible_pub = rospy.Publisher("/landing/target_visible", Bool, queue_size=10)

        rospy.loginfo("랜딩패드 탐지 노드 시작")

    def pose_callback(self, msg):
        self.current_altitude = max(msg.pose.position.z, 0.1)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn("카메라 이미지 변환 실패: %s", error)
            return

        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array([0, 80, 60])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 80, 60])
        upper_red_2 = np.array([180, 255, 255])

        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask_1, mask_2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_result) == 2:
            contours = contours_result[0]
        else:
            contours = contours_result[1]

        if not contours:
            self.publish_not_visible()
            return

        target = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(target)
        if area < self.min_area:
            self.publish_not_visible()
            return

        moments = cv2.moments(target)
        if moments["m00"] == 0:
            self.publish_not_visible()
            return

        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]

        pixel_error_x = center_x - width / 2.0
        pixel_error_y = center_y - height / 2.0
        meters_per_pixel = self.get_meters_per_pixel(width)

        body_x_error = -pixel_error_y * meters_per_pixel * self.offset_gain
        body_y_error = -pixel_error_x * meters_per_pixel * self.offset_gain

        error = Point(x=body_x_error, y=body_y_error, z=area)
        self.error_pub.publish(error)
        self.visible_pub.publish(Bool(data=True))

        rospy.loginfo_throttle(
            1.0,
            "랜딩패드 감지: body x 보정 %.3f m, body y 보정 %.3f m, 면적 %.1f",
            error.x,
            error.y,
            area,
        )

    def get_meters_per_pixel(self, image_width):
        visible_width = 2.0 * self.current_altitude * math.tan(self.horizontal_fov / 2.0)
        return visible_width / float(image_width)

    def publish_not_visible(self):
        self.error_pub.publish(Point())
        self.visible_pub.publish(Bool(data=False))
        rospy.loginfo_throttle(1.0, "랜딩패드 미검출")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        LandingDetectorNode().run()
    except rospy.ROSInterruptException:
        pass
