#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

from local_grid_planner import LocalGridPlanner



APFAvoidance = LocalGridPlanner
VFHObstacleAvoidance = LocalGridPlanner


class LocalGridPlannerDebugNode:
    def __init__(self):
        rospy.init_node("local_grid_avoidance_debug")
        self.local_setpoint_distance = rospy.get_param("~local_setpoint_distance", 0.35)
        self.avoidance = LocalGridPlanner(
            resolution=rospy.get_param("~grid_resolution", 0.25),
            forward_range=rospy.get_param("~grid_forward_range", 4.0),
            side_range=rospy.get_param("~grid_side_range", 3.5),
            backward_range=rospy.get_param("~grid_backward_range", 0.6),
            obstacle_inflation=rospy.get_param("~grid_obstacle_inflation", 0.55),
            clearance_radius=rospy.get_param("~grid_clearance_radius", 1.1),
            local_goal_distance=rospy.get_param("~local_goal_distance", 3.0),
            path_lookahead=rospy.get_param("~local_path_lookahead", 0.9),
            min_target_distance=rospy.get_param("~local_min_target_distance", 0.8),
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
            "Local A* 회피 offset: x %.3f m, y %.3f m, 활성 %s, 상태 %s, 최소 거리 %.2f m, 선택 heading %.1f deg, 목표 차이 %.1f deg",
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
        LocalGridPlannerDebugNode().run()
    except rospy.ROSInterruptException:
        pass
