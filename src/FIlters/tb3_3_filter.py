#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from threading import Thread

class RobotControl:
    def __init__(self, my_namespace, other_namespaces, rtabmap_namespace, proximity_threshold):
        self.my_namespace = my_namespace
        self.other_namespaces = other_namespaces
        self.robot_position = None
        self.robot_orientation = None
        self.proximity_threshold = proximity_threshold
        self.odom_subscriber = rospy.Subscriber(f"/{my_namespace}/ground_truth/state", Odometry, self.odom_callback)
        self.other_positions = {ns: None for ns in other_namespaces}
        self.other_orientations = {ns: None for ns in other_namespaces}
        self.slam_paused = False

        self.pause_service = rospy.ServiceProxy(f"/{rtabmap_namespace}/pause", Empty)
        self.resume_service = rospy.ServiceProxy(f"/{rtabmap_namespace}/resume", Empty)

        for ns in other_namespaces:
            rospy.Subscriber(f"/{ns}/ground_truth/state", Odometry, self.other_odom_callback, ns)

    def odom_callback(self, msg):
        self.robot_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_orientation = yaw
        self.check_proximity_and_control_slam()

    def other_odom_callback(self, msg, namespace):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.other_positions[namespace] = position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.other_orientations[namespace] = yaw

    def check_proximity_and_control_slam(self):
        # Check if self.robot_position is initialized and not None
        if self.robot_position is None:
            return

        # Check if all other positions are initialized (i.e., not None)
        if any(pos is None for pos in self.other_positions.values()):
            return

        # Iterate over each robot and check conditions
        should_resume = True  # Assume SLAM should resume unless a condition to pause is met
        for ns in self.other_namespaces:
            if self.is_within_fov(ns) and self.calculate_distance(ns) < self.proximity_threshold:
                self.pause_slam()  # Pause SLAM for this robot
                should_resume = False  # Set flag to not resume SLAM
                break  # Break the loop as we only need one condition to pause SLAM

        if should_resume:  # If no robot met the condition to pause SLAM
            self.resume_slam()  # It is safe to resume SLAM

    def calculate_distance(self, other_namespace):
        pos1 = self.robot_position
        pos2 = self.other_positions[other_namespace]
        return np.linalg.norm(pos1 - pos2)

    def is_within_fov(self, target_namespace):
        observer_pos = self.robot_position
        target_pos = self.other_positions[target_namespace]
        observer_yaw = self.robot_orientation
        robot_radius = 0.306 / 2  # Assuming a typical robot diameter
        direction_to_target = target_pos - observer_pos
        distance_to_target = np.linalg.norm(direction_to_target)

        if distance_to_target - robot_radius > self.proximity_threshold:
            rospy.loginfo(f"Distance beyond threshold for {self.my_namespace} observing {target_namespace}.")
            return False

        angle_to_target = np.arctan2(direction_to_target[1], direction_to_target[0])
        angle_difference = np.arctan2(np.sin(angle_to_target - observer_yaw), np.cos(angle_to_target - observer_yaw))
        angular_size_of_robot = np.arctan(robot_radius / distance_to_target)
        effective_half_fov_rad = np.radians(60)  
        within_fov = abs(angle_difference) <= effective_half_fov_rad
        return within_fov


    def pause_slam(self):
        if not self.slam_paused:
            Thread(target=self._pause_slam_thread).start()

    def _pause_slam_thread(self):
        try:
            self.pause_service()
            rospy.loginfo(f"SLAM paused for {self.my_namespace}")
            self.slam_paused = True
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to pause SLAM for {self.my_namespace}: {e}")

    def resume_slam(self):
        if self.slam_paused:
            Thread(target=self._resume_slam_thread).start()

    def _resume_slam_thread(self):
        try:
            self.resume_service()
            rospy.loginfo(f"SLAM resumed for {self.my_namespace}")
            self.slam_paused = False
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to resume SLAM for {self.my_namespace}: {e}")

if __name__ == "__main__":
    rospy.init_node('tb3_3_filter')

    my_namespace = 'tb3_3'  # specific to this file
    other_namespaces = ['tb3_1', 'tb3_2', 'tb3_0', 'tb3_4']
    rtabmap_namespace = 'rtabmap_3'  # specific to this file
    proximity_threshold = 5.0  # Define the threshold distance in meters

    control = RobotControl(my_namespace, other_namespaces, rtabmap_namespace, proximity_threshold)
    rospy.spin()
