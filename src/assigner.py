#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from math import sqrt, atan2
from multi_explore.msg import Frontier
from frontier_finder import FrontierFinder

class Robot:
    def __init__(self, name):
        self.name = name
        self.action_client = actionlib.SimpleActionClient(f"{name}/move_base", MoveBaseAction)
        self.action_client.wait_for_server(rospy.Duration(30))
        rospy.loginfo(f"Robot {name} connected to move_base action server")
        self.last_goal_pose = None
        self.last_robot_pose = None
        self.current_pose = None
        self.distance_threshold = 0.1  # Threshold distance between goal and robot
        self.goal_update_threshold = rospy.Duration(50)  # Update goal only after 200 seconds
        self.last_goal_time = rospy.Time(0)  # Initialize last_goal_time
        self.odom_subscriber = rospy.Subscriber(f"/{name}/ground_truth/state", Odometry, self.update_pose)

    def update_pose(self, msg):
        self.current_pose = msg.pose.pose  # Update robot's current pose from odometry messages

    def send_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*q)
        goal.target_pose.pose.position = Point(x, y, 0)
        self.action_client.send_goal(goal)
        self.last_goal_pose = goal.target_pose.pose
        rospy.loginfo(f"Goal sent to {self.name}: x={x}, y={y}, theta={theta}")
        self.last_goal_time = rospy.Time.now()  # Update last_goal_time when sending a goal


    def update_last_pose(self, current_pose):
        self.last_robot_pose = current_pose

    def is_goal_reached(self, current_pose):
        if self.last_goal_pose is None:
            return False
        dx = self.last_goal_pose.position.x - current_pose.position.x
        dy = self.last_goal_pose.position.y - current_pose.position.y
        distance = sqrt(dx**2 + dy**2)
        return distance < self.distance_threshold

    def calculate_angle_to_goal(self, current_pose):
        dx = self.last_goal_pose.position.x - current_pose.position.x
        dy = self.last_goal_pose.position.y - current_pose.position.y
        return atan2(dy, dx)

    def is_stuck(self, current_pose):
        if self.last_robot_pose is None:
            return False
        dx = self.last_robot_pose.position.x - current_pose.position.x
        dy = self.last_robot_pose.position.y - current_pose.position.y
        distance = sqrt(dx**2 + dy**2)
        return distance < 0.1  # Small threshold for movement to determine if stuck
    
    def calculate_angle_to_goal(self, current_pose):
        if not self.last_goal_pose:
            return None
        dx = self.last_goal_pose.position.x - current_pose.position.x
        dy = self.last_goal_pose.position.y - current_pose.position.y
        return atan2(dy, dx)


class FrontierAssigner:
    def __init__(self, robot_namespaces):
        self.robots = [Robot(name) for name in robot_namespaces]
        self.frontier_finder = FrontierFinder()
        self.frontiers = self.frontier_finder.frontiers.frontiers
        self.assigned_goals = {}  # Tracks assigned goals to prevent duplicates
        rospy.loginfo("FrontierAssigner initialized.")

    def update_frontiers(self, new_frontiers):
        self.frontiers = new_frontiers
        rospy.loginfo("Frontiers have been updated.")

    def run_frontier_assignment(self):
        self.frontier_finder.spin()  # Process and publish frontiers for RViz
        rospy.loginfo(f"Frontiers available: {len(self.frontier_finder.frontiers.frontiers)}")
        self.assign_frontiers()

    def assign_frontiers(self):
        for robot in self.robots:
            if robot.current_pose is None:
                rospy.logwarn(f"No current pose available for {robot.name}")
                continue

            visible_frontiers = [f for f in self.frontier_finder.frontiers.frontiers if self.is_frontier_visible(robot, robot.current_pose, f)]
            visible_frontiers = [f for f in visible_frontiers if f not in self.assigned_goals.values()]  # Filter out already assigned goals

            if visible_frontiers:
                if robot.is_stuck(robot.current_pose):
                    selected_frontier = visible_frontiers[-1]
                else:
                    selected_frontier = visible_frontiers[0]

                if selected_frontier and (rospy.Time.now() - robot.last_goal_time > robot.goal_update_threshold or robot.last_goal_pose is None):
                    robot.send_goal(selected_frontier.pose.x, selected_frontier.pose.y, 0)
                    robot.last_goal_time = rospy.Time.now()
                    self.assigned_goals[robot.name] = selected_frontier  # Track this goal as assigned
            else:
                rospy.loginfo("No appropriate frontier found for assignment.")
                if self.frontier_finder.frontiers.frontiers:
                    self.frontier_finder.frontiers.frontiers.sort(key=lambda f: self.distance(robot.current_pose, f.pose))
                    random_frontier = self.frontier_finder.frontiers.frontiers[0]  # Assign nearest unassigned frontier
                    if random_frontier not in self.assigned_goals.values():
                        robot.send_goal(random_frontier.pose.x, random_frontier.pose.y, 0)
                        robot.last_goal_time = rospy.Time.now()
                        self.assigned_goals[robot.name] = random_frontier  # Track this goal as assigned

    def release_assigned_goals(self):
        # This method can be called periodically or after a goal is reached to release goals from the tracking dictionary
        for robot in self.robots:
            if robot.is_goal_reached(robot.current_pose):
                if robot.name in self.assigned_goals:
                    del self.assigned_goals[robot.name]

    def is_frontier_visible(self, robot, robot_pose, frontier):
        angle_to_frontier = atan2(frontier.pose.y - robot_pose.position.y, frontier.pose.x - robot_pose.position.x)
        angle_to_goal = robot.calculate_angle_to_goal(robot_pose)
        if angle_to_goal is None:
            return False
        return abs(angle_to_frontier - angle_to_goal) < np.pi / 2



    def distance(self, pose1, pose2):
        # pose1 is expected to be a Pose (from Odometry), and pose2 is a Pose2D from a Frontier
        return sqrt((pose1.position.x - pose2.x)**2 + (pose1.position.y - pose2.y)**2)




if __name__ == "__main__":
    rospy.init_node('frontier_assigner')
    robot_namespaces = rospy.get_param('~robot_namespaces').split(',')
    assigner = FrontierAssigner(robot_namespaces)
    rate = rospy.Rate(50)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        # Simulate fetching new frontiers
        new_frontiers = []  # Should be updated with real-time data
        assigner.run_frontier_assignment()
        assigner.update_frontiers(new_frontiers)
        rate.sleep()
