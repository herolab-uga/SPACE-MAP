#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from threading import Lock
from matplotlib.animation import FuncAnimation

class RobotTrajectoryPlotter:
    def __init__(self):
        self.lock = Lock()
        self.robot_trajectories = {
            'tb3_0': {'odom': [], 'ground_truth/state': []},
            'tb3_1': {'odom': [], 'ground_truth/state': []},
            'tb3_2': {'odom': [], 'ground_truth/state': []},
            'tb3_3': {'odom': [], 'ground_truth/state': []},
            'tb3_4': {'odom': [], 'ground_truth/state': []}
        }
        self.fig, self.axs = plt.subplots(5, 1, figsize=(8, 20))
        self.setup_subscribers()

    def setup_subscribers(self):
        for i in range(5):
            rospy.Subscriber(f'tb3_{i}/odom', Odometry, self.odom_callback, (f'tb3_{i}', 'odom'))
            rospy.Subscriber(f'tb3_{i}/ground_truth/state', Odometry, self.odom_callback, (f'tb3_{i}', 'ground_truth/state'))

    def odom_callback(self, msg, args):
        robot_name, topic_name = args
        position = msg.pose.pose.position
        rospy.loginfo(f"{robot_name} {topic_name} received position: ({position.x}, {position.y})")
        with self.lock:
            self.robot_trajectories[robot_name][topic_name].append((position.x, position.y))

    def update_plot(self, frame):
        with self.lock:
            for i, robot_name in enumerate(self.robot_trajectories.keys()):
                self.axs[i].clear()
                self.axs[i].set_title(f"Trajectory of {robot_name}")
                self.axs[i].set_xlabel('X')
                self.axs[i].set_ylabel('Y')

                odom_traj = self.robot_trajectories[robot_name]['odom']
                odomv1_traj = self.robot_trajectories[robot_name]['ground_truth/state']

                if odom_traj:
                    odom_x, odom_y = zip(*odom_traj)
                    self.axs[i].plot(odom_x, odom_y, label='odom')

                if odomv1_traj:
                    odomv1_x, odomv1_y = zip(*odomv1_traj)
                    self.axs[i].plot(odomv1_x, odomv1_y, label='ground_truth/state')

                self.axs[i].legend()

        self.fig.canvas.draw_idle()

    def start_plotting(self):
        rospy.init_node('robot_trajectory_plotter', anonymous=True)
        ani = FuncAnimation(self.fig, self.update_plot, interval=1000)  # update every 1000 ms (1 second)
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    plotter = RobotTrajectoryPlotter()
    plotter.start_plotting()
