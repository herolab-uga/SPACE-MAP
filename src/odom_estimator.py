#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster
import math
import tf
from std_msgs.msg import Float32MultiArray

class OdomEstimator:
    def __init__(self, robot_id, initial_pos, positions, yaws):
        self.robot_id = robot_id
        self.x = initial_pos[0]
        self.y = initial_pos[1]
        self.theta = initial_pos[3]

        self.positions = positions
        self.yaws = yaws

        self.wheel_separation = 0.287
        self.wheel_radius = 0.033

        rospy.Subscriber(f"{self.robot_id}/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber(f"{self.robot_id}/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher(f"{self.robot_id}/odomv1", Odometry, queue_size=100)
        self.odom_broadcaster = TransformBroadcaster()

        self.last_time = rospy.Time.now()

    def imu_callback(self, data):
        # Update theta based on IMU data
        orientation_q = data.orientation
        explicit_quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
        self.theta = yaw

    def cmd_vel_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Compute odometry using velocity and IMU data
        delta_x = (msg.linear.x * math.cos(self.theta) - msg.linear.y * math.sin(self.theta)) * dt
        delta_y = (msg.linear.x * math.sin(self.theta) + msg.linear.y * math.cos(self.theta)) * dt
        delta_th = msg.angular.z * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # Update global position arrays
        self.positions[self.robot_id].x = self.x
        self.positions[self.robot_id].y = self.y
        self.yaws[self.robot_id] = self.theta

        # Create quaternion from yaw
        odom_quat = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.theta))

        # Publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            (odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w),
            current_time,
            f"{self.robot_id}/base_footprintv1",
            "odomv1"
        )

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odomv1"

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        # Set the velocity
        odom.child_frame_id = f"{self.robot_id}/base_footprintv1"
        odom.twist.twist.linear.x = msg.linear.x
        odom.twist.twist.linear.y = msg.linear.y
        odom.twist.twist.angular.z = msg.angular.z

        self.odom_pub.publish(odom)

def main():
    rospy.init_node('odom_estimator')

    # Initial positions are assumed as offsets from global origin (0, 0, 0, 0)
    initial_positions = [
        (0.5, 1.7, 0.0, -1.57),
        (0.5, 2.2, 0.0, 3.14159),
        (0.5, 2.7, 0.0, 0.0),
        (0.5, 3.3, 0.0, 3.14159),
        (0.5, 3.75, 0.0, 0.0)
    ]

    # Create a publisher for the combined robot positions and yaws
    positions_pub = rospy.Publisher('robot_positions', Float32MultiArray, queue_size=10)
    yaws_pub = rospy.Publisher('robot_yaws', Float32MultiArray, queue_size=10)

    # Create position and yaw arrays
    positions = {f"tb3_{i}": Point() for i in range(5)}
    yaws = {f"tb3_{i}": 0.0 for i in range(5)}

    robots = []
    for i in range(5):
        robot_id = f"tb3_{i}"
        robots.append(OdomEstimator(robot_id, initial_positions[i], positions, yaws))

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Prepare Float32MultiArray messages
        positions_msg = Float32MultiArray()
        yaws_msg = Float32MultiArray()

        # Flatten the positions and yaws into the messages
        for i in range(5):
            positions_msg.data.extend([positions[f"tb3_{i}"].x, positions[f"tb3_{i}"].y])
            yaws_msg.data.append(yaws[f"tb3_{i}"])

        # Publish the messages
        positions_pub.publish(positions_msg)
        yaws_pub.publish(yaws_msg)

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
