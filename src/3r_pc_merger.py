#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import struct

# Utility functions for converting between Open3D and ROS
def to_msg(open3d_cloud, frame_id="map"):
    header = std_msgs.msg.Header(frame_id=frame_id, stamp=rospy.Time.now())
    points = np.asarray(open3d_cloud.points)
    if open3d_cloud.has_colors():
        colors = np.asarray(open3d_cloud.colors)
        colors = (colors * 255).astype(np.uint8)
        rgb = np.array([struct.unpack('f', struct.pack('i', r << 16 | g << 8 | b))[0] for r, g, b in colors])
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.FLOAT32, 1)]
        cloud_data = np.c_[points, rgb].astype(np.float32)
    else:
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        cloud_data = points.astype(np.float32)

    cloud_msg = pc2.create_cloud(header, fields, cloud_data)
    return cloud_msg

def from_msg(ros_cloud):
    assert isinstance(ros_cloud, PointCloud2)
    open3d_cloud = o3d.geometry.PointCloud()
    gen = pc2.read_points(ros_cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = []
    colors = []
    for p in gen:
        points.append(p[:3])
        if len(p) > 3:
            rgb = struct.unpack('I', struct.pack('f', p[3]))[0]
            colors.append(((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF))
    open3d_cloud.points = o3d.utility.Vector3dVector(np.array(points))
    if colors:
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(colors) / 255.0)
    return open3d_cloud

# Main class for merging point clouds from multiple robots
class PointCloudMerger:
    def __init__(self):
        rospy.init_node('point_cloud_merger')
        # Subscribers for each robot's point cloud
        self.subscribers = {}
        self.point_clouds = {}
        self.robots = ['/rtabmap_0', '/rtabmap_1', '/rtabmap_2', '/rtabmap_3', '/rtabmap_4']
        for idx, robot in enumerate(self.robots):
            self.subscribers[robot] = rospy.Subscriber(f'{robot}/cloud_map', PointCloud2, self.callback, callback_args=robot)
            self.point_clouds[robot] = None

        self.merged_pc_pub = rospy.Publisher('/total_cloud_map', PointCloud2, queue_size=10)

    def callback(self, msg, robot):
        try:
            self.point_clouds[robot] = from_msg(msg)
            self.try_merge_and_publish()
        except Exception as e:
            rospy.logerr(f"An error occurred in callback for {robot}: {e}")

    def try_merge_and_publish(self):
        # Check if all robots have sent their point clouds
        if all(pc is not None for pc in self.point_clouds.values()):
            try:
                merged_cloud = self.align_and_merge(self.point_clouds.values())
                merged_msg = to_msg(merged_cloud, frame_id="map")
                self.merged_pc_pub.publish(merged_msg)
                # Reset the point clouds after publishing
                for key in self.point_clouds:
                    self.point_clouds[key] = None
            except Exception as e:
                rospy.logerr(f"An error occurred during merging and publishing: {e}")

    def align_and_merge(self, point_clouds):
        # Merge point clouds using ICP
        threshold = 0.02
        trans_init = np.eye(4)
        merged_cloud = next(iter(point_clouds))  # Start with the first cloud
        for pc in list(point_clouds)[1:]:
            reg_p2p = o3d.pipelines.registration.registration_icp(
                merged_cloud, pc, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            pc.transform(reg_p2p.transformation)
            merged_cloud += pc
        return merged_cloud

if __name__ == '__main__':
    try:
        pc_merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
