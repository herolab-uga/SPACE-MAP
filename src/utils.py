import rospy
import numba as nb
import numpy as np

from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


def init_marker(rgb=[0,1,0], scale = 0.2):
      marker = Marker()
      marker.header.frame_id = "map"
      marker.header.stamp = rospy.Time.now()
      marker.ns = ""
      marker.id = 0
      marker.action = 0
      marker.type = Marker.POINTS
      marker.scale.x = scale
      marker.scale.y = scale
      marker.scale.z = 0*scale
      marker.color.a = 1.0
      marker.color.r = rgb[0]
      marker.color.g = rgb[1]
      marker.color.b = rgb[0]
      marker.points = []

      return marker

@nb.njit(cache=True)
def create_map_raw(data,h,w):
	"""Create gray-scale image from Occupancy-Grid data. Also returns how many cells have been explored."""
	img = np.zeros((h, w, 1), np.uint8)
	clean_area = 0
	for i in range(h):
		for j in range(w):
			if data[i*w+j]==100:
				img[i,j]=0
				clean_area += 1
			elif data[i*w+j]==0:
				img[i,j]=255
				clean_area += 1
			elif data[i*w+j]==-1:
				img[i,j]=205
	return img, clean_area
	
def get_yaw(msg):
	"""Get yaw from odometry msg."""
	_, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
						      msg.pose.pose.orientation.y,
						      msg.pose.pose.orientation.z,
						      msg.pose.pose.orientation.w])
	return yaw

def pose_to_pixel(pose: Pose2D(), map_info):
	"""Based on the map specifications, obtain pixel coordinate from /map coordinates."""
	x_origin = map_info.origin.position.x
	y_origin = map_info.origin.position.y
	res = map_info.resolution
	px = int((pose.x - x_origin)/res)
	py = int((pose.y - y_origin)/res)
	return np.array([py, px])

def pixel_to_pose(p, map_info):
	"""Get global (/map) coordinates from pixel."""
	x_origin = map_info.origin.position.x
	y_origin = map_info.origin.position.y
	res = map_info.resolution

	xr=p[0]*res+x_origin
	yr=p[1]*res+y_origin		
	return np.array([yr, xr])

def rot_trasl_2D(p, theta, t):
	"""First rotation, then translation. (local --> global frame)"""
	R = np.array([
		[np.cos(theta), -np.sin(theta), t[0]],
		[np.sin(theta),  np.cos(theta), t[1]],
		[			0, 				0,     1]
	])
	return (R @ np.array([p[0], p[1], 1]) )[0:2]


def rot_trasl_2D_inv(p, theta, t):
	"""Go to local frame from global frame."""
	R = np.array([
		[np.cos(theta), -np.sin(theta), t[0]],
		[np.sin(theta),  np.cos(theta), t[1]],
		[			0, 				0,     1]
	])
	return (np.linalg.inv(R) @ np.array([p[0], p[1], 1]) )[0:2]

@nb.njit(cache=True)
def stay_in_grid(p, grid):
	""" Sometimes the goal is not in the "clear" area of the map. 
	From point outside of the walkable map, the closest point in the map is returned. """
	
	if grid[p[0],p[1]]:
		raw = np.where(grid==0)
		n = int(len(raw[0]))
		clean = np.zeros((n,2))
		for i in range(n):
			clean[i,:] = [raw[0][i], raw[1][i]]
		dist = np.zeros(len(clean))
		for i, pc in enumerate(clean):
			dist[i] = abs(p[0]-pc[0])+abs(p[1]-pc[1])

		return (clean[np.argmin(dist)]).astype(np.int64)
	
	else: return p

	