#!/usr/bin/env python
from math import cos, sin
import tf

def get_euler(pose):
    """ Returns roll (x), pitch (y), yaw (z) from a Quaternion.

    See ROS Quaternion Basics for usage - http://wiki.ros.org/Tutorials/Quaternions
    """
    return transformations.euler_from_quaternion(
    	[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

def closest_waypoint(waypoints, pose):
	best_distance = float('inf')
	best_wp_index = 0
	best_wp = None
	current_pos = pose.position

	for i, wp in enumerate(waypoints):
		wp_pos = wp.pose.pose.position
		dist = ((current_pos.x - wp_pos.x)**2 + (current_pos.y - wp_pose.y)**2)**0.5

		if dist < best_distance:
			best_distance = dist
			best_wp, best_wp_index = wp, i

	_, _, yaw = get_euler(waypoints[best_wp_index].pose.pose)
	nx = current_pos.x - waypoints[best_wp_index].pose.pose.position.x
	ny = current_pos.y - waypoints[best_wp_index].pose.pose.position.y

	# Check if waypoint is ahead of stop line
	ahead = nx * cos(0-yaw) - ny * sin(0-yaw)
	if ahead < 0:
		best_wp_index -= 1
	print (best_wp_index)
	return best_wp_index

def all_stop_wp(waypoints, traffic_light_posts):
	stop_wp = []
	stop_wp_index = []
	for pos in traffic_light_posts:
		best_index = closest_waypoint(waypoints, pos)
		# print (best_index)
		stop_wp.append([waypoints[best_index].pose.pose.position.x, 
						waypoints[best_index].pose.pose.position.y])

		stop_wp_index.append(best_index)
		
	return stop_wp, stop_wp_index


