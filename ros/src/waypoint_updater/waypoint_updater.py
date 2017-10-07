#!/usr/bin/env python

import rospy
from tf import transformations
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray

from math import cos, sin
from copy import deepcopy

FORWARD_SCAN_WPS = 200

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''


class WaypointUpdater(object):
    """
    *** STEP 2 ***

    This node will publish waypoints from the car's current position to some `x` distance ahead,
    with the correct target velocities, depending on traffic lights and obstacles.
    """
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)  # Simulator data

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_ego_pose = None  # ego car current position and orientation
        self.base_waypoints = None
        self.traffic_lights = None
        self.frame_id = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.current_ego_pose is None or self.frame_id is None:
                continue

            car_index = self.get_closest_waypoint_index(self.current_ego_pose, self.base_waypoints)

            lookahead_waypoints = self.get_next_waypoints(self.base_waypoints, car_index, FORWARD_SCAN_WPS)

            # Publish
            lane = self.create_lane(self.frame_id, lookahead_waypoints)
            self.final_waypoints_pub.publish(lane)

    def pose_cb(self, message):
        self.current_ego_pose = message.pose  # store location (x, y)
        self.frame_id = message.header.frame_id

    def waypoints_cb(self, message):
        self.base_waypoints = message.waypoints

    def traffic_lights_cb(self, message):
        self.traffic_lights = message.lights

    def traffic_cb(self, message):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, message):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_next_waypoints(self, waypoints, i, n):
        """ Returns a list of waypoints ahead of the ego car """
        m = min(len(waypoints), i + n)
        return deepcopy(waypoints[i:m])

    def get_closest_waypoint_index(self, pose, waypoints):
        """ Returns index of the closest waypoint """
        best_distance = float('inf')
        best_waypoint_index = 0
        my_position = pose.position

        for i, waypoint in enumerate(waypoints):

            a_waypoint_position = waypoint.pose.pose.position
            gap = self.get_distance_between_two_points(my_position, a_waypoint_position)

            if gap < best_distance:
                best_waypoint_index, best_distance = i, gap

        is_behind = self.is_waypoint_behind_ego_car(pose, waypoints[best_waypoint_index])
        if is_behind:
            best_waypoint_index += 1
        return best_waypoint_index

    def get_distance_between_two_points(self, a, b):
        """ Returns distance between two points """
        dx = a.x - b.x
        dy = a.y - b.y
        return dx * dx + dy * dy

    def is_waypoint_behind_ego_car(self, pose, waypoint):
        """ Do transformation that sets origin to the ego car position, oriented along x-axis and
        return True if the waypoint is behind the ego car,  False if in front

        See Change of basis | Essence of linear algebra, chapter 9 - https://youtu.be/P2LTAUO1TdA
        """
        _, _, yaw = self.get_euler(pose)
        origin_x = pose.position.x
        origin_y = pose.position.y

        shift_x = waypoint.pose.pose.position.x - origin_x
        shift_y = waypoint.pose.pose.position.y - origin_y

        x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

        if x > 0:
            return False
        return True

    def get_euler(self, pose):
        """ Returns roll (x), pitch (y), yaw (z) from a Quaternion.

        See ROS Quaternion Basics for usage - http://wiki.ros.org/Tutorials/Quaternions
        """
        return transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def create_lane(self, frame_id, waypoints):
        new_lane = Lane()
        new_lane.header.frame_id = frame_id
        new_lane.waypoints = waypoints
        new_lane.header.stamp = rospy.Time.now()
        return new_lane


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
