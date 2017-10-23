#!/usr/bin/env python

import rospy

from tf import transformations
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray

from math import cos, sin, sqrt
from copy import deepcopy
import numpy as np

FORWARD_SCAN_WPS = 200  # aka publish this number of waypoints
MPS = 0.44704  # is 1 MPH
MIN_VEL = -0.1
STOP = 0
GO = 1
DECEL = 2

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
    **Diagram (5) - New target trajectory planned - PLANNING**

    This node will publish waypoints from the car's current position to some `x` distance ahead,
    with the correct target velocities, depending on traffic lights and obstacles.
    """

    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)  # Simulator data
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        # Publishers
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_ego_pose = None  # ego car current position and orientation
        self.base_waypoints = None
        self.traffic_lights = None
        self.frame_id = None
        self.tlwp = 0
        self.action = GO
        self.max_vel = 0.0
        self.i = 0

        self.loop()

    def loop(self):
        """ Updates waypoints in front of the cars current position with the new target trajectory. """
        #rate = rospy.Rate(2)
	rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.current_ego_pose is None or self.frame_id is None:
                continue

            if len(self.base_waypoints) > FORWARD_SCAN_WPS and self.i > 20:
                self.car_index = self.get_closest_waypoint_index(self.current_ego_pose, self.base_waypoints)

                # lookahead_waypoints = self.get_next_waypoints(self.base_waypoints, car_index, FORWARD_SCAN_WPS)
                lookahead_waypoints = self.get_waypoints(self.car_index)

                # rospy.logwarn("Velocity 1: %s", lookahead_waypoints[0].twist.twist.linear.x)
                # rospy.logwarn("Angular  1: %s", lookahead_waypoints[0].twist.twist.angular.z)

                # Publish
                lane = self.create_lane(self.frame_id, lookahead_waypoints)
                self.final_waypoints_pub.publish(lane)
                # rospy.logwarn("action: %s", self.action)

    ######## Planning waypoints  ##############

    def get_waypoints(self, current_wp):
        """ PREDICTION DECISIONS:
        	1) If car is 5 m away from stop line, and velocity is under 1.0, STOP 
        	2) If car is 100 m away from RED light, SLOW DOWN
        	3) If car is more than 2 seconds away from GREEN light, SLOWN DOWN
        	4) If not within detection or GREEN light is less than 16 m away, go FULL SPEED
        """
        # current_wp = car_index
        if (current_wp + FORWARD_SCAN_WPS) > len(self.base_waypoints):
            _lookahead = len(self.base_waypoints) - current_wp
        else:
            _lookahead = FORWARD_SCAN_WPS
        # Calculate CTE for steering
        cte_steer, tfpx, tfpy = self.cte_calc(self.current_ego_pose, self.base_waypoints, current_wp,
                                              _lookahead)

        # Calculate TF distance
        # RED is not negative, everything else is -tfwp
        cte_tl = None
        if self.tlwp > 0:
            cte_tl = self.distance(self.base_waypoints, current_wp, self.tlwp)
        elif self.tlwp < -1:
            cte_tl = -self.distance(self.base_waypoints, current_wp, self.tlwp * (-1))
            if cte_tl > -16.0:  # if within 16 meters from GREEN light, go full speed
                cte_tl = None
            else:
                cte_tl *= -1

        """ PREDICTION DECISIONS """
        if (cte_tl is not None) and (cte_tl < 5.0) and (self.current_linear_velocity < 1.0 * MPS):
            self.action = STOP

        elif (cte_tl is not None) and (cte_tl < 100.0) and (cte_tl > 0.0):
            self.action = DECEL

        ### Consider using this if having a confident classification for far Green detection
        # elif (cte_tl is not None) and (cte_tl < 0.0) and (
        #             (self.current_linear_velocity * 2 + 2.0) < cte_tl * (-1)):
        #     self.action = DECEL
        #     cte_tl *= -1

        else:  # no within stopping range or current green light and can pass
            self.action = GO

        waypoints = []
        wp_to_tl = 0

        for i in range(_lookahead):
            # wp = Waypoint()
            wp = self.base_waypoints[current_wp + i]
            if cte_tl is not None:
                car_to_TL = (cte_tl - wp_to_tl) * (-1.0)
            else:
                car_to_TL = None

            _velocity = self.set_velocity(self.action, car_to_TL)

            wp.twist.twist.linear.x = _velocity
            wp.twist.twist.linear.y = 0.0
            wp.twist.twist.linear.z = 0.0
            wp.twist.twist.angular.x = 0.0
            wp.twist.twist.angular.y = 0.0
            wp.twist.twist.angular.z = cte_steer(tfpx[i])

            waypoints.append(wp)

            wp_to_tl += self.distance(self.base_waypoints, current_wp + i, current_wp + i + 1)

        rospy.logwarn("======================TL distance: %s m",
                      cte_tl)
        # rospy.logwarn("TL wp: %s",
        #                   self.tlwp)

        return waypoints

    ############## HELP Functions ################
    def set_velocity(self, action, distance=None):
        if action is DECEL and distance is not None:
            decelcurve = self.velocity_calc(self.max_vel)
            velocity = decelcurve(distance)
        elif action is STOP:
            velocity = MIN_VEL
        else:
            velocity = self.max_vel
        return velocity

    def velocity_calc(self, max_vel):
        # Setting points to find deceleration curve
        x = []  # distance axis
        y = []  # velocity axis
        # # Max point @20 s away
        # x.append(min(-max_vel * 20, -200))
        # y.append(max_vel)

        # x.append(min(-max_vel * 10, -100))
        # y.append(max_vel)

        # # @14 s away
        # x.append(min(-max_vel * 0.5 * 14, 75))
        # y.append(max_vel * 0.5)

        # # @13 s away
        # x.append(min(-max_vel * 0.4 * 13, -55))
        # y.append(max_vel * 0.4)

        # # @9 s away
        # x.append(min(-max_vel * 0.3 * 9, -30))
        # y.append(max_vel * 0.2)

        # # @16m away
        # x.append(-16)
        # y.append(MPS * 2)

        # # @10 m away
        # x.append(-10)
        # y.append(MPS)

        # # @5 m away
        # x.append(-5)
        # y.append(MPS * 0.5)

        # # @0 m away
        # x.append(0.)
        # y.append(MIN_VEL)

        # # @ after traffic light, all points are zeros
        # x.append(max(max_vel * 20, 200))
        # y.append(MIN_VEL)
        ############ NEW CURVE ##########
        x.append(max(-200, -max_vel * 20))
        y.append(max_vel)

        x.append(max(-100, -max_vel * 10))
        y.append(max_vel)

        x.append(max(-75, -max_vel * 7))
        y.append(max_vel * 0.5)

        x.append(max(-45, -max_vel * 4))
        y.append(max_vel * 0.2)

        x.append(max(-16, -max_vel * 1.5))
        y.append(1.0 * MPS)

        x.append(max(-10, -max_vel * 1.0))
        y.append(0.5 * MPS)

        x.append(max(-5, -max_vel * 0.5))
        y.append(0.5 * MPS)

        x.append(0.2)
        y.append(MIN_VEL)

        x.append(min(200, max_vel * 20))
        y.append(MIN_VEL)

        decel_curve = np.polyfit(x, y, 3)
        decelcurve = np.poly1d(decel_curve)
        return decelcurve

    def cte_calc(self, pose, waypoints, car_index, POINTS_TO_FIT):
        """
        Calculates the distance from the ego cars current position to the waypoints path.

        See Polynomial fitting - http://blog.mmast.net/least-squares-fitting-numpy-scipy
        """
        # rospy.logwarn("-----")

        x_coords, y_coords = self.transform_waypoints(pose, waypoints, car_index,
                                                      POINTS_TO_FIT)  # Use 10 waypoints
        coefficients = np.polyfit(x_coords, y_coords, 3)  # 3-degree polynomial fit, minimising squared error
        distance = np.poly1d(coefficients)  # distance between car position and transformed waypoint

        return distance, x_coords, y_coords

    def transform_waypoints(self, pose, waypoints, car_index, points_to_use=None):
        """
        Do transformation that sets origin of waypoints to the ego car position, oriented along x-axis and
        returns transformed waypoint co-ordinates.

        See Change of basis | Essence of linear algebra, chapter 9 - https://youtu.be/P2LTAUO1TdA
        """
        x_coords = []  # array to hold transformed waypoint x
        y_coords = []  # array to hold transformed waypoint y

        _, _, yaw = self.get_euler(pose)
        origin_x = pose.position.x
        origin_y = pose.position.y

        if points_to_use is None:
            points_to_use = len(waypoints)

        for i in range(points_to_use):
            shift_x = waypoints[car_index + i].pose.pose.position.x - origin_x
            shift_y = waypoints[car_index + i].pose.pose.position.y - origin_y

            # x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
            # y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)
            x = shift_x * cos(yaw) - shift_y * sin(yaw)
            y = shift_x * sin(yaw) + shift_y * cos(yaw)
            # rospy.logwarn("i %s - x_coord : %s", i, x)
            # rospy.logwarn("i %s - y_coord : %s", i, y)

            x_coords.append(x)
            y_coords.append(y)

        return x_coords, y_coords

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

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    ######## Obtain information from nodes ##########
    def pose_cb(self, message):
        self.i += 1
        self.current_pose = message
        self.current_ego_pose = message.pose  # store location (x, y)
        self.frame_id = message.header.frame_id

    def waypoints_cb(self, message):
        self.base_waypoints = message.waypoints
        self.max_vel = message.waypoints[0].twist.twist.linear.x

    def traffic_lights_cb(self, message):
        self.traffic_lights = message.lights

    def traffic_cb(self, message):
        # TODO: Callback for /traffic_waypoint message. Implemented.
        self.tlwp = message.data

    def obstacle_cb(self, message):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_cb(self, message):
        self.velocity = message.twist
        self.current_linear_velocity = message.twist.linear.x
        self.current_angular_velocity = message.twist.angular.z


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
