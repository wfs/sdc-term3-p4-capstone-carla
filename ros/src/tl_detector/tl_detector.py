#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from tf import transformations
from math import cos, sin, sqrt
import math
import tf
import yaml

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    """
    **Diagram (4) - Traffic light perceived - PERCEPTION**

    To build and train traffic light detection / classification ...

    1. Get and label training data into Pascal VOC format
    2. Generate TF record
    3. Feed record to object detection API TF
    4. Train
    5. Freeze weight once suitable error level reached
    6. Done
    """

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")

        # Define stopping locations
        self.config = yaml.load(config_string)
        # 
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    # def closest_wp_to_stopline(self, stopline_loc, waypoints):

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.stop_posts, self.stop_index = self.all_stop_wp(
            self.config["stop_line_positions"])  # from find_stop_pos

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if state == TrafficLight.RED:
                light_wp = light_wp
            elif state == TrafficLight.GREEN or state == TrafficLight.YELLOW:
                light_wp = light_wp * (-1)
            else:
                light_wp = -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        rospy.logwarn("TL Waypoint, state: %s, %s", self.last_wp, self.state)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # TODO implement
        return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        stop_wp = None
        if (self.pose):
            # car_position = self.get_closest_waypoint(self.pose.pose)
            stop_wp = self.get_closest_stop_wp(self.pose, self.stop_index, 100)

        # TODO find the closest visible traffic light (if one exists)

        # if light:
        if stop_wp != None:
            """ Find state with classifier """
            # state = self.get_light_state(light)
            # return light_wp, state

            """ Find state with simulator """
            state = self.get_state_in_sim(stop_wp)
            # state = self.lights[0].state
            return stop_wp, state
        else:
            return -1, TrafficLight.UNKNOWN
            # self.waypoints = None
            # return -1, TrafficLight.UNKNOWN

    #   Find the closest next stopping point 

    def get_closest_stop_wp(self, current_pose, stopping_indices, limit_distance):
        stop_wp = None
        car_index = self.get_closest_waypoint_index(current_pose.pose, self.waypoints)
        for indx in stopping_indices:
            dist = self.distance(self.waypoints, car_index, indx)
            if dist < limit_distance and not self.is_waypoint_behind_ego_car(current_pose.pose,
                                                                             self.waypoints[indx]):
                stop_wp = indx

        return stop_wp

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

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
        return sqrt(dx * dx + dy * dy)

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

    def get_state_in_sim(self, stop_wp):
        """ Use traffic pose and state from simulator to test """
        light_wp = stop_wp
        light_loc = self.waypoints[light_wp].pose.pose.position
        best_dist = float('inf')
        current_light_in_sim = None
        for i, light in enumerate(self.lights):
            pos = light.pose.pose.position
            dist = ((pos.x - light_loc.x) ** 2 + (pos.y - light_loc.y) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                current_light_in_sim = i

        return self.lights[current_light_in_sim].state

    def get_euler(self, pose):
        """ Returns roll (x), pitch (y), yaw (z) from a Quaternion.

        See ROS Quaternion Basics for usage - http://wiki.ros.org/Tutorials/Quaternions
        """
        return transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def closest_waypoint(self, pose):
        best_distance = float('inf')
        best_wp_index = 0
        best_wp = None
        # current_pos = pose.position

        for i, wp in enumerate(self.waypoints):
            wp_pos = wp.pose.pose.position
            dist = ((pose[0] - wp_pos.x) ** 2 + (pose[1] - wp_pos.y) ** 2) ** 0.5

            if dist < best_distance:
                best_distance = dist
                best_wp, best_wp_index = wp, i

        _, _, yaw = self.get_euler(self.waypoints[best_wp_index].pose.pose)
        nx = pose[0] - self.waypoints[best_wp_index].pose.pose.position.x
        ny = pose[1] - self.waypoints[best_wp_index].pose.pose.position.y

        # Check if waypoint is ahead of stop line
        ahead = nx * cos(0 - yaw) - ny * sin(0 - yaw)
        if ahead < 0:
            best_wp_index -= 1
        # print (best_wp_index)
        return best_wp_index

    def all_stop_wp(self, traffic_light_posts):
        """ Find all stop line positions from the provided location """
        stop_wp = []
        stop_wp_index = []
        for pos in traffic_light_posts:
            best_index = self.closest_waypoint(pos)
            # print (best_index)
            stop_wp.append([self.waypoints[best_index].pose.pose.position.x,
                            self.waypoints[best_index].pose.pose.position.y])

            stop_wp_index.append(best_index)

        return stop_wp, stop_wp_index


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
