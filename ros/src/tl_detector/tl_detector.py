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
import time
from tldetect import predictor

STATE_COUNT_THRESHOLD = 1
FORWARD_SCAN_WPS = 400


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
        p = predictor(modelpath="./FrozenSyam.pb")
        self.predictor = p
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

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
        self.init = False
        self.car_index = None
        self.time = 0.
        self.time_image = 0.
        self.time_image_prev = 0.

        rospy.spin()

    ### Obtain nodes information """
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.stop_posts, self.stop_index = self.all_stop_wp(
            self.config["stop_line_positions"])  # from find_stop_pos
        self.init = True

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.time_image = time.time() - self.time_image_prev
        self.time_image_prev = time.time()

        rospy.logwarn("Image call back loop: %s secs", self.time_image)

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
        # rospy.logwarn("TL Waypoint, state: %s, %s", self.last_wp, self.state)
        rospy.logwarn("Prediction time: %s", self.time)
        # rospy.logwarn("Prediction: %s, Scores: %s", pred, skores)

    # """ HELP FUNCTIONS """
    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # RED, YELLOW, GREEN = 1, 2, 3
        if (not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image)

        start = time.time()
        light = self.predictor.predict(cv_image)
        end = time.time()
        self.time = end - start

        # rospy.logwarn("Prediction: %s, Scores: %s", pred, skores)

        # Get classification
        # return self.light_classifier.get_classification(cv_image)
        return light

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_wp = None
        if (self.init):
            # car_position = self.get_closest_waypoint(self.pose.pose)
            stop_wp = self.get_closest_stop_wp(self.pose, self.stop_index)

        # TODO find the closest visible traffic light (if one exists)

        state = TrafficLight.UNKNOWN

        if stop_wp is not None:
            state = self.get_light_state()
            return stop_wp, state
        else:
            return -1, state

    ####  FIND THE CLOSET STOPPING POINT BEHIND THE TRAFFIC LIGHT ####

    def get_closest_stop_wp(self, current_pose, stopping_indices):
        stop_wp = None
        self.car_index = self.get_closest_waypoint(current_pose.pose, self.waypoints)

        for i in range(len(stopping_indices)):
            # dist = self.distance(self.waypoints, self.car_index, indx)
            # if dist < limit_distance and not self.is_waypoint_behind_ego_car(current_pose.pose, self.waypoints[indx]):
            #     stop_wp = indx
            if self.car_index < stopping_indices[i]:
                return stopping_indices[i]

        return None

    #### FIND ALL NEAREST WAYPOINTS BEHIND STOP LINE FROM GIVEN LOCATIONS ###
    def all_stop_wp(self, traffic_light_posts):
        stop_wp = []
        stop_wp_index = []
        for pos in traffic_light_posts:
            best_index = self.closest_waypoint(pos)
            stop_wp.append([self.waypoints[best_index].pose.pose.position.x,
                            self.waypoints[best_index].pose.pose.position.y])

            stop_wp_index.append(best_index)

        return stop_wp, stop_wp_index

    ### HELP FUNCTIONS ###

    def distance(self, waypoints, wp1, wp2):
        ''' Find true distance between two waypoints '''
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, pose, waypoints):
        """ Returns index of the closest waypoint ahead """
        best_distance = float('inf')
        best_waypoint_index = 0
        my_position = pose.position

        # for i, waypoint in enumerate(waypoints):

        #     a_waypoint_position = waypoint.pose.pose.position
        #     gap = self.get_distance_between_two_points(my_position, a_waypoint_position)

        #     if gap < best_distance:
        #         best_waypoint_index, best_distance = i, gap
        if self.car_index is None:
            prev_car_index = 0
            scan_waypoints = waypoints
        else:
            prev_car_index = self.car_index
            scan_waypoints = waypoints[prev_car_index: (prev_car_index + FORWARD_SCAN_WPS)]

        for i, waypoint in enumerate(scan_waypoints):
            a_waypoint_position = waypoint.pose.pose.position
            gap = self.get_distance_between_two_points(my_position, a_waypoint_position)

            if gap < best_distance:
                best_waypoint_index, best_distance = prev_car_index + i, gap

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

    def get_euler(self, pose):
        """ Returns roll (x), pitch (y), yaw (z) from a Quaternion.

        See ROS Quaternion Basics for usage - http://wiki.ros.org/Tutorials/Quaternions
        """
        return transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def closest_waypoint(self, pose):
        """ Find closet waypoint behind with given X, Y coordinate """
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


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
