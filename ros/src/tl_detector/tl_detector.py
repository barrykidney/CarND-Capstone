#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import yaml
import os

from light_classification.utils.imagesaver import ImageSaver

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.safe_visualizations = False  # saves camera images with detected bounding boxes
        self.use_classifier = True  # use the traffic light detection
        self.waypoint_range = 200
        self.current_tl_wp_idx = 0

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.camera_image = None
        self.lights = []

        self.light_classifier = TLClassifier(self.safe_visualizations)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        # rospy.logwarn(self.config)

        self.is_site = self.config['is_site']

        output_dir = os.path.join('./data/dataset/', 'site' if self.is_site else 'simulator')
        self.image_saver = ImageSaver(None, output_dir)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.no_of_wp = None
        self.idx_of_closest_wp_to_car = None
        self.line_wp_idxs_list = []
        self.has_image = False

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.no_of_wp = len(self.waypoints.waypoints)
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # rospy.logwarn(msg.lights)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        if self.is_site:
            self.has_image = True
            self.camera_image = msg
            tl_in_range = self.traffic_light_in_range()
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
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

        else:
            if self.waypoints_tree and self.light_classifier:
                tl_in_range = self.traffic_light_in_range()
                rospy.logwarn("Car wp: {},    TL in range: {}".format(self.idx_of_closest_wp_to_car, tl_in_range))
                if tl_in_range:
                    rospy.logwarn("TL wp:  {}, TL idx: {} in range {} wp"
                                  .format(self.line_wp_idxs_list[self.current_tl_wp_idx % len(self.line_wp_idxs_list)],
                                          self.current_tl_wp_idx,
                                          self.waypoint_range))
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
                        light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
                        self.last_wp = light_wp
                        self.upcoming_red_light_pub.publish(Int32(light_wp))
                    else:
                        self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                    self.state_count += 1

    def traffic_light_in_range(self):
        """Identifies if the closest traffic light to the vehicles position
            is within a given number of waypoints ahead of the vehicle.

        Returns:
            boolean: if a traffic light satisfies these parameters or not
        """

        offest = 25  # approx offset (in waypoints) from the traffic light and the stop line.

        if self.pose:
            self.idx_of_closest_wp_to_car = self.get_closest_waypoint(self.pose.pose.position.x,
                                                                      self.pose.pose.position.y)
            if len(self.line_wp_idxs_list) < 1:
                for i, light in enumerate(self.lights):
                    line = self.stop_line_positions[i]
                    self.line_wp_idxs_list.append(self.get_closest_waypoint(line[0], line[1]))

            if self.idx_of_closest_wp_to_car \
                    > self.line_wp_idxs_list[self.current_tl_wp_idx % len(self.line_wp_idxs_list)]:
                self.current_tl_wp_idx += 1
            if self.idx_of_closest_wp_to_car + self.waypoint_range \
                    > self.line_wp_idxs_list[self.current_tl_wp_idx % len(self.line_wp_idxs_list)] + offest:
                return True
            else:
                return False

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x (float): position to match a waypoint to
            y (float): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        if not self.has_image:
            return False

        if not self.use_classifier:
            rospy.loginfo('Light state: %s', light.state)
            return light.state

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, 'rgb8')
        classified_state = self.light_classifier.get_classification(cv_image)

        if not self.is_site:
            rospy.loginfo("Sim ground truth state: {}".format(self.state_to_string(light.state)))

        rospy.loginfo("Classified state:       {}".format(self.state_to_string(classified_state)))

        return classified_state

    @staticmethod
    def state_to_string(state):
        if state == TrafficLight.RED:
            return "Red"
        elif state == TrafficLight.YELLOW:
            return "Yellow"
        elif state == TrafficLight.GREEN:
            return "Green"
        return "Unknown"

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        closest_light = None
        line_wp_idx = None

        if self.pose:
            diff = self.no_of_wp
            for i, light in enumerate(self.lights):
                idx_of_closest_wp_to_line = self.line_wp_idxs_list[i]

                d = idx_of_closest_wp_to_line - self.idx_of_closest_wp_to_car
                if diff > d >= 0:
                    diff = d
                    closest_light = light
                    line_wp_idx = idx_of_closest_wp_to_line

        if closest_light:
            return line_wp_idx, self.get_light_state(closest_light)

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
