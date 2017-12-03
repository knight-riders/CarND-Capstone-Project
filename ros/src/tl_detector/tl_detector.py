#!/usr/bin/env python
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import cv2
import math
import rospy
import tf
import yaml

STATE_COUNT_THRESHOLD = 3
HIGHEST_NUM = float('inf')


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub_curr_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub_base_waypts = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub_traffic_lights = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub_image_color = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def calc_distance(self, x1, y1, x2, y2):
        """Calculates the distance between two points

        Args:
            float: First point's x-coordinate
            float: First point's y-coordinate
            float: Second point's x-coordinate
            float: Second point's y-coordinate

        Returns:
            float: Distance between the two provided points
        """
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        shortest_distance = HIGHEST_NUM
        closest_wp = -1
        for idx, wp in enumerate(self.waypoints):
            distance = self.calc_distance(pose.position.x, pose.position.y, wp.pose.pose.position.x,
                                          wp.pose.pose.position.y)
            if distance < shortest_distance:
                shortest_distance = distance
                closest_wp = idx
        return closest_wp

    def is_waypoint_ahead(self, wp_x, wp_y):
        """Determines whether a waypoint is ahead of the car

        Args:
            float: waypoint's global x-coordinate
            float: waypoint's global y-coordinate

        Returns:
            bool: Whether the waypoint is ahead of the car
        """
        car_x = self.pose.position.x
        car_y = self.pose.position.y
        car_orientation = self.pose.orientation
        euler = tf.transformations.euler_from_quaternion([car_orientation.x, car_orientation.y, car_orientation.z,
                                                          car_orientation.w])
        car_yaw = euler[2]
        return ((wp_x - car_x) * math.cos(car_yaw) +
                (wp_y - car_y) * math.sin(car_yaw)) > 0

    def get_closest_tl_idx(self):
        """Determine the index of the closest traffic light

        Returns:
             int: Index of closest traffic light
        """
        closest_tl_idx = -1
        shortest_dist = HIGHEST_NUM
        for idx, light in enumerate(self.lights):
            tl = light.pose.pose.position
            if self.is_waypoint_ahead(tl.x, tl.y):
                car_x = self.pose.position.x
                car_y = self.pose.position.y
                distance = self.calc_distance(car_x, car_y, tl.x, tl.y)
                if distance < shortest_dist:
                    shortest_dist = distance
                    closest_tl_idx = idx
                else:
                    continue
            else:
                continue
        return closest_tl_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            light_wp = self.get_closest_tl_idx()
            light = self.lights[light_wp]


        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
