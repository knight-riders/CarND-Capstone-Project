#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker

class CarlaVisualization(object):

    def __init__(self):
        rospy.init_node("carla_visualization")

        # From Waypoint Loader node to Waypoint Updater
        rospy.Subscriber("/base_waypoints", Lane, self.base_waypoints_cb)

        # From Waypoint Updater node to Waypoint Follower (Pure Pursuit)
        rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_cb)

        # From Traffic Light Detection node to Waypoint Updater
        # rospy.Subscriber("/traffic_waypoint", Lane, self.traffic_waypoint_cb)

        # From Car/Simulator
        rospy.Subscriber("/vehicle/traffic_lights", TrafficLightArray, self.traffic_light_ground_truth_cb)

        # To RVIZ
        self.base_path_pub = rospy.Publisher('carla_visualization/base_waypoints_path', Path, queue_size=1, latch=True)
        self.final_path_pub = rospy.Publisher('carla_visualization/final_waypoints_path', Path, queue_size=1)
        self.traffic_light_ground_truth_pub = rospy.Publisher("carla_visualization/traffic_light_ground_truth", MarkerArray, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            rate.sleep()

    def final_waypoints_cb(self, lane):
        path = Path()
        path = self.path_from_lane(lane, "world")

        self.final_path_pub.publish(path)

    def base_waypoints_cb(self, lane):
        path = Path()
        path = self.path_from_lane(lane, "/world")

        self.base_path_pub.publish(path)

    def path_from_lane(self, lane, frame_id):
        path = Path()
        path.header.frame_id = frame_id

        for waypoint in lane.waypoints:
            wp_pose = waypoint.pose.pose

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = wp_pose.position.x
            pose.pose.position.y = wp_pose.position.y

            path.poses.append(pose)

        return path


    def traffic_light_ground_truth_cb(self, traffic_light_array):
        marker_array = MarkerArray()

        for index, light in enumerate(traffic_light_array.lights):
            marker = self.gen_tl_marker(light, index, "/world", "tl_ground_truth")
            marker_array.markers.append(marker)

        self.traffic_light_ground_truth_pub.publish(marker_array)

    def gen_tl_marker(self, light, index, frame_id, ns):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = light.pose.pose
        marker.scale.x = 15.0
        marker.scale.y = 15.0
        marker.scale.z = 15.0

        marker.color.a = 1.0        
        marker.color.b = 0.0
        if light.state == TrafficLight.RED:
            marker.color.r = 1.0
            marker.color.g = 0.0
        elif light.state == TrafficLight.GREEN:
            marker.color.r = 0.0
            marker.color.g = 1.0
        elif light.state == TrafficLight.YELLOW:
            marker.color.r = 1.0
            marker.color.g = 1.0
        elif light.state == TrafficLight.UNKNOWN:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

        return marker

if __name__ == '__main__':
    try:
        CarlaVisualization()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Carla visualization node.')
