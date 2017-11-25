#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.base_waypoints = None
        self.final_waypoints = None
        self.traffic_waypoint = None
        self.obstacle_waypoint = None
        self.current_velocity = None
        self.psi = None

        # Publish waypoint updates at 2Hz (no need for a faster loop here)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        if self.current_pose and self.base_waypoints:

            # index of next waypoint
            next_index = self.get_next_waypoint(self.current_pose.pose)
            # index of cloosest waypoint
            closest_index = self.get_closest_waypoint(self.current_pose.pose)
            lane = Lane()

            # generate final_waypoints
            if self.traffic_waypoint and self.traffic_waypoint != -1:
                # Traffic light distance computation
                tl_dist = self.distance(self.base_waypoints.waypoints, closest_index, self.traffic_waypoint)

                # TODO begin deceleration when close enough to a stop line, else don't modify the speed set point
                # (rejects false red/yellow light detections)
            else:
                rospy.loginfo("NO TRAFFIC LIGHT")
                lane.waypoints = self.base_waypoints.waypoints[next_index : (next_index + LOOKAHEAD_WPS)]

            self.publish(lane)

    def publish(self, lane):
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints = lane.waypoints
        self.final_waypoints_pub.publish(lane)

    # Convert a waypoint from the world to the vehicle frame (from P10 - MPC project)
    def get_local_xy(self, x, y):
        shift_x = x - self.current_pose.pose.position.x
        shift_y = y - self.current_pose.pose.position.y

        ptx =  shift_x * math.cos(self.psi) + shift_y * math.sin(self.psi)
        pty = -shift_x * math.sin(self.psi) + shift_y * math.cos(self.psi)
        return ptx, pty

    # Convert a wapoint from the vehicle frame to the vehicle frame (from P10 - MPC project)
    def get_world_xy(self, ptx, pty):
        shift_x = ptx * math.cos(self.psi) - pty * math.sin(self.psi)
        shift_y = ptx * math.sin(self.psi) + pty * math.cos(self.psi)

        x = shift_x + self.current_pose.pose.position.x
        y = shift_y + self.current_pose.pose.position.y
        return x, y

    # Get closest waypoint index(from P11 - Path planning project)
    def get_closest_waypoint(self, pose):

        closest_dist = 100000

        p1 = pose.position
        waypoints = self.base_waypoints.waypoints

        for i in range(len(waypoints)):
            p2 = waypoints[i].pose.pose.position
            d = self.dist(p1, p2)
            if d < closest_dist:
                closest_dist = d
                closest_index = i

        return closest_index

    # Get next waypoint index(from P11 - Path planning project)
    def get_next_waypoint(self, pose):

        next_index = self.get_closest_waypoint(pose)
        p1 = pose.position
        p2 = self.base_waypoints.waypoints[next_index].pose.pose.position

        heading = math.atan2((p2.y-p1.y), (p2.x-p1.x))

        angle = abs(self.psi-heading)

        if angle > math.pi/4:
            next_index += 1

        return next_index

    def pose_cb(self, msg):
        self.current_pose = msg
        quaternion = (self.current_pose.pose.orientation.x,
                      self.current_pose.pose.orientation.y,
                      self.current_pose.pose.orientation.z,
                      self.current_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.psi = yaw

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. Will be implemented by Udacity later..
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # Returns the euclidian distance between 2 points (P11 - Path Planning)
    def dist(self, p1, p2):
        return math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2 + (p2.z-p1.z)**2)

    # Returns the distance between two waypoints along the path
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
