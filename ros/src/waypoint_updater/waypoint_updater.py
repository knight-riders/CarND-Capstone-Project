#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
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
MAX_ACCEL = 1 # max acceleration in m/s2
OBS_STANDOFF = 10  # Preferred stopping distance from obstacle in m
TL_STANDOFF = 5  # Preferred stopping distance from obstacle in m

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.base_waypoints = None
        self.traffic_waypoint = None
        self.obstacle_waypoint = None
        self.linear_velocity = None
        self.psi = None

        # Publish waypoint updates at 2Hz (no need for a faster loop here)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        if (self.current_pose is not None) and (self.base_waypoints is not None):

            # index of next waypoint
            next_index = self.get_next_waypoint(self.current_pose.pose)
            # index of closest waypoint
            closest_index = self.get_closest_waypoint(self.current_pose.pose)
            #rospy.loginfo("CURRENT WAYPOINT: " + str(closest_index))
            lane = Lane()

            # generate final_waypoints
            final_waypoints = self.get_final_waypoints(next_index)

            obstacles = []
            # check for obstacles
            if (self.obstacle_waypoint is not None):
                for point in self.obstacle_waypoint:
                    obs = self.get_closest_waypoint(point.pose.pose) 
                    obstacles.append((obs, OBS_STANDOFF))

            # check for red lights
            if (self.traffic_waypoint is not None) and (self.traffic_waypoint != -1):
                #rospy.loginfo("TRAFFIC WAYPOINT :" + str(self.traffic_waypoint))
                obstacles.append((self.traffic_waypoint, TL_STANDOFF))

            # Deceleration zone for smooth speed transitions in m
            decel_zone = -self.linear_velocity**2/(2 * -MAX_ACCEL)
            #rospy.loginfo("Decel Zone :" + str(decel_zone))

            if (len(obstacles) > 0):
                for (obs, standoff) in obstacles:
                    obs_dist = self.distance(self.base_waypoints.waypoints, next_index, obs) - standoff
                    
                    # Obstacle with standoff waypoint index
                    obs_idx_wp = obs
                    dist = 0
                    while dist < standoff:
                        obs_idx_wp -= 1
                        dist = self.distance(self.base_waypoints.waypoints, obs_idx_wp, obs)

                    #rospy.loginfo("Obstacle Dist :" + str(obs_dist) + " Obstacle Index :" + str(obs_idx_wp))

                    if (-standoff <= obs_dist <= decel_zone):
                        # obstacle index with standoff in /final_waypoints
                        obs_idx_final_wp = obs_idx_wp - next_index
                       
                        # every speed setpoints to 0
                        for i in range(LOOKAHEAD_WPS):
                            self.set_waypoint_velocity(final_waypoints, i, 0.0)

                        # Looping backward from the obstacle
                        for i in range(obs_idx_final_wp, 0, -1):
                            inter_wp_dist = self.distance(final_waypoints, i-1, i)
                            next_vel = math.sqrt(self.get_waypoint_velocity(final_waypoints[i])**2 + 2*MAX_ACCEL*inter_wp_dist)
                            #rospy.loginfo("next vel :" + str(next_vel) + " index :" + str(i))
                            self.set_waypoint_velocity(final_waypoints, i-1, next_vel)
                            
            self.publish(final_waypoints)

    def get_final_waypoints(self, next_index):
        final_waypoints = []
        wlen = len(self.base_waypoints.waypoints)

        for i in range(LOOKAHEAD_WPS):
            wp = Waypoint()
            wp.pose.pose.position.x = self.base_waypoints.waypoints[(next_index+i)%wlen].pose.pose.position.x
            wp.pose.pose.position.y = self.base_waypoints.waypoints[(next_index+i)%wlen].pose.pose.position.y
            wp.pose.pose.position.z = self.base_waypoints.waypoints[(next_index+i)%wlen].pose.pose.position.z
            wp.pose.pose.orientation.x = self.base_waypoints.waypoints[(next_index+i)%wlen].pose.pose.orientation.x
            wp.pose.pose.orientation.y = self.base_waypoints.waypoints[(next_index+i)%wlen].pose.pose.orientation.y
            wp.pose.pose.orientation.z = self.base_waypoints.waypoints[(next_index+i)%wlen].pose.pose.orientation.z
            wp.pose.pose.orientation.w = self.base_waypoints.waypoints[(next_index+i)%wlen].pose.pose.orientation.w

            wp.twist.twist.linear.x = self.base_waypoints.waypoints[(next_index+i)%wlen].twist.twist.linear.x
            wp.twist.twist.linear.y = 0.0
            wp.twist.twist.linear.z = 0.0
            wp.twist.twist.angular.x = 0.0
            wp.twist.twist.angular.y = 0.0
            wp.twist.twist.angular.z = 0.0

            final_waypoints.append(wp)

        return final_waypoints

    def publish(self, final_waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = final_waypoints
        self.final_waypoints_pub.publish(lane)

    # # Convert a waypoint from the world to the vehicle frame (from P10 - MPC project)
    # def get_local_xy(self, x, y):
    #     shift_x = x - self.current_pose.pose.position.x
    #     shift_y = y - self.current_pose.pose.position.y

    #     ptx =  shift_x * math.cos(self.psi) + shift_y * math.sin(self.psi)
    #     pty = -shift_x * math.sin(self.psi) + shift_y * math.cos(self.psi)
    #     return ptx, pty

    # # Convert a wapoint from the vehicle frame to the vehicle frame (from P10 - MPC project)
    # def get_world_xy(self, ptx, pty):
    #     shift_x = ptx * math.cos(self.psi) - pty * math.sin(self.psi)
    #     shift_y = ptx * math.sin(self.psi) + pty * math.cos(self.psi)

    #     x = shift_x + self.current_pose.pose.position.x
    #     y = shift_y + self.current_pose.pose.position.y
    #     return x, y

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

    def current_velocity_cb(self, msg):
        self.linear_velocity = msg.twist.linear.x

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

        # to enable loopback..
        len_wp = len(waypoints)
        if wp2 <= len_wp/2 and wp1 > len_wp/2:
            for i in range(wp1, len_wp):
                dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
                wp1 = i
            wp1 = 0

        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
