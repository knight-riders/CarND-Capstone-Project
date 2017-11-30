#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # Parameters
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)        # in kg
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)         # in US gal.
        brake_deadband = rospy.get_param('~brake_deadband', .1)         # in % ??
        decel_limit = rospy.get_param('~decel_limit', -5)               # in m/s^2
        accel_limit = rospy.get_param('~accel_limit', 1.)               # in m/s^2
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)         # in m
        wheel_base = rospy.get_param('~wheel_base', 2.8498)             # in m
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)             # dimension less
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)           # in m/s^2
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)       # in rad
        min_speed = rospy.get_param('~min_speed', 1.78816)              # 4 mph in m/s
        self.sample_time = rospy.get_param('~sample_time', 0.02)        # in s
        max_throttle_perc = rospy.get_param('~max_throttle_perc', 0.05) # in %
        max_brake_perc = rospy.get_param('~max_brake_perc', -0.05)      # in %

        # Create `TwistController` object
        kwargs = {
            'vehicle_mass'      : vehicle_mass,
            'fuel_capacity'     : fuel_capacity,
            'brake_deadband'    : brake_deadband,
            'decel_limit'       : decel_limit,
            'accel_limit'       : accel_limit,
            'wheel_radius'      : wheel_radius,
            'wheel_base'        : wheel_base,
            'steer_ratio'       : steer_ratio,
            'min_speed'         : min_speed,
            'max_lat_accel'     : max_lat_accel,
            'max_steer_angle'   : max_steer_angle,
            'sample_time'       : self.sample_time,
            'max_throttle_perc'	: max_throttle_perc,
            'max_brake_perc'	: max_brake_perc
        }
        self.controller = Controller(**kwargs)

        # Publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Subscribers
        self.twist_cmd = None
        self.current_velocity = None
        self.linear_velocity_cmd = None
        self.angular_velocity_cmd = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.dbw_enabled = False

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(1/self.sample_time) # 50Hz
        while not rospy.is_shutdown():
            kwargs = {
                'linear_velocity_cmd'  : self.linear_velocity_cmd,
                'angular_velocity_cmd' : self.angular_velocity_cmd,
                'linear_velocity'      : self.linear_velocity,
                'angular_velocity'     : self.angular_velocity,
                'dbw_enabled'          : self.dbw_enabled
            }
            throttle, brake, steer = self.controller.control(**kwargs)

            if (self.current_velocity is not None) and (self.twist_cmd is not None) and (self.dbw_enabled is True):
                self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg
        self.linear_velocity_cmd = self.twist_cmd.twist.linear.x
        self.angular_velocity_cmd = self.twist_cmd.twist.angular.z

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

if __name__ == '__main__':
    DBWNode()
