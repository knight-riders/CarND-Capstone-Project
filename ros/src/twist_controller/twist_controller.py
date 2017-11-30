from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858  # in kg/US gal
ONE_MPH = 0.44704    # in m/s
BRAKE_TORQUE_MAX = 3412.0  # in Nm

class Controller(object):
    def __init__(self, *args, **kwargs):

        self.sample_time = kwargs['sample_time']
        self.min_speed = kwargs['min_speed']
        self.max_throttle_perc = kwargs['max_throttle_perc']
        self.max_brake_perc = kwargs['max_brake_perc']
        self.wheel_base = kwargs['wheel_base']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.brake_deadband = kwargs['brake_deadband']

        self.linear_velocity_cmd = None
        self.angular_velocity_cmd = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.dbw_enabled = False

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        self.lowpassfilter_steer_cmd = LowPassFilter(0.1, self.sample_time) # 0.1s time constant
        self.lowpassfilter_accel_cmd = LowPassFilter(0.1, self.sample_time) # 0.1s time constant

    def control(self, *args, **kwargs):
        self.linear_velocity_cmd = kwargs['linear_velocity_cmd']
        self.angular_velocity_cmd = kwargs['angular_velocity_cmd']
        self.linear_velocity = kwargs['linear_velocity']
        self.angular_velocity = kwargs['angular_velocity']
        self.dbw_enabled = kwargs['dbw_enabled']

        brake = 0.
        throttle = 0.
        #rospy.loginfo(self.linear_velocity_cmd)

        if (self.dbw_enabled is True) and (self.linear_velocity_cmd is not None) and (self.linear_velocity is not None):

            # "Gain scheduling" longitudinal controller
            if self.linear_velocity_cmd > self.linear_velocity:
                accel_cmd = min([4 * (self.linear_velocity_cmd-self.linear_velocity+0.1) / self.linear_velocity_cmd,
                                 self.max_throttle_perc
                                ])
            # if linear_velocity_cmd is below or equal to actual velocity but the car is still moving
            elif self.linear_velocity > 0.1:
                accel_cmd = max([4 * (self.linear_velocity_cmd-self.linear_velocity-0.1) / self.linear_velocity,
                                 self.max_brake_perc
                                ])
            else:
                # hold the brake when the car is standstill
                accel_cmd = -0.01

            steer_cmd = self.yaw_controller.get_steering(self.linear_velocity_cmd, self.angular_velocity_cmd, self.linear_velocity)
        else:
            accel_cmd = 0.0
            steer_cmd = 0.0

        accel_cmd = self.lowpassfilter_accel_cmd.filt(accel_cmd)
        steer_cmd = self.lowpassfilter_steer_cmd.filt(steer_cmd)

        # commands limitation
        accel_cmd = max(self.max_brake_perc, min(accel_cmd, self.max_throttle_perc))
        steer_cmd = max(-self.max_steer_angle, min(steer_cmd, self.max_steer_angle))

        if accel_cmd < 0.0:
            brake = -accel_cmd * BRAKE_TORQUE_MAX
            throttle = 0.0
        else:
            brake = 0.0
            throttle = accel_cmd

        if self.dbw_enabled is True:
            rospy.loginfo("throttle: "+str(throttle)+"\tbrake: "+str(brake)+"\tsteer: "+str(steer_cmd))

        return throttle, brake, steer_cmd
