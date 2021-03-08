import rospy

from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, decel_limit, accel_limit):
        # TODO: Implement
        # Initialize Proportional Integral Differential factors.
        # Source: https://github.com/mishaboyko/CarND-PID-Control-Project/blob/master/src/main.cpp
        #tau_p = 0.2
        #tau_i = 0.00004
        #tau_d = 3.0
        # steer_pid koefficients
        #tau_p = 0.15
        #tau_i = 0.001
        #tau_d = 0.1
        # linear_pid koefficients
        tau_p = 0.8
        tau_i = 0.0
        tau_d = 0.05

        self.steer_pid = PID(tau_p, tau_i, tau_d, -max_steer_angle, max_steer_angle)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0, max_lat_accel, max_steer_angle)
        self.linear_speed_pid = PID(tau_p, tau_i, tau_d, decel_limit, 0.7 * accel_limit)


    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity, cte, delta_t):
        # Return throttle, brake, steer
        throttle = self.linear_speed_pid.step(target_linear_velocity - current_linear_velocity, delta_t)

        # Improve steer_pid?
        #steer_pid = self.steer_pid.step(cte, delta_t)
        steer_yaw = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        steer = steer_yaw
        return throttle, 0., steer
