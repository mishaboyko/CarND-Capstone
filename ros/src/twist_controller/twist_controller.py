import rospy

from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # Initialize Proportional Integral Differential factors.
        # Source: https://github.com/mishaboyko/CarND-PID-Control-Project/blob/master/src/main.cpp
        tau_p = 0.2
        tau_i = 0.00004
        tau_d = 3.0
        self.pid = PID(tau_p, tau_i, tau_d)

        rospy.loginfo("Twist Controller successfully initialized")

    def control(self, target_linear_velocity, current_linear_velocity, dbw_state):
        # Return throttle, brake, steer
        rospy.loginfo("Controller received target_linear_velocity: %s, current_linear_velocity: %s, dbw_state: %s",
        target_linear_velocity, current_linear_velocity, dbw_state)
        return 0.33, 0., 0.
