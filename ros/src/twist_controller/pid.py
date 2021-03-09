import rospy

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        # Proportional coefficient
        self.kp = kp
        # Integral coefficient
        self.ki = ki
        # Differential coefficient
        self.kd = kd
        self.min = mn
        self.max = mx
        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):
        # error is an Cross-Track-Error
        # sample_time is a time diff between two measurements

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        # val is a steering angle ?!
        val = self.kp * error + self.ki * integral + self.kd * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
