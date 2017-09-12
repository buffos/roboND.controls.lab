# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)
        self.max_windup_ = float(max_windup)
	self.alpha = 0.2

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.target_ = 0.0
        self.error_sum_ = 0.0
        self.last_error_ = 0.0


    def reset(self):
        self.target_ = 0.0
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0

        self.error_sum_ = 0.0
        self.last_timestamp_ = 0.0
        self.last_error_ = 0


    def setTarget(self, target):
        self.target_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)

    def setKI(self, ki):
        self.ki_ = float(ki)

    def setKD(self, kd):
        self.kd_ = float(kd)

    def setMaxWindup(self, max_windup):
        self.max_windup_ = float(max_windup)

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            return 0

        # Calculate the error
        error = self.target_ - measured_value

        # Set the last_timestamp_
        self.last_timestamp_ = timestamp

        # Sum the errors
        self.error_sum_ += error * delta_time

        # Find delta_error
        delta_error = error - self.last_error_

        # Update the past error
        self.last_error_ = error

        # Address max windup
        ########################################
        if self.error_sum_ > self.max_windup_:
            self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.max_windup_:
            self.error_sum_ = -self.max_windup_

        ########################################

        # Proportional error
        p = self.kp_ * error

        # Integral error
        i = self.ki_ * self.error_sum_

        # derivative error
	d = self.kd_ * (self.alpha * delta_error / delta_time + (1 - self.alpha)  * self.last_error_)
        
        # Set the control effort
        u = p + i + d

        return u
