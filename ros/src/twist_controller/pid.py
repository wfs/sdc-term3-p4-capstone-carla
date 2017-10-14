MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    """
    CONTROL SUBSYSTEM

    *** STEP 5 ***

    Calculates the appropriate steering angle adjustment to smoothly return to the reference trajectory.

    After manually fine-tuning in the Simulator, the PID parameters were kp=0.5, ki=0.004, kd=0.25.

    See Self-Driving Car Engineer ND - Term 2, Lesson 16 PID Implementation Solution
    """
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
