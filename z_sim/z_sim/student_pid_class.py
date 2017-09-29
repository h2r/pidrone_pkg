'''
CS1951R - Introduction to Robotics
Brown University
Fall 2017

pid_class.py
'''

import numpy as np

###############################################################################
# This generic PID class will be used to control throttle in project 2, but in
# future projects may be applied to control other axes on the drone. DO NOT
# change the the function specifications. You will implement the following:
###############################################################################
# __init__()
#   This function get's called once when your PID class is instantiated. Use
#   it to initialize any variables you think you need in your PID calculations
#
# step()
#   This function will get called at each sensor update, and should return 
#   a command output. This is where your actual PID calculation will occur.
#   - e:        the error at time t. for example if the drone was 10cm below
#               the setpoint, e would be 0.1
#   - t:        The time (in seconds) at which the process variable was 
#               measured
#   - return:   Your output should be a value between 1100 and 1900. This is
#               a PWM command, which will be sent to the SkyLine's throttle
#               channel.
#
# reset()
#   This function will be called each time before takeoff. Use this space
#   to do any necessary preparation before step get's called. (HINT: what 
#   happens if a lot of time elapses between calls to step? What happens
#   if start flying with a nonzero integral term?)

###############################################################################
# YOUR CODE BELOW THIS LINE
###############################################################################

class student_PID():

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.midpoint   = 1300
        self.max        = 1900
        self.min        = 1100      

        self.reset()


    def step(self, e, t):
        dt = t - self.tt
        e *= -1
        self.p = self.kp * e
        self.i += self.ki * e * dt
        self.d = self.kd * (e - self.ee) / dt 
        pwm = max(self.p,0) + self.i + self.d + self.midpoint
        print pwm, self.p, self.i, self.d
        self.tt = t
        self.ee = e

        if self.re:
            self.re = False
            return 1100
        else: 
            return min(self.max, max(self.min, pwm))

    def reset(self):
        self.p = 0
        self.i = 0
        self.d = 0
        self.tt = 0 # old timestamp
        self.ee = 0 # old error
        self.re = True

