'''
CS1951R - Introduction to Robotics
Brown University
Fall 2017

student_pid_class.py
'''

import numpy as np
import rospy

###############################################################################
# This generic PID class will be used to control throttle in project 2, but in
# future projects may be applied to control other axes on the drone. DO NOT
# change the the function specifications. You will implement the following:
###############################################################################
# __init__()
#   This function get's called once when your PID class is instantiated. Use
#   it to initialize any variables you think you need in your PID calculations.
#   PID terms will be loaded in from scripts/z_pid.yaml
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
#   if we start flying with a nonzero integral term?)

###############################################################################
# YOUR CODE BELOW THIS LINE
###############################################################################
class student_PID():

    def __init__(self, params):
        self.kp = params["kp"]
        self.ki = params["kp"]
        self.kd = params["kp"]
        self.k = params["k"]
        self.min = params['min_pwm']
        self.max = params['max_pwm']

        pass

    def step(self, e, t):
        dt = t - self.prev_t
        self.prev_t = t

        de = (e - self.old_e)/dt
        self.old_e = e

        self.int_e += e * dt

        self.p = self.kp * e
        self.i = self.ki * self.int_e
        self.d = self.kd * de
        print self.i
        u = self.p + self.i + self.d + self.k

        return max(self.min, min(self.max, u))

    def reset(self):
        self.prev_t = rospy.get_time()
        self.int_e = 0
        self.old_e = 0

#class student_PID():
#
#    def __init__(self, params):
#    	self.kp = params["kp"]
#    	self.ki = params["kp"]
#    	self.kd = params["kp"]
#    	self.k = params["k"]
#        self.max_pwm = params["max_pwm"]
#        self.min_pwm= params["min_pwm"]
#        
#        pass
#
#    def step(self, e, t):
#        return self.k
#
#    def reset(self):
#        pass
#
