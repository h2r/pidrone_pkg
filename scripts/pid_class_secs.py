#!/usr/bin/env python
from __future__ import division
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped
import time
import tf
import math
import numpy as np
from copy import deepcopy
import sys


#####################################################
#						PID							#
#####################################################
class PIDaxis():
    def __init__(self, kp, ki, kd, kp_upper = None, i_cap = None, d_cap = None, control_range = (1000,2000), midpoint = 1500, smoothing = True):
        # tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # config
        self.kp_upper = kp_upper
        self.i_cap = i_cap
        self.d_cap = d_cap
        self.control_range = control_range
        self.midpoint = midpoint
        self.smoothing = True
        # internal
        self._old_err = None
        self._p = 0
        self._i = 0
        self._d = 0
        self._dd = 0
        self._ddd = 0
        self._t = None


    def step(self, err):
        if self._t is None: time_elapsed = 0.001 # first time around prevent time spike
        else: time_elapsed = time.time() - self._t
        self._t = time.time()
        if self._old_err is None: self._old_err = err # first time around prevent d term spike	

        # find the p,i,d components
        if self.kp_upper is not None and err < 0:
            self._p = err * self.kp_upper
        else: 
            self._p = err * self.kp

        self._i += err * self.ki * time_elapsed
        if self.i_cap is not None and np.absolute(self._i) > self.i_cap:
            self._i = np.sign(self._i) * self.i_cap

        self._d = (err - self._old_err) * self.kd / time_elapsed
        if self.d_cap is not None and np.absolute(self._d) > self.d_cap:
            self._d = np.sign(self._d) * self.d_cap
        self._old_err = err

        # smooth over the last three d terms
        if self.smoothing:
            self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0)/15.0
            self._ddd = self._dd
            self._dd = self._d

        raw_output = self._p + self._i + self._d
        output = min(max(raw_output + self.midpoint, self.control_range[0]),
                self.control_range[1])

        return output

class PID:
    def __init__(self, 
        roll = PIDaxis(1.1, 0.25, 0.250),
        pitch = PIDaxis(1.1, 0.25, 0.250),
        yaw = PIDaxis(500 * 0, 0,0),
        throttle = PIDaxis(10.0, 40 * 0, 0.7, kp_upper=0, d_cap=300, midpoint =
            1200)):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.throttle = throttle
        self.sp = None
    
    def update_setpoint(self, data):
        self.sp = data

    def step(self, pos):
        if self.sp is None:
            return [1500, 1500, 1500, 1000]
        else:
            err = self.calc_err(pos)
            cmd_r = self.roll.step(err[0])
            cmd_p = self.pitch.step(err[1])
            cmd_y = self.yaw.step(err[2])
            cmd_t = self.throttle.step(err[3])

            return [cmd_r, cmd_p, cmd_y, cmd_t]

    def quat_to_rpy(self, q):
        """ takes in a quaternion (like from a pose message) and returns (roll, pitch, yaw) """
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def calc_err(self, pos):
        """ given a position and a set point (in global coordinates), this calculates the error
        in the drone's coordinate system (r,p,y,t) """
        _, _, pos_yaw = self.quat_to_rpy(pos.pose.orientation)
        _, _, sp_yaw  = self.quat_to_rpy(self.sp.pose.orientation)
        
        err_x = self.sp.pose.position.x - pos.pose.position.x
        err_y = self.sp.pose.position.y - pos.pose.position.y
        err_z = self.sp.pose.position.z - pos.pose.position.z
        err_yaw = sp_yaw - pos_yaw

        sp_angle = -np.arctan2(err_x, err_y) # the angle of the drone's pos relative to the setpoint
        sp_norm = np.sqrt(err_x**2 + err_y**2) # the distance from the drone to the setpoint (in the plane) 
        diff_angle = pos_yaw - sp_angle # the difference between the drone's yaw and its sp_angle

        err_fb = np.cos(diff_angle) * sp_norm
        err_lr = np.sin(diff_angle) * sp_norm
        return (err_lr, err_fb, err_yaw, err_z)

