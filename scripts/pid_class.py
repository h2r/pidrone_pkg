#!/usr/bin/env python
from __future__ import division
import rospy
from pidrone_pkg.msg import RC, ERR, axes_err
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
    def __init__(self, kp, ki, kd, kp_upper = None, i_range = None, d_range = None, control_range = (1000,2000), midpoint = 1500, smoothing = True):
        # tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # config
        self.kp_upper = kp_upper
        self.i_range = i_range
        self.d_range = d_range
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


    def step(self, err, time_elapsed, error = None):
        if self._old_err is None: self._old_err = err # first time around prevent d term spike	
        # find the p,i,d components
        if self.kp_upper is not None and err < 0:
            self._p = err * self.kp_upper
        else: 
            self._p = err * self.kp

        self._i += err * self.ki * time_elapsed
        if self.i_range is not None:
            self._i = max(self.i_range[0], min(self._i, self.i_range[1]))

        self._d = (err - self._old_err) * self.kd / time_elapsed
        if self.d_range is not None:
            self._d = max(self.d_range[0], min(self._d, self.d_range[1]))
        self._old_err = err
 
        # smooth over the last three d terms
        if self.smoothing:
            self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0)/15.0
            self._ddd = self._dd
            self._dd = self._d

        if error is not None:
            error.p = self._p
            error.i = self._i
            error.d = self._d

        raw_output = self._p + self._i + self._d
        output = min(max(raw_output + self.midpoint, self.control_range[0]),
                self.control_range[1])

        return output

class PID:
    def __init__(self, 
        roll = PIDaxis(1.0, 0, 0.0),
        pitch = PIDaxis(1.0, 0, 0.0),
        yaw = PIDaxis(0.0, 0.0, 0.0),
        throttle = PIDaxis(7.5, 4.0, 2.0, kp_upper = 0, i_range=(0, 400),\
            control_range=(1150,2000), d_range=(-400, 400), midpoint =
            1200), smoothing=False):
        # roll = PIDaxis(1.2, 05, 1.2),
        # pitch = PIDaxis(1.2, 0.5, 1.2),
        # yaw = PIDaxis(-1000.0, 0,0),
        # throttle = PIDaxis(7.5, 4.0, 2.0, kp_upper = 0, i_range=(0, 400),\
        #     control_range=(1150,2000), d_range=(-400, 400), midpoint =
        #     1200), smoothing=False):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.throttle = throttle
        self.sp = None
        self._t = None
    
    def update_setpoint(self, data):
        self.sp = data

    def step(self, error):
        if self._t is None: time_elapsed = 1 # first time around prevent time spike
        else: time_elapsed = rospy.get_time() - self._t
        self._t = rospy.get_time()
        cmd_r = self.roll.step(error.x.err, time_elapsed, error.x)
        cmd_p = self.pitch.step(error.y.err, time_elapsed, error.y)
        cmd_y = 0
        cmd_t = self.throttle.step(error.z.err, time_elapsed, error.z)

        return [cmd_r, cmd_p, cmd_y, cmd_t]

    # def step(self, pos, error):
    #     if self._t is None: time_elapsed = 1 # first time around prevent time spike
    #     else: time_elapsed = time.time() - self._t
    #     self._t = time.time()
    #     if self.sp is None:
    #         return [1500, 1500, 1500, 1000]
    #     else:
    #         cmd_r = self.roll.step(error[0], time_elapsed, error.x)
    #         cmd_p = self.pitch.step(error[1], time_elapsed, error.y)
    #         # cmd_y = self.yaw.step(err[2], time_elapsed, None)
    #         cmd_y = 0
    #         cmd_t = self.throttle.step(error[3], time_elapsed, error.z)
    #         # err = self.calc_err(pos)
    #         # error.x.err = err[0]
    #         # error.y.err = err[1]
    #         # error.z.err = err[3]
    #         # cmd_r = self.roll.step(err[0], time_elapsed, error.x)
    #         # cmd_p = self.pitch.step(err[1], time_elapsed, error.y)
    #         # cmd_y = self.yaw.step(err[2], time_elapsed, None)
    #         # cmd_t = self.throttle.step(err[3], time_elapsed, error.z)

    #         return [cmd_r, cmd_p, cmd_y, cmd_t]
    
    def get_roll_matrix(self, data):
        y = data['heading']/180.0*np.pi
        r = data['angx']/180.0*np.pi
        p = data['angy']/180.0*np.pi
        q = np.array(tf.transformations.quaternion_from_euler(-p, r, -y))
        return Quaternion(q).rotation_matrix


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

