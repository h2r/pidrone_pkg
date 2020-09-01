#!/usr/bin/env python
from __future__ import division
import rospy


#####################################################
#						PID							#
#####################################################
class PIDaxis():
    def __init__(self, kp, ki, kd, i_range=None, d_range=None, control_range=(1000, 2000), midpoint=1500, smoothing=True):
        # Tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Config
        self.i_range = i_range
        self.d_range = d_range
        self.control_range = control_range
        self.midpoint = midpoint
        self.smoothing = smoothing
        # Internal
        self.reset()

    def reset(self):
        self._old_err = None
        self._p = 0
        self._i = 0
        self._d = 0
        self._dd = 0
        self._ddd = 0

    def step(self, err, time_elapsed):
        if self._old_err is None:
            # First time around prevent d term spike
            self._old_err = err

        # Find the p component
        self._p = err * self.kp

        # Find the i component
        self._i += err * self.ki * time_elapsed
        if self.i_range is not None:
            self._i = max(self.i_range[0], min(self._i, self.i_range[1]))

        # Find the d component
        self._d = (err - self._old_err) * self.kd / time_elapsed
        if self.d_range is not None:
            self._d = max(self.d_range[0], min(self._d, self.d_range[1]))
        self._old_err = err

        # Smooth over the last three d terms
        if self.smoothing:
            self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0)/15.0
            self._ddd = self._dd
            self._dd = self._d

        # Calculate control output
        raw_output = self._p + self._i + self._d
        output = min(max(raw_output + self.midpoint, self.control_range[0]), self.control_range[1])

        return output


class PID:

    height_factor = 1.238
    battery_factor = 0.75

    def __init__(self, period,
                 roll=PIDaxis(300.0, 175.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 pitch=PIDaxis(300.0, 175.0, 0.0, control_range=(1400, 1600), midpoint=1520, i_range=(-100, 100)),
                 yaw=PIDaxis(0.0, 0.0, 0.0),
                 throttle=PIDaxis(100.0, 20.0, 50.0, i_range=(-400, 400), control_range=(1200, 2000), d_range=(-40, 40), midpoint=1460)
                 ):

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.throttle = throttle

        self.period = period

        # Tuning values specific to each drone
        self.reset()

    def reset(self):
        """ Reset each pid """
        # reset individual PIDs
        self.roll.reset()
        self.pitch.reset()
        self.throttle.reset()

    def step(self, velocity_error_x, velocity_error_y, velocity_error_yaw, altitude_error):
        """ Compute the control variables from the error using the step methods
        of each axis pid.
        """
        # Compute roll command
        cmd_r = self.roll.step(velocity_error_x, self.period)
        # Compute pitch command
        cmd_p = self.pitch.step(velocity_error_y, self.period)
        # Compute yaw command
        cmd_y = 1500 + velocity_error_yaw
        # Compute throttle command
        cmd_t = self.throttle.step(altitude_error, self.period)

        return [cmd_r, cmd_p, cmd_y, cmd_t]
