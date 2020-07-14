#!/usr/bin/env python
from __future__ import division
import rospy
import rospkg
import yaml
import tf


class PIDaxis():
    def __init__(self, kp, ki, kd, k, midpoint, i_range=None, d_range=None, control_range=(1000, 2000)):
        # TODO: add smoothing?
        # Tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k = k
        self.midpoint = midpoint
        # saturation values
        self.i_range = i_range
        self.d_range = d_range
        self.control_range = control_range
        # Internal
        self.reset()

    def reset(self):
        self._p = 0
        self._i = 0
        self._d = 0

    def step(self, error, derivative, elapsed_time):
        """
        Step the pid controller

        :param error: the differnece between the setpoint and the control variable
        :param derivative: the derivative of the control variable
        :param elapsed_time: the elapsed time since the previous call to the step method
        :return: a float value within the control range
        """

        # Find the p component
        self._p = self.kp * error

        # Find the i component
        self._i += self.ki * error * elapsed_time
        if self.i_range is not None:
            self._i = max(self.i_range[0], min(self._i, self.i_range[1]))

        # Find the d component
        self._d = self.kd * derivative
        if self.d_range is not None:
            self._d = max(self.d_range[0], min(self._d, self.d_range[1]))

        # Calculate control output
        raw_output = self._p + self._i - self._d + self.midpoint + self.k
        output = min(max(raw_output, self.control_range[0]), self.control_range[1])

        return output

    def increment_k(self):
        self.k += 1

    def decrement_k(self):
        self.k -= 1

    def get_k(self):
        return self.k


class CascadedPID():
    def __init__(self, v_kp, v_ki, v_kd, v_k, p_kp, p_ki, p_kd, Ts, frequency_divisor):
        self.velocity_pid = PIDaxis(kp=v_kp, ki=v_ki, kd=v_kd, k=v_k, midpoint=1500,
                                    i_range=(-50, 50), d_range=(-10, 10), control_range=(1400, 1600))
        self.position_pid = PIDaxis(kp=p_kp, ki=p_ki, kd=p_kd, k=0, midpoint=0,
                                    i_range=(-5, 5), d_range=(-5, 5), control_range=(-10, 10))
        self.frequency_divisor = frequency_divisor
        self.v_Ts = Ts
        self.p_Ts = Ts * frequency_divisor
        self.setpoint_velocity_from_position_error = 0.0
        self.counter = 0

    def reset(self):
        self.velocity_pid.reset()
        self.position_pid.reset()
        self.setpoint_velocity_from_position_error = 0.0

    def step(self, setpoint_position, setpoint_velocity, current_position,
             current_velocity, current_acceleration):
        if (self.counter % self.frequency_divisor) == 0:
            position_error = setpoint_position - current_position
            self.setpoint_velocity_from_position_error = self.position_pid.step(error=position_error,
                                                                                derivative=current_velocity,
                                                                                elapsed_time=self.v_Ts)
            self.counter = 0

        velocity_error = setpoint_velocity - current_velocity + self.setpoint_velocity_from_position_error
        velocity_cmd = self.velocity_pid.step(error=velocity_error,
                                              derivative=current_acceleration,
                                              elapsed_time=self.p_Ts)
        self.counter += 1
        return velocity_cmd


class DronePID:

    def __init__(self, velocity_controller_frequency, frequency_divisor):
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/pid.yaml" % path) as f:
            tuning_vals = yaml.load(f)
        # roll pid
        self.Ts = 1.0 / velocity_controller_frequency
        self.roll_pid = CascadedPID(v_kp=tuning_vals["roll"]["velocity"]["kp"],
                                    v_ki=tuning_vals["roll"]["velocity"]["ki"],
                                    v_kd=tuning_vals["roll"]["velocity"]["kd"],
                                    v_k=tuning_vals["roll"]["velocity"]["k"],
                                    p_kp=tuning_vals["roll"]["position"]["kp"],
                                    p_ki=tuning_vals["roll"]["position"]["ki"],
                                    p_kd=tuning_vals["roll"]["position"]["kd"],
                                    Ts=self.Ts,
                                    frequency_divisor=frequency_divisor)
        self.pitch_pid = CascadedPID(v_kp=tuning_vals["pitch"]["velocity"]["kp"],
                                     v_ki=tuning_vals["pitch"]["velocity"]["ki"],
                                     v_kd=tuning_vals["pitch"]["velocity"]["kd"],
                                     v_k=tuning_vals["pitch"]["velocity"]["k"],
                                     p_kp=tuning_vals["pitch"]["position"]["kp"],
                                     p_ki=tuning_vals["pitch"]["position"]["ki"],
                                     p_kd=tuning_vals["pitch"]["position"]["kd"],
                                     Ts=self.Ts,
                                     frequency_divisor=frequency_divisor)
        self.yaw_pid = CascadedPID(v_kp=tuning_vals["yaw"]["velocity"]["kp"],
                                   v_ki=tuning_vals["yaw"]["velocity"]["ki"],
                                   v_kd=tuning_vals["yaw"]["velocity"]["kd"],
                                   v_k=tuning_vals["yaw"]["velocity"]["k"],
                                   p_kp=tuning_vals["yaw"]["position"]["kp"],
                                   p_ki=tuning_vals["yaw"]["position"]["ki"],
                                   p_kd=tuning_vals["yaw"]["position"]["kd"],
                                   Ts=self.Ts,
                                   frequency_divisor=frequency_divisor)

        self.throttle_pid = PIDaxis(kp=tuning_vals["throttle"]["kp"],
                                    ki=tuning_vals["throttle"]["ki"],
                                    kd=tuning_vals["throttle"]["kd"],
                                    k=tuning_vals["throttle"]["k"],
                                    midpoint=1450,
                                    i_range=(-200, 200),
                                    d_range=(-40, 40),
                                    control_range=(1200, 2000))

    def reset(self):
        """ Reset each pid """
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.throttle_pid.reset()

    def get_yaw_from_pose(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def step(self, setpoint_twist, setpoint_pose, state, imu):
        """

        :param state: the current state of the drone
        :param setpoint_twist: a Twist message containing the setpoint velocities
        :param setpoint_pose: a Pose message containing the setpoint positions
        :return: a list of floats [roll, pitch, yaw, throttle] that represent the control output
        """
        current_pose = state.pose_with_covariance.pose
        current_twist = state.twist_with_covariance.twist

        # Compute roll command
        ######################
        cmd_r = self.roll_pid.step(setpoint_position=setpoint_pose.position.x,
                                   setpoint_velocity=setpoint_twist.linear.x,
                                   current_position=current_pose.position.x,
                                   current_velocity=current_twist.linear.x,
                                   current_acceleration=imu.linear_acceleration.x)
        # Compute pitch command
        #######################
        cmd_p = self.pitch_pid.step(setpoint_position=setpoint_pose.position.y,
                                    setpoint_velocity=setpoint_twist.linear.y,
                                    current_position=current_pose.position.y,
                                    current_velocity=current_twist.linear.y,
                                    current_acceleration=imu.linear_acceleration.y)

        # Compute yaw command
        cmd_y = self.yaw_pid.step(setpoint_position=self.get_yaw_from_pose(setpoint_pose),
                                  setpoint_velocity=setpoint_twist.angular.z,
                                  current_position=self.get_yaw_from_pose(current_pose),
                                  current_velocity=current_twist.angular.z,
                                  current_acceleration=0.0)

        # Compute throttle command
        cmd_t = self.throttle_pid.step(error=(setpoint_pose.position.z - current_pose.position.z),
                                       derivative=current_twist.linear.z,
                                       elapsed_time=self.Ts)
        return [cmd_r, cmd_p, cmd_y, cmd_t]
