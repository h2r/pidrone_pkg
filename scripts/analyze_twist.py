import cv2
import time
import rospy
import numpy as np
import picamera.array
from h2rMultiWii import MultiWii
from pidrone_pkg.msg import State
from geometry_msgs.msg import TwistStamped


# RASPBERRY PI
camera_matrix = np.array([[ 253.70549591,    0.,          162.39457585],
                        [ 0.,          251.25243215,  126.5400089],
                        [   0.,            0., 1.        ]])
dist_coeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,
                        0.27998478])


class AnalyzeTwist(picamera.array.PiMotionAnalysis):
    ''' A class used for real-time motion analysis of optical flow vectors to
    obtain the planar and yaw velocities of the drone.

    For more information, read the following:
    http://picamera.readthedocs.io/en/release-1.10/api_array.html#picamera.array.PiMotionAnalysis
    '''

    def setup(self, camera_wh):
        ''' Initialize the instance variables '''

        # Initialize the current twist data
        self.twist_stamped = TwistStamped()
        self.twist_stamped.header.stamp = rospy.Time.now()
        self.twist_stamped.twist.linear.x = 0
        self.twist_stamped.twist.linear.y = 0
        self.twist_stamped.twist.linear.z = 0
        self.twist_stamped.twist.angular.x = 0
        self.twist_stamped.twist.angular.y = 0
        self.twist_stamped.twist.angular.z = 0

        # Initialize angular velocity variables:
        self.ang_t = 0          # current time
        self.ang_dt = 0         # change in time
        self.delta_angvx = 0    # change in angx (roll) velocity
        self.delta_angvy = 0    # change in angy (pitch) velocity

        # set the amount that roll rate factors in
        self.ang_coefficient = 1.0

        # flow variables
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.norm_flow_to_m = self.flow_scale # the conversion from flow units to m
        self.flow_coeff = self.norm_flow_to_m/self.max_flow

        # ROS setup
        ###########
        # Publisher:
        self.twistpub = rospy.Publisher('/pidrone/picamera/twist', TwistStamped, queue_size=1)

    def analyse(self, a):
        ''' Analyze the frame, calculate the motion vectors, and publish the
        TwistStamped message. This is implicitly called by the

        a : an array of the incoming motion data that is provided by the
            PiMotionAnalysis api
        '''
        # signed 1-byte values
        x = a['x']
        y = a['y']

        # calculate the planar and yaw motions (multiply by 100 for cm to m conversion)
        x_motion = 100 * np.sum(x) * self.flow_coeff + np.arctan(self.delta_angvx * self.ang_dt) * self.ang_coefficient
        y_motion = 100 * np.sum(y) * self.flow_coeff + np.arctan(self.delta_angvy * self.ang_dt) * self.ang_coefficient
        self.twist_msg = TwistStamped()
        self.twist_msg.header.stamp = rospy.Time.now()
        self.twist_msg.twist.linear.x = self.near_zero(x_motion)
        self.twist_msg.twist.linear.y = - self.near_zero(y_motion)
        # self.twist_msg.twist.linear.z = self.near_zero(z_motion)
        # self.twist_msg.twist.angular.z = self.near_zero(yaw_motion)

        # Update and publish the twist message
        self.twistpub.publish(self.twist_msg)

    def near_zero(self, n):
        return 0 if abs(n) < 0.001 else n

    def state_callback(self, state):
        self.angdt = state.header.stamp - self.state.header.stamp
        angular_velocities = state.twist_with_covariance.twist.angular
        self.delta_angvx = angular_velocities.x - self.twist_stamped.twist.angular.x
        self.delta_angvy = angular_velocities.y - self.twist_stamped.twist.angular.y
        self.twist_stamped.twist.angular = angular_velocities
        time = rospy.get_time()
        self.ang_dt = 0 if self.ang_t is None else time - self.ang_t
        self.ang_t = time
