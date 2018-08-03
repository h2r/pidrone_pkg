import cv2
import time
import rospy
import numpy as np
import picamera.array
from pidrone_pkg.msg import Flow
from h2rMultiWii import MultiWii
from geometry_msgs.msg import TwistWithCovarianceStamped


# RASPBERRY PI?
camera_matrix = np.array([[ 253.70549591,    0.,          162.39457585],
                        [ 0.,          251.25243215,  126.5400089],
                        [   0.,            0., 1.        ]])
dist_coeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,
                        0.27998478])


class AnalyzeFlow(picamera.array.PiMotionAnalysis):
    ''' A class used for real-time motion analysis to obtain the planar and yaw
    velocities of the drone.

    For more information, read the following:
    http://picamera.readthedocs.io/en/release-1.10/api_array.html#picamera.array.PiMotionAnalysis
    '''

    def analyse(self, a):
        ''' Analyze the frame, calculate the motion vectors, and publish the
        twist message. This is implicitly called by the

        a : an array of the incoming motion data that is provided by the
            PiMotionAnalysis api
        '''
        start = time.time()
        # signed 1-byte values
        x = a['x']
        y = a['y']
        # an unsigned 2-byte value representing the sum of absolute differences
        # of the block.
        sad = a['sad']

        # store the time variables
        curr_time = time.time()
        delta_time = curr_time - self.prev_time
        self.prev_time = curr_time

        # calculate the planar and yaw motions
        self.x_motion = 0 - np.sum(x) * self.flow_coeff + np.arctan(self.ang_vx * delta_time) * self.ang_coefficient
        self.y_motion = np.sum(y) * self.flow_coeff + np.arctan(self.ang_vy * delta_time) * self.ang_coefficient
        self.z_motion = np.sum(np.multiply(x, self.z_filter_x)) + \
                np.sum(np.multiply(y, self.z_filter_y))
        # yaw is negative if drone rotated to right
        self.yaw_motion = np.sum(np.multiply(x, self.yaw_filter_x)) + \
                np.sum(np.multiply(y, self.yaw_filter_y))

# TODO smooth or not
        # Update and publish the Flow msg
        self.flow_msg.header.stamp = rospy.get_rostime()
        self.flow_msg.x_velocity = self.x_motion
        self.flow_msg.y_velocity = self.y_motion
        self.flow_msg.z_velocity = self.z_motion
        self.flow_msg.yaw_velocity = self.yaw_motion
        self.flowpub.publish(self.flow_msg)

    def get_z_filter(self, (width, height)):
        ''' Compute a divergence filter to estimate z component of flow '''
        assert width%16 == 0 and height%16 == 0
        num_rows = height/16
        num_cols = width/16
        mid_row = (num_rows - 1)/2.0
        mid_col = (num_cols - 1)/2.0
        # the picamera motion vectors have a buffer column each frame
        self.z_filter_x = np.zeros((num_rows,num_cols+1), dtype='float')
        self.z_filter_y = np.zeros((num_rows,num_cols+1), dtype='float')
        for i in range(num_cols):
            for j in range(num_rows):
                x = i - mid_col # center the coords around the middle
                y = j - mid_row
                self.z_filter_x[j,i] = x
                self.z_filter_y[j,i] = y
                # r = np.sqrt(x**2 + y**2)
                # theta = np.arctan(y/x)
                # print x,y,theta
                # z_filter_x[j,i] = r * np.cos(theta)
                # z_filter_y[j,i] = r * np.sin(theta)
        self.z_filter_x /= np.linalg.norm(self.z_filter_x)
        self.z_filter_y /= np.linalg.norm(self.z_filter_y)

    def get_yaw_filter(self, (width, height)):
        self.yaw_filter_x = self.z_filter_y
        self.yaw_filter_y = -1 * self.z_filter_x

    def imu_callback(self, msg):
        ''' Calculate angle compensation values to account for the tilt of the drone '''
        # Convert the quaternion to euler angles
        # store the orientation quaternion
        oq = msg.orientation
        orientation_list = [oq.x, oq.y, oq.z, oq.w]
        (new_angx, new_angy, new_angz) = tf.transformations.euler_from_quaternion(orientation_list)
        curr_time = msg.header.time
        # if previous data has been stored, update ang_vx and ang_vy
        if self.prev_imu_time != 0:
            self.ang_vx = (new_angx - self.prev_angx)/(curr_time - self.imu_time)
            self.ang_vy = (new_angy - self.prev_angy)/(curr_time - self.imu_time)

            self.prev_angx = new_angx
            self.prev_angy = new_angy
            self.imu_time = curr_time
        # if previous data has not been stored, just store the data
        else:
            self.prev_angx = new_angx
            self.prev_angy = new_angy
            self.imu_time = curr_time

    def setup(self, camera_wh):
        ''' Initialize the instance variables '''
        self.get_z_filter(camera_wh)
        self.get_yaw_filter(camera_wh)
        # store the angular velocities and variables to calculate these velocities
        self.ang_vx = 0
        self.ang_vy = 0
        self.prev_angx = 0
        self.prev_angy = 0
        self.prev_imu_time = 0
        self.prev_time = time.time()
        # set the amount that roll rate factors in
        self.ang_coefficient = 1.0
        # initialize the planar and yaw velocities to zero
        self.x_motion = 0
        self.y_motion = 0
        self.z_motion = 0
        self.yaw_motion = 0
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = 16.5
        self.norm_flow_to_cm = self.flow_scale # the conversion from flow units to cm
        self.flow_coeff = self.norm_flow_to_cm/self.max_flow
        # initialize the publisher for the Flow message
        self.flowpub = rospy.Publisher('/picamera/flow', Flow, queue_size=1)
        # Initialize the Flow message
        self.flow_msg = Flow()
        # set the header frame_id
        self.flow_msg.header.frame_id = 'Body'
