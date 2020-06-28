#!/usr/bin/env python

import tf
import sys
import os
import rospy
import signal
import numpy as np
from pidrone_pkg.msg import State
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header, Bool, Empty


class EMAStateEstimator(object):
    ''' A class that subscribes to data from the picamera that is published by
    the vision node and filters the data using an estimated moving average

    Publisher:
    /pidrone/state

    Subscribers:
    /pidrone/infrared
    /pidrone/imu
    /pidrone/picamera/pose
    /pidrone/picamera/twist
    '''

    def __init__(self):
        ''' A constructor for EMAStateEstimator
        '''
        # Initialize the State:
        #######################
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'Body'

        self.state_topic_str = '/pidrone/state/ema'

        self.state = State()
        self.state.header = header

        # store whether twist and range data have been received
        # (pose is not necessary to fly, only to fly with position control)
        self.received_twist_data = False
        self.received_range_data = False

        # angle compensation values (to account for tilt of drone)
        self.mw_angle_comp_x = 0
        self.mw_angle_comp_y = 0
        self.mw_angle_coeff = 0.2


    # ROS Subscriber Callback Methods:
    ##################################
    def twist_callback(self, data):
        """ Update the twist of the drone
        Does not alter the angular velocities because these are set by imu
        """
        # update the header stamp
        self.state.header.stamp = data.header.stamp
        # update linear twist data
        # TODO TEST
        self.filter_twist(data.twist)
        # update that data has been recieved
        self.received_twist_data = True

    def reset_callback(self, empty):
        """ Reset the current pose of the drone except for the z postion """
        self.state.header.stamp = rospy.Time.now()
        print 'Resetting position in x and y and orientation'
        self.state.pose_with_covariance.pose.position.x = 0
        self.state.pose_with_covariance.pose.position.y = 0
        # reset the orientation
        self.state.pose_with_covariance.pose.orientation = PoseStamped().pose.orientation

    def pose_callback(self, data):
        """ Update the pose of the drone based on the translations published
        by analyze_pose
        """
        # update the header stamp
        self.state.header.stamp = data.header.stamp
        # update the pose data
        self.filter_pose(data.pose)

    def range_callback(self, data):
        """ Update the z-position of the drone """
        # update the z position
        self.filter_range(data.range)
        # set received range data to True
        self.received_range_data = True

    # EMA Filtering Methods:
    ########################
    def filter_pose(self, pose):
        """ Calculate the position of the drone based on the pose translations
        using an EMA filter if the pose is based off of the first image, or by
        integrating if the pose is based off the previous image
        """
        # last position of the drone
        last_position = self.state.pose_with_covariance.pose.position
        # raw measured translations and rotation by analyze_transform
        position_reading = pose.position
        # constant used for the EMA filter
        alpha = 0.2
        # blend the new measurement with the old position using an EMA filter
        smoothed_x = (1.0 - alpha) * last_position.x + alpha * position_reading.x
        smoothed_y = (1.0 - alpha) * last_position.y + alpha * position_reading.y

        self.state.pose_with_covariance.pose.position.x = smoothed_x
        self.state.pose_with_covariance.pose.position.y = smoothed_y

    def imu_callback(self, data):
        """ Update the attitude of the drone """
        self.state.pose_with_covariance.pose.orientation = data.orientation
        self.state.twist_with_covariance.twist.angular = data.angular_velocity

    def filter_twist(self, twist):
        """ Run an ema filter on the velocity data and update the state velocity """
        # current position of the drone
        altitude = self.state.pose_with_covariance.pose.position.z
        # measured planar velocities of the drone
        new_vel = twist.linear
        # the current planar velocities of the drone
        velocity = self.state.twist_with_covariance.twist.linear
        # update the angular velocity coefficients to account for the angular
        # motion of the drone
        self.calc_angle_comp_values()
        # the constant for the ema filter
        alpha = 0.4
        velocity.x = self.near_zero((1.0 - alpha) * velocity.x + alpha * (new_vel.x - self.mw_angle_comp_x))
        velocity.y = self.near_zero((1.0 - alpha) * velocity.y + alpha * (new_vel.y - self.mw_angle_comp_y))
        self.state.twist_with_covariance.twist.linear = velocity

    def filter_range(self, range_reading):
        """ Smooth the range reading using and ema filter """
        # the ema filter constant
        alpha = 0.8
        # get the roll and pitch
        r,p,_ = self.get_r_p_y()
        # the z-position of the drone which is calculated by multiplying the
        # the range reading by the cosines of the roll and pitch
        curr_altitude = range_reading * np.cos(r) * np.cos(p)
        prev_altitude = self.state.pose_with_covariance.pose.position.z
        # use an ema filter to smoothe the range reading
        smoothed_altitude= (1.0 - alpha) * curr_altitude + alpha * prev_altitude
        # ensure that the range value is between 0 and 0.55 m
        smoothed_altitude = max(0, min(smoothed_altitude, 0.55))
        # update the current z position
        self.state.pose_with_covariance.pose.position.z = smoothed_altitude


    # Helper Methods:
    #################
    def get_r_p_y(self):
        """ Return the roll, pitch, and yaw from the orientation quaternion """
        x = self.state.pose_with_covariance.pose.orientation.x
        y = self.state.pose_with_covariance.pose.orientation.y
        z = self.state.pose_with_covariance.pose.orientation.z
        w = self.state.pose_with_covariance.pose.orientation.w
        quaternion = (x,y,z,w)
        r,p,y = tf.transformations.euler_from_quaternion(quaternion)
        return r,p,y

    def calc_angle_comp_values(self):
        """ Calculates angle compensation values to account for the tilt of the
        drone when calculating the velocity error """
        altitude = self.state.pose_with_covariance.pose.position.z
        angular_velocities = self.state.twist_with_covariance.twist.angular
        # v = w * r  :  (linear velocity = angular velocity * radius)
        self.mw_angle_comp_x = angular_velocities.x * altitude * self.mw_angle_coeff
        self.mw_angle_comp_y = angular_velocities.y * altitude * self.mw_angle_coeff

    def near_zero(self, n):
        """ Set a number to zero if it is below a threshold value """
        return 0.0 if abs(n) < 1e-6 else n

    def ctrl_c_handler(self, signal, frame):
        """ Exit the program """
        print '\nCaught ctrl-c. Stopping node.'
        sys.exit()


def main():
    # ROS setup
    ###########
    # Initialize the state estimator node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # Instantiate an EMAStateEstimator object
    state_estimator = EMAStateEstimator()

    # Publishers
    ############
    statepub = rospy.Publisher(state_estimator.state_topic_str, State,
                               queue_size=1, tcp_nodelay=False)

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/reset_transform", Empty, state_estimator.reset_callback)
    rospy.Subscriber('/pidrone/picamera/twist', TwistStamped, state_estimator.twist_callback)
    rospy.Subscriber('/pidrone/picamera/pose', PoseStamped, state_estimator.pose_callback)
    rospy.Subscriber('/pidrone/infrared', Range, state_estimator.range_callback)
    rospy.Subscriber('/pidrone/imu', Imu, state_estimator.imu_callback)

    # set up ctrl-c handler
    signal.signal(signal.SIGINT, state_estimator.ctrl_c_handler)
    print 'waiting for velocity and range data'
    while not state_estimator.received_twist_data and not state_estimator.received_range_data:
        pass
    print 'Publishing State'

    # set the publishing rate (Hz)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        statepub.publish(state_estimator.state)
        rate.sleep()


if __name__ == '__main__':
    main()
