#!/usr/bin/env python

# ROS imports
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import TwistStamped
from pidrone_pkg.msg import State

# UKF imports
# The matplotlib imports and the matplotlib.use('Pdf') line make it so that the
# UKF code that imports matplotlib does not complain. Essentially, the
# use('Pdf') call allows a plot to be created without a window (allows it to run
# through ssh)
import matplotlib
matplotlib.use('Pdf')
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

# Other imports
import numpy as np
import argparse


class StateEstimation(object):
    '''
    Class that estimates the state of the drone using an Unscented Kalman Filter
    (UKF) applied to raw sensor data.
    '''
    # TODO: Make a reference to the UKF math document that is being written up,
    #       once it is in a complete enough state and can be placed in a shared
    #       location.
    
    def __init__(self, ir_throttled=False, imu_throttled=False, optical_flow_throttled=False):
        # self.ready_to_filter is False until we get initial measurements in
        # order to be able to initialize the filter's state vector x and
        # covariance matrix P.
        self.ready_to_filter = False
        self.printed_filter_start_notice = False
        self.got_ir = False
        self.got_optical_flow = False
        
        self.ir_topic_str = '/pidrone/infrared'
        self.imu_topic_str = '/pidrone/imu'
        self.optical_flow_topic_str = '/pidrone/picamera/twist'
        throttle_suffix = '_throttle'
        
        if ir_throttled:
            self.ir_topic_str += throttle_suffix
        if imu_throttled:
            self.imu_topic_str += throttle_suffix
        if optical_flow_throttled:
            self.optical_flow_topic_str += throttle_suffix
            
        self.in_callback = False

        self.initialize_ukf()
        
        # The last time that we received an input and formed a prediction with
        # the state transition function
        self.last_state_transition_time = None
        
        # Time in seconds between consecutive state transitions, dictated by
        # when the inputs come in
        self.dt = None
        
        # Initialize the last control input as 0 m/s^2 along each axis
        self.last_control_input = np.array([0.0, 0.0, 0.0])
        
        self.initialize_ros()
        
    def initialize_ros(self):
        '''
        Initialize ROS-related objects, e.g., the node, subscribers, etc.
        '''
        self.node_name = 'state_estimator_ukf_test_2'
        print 'Initializing {} node...'.format(self.node_name)
        rospy.init_node(self.node_name)
        
        # Subscribe to topics to which the drone publishes in order to get raw
        # data from sensors, which we can then filter
        rospy.Subscriber(self.imu_topic_str, Imu, self.imu_data_callback)
        rospy.Subscriber(self.ir_topic_str, Range, self.ir_data_callback)
        rospy.Subscriber(self.optical_flow_topic_str, TwistStamped,
                         self.optical_flow_data_callback)
        
        # Create the publisher to publish state estimates
        self.state_pub = rospy.Publisher('/pidrone/state', State, queue_size=1,
                                        tcp_nodelay=False)
        
    def initialize_ukf(self):
        '''
        Initialize the parameters of the Unscented Kalman Filter (UKF) that is
        used to estimate the state of the drone.
        '''
        
        # Number of state variables being tracked
        self.state_vector_dim = 6
        # The state vector consists of the following column vector.
        # Note that FilterPy initializes the state vector with zeros.
        # [[x],
        #  [y],
        #  [z],
        #  [x_vel],
        #  [y_vel],
        #  [z_vel]]
        
        # Number of measurement variables that the drone receives
        self.measurement_vector_dim = 3
        # The measurement variables consist of the following vector:
        # [[slant_range],
        #  [x_vel],
        #  [y_vel]]
        
        # Function to generate sigma points for the UKF
        # TODO: Modify these sigma point parameters appropriately. Currently
        #       just guesses
        sigma_points = MerweScaledSigmaPoints(n=self.state_vector_dim,
                                              alpha=0.1,
                                              beta=2.0,
                                              kappa=(3.0-self.state_vector_dim))
        # Create the UKF object
        # Note that dt will get updated dynamically as sensor data comes in,
        # as will the measurement function, since measurements come in at
        # distinct rates. Setting compute_log_likelihood=False saves some
        # computation.
        self.ukf = UnscentedKalmanFilter(dim_x=self.state_vector_dim,
                                         dim_z=self.measurement_vector_dim,
                                         dt=1.0,
                                         hx=self.measurement_function,
                                         fx=self.state_transition_function,
                                         points=sigma_points,
                                         compute_log_likelihood=False)
        self.initialize_ukf_matrices()

    def initialize_ukf_matrices(self):
        '''
        Initialize the covariance matrices of the UKF
        '''
        # Initialize state covariance matrix P:
        # TODO: Tune these initial values appropriately. Currently these are
        #       just guesses
        self.ukf.P = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune appropriately. Currently just a guess
        self.ukf.Q = np.diag([0.01, 0.01, 0.01, 1.0, 1.0, 1.0])*0.005
        
        # Initialize the measurement covariance matrix R
        # IR slant range variance (m^2), determined experimentally in a static
        # setup with mean range around 0.335 m:
        self.measurement_cov_ir = np.array([2.2221e-05])
        self.measurement_cov_optical_flow = np.diag([0.01, 0.01])
                
    def update_input_time(self, msg):
        '''
        Update the time at which we have received the most recent input, based
        on the timestamp in the header of a ROS message
        
        msg : a ROS message that includes a header with a timestamp that
              indicates the time at which the respective input was originally
              recorded
        '''
        self.last_time_secs = msg.header.stamp.secs
        self.last_time_nsecs = msg.header.stamp.nsecs
        new_time = self.last_time_secs + self.last_time_nsecs*1e-9
        # Compute the time interval since the last state transition / input
        self.dt = new_time - self.last_state_transition_time
        # Set the current time at which we just received an input
        # to be the last input time
        self.last_state_transition_time = new_time
        
    def initialize_input_time(self, msg):
        '''
        Initialize the input time (self.last_state_transition_time) based on the
        timestamp in the header of a ROS message. This is called before we start
        filtering in order to attain an initial time value, which enables us to
        then compute a time interval self.dt by calling self.update_input_time()
        
        msg : a ROS message that includes a header with a timestamp
        '''
        self.last_time_secs = msg.header.stamp.secs
        self.last_time_nsecs = msg.header.stamp.nsecs
        self.last_state_transition_time = (self.last_time_secs +
                                           self.last_time_nsecs*1e-9)
        
    def ukf_predict(self):
        '''
        Compute the prior for the UKF based on the current state, a control
        input, and a time step.
        '''
        self.ukf.predict(dt=self.dt, u=self.last_control_input)
        
    def print_notice_if_first(self):
        if not self.printed_filter_start_notice:
            print 'Starting filter'
            self.printed_filter_start_notice = True
        
    def imu_data_callback(self, data):
        '''
        Handle the receipt of an Imu message. Only take the linear acceleration
        along the z-axis.
        
        This method PREDICTS with a control input.
        '''
        if self.in_callback:
            return
        self.in_callback = True
        self.last_control_input = np.array([data.linear_acceleration.x,
                                            data.linear_acceleration.y,
                                            data.linear_acceleration.z])
        # self.last_control_input = np.array([0.0,
        #                                     0.0,
        #                                     data.linear_acceleration.z])
        if self.ready_to_filter:
            # Wait to predict until we get an initial IR measurement to
            # initialize our state vector
            self.print_notice_if_first()
            self.update_input_time(data)
            self.ukf_predict()
            self.publish_current_state()
        self.in_callback = False
                        
    def ir_data_callback(self, data):
        '''
        Handle the receipt of a Range message from the IR sensor.
        
        This method PREDICTS with the most recent control input and UPDATES.
        '''
        if self.in_callback:
            return
        self.in_callback = True
        if self.ready_to_filter:
            self.print_notice_if_first()
            self.update_input_time(data)
            self.ukf_predict()
                        
            # Now that a prediction has been formed to bring the current prior
            # state estimate to the same point in time as the measurement,
            # perform a measurement update with the slant range reading
            measurement_z = np.array([data.range])
            self.ukf.update(measurement_z,
                            hx=self.measurement_function_ir,
                            R=self.measurement_cov_ir)
            self.publish_current_state()
        else:
            self.initialize_input_time(data)
            # Got a raw slant range reading, so update the initial state
            # vector of the UKF
            self.ukf.x[2] = data.range
            self.ukf.x[5] = 0.0 # initialize velocity as 0 m/s
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[2, 2] = self.measurement_cov_ir[0]
            self.got_ir = True
            self.check_if_ready_to_filter()
        self.in_callback = False
        
    def optical_flow_data_callback(self, data):
        '''
        Handle the receipt of a TwistStamped message from optical flow.
        The message parts that we will be using:
            - x velocity (m/s)
            - y velocity (m/s)
        
        This method PREDICTS with the most recent control input and UPDATES.
        '''
        if self.in_callback:
            return
        self.in_callback = True
        if self.ready_to_filter:
            self.print_notice_if_first()
            self.update_input_time(data)
            self.ukf_predict()
            
            # Now that a prediction has been formed to bring the current prior
            # state estimate to the same point in time as the measurement,
            # perform a measurement update with x velocity, y velocity, and yaw
            # velocity data in the TwistStamped message
            # TODO: Verify the units of these velocities that are being
            #       published
            measurement_z = np.array([data.twist.linear.x,  # x velocity
                                      data.twist.linear.y]) # y velocity
            # Ensure that we are using subtraction to compute the residual
            #self.ukf.residual_z = np.subtract
            self.ukf.update(measurement_z,
                            hx=self.measurement_function_optical_flow,
                            R=self.measurement_cov_optical_flow)
            self.publish_current_state()
        else:
            self.initialize_input_time(data)
            # Update the initial state vector of the UKF
            self.ukf.x[3] = data.twist.linear.x # x velocity
            self.ukf.x[4] = data.twist.linear.y # y velocity
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[3, 3] = self.measurement_cov_optical_flow[0, 0]
            self.ukf.P[4, 4] = self.measurement_cov_optical_flow[1, 1]
            self.got_optical_flow = True
            self.check_if_ready_to_filter()
        self.in_callback = False
            
    def check_if_ready_to_filter(self):
        self.ready_to_filter = (self.got_ir and self.got_optical_flow)
                        
    def publish_current_state(self):
        '''
        Publish the current state estimate and covariance from the UKF. This is
        a State message containing:
            - Header
            - PoseWithCovariance
            - TwistWithCovariance
        Note that a lot of these ROS message fields will be left empty, as the
        1D UKF does not track information on the entire state space of the
        drone.
        '''
        state_msg = State()
        state_msg.header.stamp.secs = self.last_time_secs
        state_msg.header.stamp.nsecs = self.last_time_nsecs
        state_msg.header.frame_id = 'global'
        
        # Get the current state estimate from self.ukf.x, and fill the rest of
        # the message with NaN
        state_msg.pose_with_covariance.pose.position.x = self.ukf.x[0]
        state_msg.pose_with_covariance.pose.position.y = self.ukf.x[1]
        state_msg.pose_with_covariance.pose.position.z = self.ukf.x[2]
        # TODO: Leave commented if JS interface Top View animation is desired
        # state_msg.pose_with_covariance.pose.orientation.x = np.nan
        # state_msg.pose_with_covariance.pose.orientation.y = np.nan
        # state_msg.pose_with_covariance.pose.orientation.z = np.nan
        # state_msg.pose_with_covariance.pose.orientation.w = np.nan
        # TODO: Leave following line if JS interface Top View animation is desired
        state_msg.pose_with_covariance.pose.orientation.w = 1
        state_msg.twist_with_covariance.twist.linear.x = self.ukf.x[3]
        state_msg.twist_with_covariance.twist.linear.y = self.ukf.x[4]
        state_msg.twist_with_covariance.twist.linear.z = self.ukf.x[5]
        state_msg.twist_with_covariance.twist.angular.x = np.nan
        state_msg.twist_with_covariance.twist.angular.y = np.nan
        state_msg.twist_with_covariance.twist.angular.z = np.nan
        
        # Prepare covariance matrices
        # TODO: Finish populating these matrices, if deemed necessary
        # 36-element array, in a row-major order, according to ROS msg docs
        pose_cov_mat = np.full((36,), np.nan)
        twist_cov_mat = np.full((36,), np.nan)
        pose_cov_mat[14] = self.ukf.P[2, 2] # z variance
        twist_cov_mat[14] = self.ukf.P[5, 5] # z velocity variance
        
        # Add covariances to message
        state_msg.pose_with_covariance.covariance = pose_cov_mat
        state_msg.twist_with_covariance.covariance = twist_cov_mat
        
        self.state_pub.publish(state_msg)

    def state_transition_function(self, x, dt, u):
        '''
        The state transition function to compute the prior in the prediction
        step, propagating the state to the next time step.
        
        x : current state. A NumPy array
        dt : time step. A float
        u : control input. A NumPy array
        '''
        dt2 = dt**2.0
        return x + np.array([x[3]*dt + 0.5*u[0]*dt2,
                             x[4]*dt + 0.5*u[1]*dt2,
                             x[5]*dt + 0.5*u[2]*dt2,
                             u[0]*dt,
                             u[1]*dt,
                             u[2]*dt])
        
    def measurement_function(self, x):
        '''
        Transform the state x into measurement space. In this simple model, we
        assume that the range measurement corresponds exactly to position along
        the z-axis, as we are assuming there is no pitch and roll.
        
        x : current state. A NumPy array
        '''
        pass
        
    def measurement_function_ir(self, x):
        return np.array([x[2]])
        
    def measurement_function_optical_flow(self, x):
        return np.array([x[3], x[4]])
        
        
def main():
    parser = argparse.ArgumentParser(description=('Estimate the drone\'s state '
                                     'with a UKF'))
    # Arguments to determine if the throttle command is being used. E.g.:
    #   rosrun topic_tools throttle messages /pidrone/infrared 40.0
    parser.add_argument('--ir_throttled', action='store_true',
            help=('Use throttled infrared topic /pidrone/infrared_throttle'))
    parser.add_argument('--imu_throttled', action='store_true',
            help=('Use throttled IMU topic /pidrone/imu_throttle'))
    parser.add_argument('--optical_flow_throttled', action='store_true',
                        help=('Use throttled optical flow topic /pidrone/picamera/twist_throttle'))
    args = parser.parse_args()
    se = StateEstimation(ir_throttled=args.ir_throttled,
                         imu_throttled=args.imu_throttled,
                         optical_flow_throttled=args.optical_flow_throttled)
    try:
        # Wait until node is halted
        rospy.spin()
    finally:
        # Upon termination of this script, print out a helpful message
        print '{} node terminating.'.format(se.node_name)
        print 'Most recent state vector:'
        print se.ukf.x
        # print 'Most recent state covariance matrix:'
        # print se.ukf.P
        
if __name__ == '__main__':
    main()
