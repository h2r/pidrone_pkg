#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Imu, Range
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
from filterpy.common.discretization import Q_discrete_white_noise

# Other imports
import numpy as np
import argparse
import os


class UKFStateEstimator2D(object):
    """
    Class that estimates the state of the drone using an Unscented Kalman Filter
    (UKF) applied to raw sensor data. The filter only tracks the quadcopter's
    motion along one spatial dimension: the global frame z-axis. In this
    simplified case, we assume that the drone's body frame orientation is
    aligned with the world frame (i.e., no roll, pitch, or yaw), and that it is
    only offset along the z-axis. It is called a 2D UKF because we track two
    state variables in the state vector: z position and z velocity.
    """
    
    def __init__(self, loop_hz, ir_throttled=False, imu_throttled=False):
        # self.ready_to_filter is False until we get initial measurements in
        # order to be able to initialize the filter's state vector x and
        # covariance matrix P.
        self.ready_to_filter = False
        self.printed_filter_start_notice = False
        self.got_ir = False
        self.loop_hz = loop_hz
        
        self.ir_topic_str = '/pidrone/infrared'
        self.imu_topic_str = '/pidrone/imu'
        throttle_suffix = '_throttle'
        
        if ir_throttled:
            self.ir_topic_str += throttle_suffix
        if imu_throttled:
            self.imu_topic_str += throttle_suffix
            
        self.in_callback = False

        self.initialize_ukf()
        
        # The last time that we received an input and formed a prediction with
        # the state transition function
        self.last_state_transition_time = None
        
        # Time in seconds between consecutive state transitions, dictated by
        # when the inputs come in
        self.dt = None
        
        # Initialize the last control input as 0 m/s^2 along the z-axis
        self.last_control_input = np.array([0.0])
        
        self.initialize_ros()
        
    def initialize_ros(self):
        """
        Initialize ROS-related objects, e.g., the node, subscribers, etc.
        """
        self.node_name = os.path.splitext(os.path.basename(__file__))[0]
        print 'Initializing {} node...'.format(self.node_name)
        rospy.init_node(self.node_name)
        
        # Subscribe to topics to which the drone publishes in order to get raw
        # data from sensors, which we can then filter
        rospy.Subscriber(self.imu_topic_str, Imu, self.imu_data_callback)
        rospy.Subscriber(self.ir_topic_str, Range, self.ir_data_callback)
        
        # Create the publisher to publish state estimates
        self.state_pub = rospy.Publisher('/pidrone/state/ukf_2d', State, queue_size=1,
                                         tcp_nodelay=False)
        
    def initialize_ukf(self):
        """
        Initialize the parameters of the Unscented Kalman Filter (UKF) that is
        used to estimate the state of the drone.
        """
        
        # Number of state variables being tracked
        self.state_vector_dim = 2
        # The state vector consists of the following column vector.
        # Note that FilterPy initializes the state vector with zeros.
        # [[z],
        #  [z_vel]]
        
        # Number of measurement variables that the drone receives
        self.measurement_vector_dim = 1
        # The measurement variables consist of the following vector:
        # [[slant_range]]
        
        # Function to generate sigma points for the UKF
        # TODO: Modify these sigma point parameters appropriately. Currently
        #       just educated guesses
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
        """
        Initialize the covariance matrices of the UKF
        """
        # Initialize state covariance matrix P:
        # TODO: Tune these initial values appropriately. Currently these are
        #       just guesses
        self.ukf.P = np.diag([0.1, 0.2])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune appropriately. Currently just a guess
        self.ukf.Q = np.diag([0.01, 1.0])*0.005
        # self.ukf.Q = Q_discrete_white_noise(self.state_vector_dim)
        
        # Initialize the measurement covariance matrix R
        # IR slant range variance (m^2), determined experimentally in a static
        # setup with mean range around 0.335 m:
        self.ukf.R = np.array([2.2221e-05])
        
    def update_input_time(self, msg):
        """
        Update the time at which we have received the most recent input, based
        on the timestamp in the header of a ROS message
        
        msg : a ROS message that includes a header with a timestamp that
              indicates the time at which the respective input was originally
              recorded
        """
        self.last_time_secs = msg.header.stamp.secs
        self.last_time_nsecs = msg.header.stamp.nsecs
        new_time = self.last_time_secs + self.last_time_nsecs*1e-9
        # Compute the time interval since the last state transition / input
        self.dt = new_time - self.last_state_transition_time
        # Assert that the time step be non-negative. Negative time steps seem to
        # be able to arise when more than one input node is running (i.e., both
        # IR and flight controller nodes). This might result in small
        # discrepencies between different nodes' delays from timestamping to
        # publishing. Another solution might be to call rospy.Time.now() right
        # before those nodes publish, or on the receiving end in this state
        # estimator node
        if self.dt < 0:
            self.dt = 0
        # Set the current time at which we just received an input
        # to be the last input time
        self.last_state_transition_time = new_time
        
    def initialize_input_time(self, msg):
        """
        Initialize the input time (self.last_state_transition_time) based on the
        timestamp in the header of a ROS message. This is called before we start
        filtering in order to attain an initial time value, which enables us to
        then compute a time interval self.dt by calling self.update_input_time()
        
        msg : a ROS message that includes a header with a timestamp
        """
        self.last_time_secs = msg.header.stamp.secs
        self.last_time_nsecs = msg.header.stamp.nsecs
        self.last_state_transition_time = (self.last_time_secs +
                                           self.last_time_nsecs*1e-9)
        
    def ukf_predict(self):
        """
        Compute the prior for the UKF based on the current state, a control
        input, and a time step.
        """
        # self.ukf.Q = Q_discrete_white_noise(self.state_vector_dim, dt=self.dt, var=1.0)
        # self.ukf.Q = np.diag([0.01, 1.0])*0.05*self.dt
        self.ukf.predict(dt=self.dt, u=self.last_control_input)
        
    def print_notice_if_first(self):
        if not self.printed_filter_start_notice:
            print 'Starting filter'
            self.printed_filter_start_notice = True
        
    def imu_data_callback(self, data):
        """
        Handle the receipt of an Imu message. Only take the linear acceleration
        along the z-axis.
        """
        if self.in_callback:
            return
        self.in_callback = True
        self.last_control_input = np.array([data.linear_acceleration.z])
        if abs(data.linear_acceleration.z) < 0.3:
            # Adaptive filtering. Lower the process noise if acceleration is low
            self.ukf.Q = np.diag([0.01, 1.0])*0.0005
        else:
            self.ukf.Q = np.diag([0.01, 1.0])*0.005
        self.in_callback = False
                        
    def ir_data_callback(self, data):
        """
        Handle the receipt of a Range message from the IR sensor.
        """
        if self.in_callback:
            return
        self.in_callback = True
        self.last_measurement_vector = np.array([data.range])
        if self.ready_to_filter:
            self.update_input_time(data)
        else:
            self.initialize_input_time(data)
            # Got a raw slant range reading, so update the initial state
            # vector of the UKF
            self.ukf.x[0] = data.range
            self.ukf.x[1] = 0.0  # initialize velocity as 0 m/s
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[0, 0] = self.ukf.R[0]
            self.got_ir = True
            self.check_if_ready_to_filter()
        self.in_callback = False
            
    def check_if_ready_to_filter(self):
        self.ready_to_filter = self.got_ir
                        
    def publish_current_state(self):
        """
        Publish the current state estimate and covariance from the UKF. This is
        a State message containing:
            - Header
            - PoseWithCovariance
            - TwistWithCovariance
        Note that a lot of these ROS message fields will be left empty, as the
        2D UKF does not track information on the entire state space of the
        drone.
        """
        state_msg = State()
        state_msg.header.stamp.secs = self.last_time_secs
        state_msg.header.stamp.nsecs = self.last_time_nsecs
        state_msg.header.frame_id = 'global'
        
        # Get the current state estimate from self.ukf.x
        state_msg.pose_with_covariance.pose.position.z = self.ukf.x[0]
        state_msg.twist_with_covariance.twist.linear.z = self.ukf.x[1]
        
        # Fill the rest of the message with NaN
        state_msg.pose_with_covariance.pose.position.x = np.nan
        state_msg.pose_with_covariance.pose.position.y = np.nan
        state_msg.pose_with_covariance.pose.orientation.x = np.nan
        state_msg.pose_with_covariance.pose.orientation.y = np.nan
        state_msg.pose_with_covariance.pose.orientation.z = np.nan
        state_msg.pose_with_covariance.pose.orientation.w = np.nan
        state_msg.twist_with_covariance.twist.linear.x = np.nan
        state_msg.twist_with_covariance.twist.linear.y = np.nan
        state_msg.twist_with_covariance.twist.angular.x = np.nan
        state_msg.twist_with_covariance.twist.angular.y = np.nan
        state_msg.twist_with_covariance.twist.angular.z = np.nan
        
        # Prepare covariance matrices
        # 36-element array, in a row-major order, according to ROS msg docs
        pose_cov_mat = np.full((36,), np.nan)
        twist_cov_mat = np.full((36,), np.nan)
        pose_cov_mat[14] = self.ukf.P[0, 0]  # z variance
        twist_cov_mat[14] = self.ukf.P[1, 1]  # z velocity variance
        
        # Add covariances to message
        state_msg.pose_with_covariance.covariance = pose_cov_mat
        state_msg.twist_with_covariance.covariance = twist_cov_mat
        
        self.state_pub.publish(state_msg)

    def state_transition_function(self, x, dt, u):
        """
        The state transition function to compute the prior in the prediction
        step, propagating the state to the next time step.
        
        x : current state. A NumPy array
        dt : time step. A float
        u : control input. A NumPy array
        """
        # State transition matrix F
        F = np.array([[1, dt],
                      [0, 1]])
        # Integrate control input acceleration to get a change in velocity
        change_from_control_input = np.array([0,
                                              u[0]*dt])
        # change_from_control_input = np.array([0.5*u[0]*(dt**2.0),
        #                                       u[0]*dt])
        x_output = np.dot(F, x) + change_from_control_input
        return x_output
        
    def measurement_function(self, x):
        """
        Transform the state x into measurement space. In this simple model, we
        assume that the range measurement corresponds exactly to position along
        the z-axis, as we are assuming there is no pitch and roll.
        
        x : current state. A NumPy array
        """
        # Measurement update matrix H
        H = np.array([[1, 0]])
        hx_output = np.dot(H, x)
        return hx_output
        
    def start_loop(self):
        """
        Begin the UKF's loop of predicting and updating. Publish a state
        estimate at the end of each loop.
        """
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            if self.ready_to_filter:
                self.print_notice_if_first()
                self.ukf_predict()
                            
                # Now that a prediction has been formed to bring the current
                # prior state estimate to the same point in time as the
                # measurement, perform a measurement update with the most recent
                # IR range reading
                self.ukf.update(self.last_measurement_vector)
                self.publish_current_state()
            rate.sleep()
            
            
def check_positive_float_duration(val):
    """
    Function to check that the --loop_hz command-line argument is a positive
    float.
    """
    value = float(val)
    if value <= 0.0:
        raise argparse.ArgumentTypeError('Loop Hz must be positive')
    return value
        
def main():
    parser = argparse.ArgumentParser(description=('Estimate the drone\'s state '
                                     'with a UKF in one spatial dimension'))
    # Arguments to determine if the throttle command is being used. E.g.:
    #   rosrun topic_tools throttle messages /pidrone/infrared 40.0
    parser.add_argument('--ir_throttled', action='store_true',
            help=('Use throttled infrared topic /pidrone/infrared_throttle'))
    parser.add_argument('--imu_throttled', action='store_true',
            help=('Use throttled IMU topic /pidrone/imu_throttle'))
    parser.add_argument('--loop_hz', '-hz', default=30.0,
                        type=check_positive_float_duration,
                        help=('Frequency at which to run the predict-update '
                              'loop of the UKF (default: 30)'))
    # NOTE: The throttle flags are deprecated in favor of the loop Hz flag.
    #       Using throttled data streams while also running the UKF on a set
    #       loop can degrade the estimates.
    args = parser.parse_args()
    se = UKFStateEstimator2D(loop_hz=args.loop_hz,
                             ir_throttled=args.ir_throttled,
                             imu_throttled=args.imu_throttled)
    try:
        se.start_loop()
    finally:
        # Upon termination of this script, print out a helpful message
        print '{} node terminating.'.format(se.node_name)
        print 'Most recent state vector:'
        print se.ukf.x
        
if __name__ == '__main__':
    main()
