#!/usr/bin/env python

# ROS imports
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import PoseStamped, TwistStamped
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
import os


class UKFStateEstimator7D(object):
    """
    Class that estimates the state of the drone using an Unscented Kalman Filter
    (UKF) applied to raw sensor data. This script tests the addition of position
    estimates from the camera, which we get from position hold, localization, or
    SLAM.
    """
    # TODO: Make a reference to the UKF math document that is being written up,
    #       once it is in a complete enough state and can be placed in a shared
    #       location.
    
    def __init__(self, loop_hz, ir_throttled=False, imu_throttled=False, optical_flow_throttled=False, camera_pose_throttled=False):
        # self.ready_to_filter is False until we get initial measurements in
        # order to be able to initialize the filter's state vector x and
        # covariance matrix P.
        self.ready_to_filter = False
        self.printed_filter_start_notice = False
        self.got_ir = False
        # self.got_camera_pose = False
        # self.got_optical_flow = False
        self.got_imu = False
        self.loop_hz = loop_hz
        
        self.ir_topic_str = '/pidrone/infrared'
        self.imu_topic_str = '/pidrone/imu'
        self.optical_flow_topic_str = '/pidrone/picamera/twist'
        self.camera_pose_topic_str = '/pidrone/picamera/pose'
        throttle_suffix = '_throttle'
        
        if ir_throttled:
            self.ir_topic_str += throttle_suffix
        if imu_throttled:
            self.imu_topic_str += throttle_suffix
        if optical_flow_throttled:
            self.optical_flow_topic_str += throttle_suffix
        if camera_pose_throttled:
            self.camera_pose_topic_str += throttle_suffix
            
        # Localization wants angular velocities, so take from IMU and republish
        # in the state message
        self.angular_velocity = None
            
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
        
        self.last_measurement_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.initialize_ros()
        
    def initialize_ros(self):
        """
        Initialize ROS-related objects, e.g., the node, subscribers, etc.
        """
        self.node_name = os.path.splitext(os.path.basename(__file__))[0]
        print 'Initializing {} node...'.format(self.node_name)
        rospy.init_node(self.node_name)
        
        # Create the publisher to publish state estimates
        self.state_pub = rospy.Publisher('/pidrone/state/ukf_7d', State, queue_size=1,
                                         tcp_nodelay=False)
        
        # Subscribe to topics to which the drone publishes in order to get raw
        # data from sensors, which we can then filter
        rospy.Subscriber(self.imu_topic_str, Imu, self.imu_data_callback)
        rospy.Subscriber(self.ir_topic_str, Range, self.ir_data_callback)
        rospy.Subscriber(self.optical_flow_topic_str, TwistStamped,
                         self.optical_flow_data_callback)
        rospy.Subscriber(self.camera_pose_topic_str, PoseStamped,
                         self.camera_pose_data_callback)
        
    def initialize_ukf(self):
        """
        Initialize the parameters of the Unscented Kalman Filter (UKF) that is
        used to estimate the state of the drone.
        """
        
        # Number of state variables being tracked
        self.state_vector_dim = 7
        # The state vector consists of the following column vector.
        # Note that FilterPy initializes the state vector with zeros.
        # [[x],
        #  [y],
        #  [z],
        #  [x_vel],
        #  [y_vel],
        #  [z_vel],
        #  [yaw]]
        
        # Number of measurement variables that the drone receives
        self.measurement_vector_dim = 6
        # The measurement variables consist of the following vector:
        # [[slant_range],
        #  [x],
        #  [y],
        #  [x_vel],
        #  [y_vel],
        #  [yaw]]
        
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
        """
        Initialize the covariance matrices of the UKF
        """
        # Initialize state covariance matrix P:
        # TODO: Tune these initial values appropriately. Currently these are
        #       just guesses
        self.ukf.P = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.0005])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune appropriately. Currently just a guess
        self.ukf.Q = np.diag([0.01, 0.01, 0.01, 1.0, 1.0, 1.0, 0.1])*0.005
        
        # Initialize the measurement covariance matrix R
        # IR slant range variance (m^2), determined experimentally in a static
        # setup with mean range around 0.335 m:
        self.measurement_cov_ir = np.array([2.2221e-05])
        self.measurement_cov_optical_flow = np.diag([0.01, 0.01])
        # Estimated standard deviation of 5 cm = 0.05 m ->
        # variance of 0.05^2 = 0.0025
        self.measurement_cov_camera_pose = np.diag([0.0025, 0.0025, 0.0003])
        self.ukf.R = np.diag([2.2221e-05, 0.0025, 0.0025, 0.01, 0.01, 0.0003])
                
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
        self.last_control_input = np.array([data.linear_acceleration.x,
                                            data.linear_acceleration.y,
                                            data.linear_acceleration.z])
        self.angular_velocity = data.angular_velocity
        if not self.ready_to_filter:
            self.initialize_input_time(data)
            self.got_imu = True
            self.check_if_ready_to_filter()
        self.in_callback = False
                        
    def ir_data_callback(self, data):
        """
        Handle the receipt of a Range message from the IR sensor.
        """
        if self.in_callback:
            return
        self.in_callback = True
        if self.ready_to_filter:
            self.update_input_time(data)
            self.last_measurement_vector[0] = data.range
        else:
            self.initialize_input_time(data)
            # Got a raw slant range reading, so update the initial state
            # vector of the UKF
            self.ukf.x[2] = data.range
            self.ukf.x[5] = 0.0  # initialize velocity as 0 m/s
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[2, 2] = self.measurement_cov_ir[0]
            self.got_ir = True
            self.check_if_ready_to_filter()
        self.in_callback = False
        
    def optical_flow_data_callback(self, data):
        """
        Handle the receipt of a TwistStamped message from optical flow.
        The message parts that we will be using:
            - x velocity (m/s)
            - y velocity (m/s)
        
        This method PREDICTS with the most recent control input and UPDATES.
        """
        if self.in_callback:
            return
        self.in_callback = True
        if self.ready_to_filter:
            self.update_input_time(data)
            self.last_measurement_vector[3] = data.twist.linear.x
            self.last_measurement_vector[4] = data.twist.linear.y
        else:
            self.initialize_input_time(data)
            # Update the initial state vector of the UKF
            self.ukf.x[3] = data.twist.linear.x  # x velocity
            self.ukf.x[4] = data.twist.linear.y  # y velocity
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[3, 3] = self.measurement_cov_optical_flow[0, 0]
            self.ukf.P[4, 4] = self.measurement_cov_optical_flow[1, 1]
            # self.got_optical_flow = True
            self.check_if_ready_to_filter()
        self.in_callback = False
        
    def camera_pose_data_callback(self, data):
        """
        Handle the receipt of a PoseStamped message from camera data.
        The message parts that we will be using:
            - x (meters)
            - y (meters)
            - yaw (radians)
        
        This method PREDICTS with the most recent control input and UPDATES.
        """
        if self.in_callback:
            return
        self.in_callback = True
        _, _, yaw = tf.transformations.euler_from_quaternion(
                                                      [data.pose.orientation.x,
                                                       data.pose.orientation.y,
                                                       data.pose.orientation.z,
                                                       data.pose.orientation.w])
        if self.ready_to_filter:
            self.update_input_time(data)
            self.last_measurement_vector[1] = data.pose.position.x
            self.last_measurement_vector[2] = data.pose.position.y
            self.last_measurement_vector[5] = yaw
        else:
            self.initialize_input_time(data)
            # Update the initial state vector of the UKF
            self.ukf.x[0] = data.pose.position.x
            self.ukf.x[1] = data.pose.position.y
            self.ukf.x[6] = yaw
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[0, 0] = self.measurement_cov_camera_pose[0, 0]
            self.ukf.P[1, 1] = self.measurement_cov_camera_pose[1, 1]
            self.ukf.P[6, 6] = self.measurement_cov_camera_pose[2, 2]
            # self.got_camera_pose = True
            self.check_if_ready_to_filter()
        self.in_callback = False
            
    def check_if_ready_to_filter(self):
        self.ready_to_filter = (self.got_ir and self.got_imu)
                        
    def publish_current_state(self):
        """
        Publish the current state estimate and covariance from the UKF. This is
        a State message containing:
            - Header
            - PoseWithCovariance
            - TwistWithCovariance
        Note that a lot of these ROS message fields will be left empty, as the
        1D UKF does not track information on the entire state space of the
        drone.
        """
        state_msg = State()
        state_msg.header.stamp.secs = self.last_time_secs
        state_msg.header.stamp.nsecs = self.last_time_nsecs
        state_msg.header.frame_id = 'global'
        
        # TODO: Consider using roll and pitch values directly from the IMU, as
        #       the IMU implements its own filter on attitude
        quaternion = self.get_quaternion_from_yaw(self.ukf.x[6])
        
        # Get the current state estimate from self.ukf.x, and fill the rest of
        # the message with NaN
        state_msg.pose_with_covariance.pose.position.x = self.ukf.x[0]
        state_msg.pose_with_covariance.pose.position.y = self.ukf.x[1]
        state_msg.pose_with_covariance.pose.position.z = self.ukf.x[2]
        state_msg.pose_with_covariance.pose.orientation.x = quaternion[0]
        state_msg.pose_with_covariance.pose.orientation.y = quaternion[1]
        state_msg.pose_with_covariance.pose.orientation.z = quaternion[2]
        state_msg.pose_with_covariance.pose.orientation.w = quaternion[3]
        state_msg.twist_with_covariance.twist.linear.x = self.ukf.x[3]
        state_msg.twist_with_covariance.twist.linear.y = self.ukf.x[4]
        state_msg.twist_with_covariance.twist.linear.z = self.ukf.x[5]
        state_msg.twist_with_covariance.twist.angular = self.angular_velocity
        
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
        
    def get_quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0.0,
                                                        0.0,
                                                        yaw)
        
    def apply_quaternion_vector_rotation(self, original_vector, yaw):
        """
        Rotate a vector from the drone's body frame to the global frame using
        quaternion-vector multiplication
        """
        # Use quaternion-vector multiplication instead of a rotation matrix to
        # rotate the vector.
        # This quaternion describes rotation from the global frame to the body
        # frame, so take the quaternion's inverse by negating its real
        # component (w)
        quat_global_to_body = self.get_quaternion_from_yaw(yaw)
        quat_body_to_global = list(quat_global_to_body) # copy the quaternion
        quat_body_to_global[3] = -quat_body_to_global[3]
        original_vector_as_quat = list(original_vector)
        original_vector_as_quat.append(0.0) # vector as quaternion with w=0
        # Apply quaternion rotation on a vector: q*v*q', where q is the rotation
        # quaternion, v is the vector (a "pure" quaternion with w=0), and q' is
        # the conjugate of the quaternion q
        original_vector_rotated = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(quat_body_to_global,
                                                   original_vector_as_quat),
            tf.transformations.quaternion_conjugate(quat_body_to_global))
        # Drop the real part w=0
        original_vector_rotated = original_vector_rotated[:3]
        return original_vector_rotated

    def state_transition_function(self, x, dt, u):
        """
        The state transition function to compute the prior in the prediction
        step, propagating the state to the next time step.
        
        x : current state. A NumPy array
        dt : time step. A float
        u : control input. A NumPy array
        """
        accels_global_frame = self.apply_quaternion_vector_rotation(u, x[6])
        
        dt2 = dt**2.0
        return x + np.array([x[3]*dt + 0.5*accels_global_frame[0]*dt2,
                             x[4]*dt + 0.5*accels_global_frame[1]*dt2,
                             x[5]*dt + 0.5*accels_global_frame[2]*dt2,
                             accels_global_frame[0]*dt,
                             accels_global_frame[1]*dt,
                             accels_global_frame[2]*dt,
                             0.0])
        
    def measurement_function(self, x):
        """
        Transform the state x into measurement space. In this simple model, we
        assume that the range measurement corresponds exactly to position along
        the z-axis, as we are assuming there is no pitch and roll.
        
        x : current state. A NumPy array
        """
        H = np.array([[0, 0, 1, 0, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1]])
        result = np.dot(H, x)
        return result
        
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
                                     'with a UKF'))
    # Arguments to determine if the throttle command is being used. E.g.:
    #   rosrun topic_tools throttle messages /pidrone/infrared 40.0
    parser.add_argument('--ir_throttled', action='store_true',
            help=('Use throttled infrared topic /pidrone/infrared_throttle'))
    parser.add_argument('--imu_throttled', action='store_true',
            help=('Use throttled IMU topic /pidrone/imu_throttle'))
    parser.add_argument('--optical_flow_throttled', action='store_true',
                        help=('Use throttled optical flow topic /pidrone/picamera/twist_throttle'))
    parser.add_argument('--camera_pose_throttled', action='store_true',
                        help=('Use throttled camera pose topic /pidrone/picamera/pose_throttle'))
    parser.add_argument('--loop_hz', '-hz', default=30.0,
                        type=check_positive_float_duration,
                        help=('Frequency at which to run the predict-update '
                              'loop of the UKF (default: 30)'))
    # NOTE: The throttle flags are deprecated in favor of the loop Hz flag.
    #       Using throttled data streams while also running the UKF on a set
    #       loop can degrade the estimates.
    args = parser.parse_args()
    se = UKFStateEstimator7D(loop_hz=args.loop_hz,
                             ir_throttled=args.ir_throttled,
                             imu_throttled=args.imu_throttled,
                             optical_flow_throttled=args.optical_flow_throttled,
                             camera_pose_throttled=args.camera_pose_throttled)
    try:
        se.start_loop()
    finally:
        # Upon termination of this script, print out a helpful message
        print '{} node terminating.'.format(se.node_name)
        print 'Most recent state vector:'
        print se.ukf.x
        # print 'Most recent state covariance matrix:'
        # print se.ukf.P
        
if __name__ == '__main__':
    main()
