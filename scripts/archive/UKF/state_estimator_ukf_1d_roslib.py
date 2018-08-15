#!/usr/bin/env python

# ROS imports
import roslibpy

# UKF imports
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

# Other imports
import numpy as np
import argparse
# To be able to get logging errors printed out in terminal:
import logging
logging.basicConfig()
 

class StateEstimation1D(object):
    
    def __init__(self, hostname, ir_throttled=False, imu_throttled=False):
        self.hostname = hostname
        
        # self.ready_to_filter is False until we get initial measurements in
        # order to be able to initialize the filter's state vector x and
        # covariance matrix P.
        self.ready_to_filter = False
        self.printed_filter_start_notice = False
        self.got_ir = False
        
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
        '''
        Initialize ROS-related objects, e.g., the connection to the ROS master,
        the subscribers, etc.
        '''
        self.node_name = 'state_estimator_ukf_1d_roslib_offboard'
        print 'Initializing {} connection to ROS master running on {}...'.format(
                self.node_name, self.hostname)
        self.ros = roslibpy.Ros(host=self.hostname, port=9090)
        
        self.ir_sub = roslibpy.Topic(ros=self.ros, name=self.ir_topic_str, message_type='sensor_msgs/Range')
        self.imu_sub = roslibpy.Topic(ros=self.ros, name=self.imu_topic_str, message_type='sensor_msgs/Imu')
        
        self.state_pub = roslibpy.Topic(ros=self.ros, name='/pidrone/state', message_type='pidrone_pkg/State')
        
    def start_subscribers(self):
        print 'Starting subscribers...'
        self.ir_sub.subscribe(self.ir_data_callback)
        self.imu_sub.subscribe(self.imu_data_callback)
        print 'Subscribers started'
        
    def initialize_ukf(self):
        '''
        Initialize the parameters of the Unscented Kalman Filter (UKF) that is
        used to estimate the state of the drone.
        '''
        
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
        self.ukf.P = np.diag([0.1, 0.2])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune appropriately. Currently just a guess
        self.ukf.Q = np.diag([0.01, 1.0])*0.005
        
        # Initialize the measurement covariance matrix R
        # IR slant range variance (m^2), determined experimentally in a static
        # setup with mean range around 0.335 m:
        self.ukf.R = np.array([2.2221e-05])
        
    def update_input_time(self, msg):
        '''
        Update the time at which we have received the most recent input, based
        on the timestamp in the header of a ROS message
        
        msg : a ROS message that includes a header with a timestamp that
              indicates the time at which the respective input was originally
              recorded
        '''
        self.last_time_secs = msg['header']['stamp']['secs']
        self.last_time_nsecs = msg['header']['stamp']['nsecs']
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
        self.last_time_secs = msg['header']['stamp']['secs']
        self.last_time_nsecs = msg['header']['stamp']['nsecs']
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
        self.last_control_input = np.array([data['linear_acceleration']['z']])
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
            measurement_z = np.array([data['range']])
            # print 'Prior:', self.ukf.x
            # print 'Measurement:', measurement_z[0]
            self.ukf.update(measurement_z)
            # print 'Posterior:', self.ukf.x
            # print 'Kalman Gain:', self.ukf.K
            # print
            self.publish_current_state()
        else:
            self.initialize_input_time(data)
            # Got a raw slant range reading, so update the initial state
            # vector of the UKF
            self.ukf.x[0] = data['range']
            self.ukf.x[1] = 0.0 # initialize velocity as 0 m/s
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
        # Prepare covariance matrices
        # 36-element array, in a row-major order, according to ROS msg docs
        pose_cov_mat = np.full((36,), np.nan)
        twist_cov_mat = np.full((36,), np.nan)
        pose_cov_mat[14] = self.ukf.P[0, 0] # z variance
        twist_cov_mat[14] = self.ukf.P[1, 1] # z velocity variance
        
        state_msg_dict = {'header' :
                            {
                            'stamp' :
                                {
                                'secs' : self.last_time_secs,
                                'nsecs' : self.last_time_nsecs
                                },
                            'frame_id' : 'global'
                            },
                          'pose_with_covariance' :
                            {
                            'pose' :
                                {
                                'position' :
                                    {
                                    'x' : np.nan,
                                    'y' : np.nan,
                                    'z' : self.ukf.x[0]
                                    },
                                'orientation' :
                                    {
                                    'x' : np.nan,
                                    'y' : np.nan,
                                    'z' : np.nan,
                                    'w' : np.nan
                                    }
                                },
                            'covariance' : list(pose_cov_mat)
                            },
                          'twist_with_covariance' :
                            {
                            'twist' :
                                {
                                'linear' :
                                    {
                                    'x' : np.nan,
                                    'y' : np.nan,
                                    'z' : self.ukf.x[1]
                                    },
                                'angular' :
                                    {
                                    'x' : np.nan,
                                    'y' : np.nan,
                                    'z' : np.nan
                                    }
                                },
                            'covariance' : list(twist_cov_mat)
                            }
                         }
        
        self.state_pub.publish(roslibpy.Message(state_msg_dict))

    def state_transition_function(self, x, dt, u):
        '''
        The state transition function to compute the prior in the prediction
        step, propagating the state to the next time step.
        
        x : current state. A NumPy array
        dt : time step. A float
        u : control input. A NumPy array
        '''
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
        '''
        Transform the state x into measurement space. In this simple model, we
        assume that the range measurement corresponds exactly to position along
        the z-axis, as we are assuming there is no pitch and roll.
        
        x : current state. A NumPy array
        '''
        # Measurement update matrix H
        H = np.array([[1, 0]])
        hx_output = np.dot(H, x)
        return hx_output
        
    def run(self):
        self.ros.on_ready(self.start_subscribers, run_in_thread=True)
        self.ros.run_forever()
        
        
def main():
    parser = argparse.ArgumentParser(description=('Estimate the drone\'s state '
                                     'with a UKF in one spatial dimension'))
    # Arguments to determine if the throttle command is being used. E.g.:
    #   rosrun topic_tools throttle messages /pidrone/infrared 40.0
    parser.add_argument('--ir_throttled', action='store_true',
            help=('Use throttled infrared topic /pidrone/infrared_throttle'))
    parser.add_argument('--imu_throttled', action='store_true',
            help=('Use throttled IMU topic /pidrone/imu_throttle'))
    args = parser.parse_args()
    se = StateEstimation1D(ir_throttled=args.ir_throttled,
                         imu_throttled=args.imu_throttled,
                         hostname='tdrone-blue')
    try:
        # Wait until node is halted
        se.run()
    finally:
        # Upon termination of this script, print out a helpful message
        print '{} node terminating.'.format(se.node_name)
        print 'Most recent state vector:'
        print se.ukf.x
        
if __name__ == '__main__':
    main()
        
    