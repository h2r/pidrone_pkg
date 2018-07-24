#!/usr/bin/env python

# ROS imports
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from pidrone_pkg.msg import State
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, TwistStamped
from std_msgs.msg import Header

# UKF imports
# The matplotlib imports and the matplotlib.use('Pdf') line make it so that the
# UKF code that imports matplotlib does not complain. Essentially, the
# use('Pdf') call allows a plot to be created without a window (allows it to run
# through ssh)
import matplotlib
matplotlib.use('Pdf')
import matplotlib.pyplot as plt
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

# Other imports
import numpy as np


class StateEstimation(object):
    '''
    Class that estimates the state of the drone using an Unscented Kalman Filter
    (UKF) applied to raw sensor data.
    '''
    # TODO: Make a reference to the UKF math document that is being written up,
    #       once it is in a complete enough state and can be placed in a shared
    #       location.
    
    def __init__(self):
        # self.ready_to_filter is False until we get initial measurements in
        # order to be able to initialize the filter's state vector x and
        # covariance matrix P.
        self.num_bad_updates = 0
        self.ready_to_filter = False
        self.printed_filter_start_notice = False
        self.got_imu = False
        self.got_optical_flow = False
        self.got_ir = False
        
        self.num_complete_ir = 0
        self.num_complete_imu = 0
        self.num_complete_optical_flow = 0
        
        self.initialize_ros()
        # TODO: Consider handling issue of callbacks referring to objects not
        #       yet initialized, such as self.ukf?
        self.initialize_ukf()
        
        # The last time that we received an input and formed a prediction with
        # the state transition function
        self.last_state_transition_time = None
        
        # Time in seconds between consecutive state transitions, dictated by
        # when the inputs come in
        self.dt = None
        
        # FilterPy requires the predict() method to be called before the first
        # call to update(), so ensure that we have computed the first prior
        #self.computed_first_prior = False
        
        # Initialize the last control input as 0 m/s^2 along each axis in the
        # body frame
        self.last_control_input = np.array([0.0, 0.0, 0.0])
        
    def initialize_ros(self):
        '''
        Initialize ROS-related objects, e.g., the node, subscribers, etc.
        '''
        print 'Initializing state_estimation node...'
        rospy.init_node('state_estimation')
        
        # Subscribe to topics to which the drone publishes in order to get raw
        # data from sensors, which we can then filter
        rospy.Subscriber('/pidrone/imu', Imu, self.imu_data_callback)
        rospy.Subscriber('/pidrone/plane_err', TwistStamped,
                         self.optical_flow_data_callback)
        rospy.Subscriber('/pidrone/infrared', Range, self.ir_data_callback) # TODO: infrared or raw_infrared?
        
        # Create the publisher to publish state estimates
        self.state_pub = rospy.Publisher('/pidrone/state', State, queue_size=1,
                                        tcp_nodelay=False)
        
    def initialize_ukf(self):
        '''
        Initialize the parameters of the Unscented Kalman Filter (UKF) that is
        used to estimate the state of the drone.
        '''
        
        # Number of state variables being tracked
        self.state_vector_dim = 12
        # The state vector consists of the following column vector.
        # Note that FilterPy initializes the state vector with zeros.
        # [[x],
        #  [y],
        #  [z],
        #  [x_vel],
        #  [y_vel],
        #  [z_vel],
        #  [roll],
        #  [pitch],
        #  [yaw],
        #  [roll_vel],
        #  [pitch_vel],
        #  [yaw_vel]]
        
        # Number of measurement variables that the drone receives
        self.measurement_vector_dim = 7
        # The measurement variables consist of the following column vector:
        # [[x_vel],
        #  [y_vel],
        #  [yaw_vel],
        #  [slant_range],
        #  [roll],
        #  [pitch],
        #  [yaw]]
        
        # Function to generate sigma points for the UKF
        # TODO: Modify these sigma point parameters appropriately. Currently
        #       just guesses
        sigma_points = MerweScaledSigmaPoints(n=self.state_vector_dim,
                                              alpha=0.1,
                                              beta=2.0,
                                              kappa=-9.0)
        
        # Create the UKF object
        # Note that dt will get updated dynamically as sensor data comes in,
        # as will the measurement function, since measurements come in at
        # distinct rates
        self.ukf = UnscentedKalmanFilter(dim_x=self.state_vector_dim,
                                         dim_z=self.measurement_vector_dim,
                                         dt=1.0,
                                         hx=self.measurement_function,
                                         fx=self.state_transition_function,
                                         points=sigma_points)
                                         #residual_x=self.residual_x_account_for_angles)
        self.initialize_ukf_matrices()

    def initialize_ukf_matrices(self):
        '''
        Initialize the covariance matrices of the UKF
        '''
        # Initialize state covariance matrix P:
        # TODO: Tune these initial values appropriately. Currently these are
        #       just guesses
        self.ukf.P = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.05, 0.05, 0.05, 0.1, 0.1, 0.1])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune appropriately. Currently just a guess
        # To consider: Changing scale factor by too much could lead to the
        # following error:
        #   "numpy.linalg.linalg.LinAlgError: 3-th leading minor not positive
        #    definite"
        self.ukf.Q = np.eye(self.state_vector_dim)*0.0001
        #self.ukf.Q = np.diag([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0])*0.001
        
        # Initialize the measurement covariance matrix R for each discrete
        # asynchronous measurement input:
        # Using np.diag makes the covariances 0
        
        # IR slant range variance (m^2), determined experimentally in a static
        # setup with mean range around 0.335 m:
        self.measurement_cov_ir = np.array([2.2221e-05])
        # TODO: Tune the following variances appropriately. Currently just
        #       guesses
        # Optical flow variance:
        self.measurement_cov_optical_flow = np.diag([0.01, 0.01, 0.01])
        # Roll-Pitch-Yaw variance:
        self.measurement_cov_rpy = np.diag([0.1, 0.1, 0.1])
        
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
        Compute the prior for the UKF, based on the current state, a control
        input, and a time step.
        '''
        self.ukf.predict(dt=self.dt, u=self.last_control_input)
        #self.computed_first_prior = True
        
    def print_notice_if_first(self):
        if not self.printed_filter_start_notice:
            print 'Starting filter'
            self.printed_filter_start_notice = True
        
    def imu_data_callback(self, data):
        '''
        Handle the receipt of an Imu message, which includes linear
        accelerations (m/s^2) to be treated as a control input in the UKF and
        orientation to be treated as measurement inputs.
        
        This method PREDICTS with a control input and UPDATES.
        '''
        euler_angles = tf.transformations.euler_from_quaternion(
                                                           [data.orientation.x,
                                                            data.orientation.y,
                                                            data.orientation.z,
                                                            data.orientation.w])
        roll = euler_angles[0]
        pitch = euler_angles[1]
        yaw = euler_angles[2]
        if self.ready_to_filter:
            self.print_notice_if_first()
            self.update_input_time(data)
            self.last_control_input = np.array([data.linear_acceleration.x,
                                                data.linear_acceleration.y,
                                                data.linear_acceleration.z])
            #self.ukf.P = (self.ukf.P + self.ukf.P.T)/2.0 # TODO: Look into
            self.ukf_predict()
            
            # Now that a prediction has been formed, perform a measurement
            # update with the roll-pitch-yaw data in the Imu message
            measurement_z = np.array([roll,
                                      pitch,
                                      yaw])
            # Ensure that we are computing the residual for angles
            #self.ukf.residual_z = self.angle_residual
            # Actually, looks like euler_from_quaternion can return negative
            # angles in radians
            self.ukf.residual_z = np.subtract
            # TODO: Look into the range of angles returned by
            #       euler_from_quaternion. As there are negatives, it would seem
            #       possible that the range be between -pi and pi radians...
            #self.ukf.P = (self.ukf.P + self.ukf.P.T)/2.0 # TODO: Look into
            self.ukf.update(measurement_z,
                            hx=self.measurement_function_rpy,
                            R=self.measurement_cov_rpy)
            self.publish_current_state()
        else:
            self.initialize_input_time(data)
            # Update the initial state vector of the UKF
            self.ukf.x[6] = roll
            self.ukf.x[7] = pitch
            self.ukf.x[8] = yaw
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[6, 6] = self.measurement_cov_rpy[0, 0]
            self.ukf.P[7, 7] = self.measurement_cov_rpy[1, 1]
            self.ukf.P[8, 8] = self.measurement_cov_rpy[2, 2]
            self.got_imu = True
            self.check_if_ready_to_filter()
        self.num_complete_imu += 1
        print '--IMU:', self.num_complete_imu
                        
    def optical_flow_data_callback(self, data):
        '''
        Handle the receipt of a TwistStamped message from optical flow.
        The message includes:
            - x velocity (m/s)
            - y velocity (m/s)
            - yaw velocity (rad/s)
        
        This method PREDICTS with the most recent control input and UPDATES.
        '''
        if self.ready_to_filter:
            self.print_notice_if_first()
            self.update_input_time(data)
            #self.ukf.P = (self.ukf.P + self.ukf.P.T)/2.0 # TODO: Look into
            self.ukf_predict()
            
            # Now that a prediction has been formed to bring the current prior
            # state estimate to the same point in time as the measurement,
            # perform a measurement update with x velocity, y velocity, and yaw
            # velocity data in the TwistStamped message
            # TODO: Verify the units of these velocities that are being
            #       published
            measurement_z = np.array([data.twist.linear.x, # x velocity
                                      data.twist.linear.y, # y velocity
                                      data.twist.angular.z]) # yaw velocity
            # Ensure that we are using subtraction to compute the residual
            self.ukf.residual_z = np.subtract
            #self.ukf.P = (self.ukf.P + self.ukf.P.T)/2.0 # TODO: Look into
            self.ukf.update(measurement_z,
                            hx=self.measurement_function_optical_flow,
                            R=self.measurement_cov_optical_flow)
            self.publish_current_state()
        else:
            self.initialize_input_time(data)
            # Update the initial state vector of the UKF
            self.ukf.x[3] = data.twist.linear.x # x velocity
            self.ukf.x[4] = data.twist.linear.y # y velocity
            self.ukf.x[11] = data.twist.angular.z # yaw velocity
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[3, 3] = self.measurement_cov_optical_flow[0, 0]
            self.ukf.P[4, 4] = self.measurement_cov_optical_flow[1, 1]
            self.ukf.P[11, 11] = self.measurement_cov_optical_flow[2, 2]
            self.got_optical_flow = True
            self.check_if_ready_to_filter()
        self.num_complete_optical_flow += 1
        print '--OPTICAL FLOW:', self.num_complete_optical_flow
                        
    def ir_data_callback(self, data):
        '''
        Handle the receipt of a Range message from the IR sensor.
        
        This method PREDICTS with the most recent control input and UPDATES.
        '''
        #print 'P diag:', np.diagonal(self.ukf.P)
        if self.ready_to_filter:
            self.print_notice_if_first()
            self.update_input_time(data)
            #self.ukf.P = (self.ukf.P + self.ukf.P.T)/2.0 # TODO: Look into
            print 'BEFORE PREDICT Z:', self.ukf.x[2]
            #print 'BEFORE PREDICT Z VEL:', self.ukf.x[5]
            self.ukf_predict()
                        
            # Now that a prediction has been formed to bring the current prior
            # state estimate to the same point in time as the measurement,
            # perform a measurement update with the slant range reading
            measurement_z = np.array([data.range])
            # Ensure that we are using subtraction to compute the residual
            self.ukf.residual_z = np.subtract
            #self.ukf.P = (self.ukf.P + self.ukf.P.T)/2.0 # TODO: Look into
            print 'BEFORE UPDATE Z:', self.ukf.x[2]
            #print 'BEFORE UPDATE Z VEL:', self.ukf.x[5]
            print 'Raw slant range:', measurement_z[0]
            self.ukf.update(measurement_z,
                            hx=self.measurement_function_ir,
                            R=self.measurement_cov_ir)
            print 'AFTER UPDATE Z:', self.ukf.x[2]
            #print 'AFTER UPDATE Z VEL:', self.ukf.x[5]
            #print 'Z-Z_vel COVARIANCE:', self.ukf.P[2, 5]
            print 'KALMAN GAIN:', self.ukf.K
            print 'RESIDUAL:', self.ukf.y
            print 'KALMAN GAIN DOT RESIDUAL:', np.dot(self.ukf.K, self.ukf.y)
            print
            if not ((measurement_z[0] <= self.ukf.x[2] <= self.ukf.x_prior[2]) or
                    (measurement_z[0] >= self.ukf.x[2] >= self.ukf.x_prior[2])):
                self.num_bad_updates += 1
                print self.num_bad_updates, 'BAD UPDATE...\n\n\n\n\n\n\n'
            self.publish_current_state()
        else:
            self.initialize_input_time(data)
            # Got a raw slant range reading, so update the initial state
            # vector of the UKF
            self.ukf.x[2] = data.range
            # Update the state covariance matrix to reflect estimated
            # measurement error. Variance of the measurement -> variance of
            # the corresponding state variable
            self.ukf.P[2, 2] = self.measurement_cov_ir[0]
            self.got_ir = True
            self.check_if_ready_to_filter()
        self.num_complete_ir += 1
        print '--IR:', self.num_complete_ir
        print
            
    def check_if_ready_to_filter(self):
        self.ready_to_filter = (self.got_imu and self.got_optical_flow and
                                self.got_ir)
                        
    def publish_current_state(self):
        '''
        Publish the current state estimate and covariance from the UKF. This is
        a State message containing:
            - PoseWithCovarianceStamped
            - TwistWithCovarianceStamped
        '''
        header = Header()
        header.stamp.secs = self.last_time_secs
        header.stamp.nsecs = self.last_time_nsecs
        header.frame_id = 'global'
        
        pose_with_cov = PoseWithCovarianceStamped()
        twist_with_cov = TwistWithCovarianceStamped()
        pose_with_cov.header = header
        twist_with_cov.header = header
        
        quaternion = tf.transformations.quaternion_from_euler(self.ukf.x[6],
                                                              self.ukf.x[7],
                                                              self.ukf.x[8])
        
        # Get the current state estimate from self.ukf.x
        pose_with_cov.pose.pose.position.x = self.ukf.x[0]
        pose_with_cov.pose.pose.position.y = self.ukf.x[1]
        pose_with_cov.pose.pose.position.z = self.ukf.x[2]
        twist_with_cov.twist.twist.linear.x = self.ukf.x[3]
        twist_with_cov.twist.twist.linear.y = self.ukf.x[4]
        twist_with_cov.twist.twist.linear.z = self.ukf.x[5]
        pose_with_cov.pose.pose.orientation.x = quaternion[0]
        pose_with_cov.pose.pose.orientation.y = quaternion[1]
        pose_with_cov.pose.pose.orientation.z = quaternion[2]
        pose_with_cov.pose.pose.orientation.w = quaternion[3]
        
        # TODO: Look into RPY velocities versus angular velocities about x, y, z axes?
        # For the time being, using Euler rates:
        twist_with_cov.twist.twist.angular.x = self.ukf.x[9]    # roll rate
        twist_with_cov.twist.twist.angular.y = self.ukf.x[10]   # pitch rate
        twist_with_cov.twist.twist.angular.z = self.ukf.x[11]   # yaw rate
        
        # Extract the relevant covariances from self.ukf.P
        pose_with_cov.pose.covariance = np.concatenate(
                     (np.concatenate((self.ukf.P[0:3, 0:3],
                                      self.ukf.P[0:3, 6:9]), axis=1),
                      np.concatenate((self.ukf.P[6:9, 0:3],
                                      self.ukf.P[6:9, 6:9]), axis=1)), axis=0)
        twist_with_cov.twist.covariance = np.concatenate(
                     (np.concatenate((self.ukf.P[3:6, 3:6],
                                      self.ukf.P[3:6, 9:12]), axis=1),
                      np.concatenate((self.ukf.P[9:12, 3:6],
                                      self.ukf.P[9:12, 9:12]), axis=1)), axis=0)
        
        state_msg = State()
        state_msg.pose_with_covariance_stamped = pose_with_cov
        state_msg.twist_with_covariance_stamped = twist_with_cov
        self.state_pub.publish(state_msg)

    def apply_rotation_matrix(self, original_matrix):
        '''
        Rotate a matrix from the drone's body frame to the global frame
        '''
        # TODO: See about using tf and potentially applying rotations with
        #       quaternions
        # TODO: Should we use the raw roll, pitch, and yaw values that come in
        #       at the same time step as the linear accelerations?
        phi = self.ukf.x[6]     # roll in radians
        theta = self.ukf.x[7]   # pitch in radians
        psi = self.ukf.x[8]     # yaw in radians
        # Set up the rotation matrix
        rotation_matrix = np.array(
               [[np.cos(theta)*np.cos(psi),
                 np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi),
                 np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi)],
                [np.cos(theta)*np.sin(psi),
                 np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi),
                 np.cos(theta)*np.sin(phi)],
                [-np.sin(theta),
                 np.cos(theta)*np.sin(phi),
                 np.cos(theta)*np.cos(phi)]])
        # Apply the rotation matrix
        return np.dot(rotation_matrix, original_matrix)

    def state_transition_function(self, x, dt, u):
        '''
        The state transition function to compute the prior in the prediction
        step, propagating the state to the next time step.
        
        x : current state. A NumPy array
        dt : time step. A float
        '''
        F = np.eye(12)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        F[6, 9] = dt
        F[7, 10] = dt
        F[8, 11] = dt
                
        # Compute the change from the control input
        accelerations_global_frame = self.apply_rotation_matrix(u)
        velocities_global_frame = accelerations_global_frame * dt
        change_from_control_input = np.array([0,
                                              0,
                                              0,
                                              velocities_global_frame[0],
                                              velocities_global_frame[1],
                                              velocities_global_frame[2],
                                              0,
                                              0,
                                              0,
                                              0,
                                              0,
                                              0])
        x_output = np.dot(F, x) + change_from_control_input
        #x_output = self.correct_fringe_angles(x_output)
        return x_output
        
    def angle_residual(self, a, b):
        '''
        Compute the residual for the measurement update step that includes only
        angles (roll, pitch, and yaw). Handle the fringe angle case. For
        example, 358 degrees - 3 degrees should give out -5 degrees and not
        355 degrees. (Example given in degrees, but in the implementation angles
        are given in radians)
        
        a : a NumPy array with shape (3,), the shape of the measurement vector
            with just roll, pitch, and yaw
        b : a NumPy array with shape (3,), the shape of the measurement vector
            with just roll, pitch, and yaw
        '''
        output_residual = np.empty_like(a)
        for num, angle1 in enumerate(a):
            angle2 = b[num]
            residual = angle1 - angle2
            if residual > np.pi:
                residual -= 2*np.pi
            elif residual < -np.pi:
                residual += 2*np.pi
            output_residual[num] = residual
        return output_residual
        
    def residual_x_account_for_angles(self, a, b):
        '''
        Compute the residual for the state vector x. For example, this is used
        in the unscented transform.
        
        a : a NumPy array with shape (12,), the shape of the state vector
        b : a NumPy array with shape (12,), the shape of the state vector
        '''
        output_residual = np.empty_like(a)
        for num, state_var1 in enumerate(a):
            state_var2 = b[num]
            residual = state_var1 - state_var2
            if 6 <= num <= 8:
                # Then the current state variable is an angle
                if residual > np.pi:
                    residual -= 2*np.pi
                elif residual < -np.pi:
                    residual += 2*np.pi
            output_residual[num] = residual
        return output_residual
        
    def correct_fringe_angles(self, x_in_transition):
        '''
        Check if the state transition involves fringe angles, i.e., if an
        orientation angle transitions from 359 degrees to 362 degrees, then the
        new angle should read 2 degrees. Likewise, a transition from 2 degrees
        to -3 degrees should read 357 degrees. (Examples given in degrees, but
        in the implementation angles are given in radians) This is just a matter
        of applying the modulo operator to each transitioned angle.
        '''
        x_in_transition[6] = np.mod(x_in_transition[6], 2*np.pi) # roll angle
        x_in_transition[7] = np.mod(x_in_transition[7], 2*np.pi) # pitch angle
        x_in_transition[8] = np.mod(x_in_transition[8], 2*np.pi) # yaw angle
        return x_in_transition
        
    def measurement_function(self, x):
        '''
        The "complete" measurement function if the measurement vector z were to
        be comprised of all measurement variables at any given timestep. This
        function does not actually get called, since measurements come in at
        distinct rates, but it is passed into the UKF object as the default
        measurement function.
        
        x : current state. A NumPy array
        '''
        # Roll and pitch values from the prior state estimate
        phi = x[6] # roll in radians
        theta = x[7] # pitch in radians
        # Conversion from altitude (alt) to slant range (r)
        alt_to_r = 1/(np.cos(theta)*np.cos(phi))
        H = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                      [0, 0, alt_to_r, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]])
        hx_output = np.dot(H, x)
        return hx_output
        
    def measurement_function_ir(self, x):
        '''
        For use when the measurement vector z is just the slant range reading
        from the IR sensor
        
        x : current state. A NumPy array
        '''
        # Roll and pitch values from the prior state estimate
        phi = x[6] # roll in radians
        theta = x[7] # pitch in radians
        # Conversion from altitude (alt) to slant range (r)
        alt_to_r = 1/(np.cos(theta)*np.cos(phi))
        H = np.array([[0, 0, alt_to_r, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        hx_output = np.dot(H, x)
        return hx_output
        
    def measurement_function_optical_flow(self, x):
        '''
        For use when the measurement vector z is comprised of x-velocity,
        y-velocity, and yaw velocity from the camera's optical flow
        
        x : current state. A NumPy array
        '''
        H = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
        hx_output = np.dot(H, x)
        return hx_output
        
    def measurement_function_rpy(self, x):
        '''
        For use when the measurement vector z is comprised of roll, pitch, and
        yaw readings from the IMU
        
        x : current state. A NumPy array
        '''
        H = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]])
        hx_output = np.dot(H, x)
        return hx_output
        
        
def main():
    se = StateEstimation()
    try:
        # Wait until node is halted
        rospy.spin()
    finally:
        # Upon termination of this script, print out a helpful message
        print 'State estimation node terminating.'
        print 'Most recent state vector:'
        print se.ukf.x
        print 'NUM BAD UPDATES:', se.num_bad_updates
        # print 'Most recent state covariance matrix:'
        # print se.ukf.P
        
if __name__ == '__main__':
    main()
