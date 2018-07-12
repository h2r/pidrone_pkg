from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np

class DroneStateEstimation(object):
    
    def __init__(self):
        state_vector_dim = 12
        measurement_vector_dim = 7
        # TODO: Modify these sigma point parameters as necessary
        sigma_points = MerweScaledSigmaPoints(n=state_vector_dim, alpha=0.1,
                                              beta=2., kappa=0.)
        # Note that dt will get updated dynamically as sensor data comes in
        self.ukf = UnscentedKalmanFilter(dim_x=state_vector_dim,
                                         dim_z=measurement_vector_dim, dt=1.0,
                                         hx=self.measurement_function,
                                         fx=self.state_transition_function,
                                         points=sigma_points)
        
        # FilterPy initializes the state vector with zeros
        # state = [[x],
        #          [y],
        #          [z],
        #          [x_vel],
        #          [y_vel],
        #          [z_vel],
        #          [roll],
        #          [pitch],
        #          [yaw],
        #          [roll_vel],
        #          [pitch_vel],
        #          [yaw_vel]]

        # Initialize state covariance matrix P:
        # TODO: Tune these initial values as necessary. Currently these are just
        #       guesses
        self.ukf.P = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 1, 1, 1, 1, 1, 1])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune as necessary. Currently just a guess
        # To consider: Changing scale factor by too much could lead to the
        # following error:
        # numpy.linalg.linalg.LinAlgError: 3-th leading minor not positive definite
        self.ukf.Q = np.eye(state_vector_dim)*0.001
        
        # Initialize the measurement covariance matrix R for each discrete
        # asynchronous measurement input:
        # Using np.diag makes the covariances 0
        # TODO: Tune as necessary. Currently just a guess
        #self.ukf.R = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 1, 1, 1, 1, 1, 1])
        #self.R_complete = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        #self.ukf.R = self.R_complete.copy()
        
        # IR slant range variance:
        self.measurement_cov_ir = np.array([0.1])
        # Optical flow variance:
        self.measurement_cov_optical_flow = np.diag([0.1, 0.1, 0.1])
        # Roll-Pitch-Yaw variance:
        self.measurement_cov_rpy = np.diag([0.1, 0.1, 0.1])
        
        # The last time that we received a control input
        self.last_state_transition_time = None
        
        # Time in seconds between consecutive state transitions, dictated by
        # when the control inputs come in
        self.dt = None
        
        # FilterPy requires the predict() method to be called before the first
        # call to update(), so ensure that we have computed the first prior
        self.computed_first_prior = False
        
        # Initialize the last control input as 0 m/s^2 along each axis in the
        # body frame
        self.last_control_input = np.array([[0], [0], [0]])

    def apply_rotation_matrix(self, original_matrix):
        '''
        Rotate a matrix from the drone's body frame to the global frame
        '''
        # Convert Euler angles from degrees to radians
        phi = np.deg2rad(self.ukf.x[6])     # roll
        theta = np.deg2rad(self.ukf.x[7])   # pitch
        psi = np.deg2rad(self.ukf.x[8])     # yaw
        # Set up the rotation matrix
        rotation_matrix = np.array([[np.cos(theta)*np.cos(psi),
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
        return x_output
        
    def measurement_function(self, x):
        '''
        The "complete" measurement function if the measurement vector z were to
        be comprised of all measurement variables at any given timestep
        
        x : current state. A NumPy array
        '''
        # Roll and pitch values from the prior state estimate
        roll_deg = x[6]
        pitch_deg = x[7]
        # Convert Euler angles from degrees to radians
        phi = np.deg2rad(roll_deg)
        theta = np.deg2rad(pitch_deg)
        H = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                      [0, 0, (1/(np.cos(theta)*np.cos(phi))), 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
        roll_deg = x[6]
        pitch_deg = x[7]
        # Convert Euler angles from degrees to radians
        phi = np.deg2rad(roll_deg)
        theta = np.deg2rad(pitch_deg)
        H = np.array([[0, 0, (1/(np.cos(theta)*np.cos(phi))), 0, 0, 0, 0, 0, 0, 0, 0, 0]])
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
        
    # TODO: Implement "sensor fusion" for data coming in at different rates,
    # which might involve modifying the measurement function dynamically. There
    # might be specific measurement functions for the IR sensor and for the
    # camera data
