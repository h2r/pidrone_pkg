from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
import math

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
        # Trust IMU roll and pitch measurements
        # TODO: Implement reading of the IMU roll and pitch (given in degrees)
        self.roll = None
        self.pitch = None
        # TODO: Incorporate camera data to estimate yaw, if possible. Or, get an
        # IMU that has a gyroscope with a magnetometer included for yaw sensing
        self.yaw = 0.0
        
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
        self.ukf.Q = np.eye(state_vector_dim)*0.1
        
        # Initialize the measurement covariance matrix R:
        # Using np.diag makes the covariances 0
        # TODO: Tune as necessary. Currently just a guess
        self.ukf.R = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 1, 1, 1, 1, 1, 1])
        
        self.last_measurement_time = None
        self.got_first_measurement = False
        # Time in seconds between consecutive measurements
        self.dt_measurement = None

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

    def state_transition_function(self, x, dt):
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
        x_output = np.dot(F, x)
        print x_output
        return x_output
        
    def measurement_function(self, x):
        '''
        x : current state. A NumPy array
        '''
        # TODO: Should the roll and pitch values from the prior state estimate be used, rather than the raw measurement?
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
        print hx_output
        return hx_output
        
    # TODO: Implement "sensor fusion" for data coming in at different rates,
    # which might involve modifying the measurement function dynamically. There
    # might be specific measurement functions for the IR sensor and for the
    # camera data
