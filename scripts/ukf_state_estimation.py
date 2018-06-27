from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
import math

class DroneStateEstimation(object):
    
    def __init__(self):
        # TODO: Modify these sigma point parameters as necessary
        sigma_points = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2., kappa=0.)
        # Note that dt will get updated dynamically as sensor data comes in
        self.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=4, dt=1.0,
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
        
        # Initialize the state variables:
        self.ukf.x = np.array([[0.0],  # x-velocity
                               [0.0],  # y-velocity
                               [0.0],  # z-velocity
                               [0.0]]) # yaw-velocity
        # Initialize state covariance matrix P:
        # TODO: Tune these initial values as necessary. Currently these are just
        #       guesses
        self.ukf.P = np.diag([0.1, 0.1, 0.1, 0.2])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune as necessary. Currently just a guess
        self.ukf.Q = np.eye(4)*0.05
        
        # Initialize the measurement covariance matrix R:
        # Using np.diag makes the covariances 0
        # TODO: Tune as necessary. Currently just a guess
        self.ukf.R = np.diag([0.1, 0.1, 0.2, 0.01])
        
        # Keep track of control input u
        self.most_recent_control_input = np.array([[0.0],  # accel x
                                                   [0.0],  # accel y
                                                   [0.0]]) # accel z

    def apply_rotation_matrix(self, original_matrix):
        '''
        Rotate a matrix from the drone's body frame to the global frame
        '''
        # Convert Euler angles from degrees to radians
        phi = np.deg2rad(self.roll)
        theta = np.deg2rad(self.pitch)
        psi = np.deg2rad(self.yaw)
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

    def state_transition_function(self, x, u, dt):
        '''
        x : current state. A NumPy column vector
        u : control inputs. A NumPy column vector
        dt : time step. A float
        '''
        x_output = np.empty_like(x)
        accelerations_global_frame = self.apply_rotation_matrix(u)
        velocities_global_frame = accelerations_global_frame * dt
        for i in range(3):
            x_output[i] = x[i] + velocities_global_frame[i]
        x_output[3] = x[3]
        return x_output
        
    def measurement_function(self, x, dt):
        '''
        x : current state. A NumPy column vector
        dt : time step. A float
        '''
        # Convert Euler angles from degrees to radians
        phi = np.deg2rad(self.roll)
        theta = np.deg2rad(self.pitch)
        return np.dot([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 0, 1],
                       [0, 0, (dt/(np.cos(theta)*np.cos(phi))), 0]], x)
