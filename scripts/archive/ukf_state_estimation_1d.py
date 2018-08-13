from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
import math

class DroneStateEstimation1D(object):
    
    def __init__(self):
        # TODO: Modify these sigma point parameters as necessary
        sigma_points = MerweScaledSigmaPoints(n=2, alpha=0.1, beta=2., kappa=0.)
        # Note that dt will get updated dynamically as sensor data comes in
        self.ukf = UnscentedKalmanFilter(dim_x=2, dim_z=1, dt=1.0,
                                         hx=self.measurement_function,
                                         fx=self.state_transition_function,
                                         points=sigma_points)
        
        # Initialize the state variables [z, z_vel]
        self.ukf.x = np.array([0.0, 0.0])
        # Initialize state covariance matrix P:
        # TODO: Tune these initial values as appropriate. Currently these are just
        #       guesses
        self.ukf.P = np.diag([0.1, 0.1])
        
        # Initialize the process noise covariance matrix Q:
        # TODO: Tune as appropriate. Currently just a guess
        # TODO: Consider using Q_discrete_white_noise() from filterpy.common
        self.ukf.Q = np.eye(2)*0.01
        
        # Initialize the measurement covariance matrix R:
        # Using np.diag makes the covariances 0
        # TODO: Tune as appropriate. Currently just a guess
        self.ukf.R = np.array([[1.5]])
        
        self.last_measurement_time = None
        self.got_first_measurement = False
        # Time in seconds between consecutive measurements
        self.dt_measurement = None

    def state_transition_function(self, x, dt):
        '''
        State transition function given the current state. No control input
        taken into consideration, as currently it appears that the IMU does not
        output linear accelerations.
        x : current state. A NumPy array
        dt : time step. A float
        '''
        state_transition_matrix = np.array([[1, dt],
                                            [0, 1]])
        output = np.dot(state_transition_matrix, x)
        return output
        
    def measurement_function(self, x):
        '''
        Transform the state x into measurement space
        x : current state. A NumPy array
        '''
        measurement_update_matrix = np.array([[1, 0]])
        output = np.dot(measurement_update_matrix, x)
        return output
    
