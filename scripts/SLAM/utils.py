"""""
Support code for fast slam
"""""

from __future__ import division
import re
import numpy as np
from numpy import dot
from numpy import identity
import math
import sys

max_float = sys.float_info.max
debug = False


def calculate_jacobian(robot_position, landmark_pos): 

    """ Calculates the Jacobian matrix of a particular observatoin

    The Jacobian matrix is a linearization of the map from (x, y) to 
    (distance, bearing) coordinates

    args: 
        robot_position: the (x, y) coordinates of the robot's position
            Note: will work fine if given (x, y, phi)
        landmark_pos: the (x, y) position of a particular landmark

    returns: 
        the Jacobian matrix of the robot_position, with respect to 
        the landmark_pos
    """

    t = robot_position 
    j = landmark_pos
    q = (j[0] - t[0])**2 + (j[1] - t[1])**2 

    jacobian = np.array([ 
        [ (j[0] - t[0])/math.sqrt(q),   (j[1] - t[1])/math.sqrt(q)], 
        [ -(j[1] - t[1])/q,              (j[0] - t[0])/q ]])
    return jacobian


def compute_measurement_covariance(jacobian, oldCovariance, sigmaObservation):

    """ Compute the measurement covariance which is used in calculating the probability
        of a new measurement given the existing measurements (the likelihood of correspondence).

        args:
            jacobian: The Jacobian matrix of the newly sampled robot position with
            respect to the existing landmark position

            oldCovariance: The covariance matrix of the existing landmark at the previous
            time step

            sigmaObservation: The sigmaObservation represents the noise added to
            the the observation and is a constant value provided in the stencil code

        returns:
            The measurement covariance matrix according to the newly sampled robot pose and
            the previous landmark position
    """    
    Q = dot( dot(jacobian, oldCovariance), np.transpose(jacobian)) + sigmaObservation
    return Q


def compute_initial_covariance(jacobian, sigmaObservation):
    """ Compute the initial covariance matrix for a landmark

    args:
        jacobian: The Jacobian matrix of the robot's position, with respect to 
        the landmark position (taken from a measurement). 

        sigmaObservation: The sigmaObservation represents the noise added to
        the the observation and is a constant value provided in the stencil code        
    """
    jacobianInverse = np.linalg.inv(jacobian)
    S = dot( dot(jacobianInverse, sigmaObservation), np.transpose(jacobianInverse) )
    return S


def compute_kalman_gain(jacobian, oldCovariance, measurementCovariance):
    """ Compute the Kalman gain 

    The Kalman gain represents how much a new landmark measurement affects an old measurement
    
    args:
        jacobian: The Jacobian matrix of the robot's position, with respect to 
        the landmark position (taken from a measurement). Here the Jacobian 
        represents the new landmark measurement
        
        oldCovariance: The covariance associated with a landmark measurement
        prior to an update. Here the old covariance matrix is representative
        of the old measurement

        measurementCovariance: The measurementCovariance represents the covariance of the previous
        measurement and the noise added to the new measurement
    """

    K = dot( dot(oldCovariance, np.transpose(jacobian)), np.linalg.inv(measurementCovariance))
    return K


def compute_new_landmark(z, z_hat, kalmanGain, old_landmark):
    """ Compute the new landmark's position estimate according to the Kalman Gain
    
    args:
        z : the current measurement of a new landmark

        z_hat : the predicted measurement from the previous landmark position estimate and
        the new robot control measurement
        
        kalmanGain: The measure of how much the new measurement affects the 
        old/believed measurment of the landmark's location
        
        old_landmark : the position esitimate of the landmark in the previous timestep

    returns:
        The updated landmark mean based on the new measurement, believed measurement,
        and Kalman Gain    
    """
    z = np.array(z)
    z[1] = z[1]
    z_hat = np.array(z)
    z_hat[1] = z_hat[1]
    d = z - z_hat
    d[1] = d[1] % (math.pi *2)

    new_landmark = old_landmark + dot(kalmanGain, d)
    return tuple(new_landmark)


def compute_new_covariance(kalmanGain, jacobian, oldCovariance):
    """ Compute the new covariance of the landmark

    The new covariance matrix of the landmark being updated
    is based on the Kalman gain, the Jacobian matrix, and the
    old covariance of this landmark

    args:
        kalmanGain: The measure of how much the new measurement affects the 
        old/believed measurment of the landmark's location

        jacobian: The Jacobian matrix of the robot's position, with respect to 
        the landmark position (taken from a measurement)
        
        oldCovariance: The covariance associated with a landmark measurement
        prior to an update
        
    returns:
        The new covariance of the landmark being updated    
    """

    I = identity(2)
    difference = I - dot(kalmanGain, jacobian)
    newCovariance = dot(difference, oldCovariance)
    return newCovariance


def adjust_angle(angle):
    """""
    keeps angle within -pi to pi
    """""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi

    return angle


def distance(x1, y1, x2, y2):
    """""
    returns the distance between two points (x1,y1) and (x2, y2)
    """""
    return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))


def normal(mu, sigma):
    """""
    samples from the Gaussian with 6 letters instead of 16
    """""
    return np.random.normal(mu, sigma)





