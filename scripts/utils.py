"""""
Support code for fast slam
"""""

from __future__ import division
import cv2
import numpy as np
from numpy import dot
from numpy import identity
import math
import sys

max_float = sys.float_info.max
MATCH_RATIO = 0.7

debug = False


class Landmark:
    """
    attributes:
    x, y: the position of the landmark
    covariance: the covariance matrix for the landmark's position, 2x2
    des: the descriptor of this landmark (landmarks are openCV features)
    counter: the number of times this landmark has been seen, initialized as 1
    """

    def __init__(self, x, y, covariance, des, count):
        self.x = x
        self.y = y
        self.covariance = covariance
        self.des = des
        self.counter = count

    def __repr__(self):
        return "Pose: " + str(self.x) + str(self.y) + "Count: " + str(self.counter)


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


def add_landmark(particle, kp, des, sigma_observation, kp_to_measurement):
    """
    adds a newly observed landmark to particle

    :param particle: the particle to add the new landmark to
    :param kp, des: the keypoint and descriptor of the new landmark to add
    :param sigma_observation: the covariance of the observation measurement
    :param kp_to_measurement: a function which computes a range/bearing measurement from the center of the
                              camera frame to kp
    """

    robot_x, robot_y = particle.pose[0], particle.pose[1]

    # get the dist and bearing from the keypoint to the center of the camera frame
    dist, bearing = kp_to_measurement(kp)

    land_x = robot_x + (dist * np.cos(bearing))
    land_y = robot_y + (dist * np.sin(bearing))

    # compute Jacobian of the robot's position and covariance of the measurement
    H = calculate_jacobian((robot_x, robot_y), (land_x, land_y))
    covariance = compute_initial_covariance(H, sigma_observation)

    # add the new landmark to this particle's list of landmarks
    particle.landmarks.append(Landmark(land_x, land_y, covariance, des, 1))


def update_landmark(particle, landmark, kp, des, sigma_observation, kp_to_measurement):
    """
    update the mean and covariance of a landmark

    uses the Extended Kalman Filter (EKF) to update the existing landmark's mean (x, y) and
    covariance according to the new measurement

    :param particle: the particle to update
    :param landmark: the landmark to update
    :param kp, des: the keypoint and descriptor of the new landmark
    :param sigma_observation: the covariance of the observation measurement
    :param kp_to_measurement: a function which computes a range/bearing measurement from the center of the
                              camera frame to kp
    """

    robot_x, robot_y = particle.pose[0], particle.pose[1]

    # get the dist and bearing from the center of the camera frame to this keypoint
    dist, bearing = kp_to_measurement(kp)

    predicted_dist = distance(landmark.x, landmark.y, robot_x, robot_y)
    predicted_bearing = (math.atan2((landmark.y - robot_y), (landmark.x - robot_x)))

    # the covariance matrix of a landmark at the previous time step
    S = landmark.covariance
    # compute the Jacobian of the robot's position
    H = calculate_jacobian((robot_x, robot_y), (landmark.x, landmark.y))
    # compute the measurement covariance matrix
    Q = compute_measurement_covariance(H, S, sigma_observation)
    # compute the Kalman gain
    K = compute_kalman_gain(H, S, Q)

    # calculate the new landmark's position estimate using the Kalman gain
    old_landmark = np.array(landmark.x, landmark.y)
    new_landmark = compute_new_landmark((dist, bearing), (predicted_dist, predicted_bearing), K, old_landmark)

    # compute the updated covariance of the landmark
    new_covariance = compute_new_covariance(K, H, S)

    return Landmark(new_landmark[0], new_landmark[1], new_covariance, des, landmark.counter)


def compute_transform(matcher, kp1, des1, kp2, des2):
    """
    computes the transformation between two sets of keypoints and descriptors
    """
    transform = None

    if des1 is not None and des2 is not None:
        matches = matcher.knnMatch(des1, des2, k=2)

        good = []
        for match in matches:
            if len(match) > 1 and match[0].distance < MATCH_RATIO * match[1].distance:
                good.append(match[0])

        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # estimateRigidTransform needs at least three pairs
        if src_pts is not None and dst_pts is not None and len(src_pts) > 3 and len(dst_pts) > 3:
            transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)

    return transform


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


def adjust_angle(angle):
    """
    keeps angle within -pi to pi
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi

    return angle





