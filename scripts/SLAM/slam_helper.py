"""
slam_helper.py

Implements fastSLAM for the pidrone
"""

import numpy as np
import math
import utils
import copy
import cv2
import threading
from thread_queue import ThreadQueue

DEBUG = False

# ----- camera parameters DO NOT EDIT ------- #
CAMERA_SCALE = 290.
CAMERA_WIDTH = 320.
CAMERA_HEIGHT = 240.

# ----- feature parameters DO NOT EDIT ------ #
DES_MATCH_THRESHOLD = 55.

# ----- SLAM parameters ------- #
PROB_THRESHOLD = 0.005
KEYFRAME_DIST_THRESHOLD = CAMERA_HEIGHT
KEYFRAME_YAW_THRESHOLD = 0.175

pose_path = '/home/pi/ws/src/pidrone_pkg/scripts/pose_data.txt'


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


class Particle:
    """
    attributes:
    robot_position: a list of the robot's position (x, y, z, yaw)
    landmarks:      a list of landmark objects
    descriptors:    a list of feature descriptors for each landmark
    weight:         the current weight for the particle
    """

    def __init__(self, x, y, z, yaw):
        self.pose = [x, y, z, yaw]
        self.landmarks = []
        self.weight = PROB_THRESHOLD

    def __str__(self):
        return "Pose: " + str(self.pose) + " Weight: " + str(self.weight)


class FastSLAM:
    def __init__(self):
        self.particles = None
        self.num_particles = None
        self.weight = PROB_THRESHOLD

        self.z = 0
        self.perceptual_range = 0.0

        # key points and descriptors from the most recent keyframe
        self.key_kp = None
        self.key_des = None

        self.file = open(pose_path, 'w')

        # we will queue threads here, and they will store their results in most_recent_map and set new_result to True
        self.thread_queue = ThreadQueue()
        self.most_recent_map = None
        self.new_result = False

        # --------------- openCV parameters --------------------- #
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
        self.landmark_matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        # ------- parameters for noise on observations ----------- #
        self.sigma_d = 3
        self.sigma_p = .30
        self.sigma_observation = np.array([[self.sigma_d ** 2, 0], [0, self.sigma_p ** 2]])

        # ------- parameters for noise on motion updates --------- #
        sigma_vx = 0.0001
        sigma_vy = 0.0001
        sigma_vz = 0.0
        sigma_yaw = 0.0001
        self.covariance_motion = np.array([[sigma_vx ** 2, 0, 0, 0],
                                           [0, sigma_vy ** 2, 0, 0],
                                           [0, 0, sigma_vz ** 2, 0],
                                           [0, 0, 0, sigma_yaw ** 2]])

    def generate_particles(self, num_particles):
        """
        Creates the initial set of particles for SLAM
        Each particle starts at (0,0) since we build the map relative to the drone's
        initial position, but with some noise

        :param num_particles: the number of particles to generate
        """
        self.particles = [Particle(abs(utils.normal(0, 0.1)),
                                   abs(utils.normal(0, 0.1)),
                                   self.z,
                                   abs(utils.normal(math.pi, 0.01))) for _ in range(num_particles)]

        self.num_particles = num_particles
        self.key_kp, self.key_des = None, None
        self.weight = PROB_THRESHOLD

        return self.estimate_pose()

    def run(self, z, prev_kp, prev_des, kp, des):
        """
        applies an iteration of the FastSLAM algorithm

        :param z: the new infrared height estimate for the drone
        :param prev_kp, prev_des: are the keypoints and descriptors from the previous frame
        :param kp, des: the current keypoints and descriptors
        """
        if DEBUG:
            print 'RUN'

        # print the average number of landmarks per particles
        print "LM: ", np.sum([len(p.landmarks) for p in self.particles]) / float(self.num_particles)

        # write poses to a text file to be animated
        for p in self.particles:
            self.file.write(str(p.pose[0]) + '\n')
            self.file.write(str(p.pose[1]) + '\n')

        self.z = z

        # reflect that the drone can see more or less if its height changes
        self.update_perceptual_range()

        # compute transformation from previous frame
        transform = utils.compute_transform(self.matcher, prev_kp, prev_des, kp, des)

        if transform is not None:
            x = -transform[0, 2]
            y = transform[1, 2]
            yaw = -np.arctan2(transform[1, 0], transform[0, 0])

            # update poses with motion prediction
            for p in self.particles:
                self.predict_particle(p, x, y, yaw)

            # (potentially) do a map update
            self.map_update_thread(kp, des)

            # replace particles with updated ones if a thread has completed
            if self.new_result:
                self.new_result = False
                most_recent_particles = self.most_recent_map
                for old, new in zip(self.particles, most_recent_particles[0]):
                    old.landmarks = new.landmarks

                # update the weight with the most recent thread result and resample
                self.weight = most_recent_particles[1]
                self.particles = resample_particles(self.particles)

        return self.estimate_pose(), self.weight

    def predict_particle(self, particle, x, y, yaw):
        """
        updates this particle's position according to the transformation from prev frame to this one

        the robot's new position is determined based on the control, with
        added noise to account for control error

        :param particle:  the particle containing the robot's position information
        :param x, y, yaw: the "controls" computed by transforming the previous camera frame to this one
        """
        if DEBUG:
            print 'PREDICT'

        noisy_x_y_z_yaw = np.random.multivariate_normal([x, y, self.z, yaw], self.covariance_motion)

        particle.pose[0] += self.pixel_to_meter(noisy_x_y_z_yaw[0])
        particle.pose[1] += self.pixel_to_meter(noisy_x_y_z_yaw[1])
        particle.pose[2] = self.z
        particle.pose[3] += noisy_x_y_z_yaw[3]
        particle.pose[3] = utils.adjust_angle(particle.pose[3])

    def estimate_pose(self):
        """
        retrieves the drone's estimated position by summing each particle's pose estimate multiplied
        by its weight

        some mathematical motivation Expectation[X] = sum over all x in X of p(x) * x
        """
        weight_sum = 0.0
        normal_weights = np.array([])

        for p in self.particles:
            w = min(math.exp(p.weight), utils.max_float)
            normal_weights = np.append(normal_weights, w)
            weight_sum += w

        normal_weights /= weight_sum

        x = 0.0
        y = 0
        z = 0
        yaw = 0

        for i, prob in enumerate(normal_weights):
            x += prob * self.particles[i].pose[0]
            y += prob * self.particles[i].pose[1]
            z += prob * self.particles[i].pose[2]
            yaw += prob * self.particles[i].pose[3]

        yaw = utils.adjust_angle(yaw)

        return [x, y, z, yaw]

    def map_update_thread(self, kp, des):
        """
        takes care of map updates with a thread, since they are slower than motion updates, this way the
        motion updates can still happen while the map is updating

        :param kp, des: the lists of keypoints and descriptors
        """
        # there is some previous keyframe
        if self.key_kp is not None and self.key_des is not None:

            transform = utils.compute_transform(self.matcher, self.key_kp, self.key_des, kp, des)

            if transform is not None:
                # distance since previous keyframe in PIXELS
                x = -transform[0, 2]
                y = transform[1, 2]
                yaw = -np.arctan2(transform[1, 0], transform[0, 0])

                if utils.distance(x, y, 0, 0) > KEYFRAME_DIST_THRESHOLD or yaw > KEYFRAME_YAW_THRESHOLD:
                    self.start_thread(kp, des)
            else:
                # moved too far to transform from last keyframe, so set a new one
                self.start_thread(kp, des)
        # there is no previous keyframe
        else:
            self.start_thread(kp, des)

    def start_thread(self, kp, des):
        """
        starts a thread to update the map

        :param kp, des: the lists of keypoints and descriptors
        """
        t = threading.Thread(target=self.keyframe_update, args=(kp, des,))
        self.thread_queue.add_thread(t)

        # set the new keyframe kp and des to the current ones
        self.key_kp, self.key_des = kp, des

    def keyframe_update(self, kp, des):
        """
        updates the map and gets the average weight of the particles, then queues those results

        :param kp, des: the lists of keypoints and descriptors
        """
        # save a copy of the particles at this time to avoid conflicts
        curr_particles = copy.deepcopy(self.particles)

        for p in curr_particles:
            self.update_map(p, kp, des)

        weight = self.get_average_weight(curr_particles)

        self.most_recent_map = (curr_particles, weight)
        self.new_result = True

    def update_map(self, particle, keypoints, descriptors):
        """
        Associate observed keypoints with an old particle's landmark set and update the EKF
        Increment the landmark's counter if it finds a match, otherwise add a new landmark
        Dcrement the counter of a landmark which is close to this particle's pose and not observed

        :param particle: the old particle to perform the data association on
        :param keypoints, descriptors: the lists of currently observed keypoints and descriptors
        """

        if DEBUG:
            print 'UPDATE'

        # if this particle has no landmarks, make all measurements into landmarks
        if len(particle.landmarks) == 0:
            for kp, des in zip(keypoints, descriptors):
                self.add_landmark(particle, kp, des)
        else:
            # find particle's landmarks in a close range, close_landmarks holds tuples of the landmark and its index
            close_landmarks = []
            for i, lm in enumerate(particle.landmarks):
                if utils.distance(lm.x, lm.y, particle.pose[0], particle.pose[1]) <= self.perceptual_range * 1.2:
                    close_landmarks.append((lm, i))

            if len(close_landmarks) != 0:
                # get the descriptors of relevant landmarks
                part_descriptors = [lm[0].des for lm in close_landmarks]

                # we will set to true indices where a landmark is matched
                matched_landmarks = [False] * len(close_landmarks)

                for kp, des in zip(keypoints, descriptors):
                    # length 1 list of the most likely match between this descriptor and all the particle's descriptors
                    match = self.landmark_matcher.match(np.array([des]), np.array(part_descriptors))

                    # there was no match
                    if match[0].distance > DES_MATCH_THRESHOLD:
                        self.add_landmark(particle, kp, des)

                        # 'punish' this particle since new landmarks decrease certainty
                        particle.weight += math.log(PROB_THRESHOLD)
                    else:
                        # get the index of the matched landmark in close_landmarks
                        close_index = match[0].trainIdx
                        matched_landmarks[close_index] = True
                        dated_landmark = close_landmarks[close_index]

                        # update the original landmark in this particle
                        updated_landmark = self.update_landmark(particle, dated_landmark[0], kp, des)
                        particle.landmarks[dated_landmark[1]] = updated_landmark

                        # 'reward' this particles since revisting landmarks increases certainty
                        particle.weight += math.log(scale_weight(match[0].distance))

                removed = []
                for i, match in enumerate(matched_landmarks):
                    tup = close_landmarks[i]
                    lm = particle.landmarks[tup[1]]
                    if match:
                        lm.counter += 1
                    else:
                        lm.counter -= 1
                        # 'punish' this particle for having a dubious landmark
                        particle.weight += math.log(0.1*PROB_THRESHOLD)
                        if lm.counter < 0:
                            removed.append(lm)

                for rm in removed:
                    particle.landmarks.remove(rm)

    def add_landmark(self, particle, kp, des):
        """
        adds a newly observed landmark to particle

        :param particle: the particle to add the new landmark to
        :param kp, des: the keypoint and descriptor of the new landmark to add
        """
        if DEBUG:
            print 'NEW LM'

        robot_x, robot_y = particle.pose[0], particle.pose[1]

        # get the dist and bearing from the keypoint to the center of the camera frame
        dist, bearing = self.kp_to_measurement(kp)

        land_x = robot_x + (dist * np.cos(bearing))
        land_y = robot_y + (dist * np.sin(bearing))

        # compute Jacobian of the robot's position and covariance of the measurement
        H = utils.calculate_jacobian((robot_x, robot_y), (land_x, land_y))
        covariance = utils.compute_initial_covariance(H, self.sigma_observation)

        # add the new landmark to this particle's list of landmarks
        particle.landmarks.append(Landmark(land_x, land_y, covariance, des, 1))

    def update_landmark(self, particle, landmark, kp, des):
        """
        update the mean and covariance of a landmark

        uses the Extended Kalman Filter (EKF) to update the existing landmark's mean (x, y) and
        covariance according to the new measurement

        :param particle: the particle to update
        :param landmark: the landmark to update
        :param kp, des: the keypoint and descriptor of the new landmark
        """

        if DEBUG:
            print 'OLD LM'

        robot_x, robot_y = particle.pose[0], particle.pose[1]

        # get the dist and bearing from the center of the camera frame to this keypoint
        dist, bearing = self.kp_to_measurement(kp)

        predicted_dist = utils.distance(landmark.x, landmark.y, robot_x, robot_y)
        predicted_bearing = (math.atan2((landmark.y - robot_y), (landmark.x - robot_x)))

        # the covariance matrix of a landmark at the previous time step
        S = landmark.covariance
        # compute the Jacobian of the robot's position
        H = utils.calculate_jacobian((robot_x, robot_y), (landmark.x, landmark.y))
        # compute the measurement covariance matrix
        Q = utils.compute_measurement_covariance(H, S, self.sigma_observation)
        # compute the Kalman gain
        K = utils.compute_kalman_gain(H, S, Q)

        # calculate the new landmark's position estimate using the Kalman gain
        old_landmark = np.array(landmark.x, landmark.y)
        new_landmark = utils.compute_new_landmark((dist, bearing), (predicted_dist, predicted_bearing), K, old_landmark)

        # compute the updated covariance of the landmark
        new_covariance = utils.compute_new_covariance(K, H, S)

        return Landmark(new_landmark[0], new_landmark[1], new_covariance, des, landmark.counter)

    def get_average_weight(self, particles):
        """
        the average weight of all the particles

        :param particles: the set of particles whose weight will be averaged
        """
        return np.sum([p.weight for p in particles]) / float(self.num_particles)

    def pixel_to_meter(self, px):
        """
        uses the camera scale to convert pixel measurements into meters

        :param px: the distance in pixels to convert
        """
        return px * self.z / CAMERA_SCALE

    def kp_to_measurement(self, kp):
        """
        Computes the range and bearing from the center of the camera frame to (kp.x, kp.y)
        bearing is in (-pi/2, pi/2) measured in the standard math way

        :param kp: they keypoint to measure from
        """
        kp_x, kp_y = kp.pt[0], kp.pt[1]
        # key point y is measured from the top left
        kp_y = CAMERA_HEIGHT - kp_y

        dx = kp_x - CAMERA_WIDTH / 2
        dy = kp_y - CAMERA_HEIGHT / 2
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        dist = self.pixel_to_meter(dist)
        bearing = math.atan2(dy, dx)

        return dist, bearing

    def update_perceptual_range(self):
        """
        computes the perceptual range of the drone: the distance from the center of the frame to the
        width-wise border of the frame
        """
        self.perceptual_range = self.pixel_to_meter(CAMERA_WIDTH / 2)


def scale_weight(w):
    """
    scales a weight from (0, DES_THRESHOLD) to (0,1)
    never returns 0 since we tale the logarithm of the weight

    :param w: the weight to scale
    """
    scaled = 1 - (w / DES_MATCH_THRESHOLD)
    if scaled == 0:
        scaled = PROB_THRESHOLD
    return scaled


def resample_particles(particles):
    """
    resample particles according to their weight

    :param particles: the set of particles to resample
    """

    if DEBUG:
        print 'RESAMPLE'

    weight_sum = 0.0
    new_particles = []
    normal_weights = np.array([])

    for p in particles:
        w = min(math.exp(p.weight), utils.max_float)
        normal_weights = np.append(normal_weights, w)
        weight_sum += w

    normal_weights /= weight_sum
    samples = np.random.multinomial(len(particles), normal_weights)
    for i, count in enumerate(samples):
        for _ in range(count):
            p = copy.deepcopy(particles[i])
            p.weight = PROB_THRESHOLD
            new_particles.append(p)

    return new_particles
