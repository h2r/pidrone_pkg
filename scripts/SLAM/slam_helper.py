"""
slam_helper.py

Implements fastSLAM for the pidrone
"""

import numpy as np
import math
import utils
import copy
import cv2

DEBUG = False

MATCH_RATIO = 0.7
PROB_THRESHOLD = 0.005
DRONE_LEVEL_THRESHOLD = 0.35  # radians, corresponds to 20 degrees
CAMERA_SCALE = 290.
CAMERA_WIDTH = 320.
CAMERA_HEIGHT = 240.
DES_MATCH_THRESHOLD = 55.
# the height of the camera frame is the shortest distance you could move and have all new features in frame, include
# some overlap for safety
KEYFRAME_DIST_THRESHOLD = CAMERA_HEIGHT - 20
KEYFRAME_YAW_THRESHOLD = 0.175

# TODO throw away frames when the drone is not level or use stack overflow suggestion


class Landmark:
    """""
    attributes:
        x, y: the position of the landmark
        covariance: the covariance matrix for the landmark's position, 2x2
        des: the descriptor of this landmark (landmarks are openCV features)
        counter: the number of times this landmark has been seen, initialized as 1
    """""

    def __init__(self, x, y, covariance, des, count):
        self.x = x
        self.y = y
        self.covariance = covariance
        self.des = des
        self.counter = count


class Particle:
    """"
    attributes:
        robot_position: a list of the robot's position (x, y, z, yaw)
        landmarks:      a list of landmark objects
        descriptors:    a list of feature descriptors for each landmark
        weight:         the current weight for the particle
    """""

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

        self.z = 0
        # distance from center of frame to ede of frame,
        self.perceptual_range = 0.0
        self.angle_x = 0
        self.angle_y = 0

        # keypoints and descriptors from the most recent keyframe
        self.key_kp = None
        self.key_des = None

        # for computing the transform
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        self.landmark_matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        # distance (range) noise
        self.sigma_d = 3
        # bearing noise
        self.sigma_p = .30
        # observation noise
        self.sigma_observation = np.array([[self.sigma_d ** 2, 0], [0, self.sigma_p ** 2]])

        sigma_vx = 0.01
        sigma_vy = 0.01
        sigma_vz = 0.0
        sigma_yaw = 0.01
        self.covariance_motion = np.array([[sigma_vx ** 2, 0, 0, 0],
                                           [0, sigma_vy ** 2, 0, 0],
                                           [0, 0, sigma_vz ** 2, 0],
                                           [0, 0, 0, sigma_yaw ** 2]])

    def run(self, z, angle_x, angle_y, prev_kp, prev_des, kp, des):
        """""
        applies an iteration of the FastSLAM algorithm
        args
            z, angle_x, angle_y are new position data
            prev_kp, prev_des are the keypoints and descriptors from the previous frame
            kp, des are the current keypoints and descriptors
        """""
        if DEBUG:
            print 'RUN'

        self.z = z
        self.angle_x = angle_x
        self.angle_y = angle_y

        # reflect that the drone can see more or less if its height changes
        self.update_perceptual_range()

        # compute transformation from previous frame
        transform = self.compute_transform(prev_kp, prev_des, kp, des)

        if transform is not None:
            # compute how much we have moved
            x = self.pixel_to_meter(-transform[0, 2])
            y = self.pixel_to_meter(transform[1, 2])
            yaw = -np.arctan2(transform[1, 0], transform[0, 0])

            # reflect that more motion causes more uncertainty and vice versa
            self.update_motion_covariance(x, y, yaw)

            print len(self.particles[0].landmarks)
            # update poses with optical flow data
            for p in self.particles:
                self.predict_particle(p, x, y, yaw)

            # if there is some previous keyframe
            if self.key_kp is not None and self.key_des is not None:

                transform = self.compute_transform(self.key_kp, self.key_des, kp, des)

                if transform is not None:
                    # distance since previous keyframe in PIXELS
                    x = -transform[0, 2]
                    y = transform[1, 2]
                    yaw = -np.arctan2(transform[1, 0], transform[0, 0])

                    # if we've moved an entire camera frame distance since the last keyframe (or yawed 10 degrees)
                    if utils.distance(x, y, 0, 0) > KEYFRAME_DIST_THRESHOLD or yaw > KEYFRAME_YAW_THRESHOLD:
                        for p in self.particles:
                            self.update_map(p, kp, des)

                        self.resample_particles()

                        # set the new keyframe to this frame
                        self.key_kp, self.key_des = kp, des

            # there is no previous keyframe
            else:
                for p in self.particles:
                    self.update_map(p, kp, des)

                self.resample_particles()
                self.key_kp, self.key_des = kp, des
        return self.estimate_pose()

    def predict_particle(self, particle, x, y, yaw):
        """"
        updates this particle's position according to transform from prev frame to this one

        the robot's new position is determined based on the control, with
        added noise to account for control error

        args:
            particle:  the particle containing the robot position information
            x, y, yaw: the "controls" computed by transforming the previous camera frame to this one
        """""
        if DEBUG:
            print 'PREDICT'

        noisy_x_y_z_yaw = np.random.multivariate_normal([x, y, self.z, yaw], self.covariance_motion)

        print 'raw x: ', x
        print 'noisy x: ', noisy_x_y_z_yaw[0]

        old_yaw = particle.pose[3]
        # I know this works but it seems like it would move the drone in the opposite direction...
        particle.pose[0] += (noisy_x_y_z_yaw[0] * np.cos(old_yaw) + noisy_x_y_z_yaw[1] * np.sin(old_yaw))
        particle.pose[1] += (noisy_x_y_z_yaw[0] * np.sin(old_yaw) + noisy_x_y_z_yaw[1] * np.cos(old_yaw))
        particle.pose[2] = self.z
        particle.pose[3] += noisy_x_y_z_yaw[3]
        particle.pose[3] = utils.adjust_angle(particle.pose[3])

    def update_map(self, particle, keypoints, descriptors):
        """"
        associates observed keypoints with an old particle's landmark set and updates the EKF
        and increments the counter if it finds a match, otherwise it adds the new keypoint, and
        if it DOESN'T observed a stored keypoint close to this particle's pose, it decrements the counter

        args:
            particle: the old particle to perform the data association on
            keypoints, descriptors:  the lists of currently observed keypoints and descriptors
        """""

        if DEBUG:
            print 'UPDATE'

        # if this particle has no landmarks, make all measurements into landmarks
        if len(particle.landmarks) == 0:
            for kp, des in zip(keypoints, descriptors):
                self.add_landmark(particle, kp, des)
        else:
            part_descriptors = [lm.des for lm in particle.landmarks]

            # we will set to true indices where a landmark is matched
            matched_landmarks = [False] * len(particle.landmarks)

            # iterate through every measurement
            for kp, des in zip(keypoints, descriptors):
                # length 1 list of the most likely match between this descriptor and all the particle's descriptors
                match = self.landmark_matcher.match(np.array([des]), np.array(part_descriptors))

                # there was no match
                if match[0].distance > DES_MATCH_THRESHOLD:
                    self.add_landmark(particle, kp, des)
                    particle.weight += math.log(PROB_THRESHOLD)
                else:
                    # get the index of the matched landmark in the old particle
                    old_index = match[0].trainIdx

                    particle.weight += math.log(scale_weight(match[0].distance))
                    self.update_landmark(particle, old_index, kp, des)
                    matched_landmarks[old_index] = True

            for i, match in enumerate(matched_landmarks):
                lm = particle.landmarks[i]
                if match:
                    lm.counter += 1
                # this landmark was not matched to, but is within sight of the drone
                elif utils.distance(lm.x, lm.y, particle.pose[0], particle.pose[1]) < self.perceptual_range:
                    lm.counter -= 1
                    if lm.counter < 0:
                        particle.landmarks.remove(lm)

    def add_landmark(self, particle, kp, des):
        """"
        adds a newly observed landmark to particle

        if a landmark has not been matched to an existing landmark,
        add it to the particle's list with the appropriate
        mean (x, y) and covariance (sigma)

        args:
            particle: the particle to add the new landmark to
            kp: the keypoint of the new landmark to add
            des: the descriptor of the new landmark to add
        """""
        if DEBUG:
            print 'NEW LM'

        robot_x, robot_y = particle.pose[0], particle.pose[1]

        # get the dist and bearing from the keypoint to the center of the camera frame
        dist, bearing = self.kp_to_measurement(kp)

        land_x = robot_x + (dist * np.cos(bearing))
        land_y = robot_y + (dist * np.sin(bearing))

        # compute the Jacobian of the robot's position
        H = utils.calculate_jacobian((robot_x, robot_y), (land_x, land_y))

        # compute the covariance matrix associated with the landmark measurement
        covariance = utils.compute_initial_covariance(H, self.sigma_observation)

        # add the new landmark to this particle's list of landmarks
        particle.landmarks.append(Landmark(land_x, land_y, covariance, des, 1))

    def update_landmark(self, particle, index, kp, des):
        """"
        update the mean and covariance of a landmark

        uses the Extended Kalman Filter (EKF) to update the existing
        landmark's mean (x, y) and covariance according to the new measurement

        args:
            particle: the particle to update
            index: the index of the landmark to update
            kp, des: the keypoint and descriptor of the new landmark
        """""

        if DEBUG:
            print 'OLD LM'

        landmark = particle.landmarks[index]

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

        # old landmark's position estimate
        old_landmark = np.array(landmark.x, landmark.y)

        # calculate the new landmark's position estimate using the Kalman gain
        new_landmark = utils.compute_new_landmark((dist, bearing), (predicted_dist, predicted_bearing), K, old_landmark)

        # compute the updated covariance of the landmark
        new_covariance = utils.compute_new_covariance(K, H, S)

        lm = Landmark(new_landmark[0], new_landmark[1], new_covariance, des, landmark.counter)
        particle.landmarks[index] = lm

    def resample_particles(self):
        """"
        resample particles according to weight
        """""

        if DEBUG:
            print 'RESAMPLE'

        weight_sum = 0.0
        new_particles = []
        normal_weights = []

        for p in self.particles:
            w = min(math.exp(p.weight), utils.max_float)
            weight_sum += w

        for p in self.particles:
            w = min(math.exp(p.weight), utils.max_float)
            normal_weights.append(w / weight_sum)

        samples = np.random.multinomial(self.num_particles, normal_weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                p = copy.deepcopy(self.particles[i])
                p.weight = PROB_THRESHOLD
                new_particles.append(p)

        self.particles = new_particles

    def generate_particles(self, num_particles):
        """""
        Creates the initial set of particles for SLAM

        Each particle should start at (0,0) since we build the map relative to the drone's
        initial position, but I want them to be a little different so Gaussian

        potential problem here is that some of them will be negative which is impossible so maybe abs?
        """""
        # since the localization code treats pi and the forward facing yaw, probably safer to initialize the
        # heading around pi...
        self.particles = [Particle(abs(utils.normal(0, 0.01)),
                                   abs(utils.normal(0, 0.01)),
                                   self.z,
                                   abs(utils.normal(math.pi, 0.01))) for _ in range(num_particles)]

        self.num_particles = num_particles

        return self.estimate_pose()

    def estimate_pose(self):
        """""
        retrieves the drone's estimated position by summing each particles estimate multiplied
        by its weight

        some mathematical motivation E[X] = sum over all x in X: p(x) * x
        """""
        weight_sum = 0.0
        normal_weights = []

        for p in self.particles:
            w = min(math.exp(p.weight), utils.max_float)
            weight_sum += w

        for p in self.particles:
            w = min(math.exp(p.weight), utils.max_float)
            normal_weights.append(w / weight_sum)

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

        # return the "average" pose and weight
        return [x, y, z, yaw], weight_sum / float(self.num_particles)

    def compute_transform(self, kp1, des1, kp2, des2):
        """""
        computes the transformation between two sets of keypoints and descriptors
        """""
        transform = None

        if des1 is not None and des2 is not None:
            matches = self.matcher.knnMatch(des1, des2, k=2)

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

    def pixel_to_meter(self, px):
        """""
        uses the camera scale to convert pixel measurements into meter
        """""
        return px * self.z / CAMERA_SCALE

    def kp_to_measurement(self, kp):
        """""
        Computes the range and bearing from the center of the camera frame to (kp.x, kp.y)
        bearing is in (-pi/2, pi/2) measured in the standard math way
        """""
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
        """""
        computes the perceptual range of the drone: the distance from the center of the frame to the
        width-wise border of the frame
        """""
        return self.pixel_to_meter(CAMERA_WIDTH / 2)

    def update_motion_covariance(self, dx, dy, dyaw):
        """""
        scales the covariance matrix for the motion model according to the velocities of the drone because
        when the drone isn't moving, we are most certain of the motion model, and vice versa
        
        args:
            dx, dy, dyaw: the displacement of the drone, in pixels, since the last frame
        """""

        # RULES
        # a delta of 0 is always 0 variance
        # an x/y delta of 10cm or more is always 0.1 variance
        # a yaw of more than 0.34 or less than -0.34 radian is always 0.1 variance
        # linear scale for all values in between

        vx = self.pixel_to_meter(dx)
        vy = self.pixel_to_meter(dy)
        vyaw = abs(dyaw)

        if vx > 0.1: vx = 0.1
        if vy > 0.1: vy = 0.1

        if vyaw >= 0.34:
            vyaw = 0.1
        else:
            # linear scale from [0,0.34] to [0,0.1]
            vyaw *= 0.294

        # motion matrix [y][x] from top left so vx is 0,0 vy is 1,1 and vyaw is 3,3
        self.covariance_motion[0][0] = vx
        self.covariance_motion[1][1] = vy
        self.covariance_motion[3][3] = vyaw


def scale_weight(w):
    """""
    scales a weight from (0, DES_THRESHOLD) to (0,1)
    never returns 0 since that logarithm is undefined
    """""
    scaled = 1 - (w / DES_MATCH_THRESHOLD)
    if scaled == 0:
        scaled = PROB_THRESHOLD
    return scaled
