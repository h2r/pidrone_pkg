import math
import numpy as np
import cv2
import copy

# first, refactor flow_pub_transform so that it computes the features
# explicitly, and then calls cv2.estimateRigidTransform using those
# features.  Verify position hold still works.  It should be faster.
#

# second, compute the local pose for each feature.  Camera frame.  For
# each feature, you'll have an x,y,z in the camera frame (where the
# camera is zero, zero,zero).  Verify it's right by publishing the
# pose as a ROS marker array.  Visualize it in rviz.

# third, make a feature map.  So given the global true pose of the
# robot from the mocap (Optitrak), store the global x,y,z location of each
# feature by transforming above to the global frame using the mocap.

# use time synchronizer to line up optitrak poses with images.
# http://docs.ros.org/api/message_filters/html/python/#message_filters.TimeSynchronizer

#

# fourth, write out the math and agree how it will work.

# publish the features in the camera frame in ros

# this node subscribes and does an update given the map and each feature.

# this node publishes a TF transform from /map to /base_link

# Use meters (not centimers) and radians.

# TODO we may use range, bearing instead of x, y for measurement, also decrease the number of features


# ----- parameters are same as picam_localization -----
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# MAP_PIXEL_WIDTH = 1700  # in pixel
# MAP_PIXEL_HEIGHT = 1213
# MAP_REAL_WIDTH = 1.4  # in meter
# MAP_REAL_HEIGHT = 1.07
MAP_PIXEL_WIDTH = 3215
MAP_PIXEL_HEIGHT = 3600
MAP_REAL_WIDTH = 2.0
MAP_REAL_HEIGHT = 2.29
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
CAMERA_SCALE_X = 320.
CAMERA_SCALE_Y = 240.

# ----- parameters for features -----
ORB_GRID_SIZE_X = 4
ORB_GRID_SIZE_Y = 5
MATCH_RATIO = 0.75  # can be tuned, taken from opencv tutorial
MIN_MATCH_COUNT = 5  # can be tuned, taken from opencv tutorial
MAP_GRID_SIZE_X = ORB_GRID_SIZE_X * 3
MAP_GRID_SIZE_Y = ORB_GRID_SIZE_Y * 3
CELL_X = float(MAP_PIXEL_WIDTH) / MAP_GRID_SIZE_X
CELL_Y = float(MAP_PIXEL_HEIGHT) / MAP_GRID_SIZE_Y
PROB_THRESHOLD = 0.001
MEASURE_WAIT_COUNT = 10

class Particle(object):
    """
    each particle holds its estimation of the robot's location and weight of particle
    z is currently not used
    """

    def __init__(self, x, y, z, yaw):
        self.position = [x, y, z, yaw]
        self.weight = PROB_THRESHOLD

    def __str__(self):
        return str(self.position) + ', ' + str(self.weight)

    def __repr__(self):
        return str(self.position) + ', ' + str(self.weight)


class LocalizationParticleFilter:
    """
    Particle filter for localization.
    """

    def __init__(self, map_kp, map_des):
        self.map_kp = map_kp
        self.map_des = map_des

        self.z = 0.0
        self.angle_x = 0.0
        self.angle_y = 0.0

        # parameters - noise on sensor.
        # TODO check the parameters and covariance matrix is proper
        sigma_vx = 0.005
        sigma_vy = 0.005
        sigma_vz = 0.0
        sigma_yaw = 0.01
        self.covariance_motion = np.array([[sigma_vx ** 2, 0, 0, 0],
                                           [0, sigma_vy ** 2, 0, 0],
                                           [0, 0, sigma_vz ** 2, 0],
                                           [0, 0, 0, sigma_yaw ** 2]])
        self.sigma_x = 0.05
        self.sigma_y = 0.05
        self.sigma_yaw = 0.01
        self.prob_threshold = PROB_THRESHOLD

        self.measure_count = 0
        self.particles = None  # initialize to random particles spread over map. x, y, z, yaw
        self.previous_time = None
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

    def sample_motion_model(self, x, y, yaw, xt):
        """
        Implement motion model from Equation 3 in PiDrone Slam with noise.
        """
        # compute xtp1, with equation 3.
        # xtp1 = 0 # A * xt + B * ut

        # add random gaussian noise, but use matpl.
        # xtp1 += np.random.normal(na.zeros(xt.shape), self.sigma)
        # return xtp1

        # add noise
        noisy_x_y_z_yaw = np.random.multivariate_normal([x, y, self.z, yaw], self.covariance_motion)
        # noisy_x_y_z_yaw = [x, y, self.z, yaw]

        old_yaw = xt.position[3]
        xt.position[0] += (noisy_x_y_z_yaw[0] * np.cos(old_yaw) + noisy_x_y_z_yaw[1] * np.sin(old_yaw))
        xt.position[1] += (noisy_x_y_z_yaw[0] * np.sin(old_yaw) + noisy_x_y_z_yaw[1] * np.cos(old_yaw))
        xt.position[2] = self.z
        xt.position[3] += noisy_x_y_z_yaw[3]

        # the range is (-pi, pi]
        while xt.position[3] > math.pi:
            xt.position[3] -= 2 * math.pi
        while xt.position[3] <= -math.pi:
            xt.position[3] += 2 * math.pi

    def measurement_model(self, kp, des, xt):
        """
        landmark_model_known_correspondence from probablistic robotics
        """

        # in this implementation zt is one feature, but really it will
        # be a vector of features, so you'd multiply q together.  We
        # should then use log probability. (So there would be an outer
        # for loop over all the detected features, and q would be a
        # log prob that we add, and return the sum.

        # in this implementation it's 2d, but really we need to do it
        # in 3d.  phi in zt is actually a quaternion defined by the
        # camera model and feature pixel location.

        # x,y,z = xt
        # j, r, phi = zt
        # mx, my = self.map[j]
        # hatr = math.sqrt(math.pow(mx - x,2) + math.pow(my - y,2))
        # hatphi = math.atan2(my - y, mx - x)
        # q = (random.normalvariate(r - hatr, self.sigmar) *
        #      random.normalvariate(phi - hatphi, self.sigmaphi))
        # return q

        # get global and grid pose
        grid_x = int(xt.position[0] * METER_TO_PIXEL / CELL_X)
        grid_x = max(min(grid_x, MAP_GRID_SIZE_X - 1), 0)
        grid_y = int(xt.position[1] * METER_TO_PIXEL / CELL_Y)
        grid_y = max(min(grid_y, MAP_GRID_SIZE_Y - 1), 0)
        sub_map_kp = self.map_kp[grid_x][grid_y]
        sub_map_des = self.map_des[grid_x][grid_y]

        # measure current global position
        pose, num = self.compute_location(kp, des, sub_map_kp, sub_map_des)

        # compute weight of particle
        if pose is None:
            q = self.prob_threshold
            # print 'none', num, xt.position[0], xt.position[1]
        else:
            # add noise
            noisy_pose = [np.random.normal(pose[0], self.sigma_x), np.random.normal(pose[1], self.sigma_y), pose[2],
                          np.random.normal(pose[3], self.sigma_yaw)]
            # keep in (-pi, pi]
            while noisy_pose[3] > math.pi:
                noisy_pose[3] -= 2 * math.pi
            while noisy_pose[3] <= -math.pi:
                noisy_pose[3] += 2 * math.pi
            # noisy_pose = pose

            yaw_difference = noisy_pose[3] - xt.position[3]
            if yaw_difference > math.pi:
                yaw_difference = yaw_difference - 2 * math.pi
            elif yaw_difference <= -math.pi:
                yaw_difference = yaw_difference + 2 * math.pi
            # TODO the log and addition for bearing and velocity ?
            q = min(2., norm_pdf(noisy_pose[0] - xt.position[0], 0, self.sigma_x)) \
                * min(2., norm_pdf(noisy_pose[1] - xt.position[1], 0, self.sigma_y)) \
                * min(2., norm_pdf(yaw_difference, 0, self.sigma_yaw))

            # print 'true', noisy_pose[0], noisy_pose[1], \
            #     '\nparticle', xt.position[0], xt.position[1], \
            #     '\ndifference', noisy_pose[3], xt.position[3], \
            #     '\nweight', q, \
            #     '\n-----'
        xt.weight = max(q, self.prob_threshold)

    def update(self, z, angle_x, angle_y, prev_kp, prev_des, kp, des):
        """
        We implement the MCL algorithm from probabilistic robotics (Table 8.2)
        kp is the position of detected features
        des is the description of detected features
        """
        # newparticles = []
        # weights = []
        # for (xtm1, wtm) in self.particles:
        #     xtm = self.sample_motion_model(ut, xtm1)
        #     wtm = self.measurement_model(zt, xtm)
        #     newparticles.append((xtm, wtm))
        #     weights.append(wtm)
        #
        # weights = weights / np.sum(weights) # normalize
        #
        # samples = np.random.multinomial(len(self.particles), weights)
        #
        # self.particles = [newparticles[i] for i in samples]

        # update parameters
        self.z = z
        self.angle_x = angle_x
        self.angle_y = angle_y

        transform = self.compute_transform(prev_kp, prev_des, kp, des)
        if transform is not None:
            x = -transform[0, 2] * self.z / CAMERA_SCALE_X
            y = transform[1, 2] * self.z / CAMERA_SCALE_Y
            yaw = -np.arctan2(transform[1, 0], transform[0, 0])

        for p in self.particles:
            # update the position of each particle
            if transform is not None:
                self.sample_motion_model(x, y, yaw, p)
            # update the weight of each particle
            if self.measure_count > MEASURE_WAIT_COUNT:
                self.measurement_model(kp, des, p)
        # don't measure every time
        if self.measure_count > MEASURE_WAIT_COUNT:
            self.measure_count = 0
        self.measure_count += 1

        # resample particles
        self.resample_particles()

        # return the estimated position
        return self.get_estimated_position()

    def resample_particles(self):
        weights_sum = 0.0
        weights = []
        new_particles = []

        for p in self.particles:
            weights_sum += p.weight
            weights.append(p.weight)
        weights = np.array(weights)
        weights = weights / weights_sum  # normalize
        samples = np.random.multinomial(len(self.particles), weights)  # sample
        for i, count in enumerate(samples):
            for _ in range(count):
                new_particles.append(copy.deepcopy(self.particles[i]))

        self.particles = np.array(new_particles)  # replace particles

    def get_estimated_position(self):
        weights_sum = 0.0
        weights = []
        x = 0
        y = 0
        z = 0
        yaw = 0

        for p in self.particles:
            weights_sum += p.weight
            weights.append(p.weight)
        weights = np.array(weights)
        weights = weights / weights_sum  # get the probability

        for i, prob in enumerate(weights):
            x += prob * self.particles[i].position[0]
            y += prob * self.particles[i].position[1]
            z += prob * self.particles[i].position[2]
            yaw += prob * self.particles[i].position[3]

        particle = Particle(x, y, z, yaw)
        particle.weight = weights_sum / len(weights)

        return particle

    def initialize_particles(self, num_particles, kp, des):
        """
        find most possible location to start
        :param num_particles: number of particles we are using
        :param kp: the keyPoints of the first captured image
        :param des: the descriptions of the first captured image
        """
        weights_sum = 0.0
        weights = []
        particles = []
        new_particles = []

        # go through every grid, trying to find matched features
        for x in range(MAP_GRID_SIZE_X):
            for y in range(MAP_GRID_SIZE_Y):
                p, w = self.compute_location(kp, des, self.map_kp[x][y], self.map_des[x][y])
                if p is not None:
                    particles.append(Particle(p[0], p[1], p[2], p[3]))
                    weights_sum += w
                    weights.append(w)

        # cannot find a match
        if len(particles) == 0:
            print "Random Initialization"
            for x in range(MAP_GRID_SIZE_X):
                for y in range(MAP_GRID_SIZE_Y):
                    particles.append(Particle((x * CELL_X + CELL_X / 2.0) / METER_TO_PIXEL,
                                              (y * CELL_Y + CELL_Y / 2.0) / METER_TO_PIXEL,
                                              self.z,
                                              np.random.random_sample() * 2 * np.pi - np.pi))
                    weights_sum += 1.0  # uniform sample
                    weights.append(1.0)

        # sample particles based on the number of matched features
        weights = np.array(weights) / weights_sum  # normalize
        samples = np.random.multinomial(num_particles, weights)  # sample
        for i, count in enumerate(samples):
            for _ in range(count):
                new_particles.append(copy.deepcopy(particles[i]))

        self.particles = np.array(new_particles)  # replace particles

        return self.get_estimated_position()

    def compute_location(self, kp1, des1, kp2, des2):
        """
        compute the global location of center of current image
        :param kp1: captured keyPoints
        :param des1: captured descriptions
        :param kp2: map keyPoints
        :param des2: map descriptions
        :return: global pose
        """

        good = []
        pose = None

        if des1 is not None and des2 is not None:
            matches = self.matcher.knnMatch(des1, des2, k=2)

            for match in matches:
                if len(match) > 1 and match[0].distance < MATCH_RATIO * match[1].distance:
                    good.append(match[0])

            if len(good) > MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
                if transform is not None:
                    transformed_center = cv2.transform(CAMERA_CENTER, transform)  # get global pixel
                    transformed_center = [transformed_center[0][0][0] / METER_TO_PIXEL,  # map to global pose
                                          (MAP_PIXEL_HEIGHT - 1 - transformed_center[0][0][1]) / METER_TO_PIXEL]
                    yaw = np.arctan2(transform[1, 0], transform[0, 0])  # get global heading

                    # correct the pose if the drone is not level
                    z = math.sqrt(self.z ** 2 / (1 + math.tan(self.angle_x) ** 2 + math.tan(self.angle_y) ** 2))
                    offset_x = np.tan(self.angle_x) * z
                    offset_y = np.tan(self.angle_y) * z
                    global_offset_x = math.cos(yaw) * offset_x + math.sin(yaw) * offset_y
                    global_offset_y = math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
                    pose = [transformed_center[0] + global_offset_x, transformed_center[1] + global_offset_y, z, yaw]

        return pose, len(good)

    def compute_transform(self, kp1, des1, kp2, des2):
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


def create_map(file_name):
    """
    create a feature map, extract features from each bigger cell.
    :param file_name: the image of map
    :return: a list of center of bigger cells (kp and des), each bigger cell is a 3 by 3 grid (9 cells).
    """

    # read image and extract features
    image = cv2.imread(file_name)
    # the edgeThreshold and patchSize can be tuned if the gap between cell is too large
    detector = cv2.ORB(nfeatures=3000, scoreType=cv2.ORB_FAST_SCORE, edgeThreshold=5)
    maxTotalKeypoints = 3000 * ORB_GRID_SIZE_X * ORB_GRID_SIZE_Y
    detector_grid = cv2.GridAdaptedFeatureDetector(detector, maxTotalKeypoints=maxTotalKeypoints,
                                                   gridCols=ORB_GRID_SIZE_X, gridRows=ORB_GRID_SIZE_Y)
    kp = detector_grid.detect(image, None)
    kp, des = detector.compute(image, kp)

    # rearrange kp and des into grid
    grid_kp = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    grid_des = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    for i in range(len(kp)):
        x = int(kp[i].pt[0] / CELL_X)
        y = MAP_GRID_SIZE_Y - 1 - int(kp[i].pt[1] / CELL_Y)
        grid_kp[x][y].append(kp[i])
        grid_des[x][y].append(des[i])

    # group every 3 by 3 grid, so we can access each center of grid and its 8 neighbors easily
    map_grid_kp = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    map_grid_des = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    for i in range(MAP_GRID_SIZE_X):
        for j in range(MAP_GRID_SIZE_Y):
            for k in range(-1, 2):
                for l in range(-1, 2):
                    x = i + k
                    y = j + l
                    if 0 <= x < MAP_GRID_SIZE_X and 0 <= y < MAP_GRID_SIZE_Y:
                        map_grid_kp[i][j].extend(grid_kp[x][y])
                        map_grid_des[i][j].extend(grid_des[x][y])
            map_grid_des[i][j] = np.array(map_grid_des[i][j])

    return map_grid_kp, map_grid_des


def norm_pdf(x, mu, sigma):
    u = (x - mu) / float(abs(sigma))
    y = (1 / (np.sqrt(2 * np.pi) * abs(sigma))) * np.exp(-u * u / 2.)
    return y
