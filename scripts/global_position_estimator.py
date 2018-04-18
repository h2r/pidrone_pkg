import rospy
import math
import random
import numpy as np
import cv2
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


SIZE_GRID_X = 8
SIZE_GRID_Y = 8
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
MAP_WIDTH = 1024    # in pixel
MAP_HEIGHT = 768
MAP_REAL_WIDTH = 0.7    # in meter
MAP_REAL_HEIGHT = 0.54
METER_TO_PIXEL = (MAP_WIDTH / MAP_REAL_WIDTH + MAP_HEIGHT / MAP_REAL_HEIGHT) / 2.
MIN_MATCH_COUNT = 10
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)


class LocalizationParticleFilter:
    """
    Particle filter for localization.  
    """
    
    def __init__(self):
        self.map_kp = []
        self.map_des = []

        self.z = 0
        self.angle_x = 0
        self.angle_y = 0

        # parameters - noise on sensor.
        # TODO check the parameters and covariance matrix is proper
        sigma_vx = 0.1
        sigma_vy = 0.1
        sigma_vz = 0.0
        sigma_yaw = 0.01
        self.covariance_motion = np.array([[sigma_vx**2, 0, 0, 0],
                                           [0, sigma_vy**2, 0, 0],
                                           [0, 0, sigma_vz**2, 0],
                                           [0, 0, 0, sigma_yaw**2]])
        self.sigma_x = 0.1
        self.sigma_y = 0.1
        self.sigma_yaw = 0.1
        self.particles = [] # initialize to random particles spread over map. x, y, z, yaw
        self.previous_time = None
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
    
    def sample_motion_model(self, ut):
        """
        Implement motion model from Equation 3 in PiDrone Slam with noise. 
        """
        # compute xtp1, with equation 3.
        # xtp1 = 0 # A * xt + B * ut

        # add random gaussian noise, but use matpl.
        # xtp1 += np.random.normal(na.zeros(xt.shape), self.sigma)

        # update particles as matrix
        current_rostime = rospy.Time.now()
        current_time = current_rostime.to_sec()
        delta_time = current_time - self.previous_time

        noisy_ut = np.random.multivariate_normal(ut, self.covariance_motion, len(self.particles))

        old_yaw = self.particles[:, 3]
        self.particles[:, 0] += (noisy_ut[:, 0] * np.cos(old_yaw) + noisy_ut[:, 1] * np.sin(old_yaw)) * delta_time
        self.particles[:, 1] += (noisy_ut[:, 0] * np.sin(old_yaw) + noisy_ut[:, 1] * np.cos(old_yaw)) * delta_time
        self.particles[:, 3] += noisy_ut[:, 3] * delta_time

        self.previous_time = current_time

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
        
        # j, r, phi = zt
        # mx, my = self.map[j]
        # hatr = math.sqrt(math.pow(mx - x,2) + math.pow(my - y,2))
        # hatphi = math.atan2(my - y, mx - x)
        # q = (random.normalvariate(r - hatr, self.sigmar) *
        #      random.normalvariate(phi - hatphi, self.sigmaphi))
        # return q

        # pick 8 cells around the center
        num_grid_x = MAP_REAL_WIDTH / SIZE_GRID_X
        num_grid_y = MAP_REAL_HEIGHT / SIZE_GRID_Y
        drone_grid_x = int(xt[0] / num_grid_x)
        drone_grid_y = int(xt[1] / num_grid_y)
        sub_map_kp = []
        sub_map_des = []
        for i in range(-1,2):
            for j in range(-1,2):
                x = drone_grid_x + i
                y = drone_grid_y + j
                if 0 <= x < SIZE_GRID_X and 0 <= y < SIZE_GRID_Y:
                    sub_map_kp.append(self.map_kp[x][y])
                    sub_map_des.append(self.map_des[x][y])

        # measure current global position
        matches = self.matcher.knnMatch(des, sub_map_des, k=2)
        good = []
        pose = None
        for match in matches:
            if len(match) > 1 and match[0].distance < 0.7 * match[1].distance:
                good.append(match[0])
        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([sub_map_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
            if transform is not None:
                transformed_center = cv2.transform(CAMERA_CENTER, transform)  # get global pixel
                transformed_center = [transformed_center[0][0][0] / float(METER_TO_PIXEL),  # map to global pose
                                      (MAP_HEIGHT - transformed_center[0][0][1]) / float(METER_TO_PIXEL)]
                yaw = np.arctan2(transform[1, 0], transform[0, 0])  # get global heading

                # correct the pose if the drone is not level
                z = math.sqrt(self.z ** 2 / (1 + math.tan(self.angle_x) ** 2 + math.tan(self.angle_y) ** 2))
                offset_x = np.tan(self.angle_x) * z
                offset_y = np.tan(self.angle_y) * z
                global_offset_x = math.cos(yaw) * offset_x + math.sin(yaw) * offset_y
                global_offset_y = math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
                pose = [transformed_center[0] + global_offset_x, transformed_center[1] + global_offset_y, self.z, yaw]

        # compute weight of particle
        if pose is None:
            q = 0
        else:
            q = random.normalvariate(pose[0]-xt[0], self.sigma_x) * random.normalvariate(pose[1]-xt[1], self.sigma_y) * random.normalvariate(pose[3]-xt[3], self.sigma_yaw)

        return q

    def update(self, ut, zt):
        """
        We implement the MCL algorithm from probabilistic robotics (Table 8.2)
        zt is the array of detected features.
        """
        newparticles = []
        weights = []
        for (xtm1, wtm) in self.particles:
            xtm = self.sample_motion_model(ut, xtm1)
            wtm = self.measurement_model(zt, xtm)
            newparticles.append((xtm, wtm))
            weights.append(wtm)

        weights = weights / np.sum(weights) # normalize
        
        samples = np.random.multinomial(len(self.particles), weights)

        self.particles = [newparticles[i] for i in samples]


def split_map_into_grid(kp, des):
    grid_k = []
    grid_d = []
    for i in range(SIZE_GRID_X):
        grid_k.append([])
        grid_d.append([])
        for j in range(SIZE_GRID_Y):
            grid_k[i].append([])
            grid_d[i].append([])

    num_grid_x = MAP_WIDTH / SIZE_GRID_X
    num_grid_y = MAP_HEIGHT / SIZE_GRID_Y
    for i in range(len(kp)):
        x = int(kp[i].pt[0] / num_grid_x)
        y = int(kp[i].pt[1] / num_grid_y)
        grid_k[x][y].append(kp[i])
        grid_d[x][y].append(des[i])

    return grid_k, grid_d

        
def main():
    rospy.init_node('global_position_estimator')

if __name__ == '__main__':
    main()
