"""""
MATL.py

Runs "MATL" for the PiDrone, MATL being "Mapping and then Localization" (as opposed to "Simultaneous Mapping and 
Localization" for SLAM). To use, run the script and press 'm' to begin mapping. The image data are saved and then run
through FastSLAM once you land and press 'm' a second time. Once the map is complete, press 'r' to start localization
and fly around!
"""""

import numpy as np
import picamera
import picamera.array
import cv2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Range, CameraInfo
import rospy
import tf
import sys
from MATL_slam_helper import FastSLAM
from MATL_helper import PROB_THRESHOLD, LocalizationParticleFilter

# ---------- camera parameters DO NOT EDIT ----------- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)

# ---------- SLAM parameters ----------- #
MAX_BAD_COUNT = -1000
SLAM_PARTICLE = 20
LOCALIZATION_PARTICLE = 40
NUM_FEATURES = 50


class MATL(picamera.array.PiMotionAnalysis):
    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.br = tf.TransformBroadcaster()

        self.posepub = rospy.Publisher('/pidrone/picamera/pose', PoseStamped, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
        self.SLAM_estimator = FastSLAM()
        self.localization_estimator = None

        self.angle_x = 0.0
        self.angle_y = 0.0

        self.first_locate = True

        self.map_built = False
        self.build_map = False
        self.compute_map = False

        self.state = "START"

        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None

        self.prev_rostime = None
        self.pos = [0, 0, 0, 0]
        self.posemsg = PoseStamped()
        self.z = 0.0

        self.map_counter = 0
        self.max_map_counter = 0

        self.map_data = []
        self.z_data = []

        # constant
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_rostime = rospy.Time.now()
        self.posemsg.header.stamp = curr_rostime
        curr_time = curr_rostime.to_sec()

        # The standby state does not require kp and des
        if self.state == "STANDBY" and self.compute_map:

            # create a SLAM object and the denominator for progress percentage
            self.SLAM_estimator.generate_particles(SLAM_PARTICLE)
            denominator = len(self.map_data)

            # run stored map data through fastSLAM to produce a map
            for i in range(1, len(self.map_data)):
                self.SLAM_estimator.run(self.z_data[i-1], self.map_data[i-1][0], self.map_data[i-1][1], self.map_data[i][0],
                                        self.map_data[i][1])
                # make a nice-looking progress bar
                percent = int(round(float(i) / denominator, 2) * 100)
                sys.stdout.write("\rIncorporated %d%% of data into map!" % percent)
                sys.stdout.flush()

            # find the particle with the best map
            particle_weights = [p.weight for p in self.SLAM_estimator.particles]
            best_index = particle_weights.index(max(particle_weights))
            map = self.SLAM_estimator.particles[best_index].landmarks
            self.compute_map = False

            # initialize a localization object and sort the kp and des into grids
            map_kp = [[lm.x, lm.y] for lm in map]
            map_des = [[d for d in lm.des] for lm in map]
            self.localization_estimator = LocalizationParticleFilter()
            self.localization_estimator.create_map(map_kp, map_des)

            print '\n'
            print "The map is completed with ", len(map), "landmarks!"
            print self.state
        else:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and curr_kp is not None:

                if self.state == "MAPPING":
                    self.map_data.append((curr_kp, curr_des))
                    self.z_data.append(self.z)
                elif self.state == "LOCALIZING":
                        if self.first_locate:
                            particle = self.localization_estimator.initialize_particles(LOCALIZATION_PARTICLE, curr_kp,
                                                                                        curr_des)
                            self.first_locate = False
                            self.pos = [particle.x(), particle.y(), particle.z(), particle.yaw()]

                            self.posemsg.pose.position.x = self.pos[0]
                            self.posemsg.pose.position.y = self.pos[1]
                            self.posemsg.pose.position.z = self.pos[2]
                            x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.pos[3])

                            self.posemsg.pose.orientation.x = x
                            self.posemsg.pose.orientation.y = y
                            self.posemsg.pose.orientation.z = z
                            self.posemsg.pose.orientation.w = w

                            self.posepub.publish(self.posemsg)

                            print 'first', particle
                        else:
                            particle = self.localization_estimator.update(self.z, self.angle_x, self.angle_y,
                                                                        self.prev_kp, self.prev_des, curr_kp, curr_des)

                            # update position
                            self.pos = [self.hybrid_alpha * particle.x() + (1.0 - self.hybrid_alpha) * self.pos[0],
                                        self.hybrid_alpha * particle.y() + (1.0 - self.hybrid_alpha) * self.pos[1],
                                        self.z,
                                        self.alpha_yaw * particle.yaw() + (1.0 - self.alpha_yaw) * self.pos[3]]

                            self.posemsg.pose.position.x = self.pos[0]
                            self.posemsg.pose.position.y = self.pos[1]
                            self.posemsg.pose.position.z = self.pos[2]
                            x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.pos[3])

                            self.posemsg.pose.orientation.x = x
                            self.posemsg.pose.orientation.y = y
                            self.posemsg.pose.orientation.z = z
                            self.posemsg.pose.orientation.w = w

                            self.posepub.publish(self.posemsg)

                            print '--pose', self.pos[0], self.pos[1], self.pos[3]

                            # if average particle weight is close to initial weight
                            if is_almost_equal(particle.weight(), PROB_THRESHOLD):
                                self.map_counter = self.map_counter - 1
                            elif self.map_counter <= 0:
                                self.map_counter = 1
                            else:
                                self.map_counter = min(self.map_counter + 1, -MAX_BAD_COUNT)

                            # if it's been a while without a significant average weight
                            if self.map_counter < MAX_BAD_COUNT:
                                self.first_locate = True
                                self.map_counter = 0
                                print 'Restart localization'
                            print 'count', self.map_counter

                self.prev_kp = curr_kp
                self.prev_des = curr_des

            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.pos[3]),
                              rospy.Time.now(),
                              "base",
                              "world")

    def reset_callback(self, data):
        """
        start localization when '/pidrone/reset_transform' is published to (press 'r')
        """
        self.first_locate = True
        self.map_counter = 0
        self.max_map_counter = 0

        if self.state == "LOCALIZING":
            self.state = "STANDBY"
        elif self.state == "STANDBY":
            self.state = "LOCALIZING"
        else:
            print "ILLEGAL STATE"

        print self.state

    def map_callback(self, data):
        """
        gets called when you press 'm' in the web interface (publish to /pidrone/map)
        the first time you press, it will start saving the kp and des, and the second you press, it will
        start building the map, and from then on it starts alternating
        """
        if self.state == "START":
            self.state = "MAPPING"
        elif self.state == "MAPPING":
            self.state = "STANDBY"
            self.compute_map = True
        elif self.state == "STANDBY":
            self.state = "MAPPING"
        else:
            print "ILLEGAL STATE"

        print self.state

    def state_callback(self, data):
        """ update z, angle x, and angle y data when /pidrone/state is published to """
        self.z = data.pose_with_covariance.pose.position.z
        self.angle_x = data.twist_with_covariance.twist.angular.x
        self.angle_y = data.twist_with_covariance.twist.angular.y


def is_almost_equal(x, y):
    epsilon = 1 * 10 ** (-8)
    return abs(x - y) <= epsilon


