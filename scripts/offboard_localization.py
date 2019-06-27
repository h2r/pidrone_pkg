"""""
picam_localization_distance

implements Monte-Carlo Localization using the pi-camera
"""""

import numpy as np
import cv2
from pidrone_pkg.msg import Mode
from sensor_msgs.msg import Image, Range, CameraInfo
from std_msgs.msg import Empty
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from pidrone_pkg.msg import State
from localization_helper import LocalizationParticleFilter, create_map, PROB_THRESHOLD
import os

# ---------- map parameters ----------- #
MAP_PIXEL_WIDTH = 3227  # in pixel
MAP_PIXEL_HEIGHT = 2447
MAP_REAL_WIDTH = 1.4  # in meter
MAP_REAL_HEIGHT = 1.07
# ------------------------------------- #

# ---------- camera parameters ----------- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
# ---------------------------------------- #

# ---------- localization parameters ----------- #
MAX_BAD_COUNT = -10
NUM_PARTICLE = 30
NUM_FEATURES = 200
# ---------------------------------------------- #


class AnalyzePhase:

    def __init__(self):
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        self.posepub = rospy.Publisher('/pidrone/picamera/pose', PoseStamped, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.detector = cv2.ORB_create(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
        map_grid_kp, map_grid_des = create_map('map.jpg')
        self.estimator = LocalizationParticleFilter(map_grid_kp, map_grid_des)

        # [x, y, z, yaw]
        self.pos = [0, 0, 0, 0]
        self.posemsg = PoseStamped()

        self.angle_x = 0.0
        self.angle_y = 0.0
        # localization does not estimate z
        self.z = 0.0

        self.first_locate = True
        self.locate_position = False

        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None
        self.prev_rostime = None

        self.map_counter = 0
        self.max_map_counter = 0

        # constant
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int

    def image_callback(self, data):
        curr_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        curr_rostime = rospy.Time.now()
        self.posemsg.header.stamp = curr_rostime
        curr_time = curr_rostime.to_sec()

        # start MCL localization
        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and curr_kp is not None:
                # generate particles for the first time
                if self.first_locate:
                    particle = self.estimator.initialize_particles(NUM_PARTICLE, curr_kp, curr_des)
                    self.first_locate = False
                    self.pos = [particle.x(), particle.y(), particle.z(), particle.yaw()]

                    self.posemsg.pose.position.x = particle.x()
                    self.posemsg.pose.position.y = particle.y()
                    self.posemsg.pose.position.z = particle.z()
                    x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.pos[3])

                    self.posemsg.pose.orientation.x = x
                    self.posemsg.pose.orientation.y = y
                    self.posemsg.pose.orientation.z = z
                    self.posemsg.pose.orientation.w = w

                    print 'first', particle
                else:
                    particle = self.estimator.update(self.z, self.angle_x, self.angle_y, self.prev_kp, self.prev_des,
                                                     curr_kp, curr_des)

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

                    # if all particles are not good estimations
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
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

            self.prev_kp = curr_kp
            self.prev_des = curr_des

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.pos[3]),
                              rospy.Time.now(),
                              "base",
                              "world")

    def state_callback(self, data):
        """ update z, angle x, and angle y data when /pidrone/state is published to """
        self.z = data.pose_with_covariance.pose.position.z
        self.angle_x = data.twist_with_covariance.twist.angular.x
        self.angle_y = data.twist_with_covariance.twist.angular.y

    def reset_callback(self, data):
        """start localization when '/pidrone/reset_transform' is published to (press 'r')"""
        print "Start localization"
        self.locate_position = True
        self.first_locate = True
        self.map_counter = 0
        self.max_map_counter = 0


def is_almost_equal(x, y):
    epsilon = 1*10**(-8)
    return abs(x-y) <= epsilon


def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    
    phase_analyzer = AnalyzePhase()

    phase_analyzer = AnalyzePhase()
    rospy.Subscriber("/pidrone/reset_transform", Empty, phase_analyzer.reset_callback)
    rospy.Subscriber('/pidrone/picamera/image_raw', Image, phase_analyzer.image_callback)
    rospy.Subscriber('/pidrone/state', State, phase_analyzer.state_callback)

    print "Start"

    rospy.spin()


if __name__ == '__main__':
    main()
