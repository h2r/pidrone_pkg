"""
slam.py

Runs fastSLAM algorithm for the PiDrone. Helper code located in slam_helper.py
"""

import numpy as np
import picamera
import picamera.array
import cv2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Range, CameraInfo
import rospy
import tf
from slam_helper import FastSLAM

# ---------- camera parameters DO NOT EDIT ----------- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)

# ---------- SLAM parameters ----------- #
NUM_PARTICLE = 7
NUM_FEATURES = 30


class SLAM(picamera.array.PiMotionAnalysis):

    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.br = tf.TransformBroadcaster()

        # setup publishers
        self.posepub = rospy.Publisher('/pidrone/picamera/pose', PoseStamped, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
        self.estimator = FastSLAM()

        self.angle_x = 0.0
        self.angle_y = 0.0

        self.first_locate = True
        self.locate_position = False

        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None

        self.prev_rostime = None
        self.pos = [0, 0, 0, 0]
        self.posemsg = PoseStamped()
        self.z = 0.0

        # constant
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_rostime = rospy.Time.now()
        self.posemsg.header.stamp = curr_rostime
        curr_time = curr_rostime.to_sec()

        # start SLAM
        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and len(curr_kp) != 0:
                # generate particles for the first time
                if self.first_locate:
                    pose = self.estimator.generate_particles(NUM_PARTICLE)
                    self.first_locate = False
                    self.pos = pose

                    self.posemsg.pose.position.x = self.pos[0]
                    self.posemsg.pose.position.y = self.pos[1]
                    self.posemsg.pose.position.z = self.pos[2]
                    x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.pos[3])

                    self.posemsg.pose.orientation.x = x
                    self.posemsg.pose.orientation.y = y
                    self.posemsg.pose.orientation.z = z
                    self.posemsg.pose.orientation.w = w

                    self.posepub.publish(self.posemsg)
                    print 'first', pose
                else:
                    pose, weight = self.estimator.run(self.z, self.prev_kp, self.prev_des,
                                                      curr_kp, curr_des)

                    # update position
                    self.pos = [self.hybrid_alpha * pose[0] + (1.0 - self.hybrid_alpha) * self.pos[0],
                                self.hybrid_alpha * pose[1] + (1.0 - self.hybrid_alpha) * self.pos[1],
                                self.z,
                                self.alpha_yaw * pose[3] + (1.0 - self.alpha_yaw) * self.pos[3]]

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
                    print '--weight', weight

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

