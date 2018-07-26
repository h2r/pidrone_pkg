"""""
offboard_slam.py

Runs fastSLAM for the pidrone off board, on the pi run 'image_pub.py'
"""""

import numpy as np
import cv2
from pidrone_pkg.msg import axes_err, Mode, ERR
from sensor_msgs.msg import Image, Range, CameraInfo
from std_msgs.msg import Empty
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from pid_class import PIDaxis
from geometry_msgs.msg import TwistStamped
from slam_helper import FastSLAM, PROB_THRESHOLD
import math


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# assume a pixel in x and y has the same length
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
MAX_BAD_COUNT = -1000
NUM_PARTICLE = 10
NUM_FEATURES = 50


class AnalyzePhase:

    def __init__(self):
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        self.lr_pid = PIDaxis(10.0, 0.000, 0.0, midpoint=0, control_range=(-5.0, 5.0))
        self.fb_pid = PIDaxis(10.0, 0.000, 0.0, midpoint=0, control_range=(-5.0, 5.0))

        self.detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
        self.estimator = FastSLAM()

        self.first_locate = True
        self.first_hold = True
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.locate_position = False
        self.prev_time = None
        self.prev_rostime = None
        self.pos = [0, 0, 0]
        self.yaw = 0.0
        self.z = 0.075
        self.iacc_yaw = 0.0
        self.hold_position = False
        self.target_pos = [0, 0, 0]
        self.target_yaw = 0.0
        self.map_counter = 0
        self.max_map_counter = 0
        self.mode = Mode()
        self.mode.mode = 5
        # constant
        self.kp_yaw = 50.0
        self.ki_yaw = 0.1
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int
        # angle
        self.angle_x = 0.0  # the hz of state_controller is different
        self.angle_y = 0.0

        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber('/pidrone/angle', TwistStamped, self.angle_callback)
        rospy.Subscriber('/pidrone/picamera/image_raw', Image, self.image_callback)
        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

    def image_callback(self, data):
        curr_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        # start SLAM
        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and curr_kp is not None:
                # generate particles for the first time
                if self.first_locate:
                    pose = self.estimator.generate_particles(NUM_PARTICLE)
                    self.first_locate = False
                    self.pos = pose
                    self.yaw = pose[3]
                    print 'first', pose
                else:
                    # get a estimate velocity over time
                    pose, weight = self.estimator.run(self.z, self.angle_x,
                                                      self.angle_y, self.prev_kp, self.prev_des, curr_kp, curr_des)

                    # update position
                    self.pos = [self.hybrid_alpha * pose[0] + (1.0 - self.hybrid_alpha) * self.pos[0],
                                self.hybrid_alpha * pose[1] + (1.0 - self.hybrid_alpha) * self.pos[1],
                                self.z]
                    self.yaw = self.alpha_yaw * pose[3] + (1.0 - self.alpha_yaw) * self.yaw

                    print '--pose', self.pos[0], self.pos[1], self.yaw
                    print '--weight', weight

                    # if all particles are not good estimations
                    if is_almost_equal(weight, NUM_FEATURES*math.log(PROB_THRESHOLD)):
                        self.map_counter = self.map_counter - 1
                    elif self.map_counter <= 0:
                        self.map_counter = 1
                    else:
                        self.map_counter = min(self.map_counter + 1, -MAX_BAD_COUNT)

                    # if it's been a while without a high average particle weight
                    if self.map_counter < MAX_BAD_COUNT:
                        self.first_locate = True
                        self.fb_pid._i = 0
                        self.lr_pid._i = 0
                        self.iacc_yaw = 0.0
                        self.map_counter = 0
                        self.mode.x_velocity = 0
                        self.mode.y_velocity = 0
                        self.mode.yaw_velocity = 0
                        self.pospub.publish(self.mode)
                        print 'Restart SLAM'
                    else:
                        if self.hold_position:
                            if self.first_hold:
                                self.target_pos = self.pos
                                self.target_yaw = 0  # rotate is not implement
                                self.first_hold = False
                                image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
                                self.first_image_pub.publish(image_message)
                            else:
                                err_x = self.target_pos[0] - self.pos[0]
                                err_y = self.target_pos[1] - self.pos[1]
                                self.mode.x_velocity = self.lr_pid.step(err_x, curr_time - self.prev_time)
                                self.mode.y_velocity = self.fb_pid.step(err_y, curr_time - self.prev_time)
                                err_yaw = self.target_yaw - self.yaw
                                self.iacc_yaw += err_yaw * self.ki_yaw
                                self.mode.yaw_velocity = err_yaw * self.kp_yaw + self.iacc_yaw
                                self.pospub.publish(self.mode)
                            print '--target', self.target_pos[0], self.target_pos[1], self.target_yaw

                    print 'count', self.map_counter
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

            self.prev_kp = curr_kp
            self.prev_des = curr_des

        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                              rospy.Time.now(),
                              "base",
                              "world")

    # the angle is just estimate
    def angle_callback(self, data):
        self.angle_x = data.twist.angular.x
        self.angle_y = data.twist.angular.y

    def range_callback(self, data):
        if data.range != -1:
            self.z = data.range

    def reset_callback(self, data):
        print "Start localization"
        self.locate_position = True
        self.first_locate = True
        self.hold_position = False
        self.map_counter = 0
        self.max_map_counter = 0

    def toggle_callback(self, data):
        self.hold_position = not self.hold_position
        self.first_hold = True
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        self.iacc_yaw = 0.0
        print "Position hold", "enabled." if self.hold_position else "disabled."

    def mode_callback(self, data):
        if not self.hold_position or data.mode == 4 or data.mode == 3:
            print "VELOCITY"
            # TODO scale is not consistent, check index.html and pid_class.py
            data.z_velocity = data.z_velocity * 100
            self.pospub.publish(data)
        else:
            self.target_pos[0] += data.x_velocity / 100.
            self.target_pos[1] += data.y_velocity / 100.
            print "Target position", self.target_pos


def is_almost_equal(x, y):
    """""
    Is the difference between average weight and the lower bound on weight within 25% of the lower bound?
    """""
    epsilon = 0.25 * y
    return abs(x-y) <= abs(epsilon)


def main():
    rospy.init_node('camera_off_board')

    phase_analyzer = AnalyzePhase()
    print "Start"

    rospy.spin()


if __name__ == '__main__':
    main()