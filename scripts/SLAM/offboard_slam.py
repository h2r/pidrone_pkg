"""""
offboard_slam.py

Runs fastSLAM for the pidrone off board, on the pi run 'image_pub.py'
"""""

import numpy as np
import cv2
from pidrone_pkg.msg import Mode
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Empty
import rospy
import tf
from cv_bridge import CvBridge
from pid_class import PIDaxis
from geometry_msgs.msg import TwistStamped
from slam_helper import FastSLAM, PROB_THRESHOLD
import math


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# assume a pixel in x and y has the same length
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
MAX_BAD_COUNT = -100
NUM_PARTICLE = 20
NUM_FEATURES = 50


class AnalyzePhase:

    def __init__(self):
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        # bind method calls to subscribed topics
        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber('/pidrone/angle', TwistStamped, self.angle_callback)
        rospy.Subscriber('/pidrone/picamera/image_raw', Image, self.image_callback)

        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.lr_pid = PIDaxis(10.0, 0.000, 0.0, midpoint=0, control_range=(-5.0, 5.0))
        self.fb_pid = PIDaxis(10.0, 0.000, 0.0, midpoint=0, control_range=(-5.0, 5.0))

        self.detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
        self.estimator = FastSLAM()

        self.angle_x = 0.0
        self.angle_y = 0.0

        self.first_locate = True
        self.locate_position = False

        self.first_hold = True
        self.hold_position = False

        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None

        self.prev_rostime = None
        self.pos = [0, 0, 0, 0]
        self.target_pos = [0, 0, 0, 0]
        self.z = 0.0

        self.map_counter = 0
        self.max_map_counter = 0

        self.mode = Mode()
        self.mode.mode = 5

        # constant
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int

    def image_callback(self, data):
        curr_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        curr_rostime = rospy.Time.now()
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
                    print 'first', pose
                else:
                    pose, weight = self.estimator.run(self.z, self.prev_kp, self.prev_des,
                                                      curr_kp, curr_des)

                    # update position
                    self.pos = [self.hybrid_alpha * pose[0] + (1.0 - self.hybrid_alpha) * self.pos[0],
                                self.hybrid_alpha * pose[1] + (1.0 - self.hybrid_alpha) * self.pos[1],
                                self.z,
                                self.alpha_yaw * pose[3] + (1.0 - self.alpha_yaw) * self.pos[3]]

                    print '--pose', self.pos[0], self.pos[1], self.pos[3]
                    print '--weight', weight

                    # if average weight is close to the worst possible weight
                    if scale_weight(weight) < 0.2:
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
                                self.first_hold = False
                                image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
                                self.first_image_pub.publish(image_message)
                            else:
                                err_x = self.target_pos[0] - self.pos[0]
                                err_y = self.target_pos[1] - self.pos[1]
                                self.mode.x_velocity = self.lr_pid.step(err_x, curr_time - self.prev_time)
                                self.mode.y_velocity = self.fb_pid.step(err_y, curr_time - self.prev_time)
                                self.mode.yaw_velocity = 0
                                self.pospub.publish(self.mode)
                            print '--target', self.target_pos[0], self.target_pos[1], self.target_pos[3]

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

    def angle_callback(self, data):
        """
        updates angle data when '/pidrone/angle' is published to
        """
        self.angle_x = data.twist.angular.x
        self.angle_y = data.twist.angular.y

    def range_callback(self, data):
        """
        update z when '/pidrone/infrared' is published to
        """
        if data.range != -1:
            self.z = data.range

    def reset_callback(self, data):
        """
        start localization when '/pidrone/reset_transform' is published to (press 'r')
        """
        print "Start SLAM"
        self.locate_position = True
        self.first_locate = True
        self.hold_position = False
        self.map_counter = 0
        self.max_map_counter = 0

    def toggle_callback(self, data):
        """
        toggle position hold when '/pidrone/toggle_transform' is published to (press 'p')
        """
        self.hold_position = not self.hold_position
        self.first_hold = True
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        print "Position hold", "enabled." if self.hold_position else "disabled."

    def mode_callback(self, data):
        """
        update the mode whenever '/pidrone/set_mode' is published to, velocity mode is default
        """
        self.mode.mode = data.mode
        if not self.hold_position or data.mode == 4 or data.mode == 3:
            print "VELOCITY"
            # TODO scale is not consistent, check index.html and pid_class.py
            data.z_velocity = data.z_velocity * 100
            self.pospub.publish(data)
        else:
            self.target_pos[0] += data.x_velocity / 100.
            self.target_pos[1] += data.y_velocity / 100.
            print "Target position", self.target_pos


def scale_weight(w):
    """
    :param w: the current average weight of the particles
    :return: the weight scaled from the (0, worst_possible_weight) to (0,1)
    """
    worst_possible_weight = NUM_FEATURES * (math.log(PROB_THRESHOLD) + math.log(0.1 * PROB_THRESHOLD))
    # subtract from 1 so that higher is better
    return 1 - (w / worst_possible_weight)


def main():
    rospy.init_node('camera_off_board')

    phase_analyzer = AnalyzePhase()
    print "Start"

    rospy.spin()


if __name__ == '__main__':
    main()
