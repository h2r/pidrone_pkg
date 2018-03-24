import numpy as np
import picamera
import picamera.array
import cv2
import rospy
import time
import sys
from h2rMultiWii import MultiWii
from picam_flow_class import AnalyzeFlow
from pidrone_pkg.msg import axes_err, Mode, ERR
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Range, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty
import rospy
import tf
import copy
from pyquaternion import Quaternion
import time
from cv_bridge import CvBridge, CvBridgeError
import sys
from pid_class import PIDaxis
import math
import camera_info_manager


class AnalyzePhase(picamera.array.PiMotionAnalysis):

    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.br = tf.TransformBroadcaster()

        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.lr_pid = PIDaxis(0.0400, -0.00000, 0.000, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(0.0400, 0.0000, -0.000, midpoint=0, control_range=(-10.0, 10.0))

        self.prev_img = None
        self.detector = cv2.ORB(nfeatures=120)
        self.index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        self.search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        self.first_kp = None
        self.first_des = None
        self.prev_kp = None
        self.prev_des = None
        self.first = True
        self.lr_err = ERR()
        self.fb_err = ERR()
        self.prev_time = None
        self.prev_rostime = None
        self.pos = [0, 0]
        self.z = 0.075
        self.smoothed_yaw = 0.0
        self.yaw_observed = 0.0
        self.iacc_yaw = 0.0
        self.transforming = False
        self.last_first_time = None
        self.target_x = 0
        self.target_y = 0
        self.first_counter = 0
        self.max_first_counter = 0
        self.mode = Mode()
        # constant
        self.kp_yaw = 80.0
        self.ki_yaw = 0.1
        self.kpi_yaw = 15.0
        self.kpi_max_yaw = 0.01
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.1  # blend position with first frame and int

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
        curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        if self.first:
            print "taking new first"
            self.first = False
            self.first_kp = curr_kp
            self.first_des = curr_des
            self.last_first_time = curr_time

            image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
            self.first_image_pub.publish(image_message)

        elif self.transforming:
            # move the drone to the right, x is negative.
            # move the drone to the front, y is positive.
            corr_first = self.feature_transform(self.first_kp, self.first_des, curr_kp, curr_des)

            if corr_first is not None:
                self.last_first_time = curr_time
                self.first_counter += 1
                self.max_first_counter = max(self.first_counter, self.max_first_counter)

                # X and Y
                first_displacement = [-corr_first[0, 2] / 320., corr_first[1, 2] / 240.]
                self.pos = [
                    self.hybrid_alpha * first_displacement[0] * self.z + (1.0 - self.hybrid_alpha) * self.pos[0],
                    self.hybrid_alpha * first_displacement[1] * self.z + (1.0 - self.hybrid_alpha) * self.pos[1]]
                self.lr_err.err = self.target_x - self.pos[0]
                self.fb_err.err = self.target_y - self.pos[1]
                print "ERR", self.lr_err.err, self.fb_err.err
                self.mode.x_i = self.lr_pid.step(self.lr_err.err, curr_time - self.prev_time)
                self.mode.y_i = self.fb_pid.step(self.fb_err.err, curr_time - self.prev_time)
                self.mode.x_velocity = self.mode.x_i
                self.mode.y_velocity = self.mode.y_i

                # Yaw
                scalex = np.linalg.norm(corr_first[:, 0])
                scalez = np.linalg.norm(corr_first[:, 1])
                corr_first[:, 0] /= scalex
                corr_first[:, 1] /= scalez
                self.yaw_observed = np.arctan2(corr_first[1, 0], corr_first[0, 0])
                self.smoothed_yaw = (1.0 - self.alpha_yaw) * self.smoothed_yaw + self.alpha_yaw * self.yaw_observed
                yaw = self.smoothed_yaw
                self.iacc_yaw += yaw * self.ki_yaw
                yaw_kpi_term = np.sign(yaw) * yaw * yaw * self.kpi_yaw
                if abs(yaw_kpi_term) < self.kpi_max_yaw:
                    self.iacc_yaw += yaw_kpi_term
                else:
                    self.iacc_yaw += np.sign(yaw) * self.kpi_max_yaw
                self.mode.yaw_velocity = yaw * self.kp_yaw + self.iacc_yaw
                print "yaw iacc: ", self.iacc_yaw

                self.pospub.publish(self.mode)
                print "first", self.max_first_counter, self.first_counter
            else:
                self.first_counter = 0
                corr_int = self.feature_transform(self.prev_kp, self.prev_des, curr_kp, curr_des)

                if corr_int is not None:
                    time_since_first = curr_time - self.last_first_time
                    print "integrated", time_since_first
                    print "max_first_counter: ", self.max_first_counter

                    # X and Y
                    int_displacement = [-corr_int[0, 2] / 320., corr_int[1, 2] / 240.]
                    self.pos[0] += int_displacement[0] * self.z
                    self.pos[1] += int_displacement[1] * self.z
                    self.lr_err.err = self.target_x - self.pos[0]
                    self.fb_err.err = self.target_y - self.pos[1]
                    print "ERR", self.lr_err.err, self.fb_err.err
                    self.mode.x_i = self.lr_pid.step(self.lr_err.err, curr_time - self.prev_time)
                    self.mode.y_i = self.fb_pid.step(self.fb_err.err, curr_time - self.prev_time)
                    cvc_vel = 1.1  # 0.25 #1.0
                    self.mode.x_velocity = cvc_vel * self.mode.x_i
                    self.mode.y_velocity = cvc_vel * self.mode.y_i

                    # Yaw
                    self.mode.yaw_velocity = self.iacc_yaw
                    print "yaw iacc: ", self.iacc_yaw

                    self.pospub.publish(self.mode)
                else:
                    print "LOST"
        else:
            corr_first = self.feature_transform(self.first_kp, self.first_des, curr_kp, curr_des)
            if corr_first is not None:
                self.last_first_time = rospy.get_time()
                if curr_time - self.last_first_time > 2:
                    self.first = True
            else:
                print "no first", curr_time - self.last_first_time

        self.prev_img = curr_img
        self.prev_kp = curr_kp
        self.prev_des = curr_des
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        # TODO should be z instead of pos[2], which has never been updated
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.yaw_observed),
                              rospy.Time.now(),
                              "base",
                              "world")

    def feature_transform(self, kp1, des1, kp2, des2):
        transform = None

        if des1 is not None and des2 is not None:
            matches = self.matcher.knnMatch(des1, des2, k=2)

            good = []
            for match in matches:
                if len(match) > 1 and match[0].distance < 0.7 * match[1].distance:
                    good.append(match[0])

            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            if src_pts is not None and dst_pts is not None and len(src_pts) > 3 and len(dst_pts) > 3:
                transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)

        return transform

    def range_callback(self, data):
        if data.range != -1:
            self.z = data.range

    def reset_callback(self, data):
        print "Resetting Phase"
        self.first = True
        self.pos = [0, 0]
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        self.target_x = 0
        self.target_y = 0
        self.smoothed_yaw = 0.0
        self.iacc_yaw = 0.0
        self.first_counter = 0
        self.max_first_counter = 0

    def toggle_callback(self, data):
        self.transforming = not self.transforming
        print "Position hold", "enabled." if self.transforming else "disabled."

    def mode_callback(self, data):
        self.mode.mode = data.mode
        if not self.transforming or data.mode == 4 or data.mode == 3:
            print "VELOCITY"
            self.pospub.publish(data)
        else:
            # TODO 4 is used for what ? Should the target be accumulated ?
            self.target_x = data.x_velocity * 4.
            self.target_y = data.y_velocity * 4.
            print "POSITION", self.target_x, self.target_y


def main():
    rospy.init_node('flow_pub')

    velpub = rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)
    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")

    try:
        velocity = axes_err()
        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with AnalyzeFlow(camera) as flow_analyzer:
                flow_analyzer.bridge = bridge
                flow_analyzer.setup(camera.resolution)
                phase_analyzer = AnalyzePhase(camera, bridge)

                camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                print "Starting Flow"
                camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
                last_time = None
                while not rospy.is_shutdown():
                    velocity.x.err = flow_analyzer.x_motion
                    velocity.y.err = flow_analyzer.y_motion
                    velocity.z.err = flow_analyzer.z_motion
                    camera.wait_recording(1 / 100.0)
                    velpub.publish(velocity)

                    if phase_analyzer.prev_img is not None and phase_analyzer.prev_time != last_time:
                        image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
                        image_message.header.stamp = phase_analyzer.prev_rostime
                        # print "stamp", image_message.header.stamp
                        last_time = phase_analyzer.prev_rostime
                        image_pub.publish(image_message)
                        camera_info_pub.publish(cim.getCameraInfo())

                camera.stop_recording(splitter_port=1)
                camera.stop_recording(splitter_port=2)
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise


if __name__ == '__main__':
    main()
