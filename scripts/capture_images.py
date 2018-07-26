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
from h2rMultiWii import MultiWii


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240


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

        self.prev_img = None
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

        self.i = 1
        self.board = MultiWii("/dev/ttyUSB0")

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime

    def range_callback(self, data):
        if data.range != -1:
            self.z = data.range

    def reset_callback(self, data):
        print "Taking picture"
        cv2.imwrite("img" + str(self.i) + ".jpg", self.prev_img)
        self.i += 1
        mw_data = self.board.getData(MultiWii.ATTITUDE)
        curr_angx = mw_data['angx'] / 180.0 * np.pi
        curr_angy = mw_data['angy'] / 180.0 * np.pi
        print curr_angx, curr_angy, self.z

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

        with picamera.PiCamera(resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=30) as camera:
            phase_analyzer = AnalyzePhase(camera, bridge)

            print "Starting Flow"
            camera.start_recording(phase_analyzer, format='bgr', splitter_port=1)
            last_time = None
            while not rospy.is_shutdown():
                camera.wait_recording(1 / 30.0)
                velpub.publish(velocity)

                if phase_analyzer.prev_img is not None and phase_analyzer.prev_time != last_time:
                    image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
                    image_message.header.stamp = phase_analyzer.prev_rostime
                    # print "stamp", image_message.header.stamp
                    last_time = phase_analyzer.prev_rostime
                    image_pub.publish(image_message)
                    camera_info_pub.publish(cim.getCameraInfo())

            camera.stop_recording(splitter_port=1)
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise


if __name__ == '__main__':
    main()
