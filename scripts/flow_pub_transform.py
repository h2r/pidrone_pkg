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
from sensor_msgs.msg import Image, Range
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

keynumber = 5

current_mode = 0
phase_started = False

first_counter = 0
max_first_counter = 0

def mode_callback(data):
    global current_mode
    current_mode = data.mode

class AnalyzePhase(picamera.array.PiMotionAnalysis):

    def write(self, data):
        global first_counter
        global max_first_counter
        img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
#       cv2.imshow("img", img)
#       cv2.waitKey(1)
        curr_time = rospy.get_time()
        if self.first:
            print "taking new first"
            self.first = False
            #self.first_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)            
            self.first_img = img
            cv2.imwrite("first_img" + str(self.i) + ".jpg", self.first_img)
            #self.prev_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            self.prev_img = img
            self.prev_time = rospy.get_time()
            self.i += 1
        elif self.transforming:
            #curr_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            curr_img = img
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            corr_int = cv2.estimateRigidTransform(self.prev_img, curr_img, False)
            self.prev_img = curr_img
            if corr_first is not None:
                self.last_first_time = curr_time
                if curr_time - self.last_first_time > 2:
                    self.first = True
                first_displacement = [corr_first[0, 2] / 320., corr_first[1, 2] / 240.]
                scalex = np.linalg.norm(corr_first[:, 0])
                scalez = np.linalg.norm(corr_first[:, 1])
                corr_first[:, 0] /= scalex
                corr_first[:, 1] /= scalez
                yaw = math.atan2(corr_first[1, 0], corr_first[0, 0])
                print first_displacement, yaw
                # jgo XXX see what happens if we dont reset upon seeing first
                #self.pos = [first_displacement[0] * self.z, first_displacement[1] * self.z / 240., yaw]
                # jgo XXX see what happens if we use alpha blending 
                hybrid_alpha = 0.1 # needs to be between 0 and 1.0
                self.pos = [(hybrid_alpha) * first_displacement[0] * self.z + (1.0 - hybrid_alpha) * self.pos[0], (hybrid_alpha) * first_displacement[1] * self.z / 240. + (1.0 - hybrid_alpha) * self.pos[1], yaw]
                self.lr_err.err = self.pos[0]
                self.fb_err.err = self.pos[1]
                mode = Mode()
                mode.mode = 5
                mode.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
                mode.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
                #mode.x_velocity = mode.x_i
                #mode.y_velocity = mode.y_i
                # jgo XXX LOLOL constant velocity controller 
                first_counter = first_counter + 1
                max_first_counter = max(first_counter, max_first_counter)
                cvc_norm = np.sqrt(mode.x_i * mode.x_i + mode.y_i * mode.y_i)
                if cvc_norm <= 0.01:
                    cvc_norm = 1.0
                cvc_vel = 0.75
                mode.x_velocity = cvc_vel * mode.x_i / cvc_norm
                mode.y_velocity = cvc_vel * mode.y_i / cvc_norm
                mode.yaw_velocity = yaw * self.kp_yaw
                self.pospub.publish(mode)
                print "first", max_first_counter, first_counter
            elif corr_int is not None:
                print "integrated", curr_time - self.last_first_time
                print "max_first_counter: ", max_first_counter
                int_displacement = [corr_int[0, 2] / 320., corr_int[1, 2] / 240.]
                scalex = np.linalg.norm(corr_int[:, 0])
                scalez = np.linalg.norm(corr_int[:, 1])
                corr_int[:, 0] /= scalex
                corr_int[:, 1] /= scalez
                yaw = math.atan2(corr_int[1, 0], corr_int[0, 0])
                print int_displacement, yaw
                self.pos[0] += int_displacement[0] * self.z
                self.pos[1] += int_displacement[1] * self.z
#               self.pos[2] = yaw
                self.lr_err.err = self.pos[0]
                self.fb_err.err = self.pos[1]
                mode = Mode()
                mode.mode = 5
                mode.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
                mode.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
                # jgo XXX LOLOL constant velocity controller 
                first_counter = 0
                cvc_norm = np.sqrt(mode.x_i * mode.x_i + mode.y_i * mode.y_i)
                if cvc_norm <= 0.01:
                    cvc_norm = 1.0
                cvc_vel = 0.750
                mode.x_velocity = cvc_vel * mode.x_i / cvc_norm
                mode.y_velocity = cvc_vel * mode.y_i / cvc_norm
                #mode.yaw_velocity = yaw * self.kp_yaw
                self.pospub.publish(mode)
            else:
                print "LOST"
        else:
            print "Not transforming"
            #curr_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            curr_img = img
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            if corr_first is not None:
                print "first"
                self.last_first_time = rospy.get_time()
                if curr_time - self.last_first_time > 2:
                    self.first = True
            else:
                print "no first", curr_time - self.last_first_time
            self.prev_img = curr_img
        prev_time = curr_time

        
    def setup(self):
        self.first_img = None
        self.prev_img = None
        self.first = True
        self.alpha = 0.7
        self.lr_err = ERR()
        self.fb_err = ERR()
        self.prev_time = None
        self.pospub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)
        self.pos = [0, 0, 0]
# -, -, 0.1
        self.lr_pid = PIDaxis(0.0200, 0.00, 0.00, midpoint=0, control_range=(-15., 15.))
        self.fb_pid = PIDaxis(-0.0200, -0.00, -0.00, midpoint=0, control_range=(-15., 15.))
        #self.lr_pid = PIDaxis(0.05, 0., 0.001, midpoint=0, control_range=(-15., 15.))
        #self.fb_pid = PIDaxis(-0.05, 0., -0.001, midpoint=0, control_range=(-15., 15.))
        self.index_params = dict(algorithm = 6, table_number = 6,
                                key_size = 12, multi_probe_level = 1)
        self.search_params = dict(checks = 50)
        self.min_match_count = 10
        self.camera_matrix = np.array([[ 250.0, 0., 160.0], 
                                    [ 0., 250.0, 120.0], 
                                    [   0., 0., 1.]])
        self.z = 7.5
        self.est_RT = np.identity(4)
        self.threshold = 0.5
        self.kp_yaw = 100.0
        self.transforming = False
        self.i = 0
        self.last_first_time = None

camera = picamera.PiCamera(framerate=90)
# camera = picamera.PiCamera(framerate=40, sensor_mode=4)
phase_analyzer = AnalyzePhase(camera)

def range_callback(data):
    global phase_analyzer
    if data.range != -1:
        phase_analyzer.z = data.range

def reset_callback(data):
    global phase_analyzer
    print "Resetting Phase"
    phase_analyzer.first = True
    phase_analyzer.pos = [0, 0]
    phase_analyzer.fb_pid._i = 0
    phase_analyzer.lr_pid._i = 0

def toggle_callback(data):
    global phase_analyzer
    phase_analyzer.transforming = not phase_analyzer.transforming

if __name__ == '__main__':
    rospy.init_node('flow_pub')
    velpub= rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)
    imgpub = rospy.Publisher("/pidrone/camera", Image, queue_size=1)
    rospy.Subscriber("/pidrone/mode", Mode, mode_callback)
    rospy.Subscriber("/pidrone/reset_transform", Empty, reset_callback)
    rospy.Subscriber("/pidrone/toggle_transform", Empty, toggle_callback)
    rospy.Subscriber("/pidrone/infrared", Range, range_callback)
    global current_mode
    global phase_started
    global camera
    global phase_analyzer
    try:
        velocity = axes_err()
        bridge = CvBridge()
        with AnalyzeFlow(camera) as flow_analyzer:
            camera.resolution = (320, 240)
            flow_analyzer.setup(camera.resolution)
            phase_analyzer.setup()
            camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
            print "Starting Flow"
            camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
            phase_started = True
            i = 0
            while not rospy.is_shutdown():
                velocity.x.err = flow_analyzer.x_motion 
                velocity.y.err = flow_analyzer.y_motion
                velocity.z.err = flow_analyzer.z_motion
                camera.wait_recording(1/100.0)
                velpub.publish(velocity)

            camera.stop_recording(splitter_port=1)
            if phase_started:
                camera.stop_recording(splitter_port=2)
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise 
