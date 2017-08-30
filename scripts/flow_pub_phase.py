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
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import sys
from pid_class import PIDaxis

keynumber = 5

current_mode = 0
phase_started = False

def mode_callback(data):
    global current_mode
    current_mode = data.mode

class AnalyzePhase(picamera.array.PiMotionAnalysis):

    def write(self, data):
        img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
#       cv2.imshow("img", img)
#       cv2.waitKey(1)
        if self.first:
            self.first = False
            self.first_img = cv2.cvtColor(np.float32(img), cv2.COLOR_RGB2GRAY)            
            self.prev_img = deepcopy(self.first_img)
            self.prev_time = rospy.get_time()
        else:
            curr_img = cv2.cvtColor(np.float32(img), cv2.COLOR_RGB2GRAY)
            curr_time = rospy.get_time()
            corr_first = cv2.phaseCorrelate(self.first_img, curr_img)
            corr_int = cv2.phaseCorrelate(self.prev_img, curr_img)
            self.prev_img = curr_img
            print 'first', corr_first
            print 'int', corr_int
            if corr_first[1] > self.threshold: # not lost
                print "first"
                self.pos = [corr_first[0][0] * self.z / 320., corr_first[0][1] * self.z / 240.]
            else:
                print "integrated"
                self.pos[0] += corr_int[0][0] * self.z / 320.
                self.pos[1] += corr_int[0][1] * self.z / 240.
            self.lr_err.err = self.pos[0]
            self.fb_err.err = self.pos[1]
            mode = Mode()
            mode.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
            mode.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
            self.pospub.publish(mode)
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
        self.pos = [0, 0]
        self.lr_pid = PIDaxis(-0.00001, -0., -0.0, midpoint=0, control_range=(-15., 15.))
        self.fb_pid = PIDaxis(0.00001, 0., 0.0, midpoint=0, control_range=(-15., 15.))
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

camera = picamera.PiCamera(framerate=90)
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

if __name__ == '__main__':
    rospy.init_node('flow_pub')
    velpub= rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)
    imgpub = rospy.Publisher("/pidrone/camera", Image, queue_size=1)
    rospy.Subscriber("/pidrone/mode", Mode, mode_callback)
    rospy.Subscriber("/pidrone/reset_phase", Empty, reset_callback)
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
