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
            
keynumber = 5

current_mode = 0
phase_started = False

first_counter = 0
max_first_counter = 0


replan_period = 1.0 # seconds
replan_scale = 0.5 
replan_last_reset = 0
replan_next_deadline = 0
replan_until_deadline = 0
replan_vel_x = 0
replan_vel_y = 0

vel_average = [0,0]
vel_alpha = 1.0
vel_average_time = 0.0




def mode_callback(data):
    global current_mode
    global phase_analyzer
    current_mode = data.mode
    if not phase_analyzer.transforming or data.mode == 4 or data.mode == 3:
        print "VELOCITY"
        phase_analyzer.pospub.publish(data)
    else:
        phase_analyzer.target_x = data.x_velocity * 4.
        phase_analyzer.target_y = -data.y_velocity * 4.
        print "POSITION", phase_analyzer.target_x, phase_analyzer.target_y

class AnalyzePhase(picamera.array.PiMotionAnalysis):
    def __init__(self, camera):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.br = tf.TransformBroadcaster()


    def write(self, data):
        global first_counter
        global max_first_counter
        global replan_period
        global replan_last_reset
        global replan_next_deadline
        global replan_until_deadline
        global replan_vel_x
        global replan_vel_y
        global replan_scale
        global vel_average
        global vel_alpha
        global vel_average_time
        img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
#       cv2.imshow("img", img)
#       cv2.waitKey(1)
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        shouldi_set_velocity = 1 #0
        if np.abs(curr_time - replan_last_reset) > replan_period:
            replan_last_reset = curr_time
            replan_next_deadline = replan_last_reset + replan_period
            shouldi_set_velocity = 1
        replan_until_deadline = replan_next_deadline-curr_time
        #print curr_time, replan_last_reset, replan_next_deadline, replan_until_deadline, replan_vel_x, replan_vel_y


        if self.first:
            print "taking new first"
            self.first = False
            #self.first_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)            
            self.first_img = img
            cv2.imwrite("first_img" + str(self.i) + ".jpg", self.first_img)
            #self.prev_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            self.prev_img = img
            self.prev_rostime = curr_rostime
            self.prev_time = curr_time

            self.i += 1
            image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.first_image_pub.publish(image_message)

            
        elif self.transforming:
            #curr_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            curr_img = img
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            corr_int = cv2.estimateRigidTransform(self.prev_img, curr_img, False)
            self.prev_img = curr_img
            self.prev_rostime = curr_rostime
            self.prev_time = curr_time


            if corr_first is not None:
                self.last_first_time = curr_time
                if curr_time - self.last_first_time > 2:
                    self.first = True
                first_displacement = [corr_first[0, 2] / 320., corr_first[1, 2] / 240.]
                scalex = np.linalg.norm(corr_first[:, 0])
                scalez = np.linalg.norm(corr_first[:, 1])
                corr_first[:, 0] /= scalex
                corr_first[:, 1] /= scalez
                self.yaw_observed = math.atan2(corr_first[1, 0], corr_first[0, 0])
                #print first_displacement, yaw_observed
                #yaw = yaw_observed
                self.smoothed_yaw = (1.0 - self.alpha_yaw) * self.smoothed_yaw + (self.alpha_yaw) * self.yaw_observed 
                yaw = self.smoothed_yaw
                vel_average[0] = (1.0 - vel_alpha) * vel_average[0] + (vel_alpha) * self.pos[0]
                # jgo XXX see what happens if we dont reset upon seeing first
                #self.pos = [first_displacement[0] * self.z, first_displacement[1] * self.z / 240., yaw]
                # jgo XXX see what happens if we use alpha blending 
                hybrid_alpha = 0.1 # needs to be between 0 and 1.0
                self.pos[0:2] = [(hybrid_alpha) * first_displacement[0] * self.z + (1.0 - hybrid_alpha) * self.pos[0],
                            (hybrid_alpha) * first_displacement[1] * self.z  + (1.0 - hybrid_alpha) * self.pos[1]]
                vel_average[0] = (1.0 - vel_alpha) * vel_average[0] + (vel_alpha) * self.pos[0]
                vel_average[1] = (1.0 - vel_alpha) * vel_average[1] + (vel_alpha) * self.pos[1]
                vel_average_time = (1.0 - vel_alpha) * vel_average_time + (vel_alpha) * curr_time
                #print "times: ", vel_average_time, curr_time, curr_time - vel_average_time
                #self.lr_err.err = vel_average[0] + self.target_x
                #self.fb_err.err = vel_average[1] + self.target_y
                self.lr_err.err = self.pos[0] + self.target_x
                self.fb_err.err = self.pos[1] + self.target_y
                print "ERR", self.lr_err.err, self.fb_err.err
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


                # XXX
                cvc_vel = 1.00#0.15#0.25

                if shouldi_set_velocity:
                    replan_vel_x = mode.x_i * replan_scale / max(replan_until_deadline, 0.1)
                    replan_vel_y = mode.y_i * replan_scale / max(replan_until_deadline, 0.1)
                    replan_vel_x = min(replan_vel_x, 1.0)
                    replan_vel_y = min(replan_vel_y, 1.0)
                # XXX coast if first frame found but still do PID update to
                # integrate!
                #mode.x_velocity = 0
                #mode.y_velocity = 0
                mode.x_velocity = cvc_vel * mode.x_i 
                mode.y_velocity = cvc_vel * mode.y_i 
                #mode.x_velocity = cvc_vel * mode.x_i / cvc_norm
                #mode.y_velocity = cvc_vel * mode.y_i / cvc_norm
                #mode.x_velocity = replan_vel_x
                #mode.y_velocity = replan_vel_y
                self.iacc_yaw += yaw * self.ki_yaw

                yaw_kpi_term = np.sign(yaw) * yaw * yaw * self.kpi_yaw
                if abs(yaw_kpi_term) < self.kpi_max_yaw:
                    self.iacc_yaw += yaw_kpi_term
                else:
                    self.iacc_yaw += np.sign(yaw) * self.kpi_max_yaw

                mode.yaw_velocity = yaw * self.kp_yaw + self.iacc_yaw
                print "yaw iacc: ", self.iacc_yaw
                self.pospub.publish(mode)
                print "first", max_first_counter, first_counter
            elif corr_int is not None:
                time_since_first = curr_time - self.last_first_time
                print "integrated", time_since_first
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
                #self.pos[2] = yaw
                #vel_average = (1.0 - vel_alpha) * vel_average[0] + (vel_alpha) * self.pos[0]
                #vel_average = (1.0 - vel_alpha) * vel_average[1] + (vel_alpha) * self.pos[1]
                vel_average_time = (1.0 - vel_alpha) * vel_average_time + (vel_alpha) * curr_time
                #print "times: ", vel_average_time, curr_time, curr_time - vel_average_time
                #self.lr_err.err = vel_average[0] + self.target_x
                #self.fb_err.err = vel_average[1] + self.target_y
                self.lr_err.err = self.pos[0] + self.target_x
                self.fb_err.err = self.pos[1] + self.target_y
                print "ERR", self.lr_err.err, self.fb_err.err
                mode = Mode()
                mode.mode = 5
                mode.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
                mode.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
                # jgo XXX LOLOL constant velocity controller 
                first_counter = 0
                cvc_norm = np.sqrt(mode.x_i * mode.x_i + mode.y_i * mode.y_i)
                if cvc_norm <= 0.01:
                    cvc_norm = 1.0
                cvc_vel = 3.0#0.25 #1.0 

                if shouldi_set_velocity:
                    #replan_vel_x = mode.x_i * replan_scale# * cvc_vel
                    #replan_vel_y = mode.y_i * replan_scale# * cvc_vel
                    replan_vel_x = mode.x_i * replan_scale / max(replan_until_deadline, 0.1)
                    replan_vel_y = mode.y_i * replan_scale / max(replan_until_deadline, 0.1)
                    replan_vel_x = min(replan_vel_x, 1.0)
                    replan_vel_y = min(replan_vel_y, 1.0)
                #cvc_vel = max(min(time_since_first * 0.1, 1.0), 0.0)
                mode.x_velocity = cvc_vel * mode.x_i 
                mode.y_velocity = cvc_vel * mode.y_i 
                #mode.x_velocity = cvc_vel * mode.x_i / cvc_norm
                #mode.y_velocity = cvc_vel * mode.y_i / cvc_norm
                #mode.x_velocity = replan_vel_x
                #mode.y_velocity = replan_vel_y
                #mode.yaw_velocity = yaw * self.kp_yaw
                # yaw i term only
                mode.yaw_velocity = self.iacc_yaw
                print "yaw iacc: ", self.iacc_yaw
                self.pospub.publish(mode)
            else:
                print "LOST"

        else:
            #print "Not transforming"
            #curr_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            curr_img = img
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            if corr_first is not None:
                #print "first"
                self.last_first_time = rospy.get_time()
                if curr_time - self.last_first_time > 2:
                    self.first = True
            else:
                print "no first", curr_time - self.last_first_time
            self.prev_img = curr_img
            self.prev_rostime = curr_rostime
            self.prev_time = curr_time
            
        prev_time = curr_time
        #print "smoothed yaw: ", self.smoothed_yaw
        self.br.sendTransform((self.pos[0] * 0.01, self.pos[1] * 0.01, self.pos[2] * 0.01),
                              tf.transformations.quaternion_from_euler(0, 0, self.yaw_observed),
                              rospy.Time.now(),
                              "base",
                              "world")

        
    def setup(self):
        self.first_img = None
        self.prev_img = None
        self.first = True
        self.lr_err = ERR()
        self.fb_err = ERR()
        self.prev_time = None
        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.pos = [0, 0, 0]
# -, -, 0.1
        #self.lr_pid = PIDaxis(0.100, -0.000100, 0.0050, midpoint=0, control_range=(-10.0, 10.0))
        self.lr_pid = PIDaxis(0.0500, -0.00000, 0.000, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(-0.0500, 0.0000, -0.000, midpoint=0, control_range=(-10.0, 10.0))
        #self.lr_pid = PIDaxis(0.0500, 0.001, 0.04, midpoint=0, control_range=(-15., 15.))
        #self.fb_pid = PIDaxis(-0.0500, -0.0010, -0.04, midpoint=0, control_range=(-15., 15.))
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
        self.ki_yaw = 0.1
        self.kpi_yaw = 20.0
        self.kpi_max_yaw = 0.01
        self.alpha_yaw = 0.1
        self.smoothed_yaw = 0.0
        self.yaw_observed = 0.0
        self.iacc_yaw = 0.0
        self.transforming = False
        self.i = 0
        self.last_first_time = None
        self.target_x = 0
        self.target_y = 0

camera = picamera.PiCamera(framerate=90)
# camera = picamera.PiCamera(framerate=40, sensor_mode=4)
phase_analyzer = AnalyzePhase(camera)

def range_callback(data):
    global phase_analyzer
    if data.range != -1:
        phase_analyzer.z = data.range * 100

def reset_callback(data):
    global phase_analyzer
    print "Resetting Phase"
    phase_analyzer.first = True
    phase_analyzer.pos = [0, 0, 0]
    phase_analyzer.fb_pid._i = 0
    phase_analyzer.lr_pid._i = 0
    phase_analyzer.target_x = 0
    phase_analyzer.target_y = 0

def toggle_callback(data):
    global phase_analyzer
    phase_analyzer.transforming = not phase_analyzer.transforming

def main():
    rospy.init_node('flow_pub')

<<<<<<< HEAD


=======
>>>>>>> 1d6eb550eab108dfc7c3e3b3b6fcca8777ced052
    first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)
    
    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    #image_mono_pub = rospy.Publisher("/pidrone/picamera/image_mono",Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

                                                                        
    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")
                                             
    
    global current_mode
    global phase_started
    global camera
    global phase_analyzer
    try:
        velocity = axes_err()
        bridge = CvBridge()
        
        with AnalyzeFlow(camera) as flow_analyzer:
            flow_analyzer.bridge = bridge
            camera.resolution = (320, 240)
            flow_analyzer.setup(camera.resolution)
            phase_analyzer.setup()
            rospy.Subscriber("/pidrone/set_mode", Mode, mode_callback)
            rospy.Subscriber("/pidrone/reset_transform", Empty, reset_callback)
            rospy.Subscriber("/pidrone/toggle_transform", Empty, toggle_callback)
            rospy.Subscriber("/pidrone/infrared", Range, range_callback)
            phase_analyzer.bridge = bridge
            phase_analyzer.first_image_pub = first_image_pub

            camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
            print "Starting Flow"
            camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
            phase_started = True
            i = 0
            last_time = None
            while not rospy.is_shutdown():
                camera.wait_recording(1/100.0)


                if phase_analyzer.prev_img != None and phase_analyzer.prev_time != last_time:
                    image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
                    image_message.header.stamp = phase_analyzer.prev_rostime
                    #print "stamp", image_message.header.stamp
                    last_time = phase_analyzer.prev_rostime
                    image_pub.publish(image_message)
                    

                    camera_info_pub.publish(cim.getCameraInfo())
                
                

            camera.stop_recording(splitter_port=1)
            if phase_started:
                camera.stop_recording(splitter_port=2)
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise 
    

if __name__ == '__main__':
    main()
