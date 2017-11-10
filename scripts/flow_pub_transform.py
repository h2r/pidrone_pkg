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

    def setup(self):
        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        self.velcmdpub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_img = None
        self.prev_img = None
        self.first = True
        self.transforming = False
        self.lr_err = ERR()
        self.fb_err = ERR()
        self.prev_time = None
        self.pos = [0, 0, 0]
        self.lr_pid = PIDaxis(0.0500, -0.0000, 0.000, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(-0.0500, 0.0000, -0.000, midpoint=0, control_range=(-10.0, 10.0))
        self.z = 7.5
        self.kp_yaw = 100.0
        self.ki_yaw = 0.1
        self.iacc_yaw = 0.0
        self.smoothed_yaw = 0.0
        self.alpha_yaw = 0.1    # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.1 # blend position with first frame and int
        self.last_first_time = None
        self.target_x = 0
        self.target_y = 0
        self.first_counter = 0
        self.max_first_counter = 0
        self.mode = Mode()
        self.mode.mode = 5

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
        self.curr_time = rospy.get_time() 

        # capture a first frame is commanded
        if self.first:
            print "taking new first"
            self.first = False
            self.first_img = curr_img
            self.prev_img = curr_img
            self.prev_time = rospy.get_time()
            self.publish_image(self.first_image_pub, curr_img)

        elif self.transforming:
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            if corr_first is not None: # if we matched with the first
                self.last_first_time = self.curr_time
                self.first_counter+=1
                self.max_first_counter = max(self.max_first_counter,self.first_counter)

                self.step_pos_controller(corr_first, True, 1.5)            
                
                yaw_observed = math.atan2(corr_first[1, 0], corr_first[0, 0])
                self.smoothed_yaw = (1.0 - self.alpha_yaw) * self.smoothed_yaw + (self.alpha_yaw) * yaw_observed 
                yaw = self.smoothed_yaw
                self.iacc_yaw += yaw * self.ki_yaw
                self.mode.yaw_velocity = yaw * self.kp_yaw + self.iacc_yaw
                print "yaw iacc: ", self.iacc_yaw, "kp_yaw: ", self.kp_yaw * yaw
                
                self.velcmdpub.publish(self.mode)
                print "first", self.max_first_counter, self.first_counter
            else:
                self.first_counter = 0
                # integration if we failed to find the first frame
                corr_int = cv2.estimateRigidTransform(self.prev_img, curr_img, False)
                if corr_int is not None:
                    time_since_first = self.curr_time - self.last_first_time
                    print "integrated", time_since_first
                    print "self.max_first_counter: ", self.max_first_counter
                    # yaw i term only

                    self.step_pos_controller(corr_int, False, 1.5)            
                
                    self.mode.yaw_velocity = self.iacc_yaw
                    self.velcmdpub.publish(self.mode)
                
                else:
                    print "LOST"
        else:
            # while position hold is disabled, keep running the vision to keep
            # the prev image and self.first updated
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            if corr_first is not None:
                self.last_first_time = rospy.get_time()
                if self.curr_time - self.last_first_time > 2:
                    self.first = True
            else:
                print "no first", self.curr_time - self.last_first_time
        self.prev_img = curr_img
        prev_time = self.curr_time

        
    def step_pos_controller(self, est_transform, first_frame_bool, control_coeff):
        first_displacement = [est_transform[0, 2] / 320., est_transform[1, 2] / 240.] # take the x and the y from the transform
        scalex = np.linalg.norm(est_transform[:, 0])    # normalize by the first and second columns
        scalez = np.linalg.norm(est_transform[:, 1])
        est_transform[:, 0] /= scalex
        est_transform[:, 1] /= scalez
        if first_frame_bool:    
            self.pos[0:2] = [(self.hybrid_alpha) * first_displacement[0] * self.z + (1.0 - self.hybrid_alpha) * self.pos[0], 
            (self.hybrid_alpha) * first_displacement[1] * self.z / 240. + (1.0 - self.hybrid_alpha) * self.pos[1]]
        else:
            self.pos[0] += first_displacement[0] * self.z
            self.pos[1] += first_displacement[1] * self.z
        
        self.lr_err.err = self.pos[0] + self.target_x
        self.fb_err.err = self.pos[1] + self.target_y
        #print "ERR", self.lr_err.err, self.fb_err.err
        # self.mode.x_i = self.lr_pid.step(self.lr_err.err, self.prev_time - self.curr_time)
        # self.mode.y_i = self.fb_pid.step(self.fb_err.err, self.prev_time - self.curr_time)
        self.mode.x_i = min(max(self.lr_err.err * 0.05, -10.), 10) # p control with a range of -10 to 10
        self.mode.y_i = min(max(self.fb_err.err * -0.05, -10.), 10)
        self.mode.x_velocity = control_coeff * self.mode.x_i 
        self.mode.y_velocity = control_coeff * self.mode.y_i 


    def range_callback(self, data):
        if data.range != -1:
            self.z = data.range

    def reset_callback(self, data):
        print "Resetting Phase"
        self.first = True
        self.pos = [0, 0, 0]
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        self.target_x = 0
        self.target_y = 0

    def toggle_callback(self, data):
        self.transforming = not self.transforming
        print "Position hold", "enabled." if self.transforming else "disabled."

    def mode_callback(self, data):
        if not self.transforming or data.mode == 4 or data.mode == 3:
            # republish velocity commands on /pidrone/set_mode_vel if we're not holding pos
            print "VELOCITY"
            self.velcmdpub.publish(data)
        else:
            # accept position control commands
            self.target_x = data.x_velocity * 4.
            self.target_y = -data.y_velocity * 4.
            print "POSITION", self.target_x, self.target_y

    def publish_image(self, pub, img):
        image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        pub.publish(image_message)

def main():
    rospy.init_node('flow_pub')
   
    flowpub = rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)

    # image publishing for the web interface
    first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)
    image_pub       = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

    # camera info manager stuff                                                              
    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")
                                             
    try:
        flow_msg = axes_err()
        bridge = CvBridge()
        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with AnalyzePhase(camera) as phase_analyzer:
                with AnalyzeFlow(camera) as flow_analyzer:
                    flow_analyzer.bridge = bridge
                    flow_analyzer.setup(camera.resolution)
                    phase_analyzer.setup()
                    phase_analyzer.bridge = bridge
                    phase_analyzer.first_image_pub = first_image_pub

                    camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                    camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
                    while not rospy.is_shutdown():
                        flow_msg.x.err = flow_analyzer.x_motion 
                        flow_msg.y.err = flow_analyzer.y_motion
                        flow_msg.z.err = flow_analyzer.z_motion
                        camera.wait_recording(1/100.0)
                        flowpub.publish(flow_msg)

                        if phase_analyzer.prev_img != None:
                            phase_analyzer.publish_image(image_pub, phase_analyzer.prev_img)
                            camera_info_pub.publish(cim.getCameraInfo())
                        

            camera.stop_recording(splitter_port=1)  # stop recording both the flow 
            camera.stop_recording(splitter_port=2)  # and the images

        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise 
    
if __name__ == '__main__':
    main()
