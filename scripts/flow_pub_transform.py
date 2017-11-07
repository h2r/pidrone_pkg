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

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
        curr_time = rospy.get_time() 
        self.replan_until_deadline = self.replan_next_deadline-curr_time


        if self.first:
            print "taking new first"
            self.first = False
            self.first_img = curr_img
            cv2.imwrite("first_img" + str(self.i) + ".jpg", self.first_img)
            self.prev_img = curr_img
            self.prev_time = rospy.get_time()
            self.i += 1
            image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
            self.first_image_pub.publish(image_message)

            
        elif self.transforming:
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            if corr_first is not None:
                self.last_first_time = curr_time
                if curr_time - self.last_first_time > 2:
                    self.first = True
#                first_displacement = [corr_first[0, 2] / 320., corr_first[1, 2] / 240.]
#                scalex = np.linalg.norm(corr_first[:, 0])
#                scalez = np.linalg.norm(corr_first[:, 1])
#                corr_first[:, 0] /= scalex
#                corr_first[:, 1] /= scalez
#                self.vel_average[0] = (1.0 - self.vel_alpha) * self.vel_average[0] + (self.vel_alpha) * self.pos[0]
#                # jgo XXX see what happens if we use alpha blending 
#                self.hybrid_alpha = 0.1 # needs to be between 0 and 1.0
#                self.pos = [(self.hybrid_alpha) * first_displacement[0] * self.z + (1.0 - self.hybrid_alpha) * self.pos[0], (self.hybrid_alpha) * first_displacement[1] * self.z / 240. + (1.0 - self.hybrid_alpha) * self.pos[1], yaw]
#                self.vel_average[0] = (1.0 - self.vel_alpha) * self.vel_average[0] + (self.vel_alpha) * self.pos[0]
#                self.vel_average[1] = (1.0 - self.vel_alpha) * self.vel_average[1] + (self.vel_alpha) * self.pos[1]
#                self.vel_average_time = (1.0 - self.vel_alpha) * self.vel_average_time + (self.vel_alpha) * curr_time
#                self.lr_err.err = self.pos[0] + self.target_x
#                self.fb_err.err = self.pos[1] + self.target_y
#                print "ERR", self.lr_err.err, self.fb_err.err
#                mode = Mode()
#                mode.mode = 5
#                mode.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
#                mode.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
#                # jgo XXX LOLOL constant velocity controller 
#                self.first_counter = self.first_counter + 1
#                self.max_first_counter = max(self.first_counter, self.max_first_counter)
#
#
#                # XXX
#                cvc_vel = 1.00
#
#                self.replan_vel_x = mode.x_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
#                self.replan_vel_y = mode.y_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
#                self.replan_vel_x = min(self.replan_vel_x, 1.0)
#                self.replan_vel_y = min(self.replan_vel_y, 1.0)
#                # XXX coast if first frame found but still do PID update to
#                # integrate!
#                mode.x_velocity = cvc_vel * mode.x_i 
#                mode.y_velocity = cvc_vel * mode.y_i 
                self.step_pos_controller(corr_first, True, 1.0)            
                yaw_observed = math.atan2(corr_first[1, 0], corr_first[0, 0])
                self.smoothed_yaw = (1.0 - self.alpha_yaw) * self.smoothed_yaw + (self.alpha_yaw) * yaw_observed 
                yaw = self.smoothed_yaw
                self.iacc_yaw += yaw * self.ki_yaw

                yaw_kpi_term = np.sign(yaw) * np.min(yaw * yaw * self.kpi_yaw, self.kpi_max_yaw)
                self.iacc_yaw += yaw_kpi_term
                self.mode.yaw_velocity = yaw * self.kp_yaw + self.iacc_yaw
                print "yaw iacc: ", self.iacc_yaw
                
                self.pospub.publish(self.mode)
                print "first", self.max_first_counter, self.first_counter
            else:
                # integration if we failed to find the first frame
                corr_int = cv2.estimateRigidTransform(self.prev_img, curr_img, False)
                if corr_int is not None:
                    time_since_first = curr_time - self.last_first_time
                    print "integrated", time_since_first
                    print "self.max_first_counter: ", self.max_first_counter
#                    int_displacement = [corr_int[0, 2] / 320., corr_int[1, 2] / 240.]
#                    scalex = np.linalg.norm(corr_int[:, 0])
#                    scalez = np.linalg.norm(corr_int[:, 1])
#                    corr_int[:, 0] /= scalex
#                    corr_int[:, 1] /= scalez
#                    self.pos[0] += int_displacement[0] * self.z
#                    self.pos[1] += int_displacement[1] * self.z
#                    self.vel_average_time = (1.0 - self.vel_alpha) * self.vel_average_time + (self.vel_alpha) * curr_time
#                    self.lr_err.err = self.pos[0] + self.target_x
#                    self.fb_err.err = self.pos[1] + self.target_y
#                    print "ERR", self.lr_err.err, self.fb_err.err
#                    mode = Mode()
#                    mode.mode = 5
#                    mode.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
#                    mode.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
#                    # jgo XXX LOLOL constant velocity controller 
#                    self.first_counter = 0
#                    cvc_vel = 3.0
#                    self.replan_vel_x = mode.x_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
#                    self.replan_vel_y = mode.y_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
#                    self.replan_vel_x = min(self.replan_vel_x, 1.0)
#                    self.replan_vel_y = min(self.replan_vel_y, 1.0)
#                    mode.x_velocity = cvc_vel * mode.x_i 
#                    mode.y_velocity = cvc_vel * mode.y_i 
                    # yaw i term only

                    self.step_pos_controller(corr_int, False, 3.0)            
                    yaw = math.atan2(corr_int[1, 0], corr_int[0, 0])
                    print int_displacement, yaw
                    self.mode.yaw_velocity = self.iacc_yaw
                    print "yaw iacc: ", self.iacc_yaw
                    self.pospub.publish(self.mode)
                else:
                    print "LOST"
        else:
            # while position hold is disabled, keep running the vision to keep
            # the prev image and self.first updated
            corr_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            if corr_first is not None:
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
        self.lr_err = ERR()
        self.fb_err = ERR()
        self.prev_time = None
        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.pos = [0, 0, 0]
        self.lr_pid = PIDaxis(0.0500, -0.00000, 0.000, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(-0.0500, 0.0000, -0.000, midpoint=0, control_range=(-10.0, 10.0))
        self.z = 7.5
        self.kp_yaw = 100.0
        self.ki_yaw = 0.1
        self.kpi_yaw = 20.0
        self.kpi_max_yaw = 0.01
        self.alpha_yaw = 0.1
        self.smoothed_yaw = 0.0
        self.iacc_yaw = 0.0
        self.transforming = False
        self.i = 0
        self.last_first_time = None
        self.target_x = 0
        self.target_y = 0
        self.replan_last_reset = 0
        self.replan_period = 1.0
        self.replan_scale = 0.5 
        self.replan_next_deadline = 0
        self.replan_until_deadline = 0
        self.replan_vel_x = 0
        self.replan_vel_y = 0
        self.first_counter = 0
        self.max_first_counter = 0
        self.vel_alpha = 1.0
        self.mode = Mode()
        self.hybrid_alpha = 0.1

    def step_pos_controller(self, est_transform, first_frame_bool, cvc_vel):
        first_displacement = [est_transform[0, 2] / 320., est_transform[1, 2] / 240.]
        scalex = np.linalg.norm(est_transform[:, 0])
        scalez = np.linalg.norm(est_transform[:, 1])
        est_transform[:, 0] /= scalex
        est_transform[:, 1] /= scalez
        if first_frame_bool:    
            self.pos = [(self.hybrid_alpha) * first_displacement[0] * self.z + (1.0 - self.hybrid_alpha) * self.pos[0], 
            (self.hybrid_alpha) * first_displacement[1] * self.z / 240. + (1.0 - self.hybrid_alpha) * self.pos[1], 
            yaw]
        else:
            self.pos[0] += first_displacement[0] * self.z
            self.pos[1] += first_displacement[1] * self.z
        
        
        
        self.lr_err.err = self.pos[0] + self.target_x
        self.fb_err.err = self.pos[1] + self.target_y
        print "ERR", self.lr_err.err, self.fb_err.err
        self.mode.mode = 5
        self.mode.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
        self.mode.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
        self.replan_vel_x = self.mode.x_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
        self.replan_vel_y = self.mode.y_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
        self.replan_vel_x = min(self.replan_vel_x, 1.0)
        self.replan_vel_y = min(self.replan_vel_y, 1.0)
        self.mode.x_velocity = cvc_vel * self.mode.x_i 
        self.mode.y_velocity = cvc_vel * self.mode.y_i 


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
        if self.transforming:
            print "Position is now on"
        else:
            print "Position is now off"

    def mode_callback(self, data):
        if not self.transforming or data.mode == 4 or data.mode == 3:
            print "VELOCITY"
            self.pospub.publish(data)
        else:
            self.target_x = data.x_velocity * 4.
            self.target_y = -data.y_velocity * 4.
            print "POSITION", self.target_x, self.target_y

def main():
    rospy.init_node('flow_pub')
    velpub= rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)

    camera = picamera.PiCamera(framerate=90)
    phase_analyzer = AnalyzePhase(camera)

    rospy.Subscriber("/pidrone/set_mode", Mode, phase_analyzer.mode_callback)
    rospy.Subscriber("/pidrone/reset_transform", Empty, phase_analyzer.reset_callback)
    rospy.Subscriber("/pidrone/toggle_transform", Empty, phase_analyzer.toggle_callback)
    rospy.Subscriber("/pidrone/infrared", Range, phase_analyzer.range_callback)




    first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)
    
    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    #image_mono_pub = rospy.Publisher("/pidrone/picamera/image_mono",Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

                                                                        
    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")
                                             
    
    try:
        velocity = axes_err()
        bridge = CvBridge()
        
        with AnalyzeFlow(camera) as flow_analyzer:
            flow_analyzer.bridge = bridge
            camera.resolution = (320, 240)
            flow_analyzer.setup(camera.resolution)
            phase_analyzer.setup()
            phase_analyzer.bridge = bridge
            phase_analyzer.first_image_pub = first_image_pub

            camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
            print "Starting Flow"
            camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
            i = 0
            while not rospy.is_shutdown():
                velocity.x.err = flow_analyzer.x_motion 
                velocity.y.err = flow_analyzer.y_motion
                velocity.z.err = flow_analyzer.z_motion
                camera.wait_recording(1/100.0)
                velpub.publish(velocity)


                if phase_analyzer.prev_img != None:
                    image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
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
