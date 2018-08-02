import tf
import sys
import cv2
import math
import time
import rospy
import signal
import picamera
import numpy as np
import picamera.array
import camera_info_manager
from pid_class import PIDaxis
from std_msgs.msg import Empty
from analyze_flow import AnalyzeFlow
from three_dim_vec import Position, Error
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Range, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped


class AnalyzePhase(picamera.array.PiMotionAnalysis):

    def __init__(self, camera):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.br = tf.TransformBroadcaster()
        # used to safely end the connection to the camera
        self.phase_started = False
        # initialize the variables relating to the first image
        self.first_img = None
        self.last_first_time = None
        self.has_first = False
        self.prev_img = None
        self.prev_time = None
        # the number of images in which the first image was visible
        self.first_counter = 0
        # the maximum number of consecutive images in which the first image was visible
        self.max_first_counter = 0
        # initialize the control mode as velocity control
        self.control_mode = 'VELOCITY'
        # current position
        self.position = Position(0.0, 0.0, 0.0)
        # target position
        self.target_position = Position(0.0, 0.0, 0.0)
        # PIDs:
        # left/right (roll) pid
        self.lr_pid = PIDaxis(0.0500, -0.00000, 0.000, midpoint=0, control_range=(-10.0, 10.0))
        # front/back (pitch) pid
        self.fb_pid = PIDaxis(-0.0500, 0.0000, -0.000, midpoint=0, control_range=(-10.0, 10.0))
        # errors:
        self.lr_err = Error(0.0, 0.0, 0.0)
        self.fb_err = Error(0.0, 0.0, 0.0)
        # initialize variables for yaw velocity PI controller:
        # propotional constant
        self.kp_yaw = 100.0
        # integral constant
        self.ki_yaw = 0.1
        self.kpi_yaw = 20.0
        self.kpi_max_yaw = 0.01
        # smoothing value for complimentary filter
        self.alpha_yaw = 0.1
        self.smoothed_yaw = 0.0
        self.yaw_observed = 0.0
        # yaw accumulated integral
        self.iacc_yaw = 0.0
        # initial altitude used for camera movement calculations
        self.z = 7.5


# TODO determine if these do anything and comment these
        self.est_RT = np.identity(4)
        self.threshold = 0.5
        self.index_params = dict(algorithm = 6, table_number = 6,
                                key_size = 12, multi_probe_level = 1)
        self.search_params = dict(checks = 50)
        self.min_match_count = 10
        self.camera_matrix = np.array([[ 250.0, 0., 160.0],
                                    [ 0., 250.0, 120.0],
                                    [   0., 0., 1.]])


    def setup(self):


        self.set_vel_pub = rospy.Publisher('/pidrone/set_vel', Velocity, queue_size=1)




    def write(self, data):
        img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
        current_time = rospy.get_time()

        # if there is no first image
        if not self.has_first:
            print 'Captured the first image'
            self.has_first = True
            self.first_img = img
            self.previous_img = img
            self.previous_time = current_time

            cv2.imwrite("first_img" + str(self.i) + ".jpg", self.first_img)
            image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.first_image_pub.publish(image_message)

        # if there is a first image and the position control is enabled
        elif self.control_mode = 'POSITION':
            current_img = img
            transform_first = cv2.estimateRigidTransform(self.first_img, current_img, False)
            transform_integrated = cv2.estimateRigidTransform(self.prev_img, curr_img, False)
            delta_time = current_time - self.previous_time
            self.previous_img = current_img
            self.previous_time = current_time

            # if the first image is visible
            if transform_first is not None:
                self.last_first_time = curr_time
                if curr_time - self.last_first_time > 2:
                    self.has_first = False
                first_displacement = [transform_first[0, 2] / 320., transform_first[1, 2] / 240.]
                scalex = np.linalg.norm(transform_first[:, 0])
                scalez = np.linalg.norm(transform_first[:, 1])
                transform_first[:, 0] /= scalex
                transform_first[:, 1] /= scalez
                self.yaw_observed = math.atan2(transform_first[1, 0], transform_first[0, 0])
                #print first_displacement, yaw_observed
                #yaw = yaw_observed
                self.smoothed_yaw = (1.0 - self.alpha_yaw) * self.smoothed_yaw + (self.alpha_yaw) * self.yaw_observed
                yaw = self.smoothed_yaw
                # jgo XXX see what happens if we dont reset upon seeing first
                #self.pos = [first_displacement[0] * self.z, first_displacement[1] * self.z / 240., yaw]
                # jgo XXX see what happens if we use alpha blending
                hybrid_alpha = 0.1 # needs to be between 0 and 1.0
                self.pos[0:2] = [(hybrid_alpha) * first_displacement[0] * self.z + (1.0 - hybrid_alpha) * self.pos[0],
                            (hybrid_alpha) * first_displacement[1] * self.z  + (1.0 - hybrid_alpha) * self.pos[1]]
                self.lr_err.err = self.pos[0] + self.target_x
                self.fb_err.err = self.pos[1] + self.target_y
                print "ERR", self.lr_err.err, self.fb_err.err
                set_vel = Velocity()
                set_vel.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
                set_vel.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
                #set_vel.x_velocity = set_vel.x_i
                #set_vel.y_velocity = set_vel.y_i
                # jgo XXX LOLOL constant velocity controller
                self.first_counter = self.first_counter + 1
                self.max_first_counter = max(first_counter, self.max_first_counter)
                cvc_norm = np.sqrt(set_vel.x_i * set_vel.x_i + set_vel.y_i * set_vel.y_i)
                if cvc_norm <= 0.01:
                    cvc_norm = 1.0


                # XXX
                cvc_vel = 1.00#0.15#0.25

                if shouldi_set_velocity:
                    self.replan_vel_x = set_vel.x_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
                    self.replan_vel_y = set_vel.y_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
                    self.replan_vel_x = min(replan_vel_x, 1.0)
                    self.replan_vel_y = min(self.replan_vel_y, 1.0)
                # XXX coast if first frame found but still do PID update to
                # integrate!
                #set_vel.x_velocity = 0
                #set_vel.y_velocity = 0
                set_vel.x_velocity = cvc_vel * set_vel.x_i
                set_vel.y_velocity = cvc_vel * set_vel.y_i
                #set_vel.x_velocity = cvc_vel * set_vel.x_i / cvc_norm
                #set_vel.y_velocity = cvc_vel * set_vel.y_i / cvc_norm
                #set_vel.x_velocity = replan_vel_x
                #set_vel.y_velocity = replan_vel_y
                self.iacc_yaw += yaw * self.ki_yaw

                yaw_kpi_term = np.sign(yaw) * yaw * yaw * self.kpi_yaw
                if abs(yaw_kpi_term) < self.kpi_max_yaw:
                    self.iacc_yaw += yaw_kpi_term
                else:
                    self.iacc_yaw += np.sign(yaw) * self.kpi_max_yaw

                set_vel.yaw_velocity = yaw * self.kp_yaw + self.iacc_yaw
                print "yaw iacc: ", self.iacc_yaw
                self.set_vel_pub.publish(set_vel)
                print "first", self.max_first_counter, self.first_counter

            # if the first image is lost, but the previous image is visible
            elif transform_integrated is not None:
                time_since_first = curr_time - self.last_first_time
                print "integrated", time_since_first
                print "max_first_counter: ", self.max_first
                int_displacement = [transform_integrated[0, 2] / 320., transform_integrated[1, 2] / 240.]
                scalex = np.linalg.norm(transform_integrated[:, 0])
                scalez = np.linalg.norm(transform_integrated[:, 1])
                transform_integrated[:, 0] /= scalex
                transform_integrated[:, 1] /= scalez
                yaw = math.atan2(transform_integrated[1, 0], transform_integrated[0, 0])
                print int_displacement, yaw
                self.pos[0] += int_displacement[0] * self.z
                self.pos[1] += int_displacement[1] * self.z
                #self.pos[2] = yaw
                self.lr_err.err = self.pos[0] + self.target_x
                self.fb_err.err = self.pos[1] + self.target_y
                print "ERR", self.lr_err.err, self.fb_err.err
                set_vel.x_i += self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
                set_vel.y_i += self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
                # jgo XXX LOLOL constant velocity controller
                self.first_counter = 0
                cvc_norm = np.sqrt(set_vel.x_i * set_vel.x_i + set_vel.y_i * set_vel.y_i)
                if cvc_norm <= 0.01:
                    cvc_norm = 1.0
                cvc_vel = 3.0#0.25 #1.0

                if shouldi_set_velocity:
                    #replan_vel_x = set_vel.x_i * replan_scale# * cvc_vel
                    #replan_vel_y = set_vel.y_i * replan_scale# * cvc_vel
                    self.replan_vel_x = set_vel.x_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
                    self.replan_vel_y = set_vel.y_i * self.replan_scale / max(self.replan_until_deadline, 0.1)
                    self.replan_vel_x = min(self.replan_vel_x, 1.0)
                    self.replan_vel_y = min(self.replan_vel_y, 1.0)
                #cvc_vel = max(min(time_since_first * 0.1, 1.0), 0.0)
                set_vel.x_velocity = cvc_vel * set_vel.x_i
                set_vel.y_velocity = cvc_vel * set_vel.y_i
                #set_vel.x_velocity = cvc_vel * set_vel.x_i / cvc_norm
                #set_vel.y_velocity = cvc_vel * set_vel.y_i / cvc_norm
                #set_vel.x_velocity = replan_vel_x
                #set_vel.y_velocity = replan_vel_y
                #set_vel.yaw_velocity = yaw * self.kp_yaw
                # yaw i term only
                set_vel.yaw_velocity = self.iacc_yaw
                print "yaw iacc: ", self.iacc_yaw
                self.set_vel_pub.publish(set_vel)

            # if both the first and previous images are lost
            else:
                print "LOST"

        else:
            #print "Not transforming"
            #curr_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            curr_img = img
            transform_first = cv2.estimateRigidTransform(self.first_img, curr_img, False)
            if transform_first is not None:
                #print "first"
                self.last_first_time = rospy.get_time()
                if curr_time - self.last_first_time > 2:
                    self.has_first = False
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




    # ROS Subscriber Callback Methods
    #################################
    def range_callback(self, msg):
        if msg.range != -1:
            self.z = msg.range * 100

    def reset_callback(self, msg):
        print "Resetting Phase"
        self.has_first = False
        self.pos = [0, 0, 0]
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        self.target_x = 0
        self.target_y = 0

    def toggle_callback(self, msg):
        self.transforming = not self.transforming

    def velocity_callback(self, msg):
        if not self.transforming:
            print "VELOCITY"
            msg.z_velocity = msg.z_velocity * 100
            self.set_vel_pub.publish(msg)
        else:
            self.target_x = msg.x_velocity * 4.
            self.target_y = -msg.y_velocity * 4.
            print "POSITION", self.target_x, self.target_y

    # Helper Methods
    ################
    def ctrl_c_handler(self, signal, frame):
        """Gracefully quit the flow_pub_transform node"""
        print "\nCaught ctrl-c! Stopping node."
        sys.exit()

def main():

    # ROS setup
    ###########
    rospy.init_node('flow_pub')

    # Publishers:
    #############
    first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)
    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    #image_mono_pub = rospy.Publisher("/pidrone/picamera/image_mono",Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

    # camera = picamera.PiCamera(framerate=40, sensor_mode=4)
    camera = picamera.PiCamera(framerate=90)
    bridge = CvBridge()
    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")

    # Create the AnalyzePhase object
    phase_analyzer = AnalyzePhase(camera)
    phase_analyzer.setup()
    phase_analyzer.bridge = bridge
    phase_analyzer.first_image_pub = first_image_pub

    # Create AnaylzeFlow object
    flow_analyzer = AnalyzeFlow(camera)
    flow_analyzer.bridge = bridge
    camera.resolution = (320, 240)
    flow_analyzer.setup(camera.resolution)

    # Subscribers:
    ##############
    rospy.Subscriber("/pidrone/reset_transform", Empty, phase_analyzer.reset_callback)
    rospy.Subscriber("/pidrone/toggle_transform", Empty, phase_analyzer.toggle_callback)
    rospy.Subscriber("/pidrone/infrared", Range, phase_analyzer.range_callback)
    rospy.Subscriber("/pidrone/desired_vel", Velocity, phase_analyzer.velocity_callback)

    camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
    print "Starting Flow"
    camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
    phase_analyzer.phase_started = True
    i = 0
    last_time = None

    # set up the ctrl-c handler
    signal.signal(signal.SIGINT, phase_analyzer.ctrl_c_handler)
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
    if phase_analyzer.phase_started:
        camera.stop_recording(splitter_port=2)
    print "Shutdown Recieved"
    sys.exit()


if __name__ == '__main__':
    main()
