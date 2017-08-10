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
homography_started = False

def mode_callback(data):
    global current_mode
    current_mode = data.mode

class AnalyzeHomography(picamera.array.PiMotionAnalysis):

    def compose_pose(self, RT):
        pos = PoseStamped()
        quat_r = tf.transformations.quaternion_from_matrix(RT)
        pos.header.frame_id = 'world'
        pos.header.seq += 1
        pos.header.stamp = rospy.Time.now()
        pos.pose.position.x = RT[0][3]
        pos.pose.position.y = RT[1][3]
        pos.pose.position.z = RT[2][3]
        pos.pose.orientation.x = quat_r[0]
        pos.pose.orientation.y = quat_r[1]
        pos.pose.orientation.z = quat_r[2]
        pos.pose.orientation.w = quat_r[3]

        return pos

    def get_H(self, curr_img):
        curr_kp, curr_des = self.orb.detectAndCompute(curr_img,None)

        good = []
        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        if self.first_des is not None and curr_des is not None and len(self.first_des) > 3 and len(curr_des) > 3:
            matches = flann.knnMatch(self.first_des, curr_des, k=2)
            # store all the good matches as per Lowe's ratio test.
            for test in matches:
                if len(test) > 1:
                    m, n = test
                    if m.distance < 0.7*n.distance:
                         good.append(m)

            H = None
            if len(good) > self.min_match_count:
                src_pts = np.float32([self.first_kp[m.queryIdx].pt for m in good]).reshape(-1,1,2) 
                dst_pts = np.float32([curr_kp[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

            return H

        else:
            print "Not enough matches are found - %d/%d" % (len(good), self.min_match_count)
            return None

    def get_RT(self, H):
        retval, Rs, ts, norms = cv2.decomposeHomographyMat(H,
                self.camera_matrix)
        min_index = 0
        min_z_norm = 0
        for i in range(len(Rs)):
            if norms[i][2] < min_z_norm:
                min_z_norm = norms[i][2]
                min_index = i

        T = ts[min_index] * self.z

        return Rs[min_index], T, norms[min_index]

    def write(self, data):
        print "data"
        img = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
#       cv2.imshow("img", img)
#       cv2.waitKey(1)
        if self.first:
            self.first = False
            self.first_kp, self.first_des = self.orb.detectAndCompute(img, None)
            print len(self.first_kp), "features for first"
            self.prev_time = rospy.get_time()
        else:
            curr_time = rospy.get_time()
            H = self.get_H(img)
            if H is not None:
                R, t, n = self.get_RT(H)
                RT = np.identity(4)
                RT[0:3, 0:3] = R[0:3, 0:3]
                if np.linalg.norm(t) < 50:
                    RT[0:3, 3] = t.T[0]
                new_pos = self.compose_pose(RT)
                self.smoothed_pos.header = new_pos.header
                self.smoothed_pos.pose.position.x = (1. - self.alpha) * self.smoothed_pos.pose.position.x + self.alpha * new_pos.pose.position.x
                self.smoothed_pos.pose.position.y = (1. - self.alpha) * self.smoothed_pos.pose.position.y + self.alpha * new_pos.pose.position.y
                mode = Mode()
                mode.mode = 5
                self.lr_err.err = self.smoothed_pos.pose.position.x
                self.fb_err.err = self.smoothed_pos.pose.position.y
                mode.x_velocity = self.lr_pid.step(self.lr_err.err, self.prev_time - curr_time)
                mode.y_velocity = self.fb_pid.step(self.fb_err.err, self.prev_time - curr_time)
                print 'lr', mode.x_velocity, 'fb', mode.y_velocity
                self.prev_time = curr_time
                self.pospub.publish(mode)
            else:
                mode = Mode()
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                print "LOST"

        
    def setup(self):
        self.first_kp = None
        self.first_des = None
        self.first = True
        self.orb = cv2.ORB_create(nfeatures=500, nlevels=8, scaleFactor=1.1)
        self.alpha = 0.6
        self.lr_err = ERR()
        self.fb_err = ERR()
        self.prev_time = None
        self.pospub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)
        self.smoothed_pos = PoseStamped()
        self.lr_pid = PIDaxis(-0.01, -0., -0.01, midpoint=0, control_range=(-15., 15.))
        self.fb_pid = PIDaxis(0.01, 0., 0.01, midpoint=0, control_range=(-15., 15.))
        self.index_params = dict(algorithm = 6, table_number = 6,
                                key_size = 12, multi_probe_level = 1)
        self.search_params = dict(checks = 50)
        self.min_match_count = 10
        self.camera_matrix = np.array([[ 250.0, 0., 160.0], 
                                    [ 0., 250.0, 120.0], 
                                    [   0., 0., 1.]])
        self.z = 100

camera = picamera.PiCamera(framerate=90)
homography_analyzer = AnalyzeHomography(camera)

def reset_callback(data):
    print "Resetting Homography"
    homography_analyzer.first = True

if __name__ == '__main__':
    rospy.init_node('flow_pub')
    velpub= rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)
    imgpub = rospy.Publisher("/pidrone/camera", Image, queue_size=1)
    rospy.Subscriber("/pidrone/mode", Mode, mode_callback)
    rospy.Subscriber("/pidrone/reset_homography", Empty, reset_callback)
    global current_mode
    global homography_started
    global camera
    global homography_analyzer
    try:
        velocity = axes_err()
        bridge = CvBridge()
        with AnalyzeFlow(camera) as flow_analyzer:
            camera.resolution = (320, 240)
            flow_analyzer.setup(camera.resolution)
            homography_analyzer.setup()
            camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
            print "Starting Homography"
            camera.start_recording(homography_analyzer, format='bgr', splitter_port=2)
            homography_started = True
            i = 0
            while not rospy.is_shutdown():
                velocity.x.err = flow_analyzer.x_motion 
                velocity.y.err = flow_analyzer.y_motion
                velocity.z.err = flow_analyzer.z_motion
                camera.wait_recording(1/100.0)
                velpub.publish(velocity)

            camera.stop_recording(splitter_port=1)
            if homography_started:
                camera.stop_recording(splitter_port=2)
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise 
