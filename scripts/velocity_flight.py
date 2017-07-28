from pid_class import PID
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from h2rPiCam import streamPi
import cv2
import rospy
import numpy as np
from pidrone_pkg.msg import RC, axes_err
from h2rMultiWii import MultiWii
import time
import sys

class Homography:
    """ Runs Homography AR"""
    
    def __init__(self, min_match_count = 10, width = 320, height = 240,
    camera_matrix = np.array([[ 250.0,    0.,          160.0], 
                            [ 0.,          250.0,  120.0], 
                            [   0.,            0., 1.        ]]), 
    dist_coeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,
                            0.27998478]), 
    flann_index_kdtree = 0, 
    index_params = dict(algorithm = 6, table_number = 6,
                        key_size = 12, multi_probe_level = 1), 
    search_params = dict(checks = 50)):

        self.min_match_count = min_match_count
        self.width = width
        self.height = height
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.curr_kp = None
        self.curr_des = None
        self.flann_index_kdtree = flann_index_kdtree
        self.index_params = index_params
        self.search_params = search_params
        self.est_H = np.identity(3)
        self.good = []
        self.prev_time = None
        self.curr_z = None

    def update_H(self, curr_img, prev_img = None):
        orb = cv2.ORB_create(nfeatures=500, nlevels=8, scaleFactor=2.0)
        if self.curr_kp is None or self.curr_des is None:
            self.curr_kp, self.curr_des = orb.detectAndCompute(prev_img,None)
        prev_kp, prev_des = orb.detectAndCompute(curr_img,None)
        print 'Found {} features!'.format(len(prev_kp))

        self.good = []
        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        if self.curr_des is not None and prev_des is not None and len(self.curr_des) > 3 and len(prev_des) > 3:
            matches = flann.knnMatch(self.curr_des,prev_des,k=2)
            # store all the good matches as per Lowe's ratio test.
            for test in matches:
                if len(test) > 1:
                    m, n = test
                    # if m.distance < 0.7*n.distance:
                    #     self.good.append(m)
                    if True:
                        self.good.append(m)

            if len(self.good) > self.min_match_count:
                src_pts = np.float32([self.curr_kp[m.queryIdx].pt for m in self.good]).reshape(-1,1,2)
                dst_pts = np.float32([prev_kp[m.trainIdx].pt for m in self.good]).reshape(-1,1,2)

                H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                self.est_H = H

                self.curr_kp = prev_kp
                self.curr_des = prev_des

            return True

        else:
            print "Not enough matches are found - %d/%d" % (len(self.good),self.min_match_count)
            return False

    def update_z(self, t):
        if np.abs(t[2] * self.curr_z) < 10:
            self.curr_z += t[2] * self.curr_z

    def set_z(self, z):
        self.curr_z = z

    def get_vel_and_z(self):
        if self.prev_time == None:
            self.prev_time = rospy.get_time()
            return np.array([0, 0, 0]), self.curr_z
        else:
            retval, Rs, ts, norms = cv2.decomposeHomographyMat(self.est_H,
                    self.camera_matrix)
            min_index = 0
            min_z_norm = 0
            for i in range(len(Rs)):
                if norms[i][2] < min_z_norm:
                    min_z_norm = norms[i][2]
                    min_index = i
#           self.update_z(ts[min_index].T[0]) # uncomment for z estimation from camera

            T = ts[min_index] * self.curr_z

            curr_time = rospy.get_time()
            timestep = curr_time - self.prev_time
            print timestep, self.curr_z
            ret_val = T.T[0] / timestep, self.curr_z
            self.prev_time = curr_time
            print ret_val
            return ret_val


vrpn_pos = None
init_z = None
smoothed_vel = np.array([0, 0, 0])
alpha = 1.0
ultra_z = 0

def vrpn_update_pos(data):
    global vrpn_pos
    global init_z
    vrpn_pos = data
    if init_z is None:
        init_z = vrpn_pos.pose.position.z

def ultra_callback(data):
    global ultra_z
    if data.range != -1:
        ultra_z = data.range

if __name__ == '__main__':
    rospy.init_node('velocity_flight')
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_update_pos)
    rospy.Subscriber("/pidrone/ultrasonic", Range, ultra_callback)
    homography = Homography()
    pid = PID()
    first = True
    board = MultiWii("/dev/ttyACM0")
    while vrpn_pos is None:
        if not rospy.is_shutdown():
            print "No VRPN :("
            time.sleep(0.01)
        else:
            print "Shutdown Recieved"
            sys.exit()
    stream = streamPi()
    try:
        while not rospy.is_shutdown():
            curr_img = cv2.cvtColor(np.array(stream.next()), cv2.COLOR_RGB2GRAY)
            if first:
                homography.update_H(curr_img, curr_img)
                first = False
                homography.set_z(vrpn_pos.pose.position.z)
                board.arm()
            else:
                homography.set_z(ultra_z)
                error = axes_err()
                if homography.update_H(curr_img):
                    vel, z = homography.get_vel_and_z()
                    if np.abs(np.linalg.norm(vel)) < 2500:
                        print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
                        smoothed_vel = (1 - alpha) * smoothed_vel + alpha * vel
                        error.x.err = smoothed_vel[0]
                        error.y.err = smoothed_vel[1]
                    else:
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print np.abs(np.linalg.norm(vel))
                else:
                    print "###################################################################################################"
                    print "Couldn't update H"
                error.z.err = init_z - homography.curr_z + 40
                cmds = pid.step(error)
                print 'vrpn-ultra-error', homography.curr_z - vrpn_pos.pose.position.z
                print error
                print cmds
                board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
        print "Shutdown Recieved"
        board.disarm()
        sys.exit()
    except Exception as e:
        board.disarm()
        raise
