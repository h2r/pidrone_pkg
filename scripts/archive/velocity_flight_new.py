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
    index_params = dict(algorithm = 6, trees = 5), 
    #index_params = dict(algorithm = 6, table_number = 6,
    #                    key_size = 12, multi_probe_level = 1), 
    search_params = dict(checks = 50)):

        self.min_match_count = min_match_count
        self.width = width
        self.height = height
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.prev_kp = None
        self.prev_des = None
        self.flann_index_kdtree = flann_index_kdtree
        self.index_params = index_params
        self.search_params = search_params
        self.est_H = np.identity(3)
        self.good = []
        self.prev_time = None
        self.curr_z = None
        self.orb = cv2.ORB_create(nfeatures=500, nlevels=8, scaleFactor=1.2)
        self.sift = cv2.xfeatures2d.SIFT_create(nOctaveLayers=8)

    def update_H(self, curr_img, prev_img = None):
        if self.prev_kp is None or self.prev_des is None:
            self.prev_kp, self.prev_des = self.orb.detectAndCompute(prev_img,None)
            #self.prev_kp, self.prev_des = self.sift.detectAndCompute(prev_img,None)
        curr_kp, curr_des = self.orb.detectAndCompute(curr_img,None)
        #curr_kp, curr_des = self.sift.detectAndCompute(curr_img,None)

        self.good = []
        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        if self.prev_des is not None and curr_des is not None and len(self.prev_des) > 3 and len(curr_des) > 3:
            matches = flann.knnMatch(self.prev_des,curr_des,k=2)
            # store all the good matches as per Lowe's ratio test.
            for test in matches:
                if len(test) > 1:
                    m, n = test
                    # if m.distance < 0.7*n.distance:
                    #     self.good.append(m)
                    if True:
                        self.good.append(m)

            if len(self.good) > self.min_match_count:
                src_pts = np.float32([self.prev_kp[m.queryIdx].pt for m in self.good]).reshape(-1,1,2)
                dst_pts = np.float32([curr_kp[m.trainIdx].pt for m in self.good]).reshape(-1,1,2)

                H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                self.est_H = H

            self.prev_kp = curr_kp
            self.prev_des = curr_des

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
            try:
                retval, Rs, ts, norms = cv2.decomposeHomographyMat(self.est_H,
                    self.camera_matrix)
            except Exception as e:
                return np.array([0, 0, 0]), self.curr_z

            min_index = 0
            min_z_norm = 0
            for i in range(len(Rs)):
                if norms[i][2] < min_z_norm:
                    min_z_norm = norms[i][2]
                    min_index = i
#           self.update_z(ts[min_index].T[0]) # uncomment for z estimation from camera

            #print 'raw', ts[min_index].T
            T = ts[min_index] * self.curr_z

            curr_time = rospy.get_time()
            timestep = curr_time - self.prev_time
            ret_val = T.T[0] / timestep, self.curr_z
            self.prev_time = curr_time
            return ret_val


vrpn_pos = None
init_z = None
smoothed_vel = np.array([0, 0, 0])
alpha = 0.6
ultra_z = 0

def vrpn_update_pos(data):
    global vrpn_pos
    global init_z
    vrpn_pos = data
    if init_z is None:
        init_z = vrpn_pos.pose.position.z

#def ultra_callback(data):
    #global ultra_z
    #if data.pose.position.z != -1:
        #ultra_z = data.pose.position.z

def ultra_callback(data):
    global ultra_z
    #if data.range != -1:
        #ultra_z = data.range
    ultra_z = data.range

if __name__ == '__main__':
    rospy.init_node('velocity_flight')
    cmdpub = rospy.Publisher('/pidrone/plane_cmds', RC, queue_size=1)
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_update_pos)
    rospy.Subscriber("/pidrone/infrared", Range, ultra_callback)
    homography = Homography()
    pid = PID()
    first = True
    stream = streamPi()
    try:
        while not rospy.is_shutdown():
            curr_img = cv2.cvtColor(np.array(stream.next()), cv2.COLOR_RGB2GRAY)
            if first:
                homography.update_H(curr_img, curr_img)
                first = False
                homography.set_z(300.0 - ultra_z)
            else:
                error = axes_err()
                if homography.update_H(curr_img):
                    homography.set_z(300.0 - ultra_z)
                    vel, z = homography.get_vel_and_z()
                    if np.abs(np.linalg.norm(vel)) < 2500:
                        ##print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
                        ##print vel
                        smoothed_vel = (1 - alpha) * smoothed_vel + alpha * vel
                        error.x.err = smoothed_vel[0]
                        error.y.err = smoothed_vel[1]
                        print vel, smoothed_vel
                    else:
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        #print np.linalg.norm(vel)
                        #smoothed_vel = (1 - alpha) * smoothed_vel + alpha * vel
                        error.x.err = 0
                        error.y.err = 0
                        #print vel, smoothed_vel
                    cmds = pid.step(error)
                    rc = RC()
                    rc.roll = cmds[0]
                    rc.pitch = cmds[1]
                    cmdpub.publish(rc)
                else:
                    ##print "###################################################################################################"
                    print "Couldn't update H"
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise
