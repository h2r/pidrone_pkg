import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty
import rospy
import tf
import copy
from pyquaternion import Quaternion
from camera_class import Camera
from h2rPiCam import streamPi
import time
from copy import deepcopy

class Homography:

    def __init__(self, min_match_count = 10, width = 320, height = 240,
    camera_matrix = np.array([[ 250.0,    0.,          160.0], 
                            [ 0.,          250.0,  120.0], 
                            [   0.,            0., 1.        ]]), 
    dist_coeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,
                            0.27998478]), 
    first_kp=None, first_des=None, flann_index_kdtree = 0, 
    index_params = dict(algorithm = 6, table_number = 6,
                        key_size = 12, multi_probe_level = 1), 
    search_params = dict(checks = 50)):

        self.min_match_count = min_match_count
        self.width = width
        self.height = height
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.first_kp = None
        self.first_des = None
        self.prev_kp = None
        self.prev_des = None
        self.flann_index_kdtree = flann_index_kdtree
        self.index_params = index_params
        self.search_params = search_params
        self.curr_pos = PoseStamped()
        self.curr_pos.header.frame_id = 'world'
        self.est_RT = None
        self.est_RT = np.identity(4)
        self.good = []
        self.alpha = 0.05
        self.orb = cv2.ORB_create(nfeatures=500, nlevels=8, scaleFactor=1.1)
        self.curr_kp = None
        self.curr_des = None

    def get_H(self, curr_img, prev_img, first):
        H = None
        if self.curr_kp is None and self.curr_des is None:
            self.curr_kp, self.curr_des = self.orb.detectAndCompute(curr_img,None)
        if first:
            if self.first_kp is None or self.first_des is None:
                self.first_kp, self.first_des = self.orb.detectAndCompute(prev_img,None)
        else:
            if self.prev_kp is None or self.prev_des is None:
                self.prev_kp, self.prev_des = self.orb.detectAndCompute(prev_img,None)

        self.good = []
        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        if first:
            if self.first_des is not None and self.curr_des is not None and len(self.first_des) > 3 and len(self.curr_des) > 3:
                matches = flann.knnMatch(self.first_des,self.curr_des,k=2)
                # store all the good matches as per Lowe's ratio test.
                for test in matches:
                    if len(test) > 1:
                        m, n = test
                        # if m.distance < 0.7*n.distance:
                        #     self.good.append(m)
                        if True:
                            self.good.append(m)

                if len(self.good) > self.min_match_count:
                    src_pts = np.float32([self.first_kp[m.queryIdx].pt for m in self.good]).reshape(-1,1,2)
                    dst_pts = np.float32([self.curr_kp[m.trainIdx].pt for m in self.good]).reshape(-1,1,2)

                    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                return H

        if self.prev_des is not None and self.curr_des is not None and len(self.prev_des) > 3 and len(self.curr_des) > 3:
            matches = flann.knnMatch(self.prev_des,self.curr_des,k=2)
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
                dst_pts = np.float32([self.curr_kp[m.trainIdx].pt for m in self.good]).reshape(-1,1,2)

                H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

                self.prev_kp = self.curr_kp
                self.prev_des = self.curr_des

            return H

        else:
            print "Not enough matches are found - %d/%d" % (len(self.good),self.min_match_count)
            return None

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
 
    def decompose_pose(self,p):
        q = Quaternion(p.pose.orientation.w, p.pose.orientation.x,
            p.pose.orientation.y, p.pose.orientation.z)
        T = np.identity(4)
        T[0, 3] = p.pose.position.x
        T[1, 3] = p.pose.position.y
        T[2, 3] = p.pose.position.z
        R = q.rotation_matrix
        RT = np.identity(4)
        RT[0:3, 0:3] = R
        RT[0:3, 3] = T[0:3, 3]
        return RT

    def get_RT(self, start_RT, H):
        if H is None:
            return None
        retval, Rs, ts, norms = cv2.decomposeHomographyMat(H,
                self.camera_matrix)
        min_index = 0
        min_z_norm = 0
        for i in range(len(Rs)):
            if norms[i][2] < min_z_norm:
                min_z_norm = norms[i][2]
                min_index = i
        T = np.zeros(3)
        T = ts[min_index] * start_RT[2,3]

        return Rs[min_index], T, norms[min_index]


    def get_fixed_RT(self, start_RT, H):
        return self.get_RT(start_RT, H)


    def get_integrated_RT(self, start_RT, H):
        RTn = self.get_RT(start_RT, H)
        if RTn is None:
            return None
        else:
            R, T, norm = RTn
        RT = np.identity(4)
        RT[0:3, 0:3] = np.identity(3)
        RT[0:3, 3] = T.T
        est_RT = np.dot(RT, self.est_RT)
        return est_RT[0:3, 0:3], est_RT[0:3, 3], norm
        
imu_R = None
homography = Homography()
start_RT= None
vrpn_pos = None

def reset_callback(data):
    global homography
    vrpn_RT = homography.decompose_pose(vrpn_pos)
    homography.est_RT[0:3, 3] = vrpn_RT[0:3, 3] - start_RT[0:3, 3]

def imu_callback(data):
    global imu_R
    imu_R = Quaternion(data.pose.orientation.w, data.pose.orientation.x,
            data.pose.orientation.y, data.pose.orientation.z).rotation_matrix


def vrpn_callback(data):
    global start_RT
    if start_RT is None:
        start_RT = homography.decompose_pose(data)
    global vrpn_pos
    vrpn_pos = data
    

if __name__ == '__main__':
    global start_RT
    global imu_R
    rospy.init_node("homography_flight")
    homopospub = rospy.Publisher('/pidrone/homo_pos', PoseStamped, queue_size=1)
    homovelpub = rospy.Publisher('/pidrone/homo_vel', Marker, queue_size=1)
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_callback)
    rospy.Subscriber("/pidrone/multiwii_attitude", PoseStamped, imu_callback)
    rospy.Subscriber("/pidrone/reset_pos", Empty, reset_callback)
    prev_img = None
    init_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    homo_pos = None
    plane_smooth = 1.0
    z_smooth = 1.0
    prev_time = None

    for curr_img in streamPi():
        curr_img = np.array(curr_img)
        curr_img = cv2.cvtColor(np.array(curr_img), cv2.COLOR_RGB2GRAY)
        
        if prev_img is None:
            print "first prev"
            prev_img = deepcopy(curr_img)
        else:
            if start_RT is not None:
                # run homography on a new image and integrate H
                first_H = homography.get_H(curr_img, prev_img=prev_img, first=True)
                integrated_H = homography.get_H(curr_img, prev_img=prev_img,
                        first=False)
                homography.curr_kp = None
                homography.curr_des = None

# Ommitting rotation right now
                first_RTn = homography.get_fixed_RT(start_RT, first_H)
                integrated_RTn = homography.get_integrated_RT(start_RT,
                        integrated_H)
                homo_RT = None
                if integrated_RTn is None and first_RTn is None:
                    pass
                else:
                    if integrated_RTn is not None and first_RTn is None:
                        print 1
                        homography.est_RT[0:3, 3] = integrated_RTn[1]
                    elif integrated_RTn is None and first_RTn is not None:
                        homography.est_RT[0:3, 3] = first_RTn[1][0]
                    else:
                        print integrated_RTn[1], first_RTn[1].T[0]
                        homography.est_RT[0:3, 3] = (1.0 - homography.alpha) * integrated_RTn[1] + homography.alpha * first_RTn[1].T[0]
                    homo_RT = homography.est_RT
                    print homo_RT[0:3, 3]

                if homo_RT is not None:
                    homo_R = homo_RT[0:3, 0:3]
                    homo_T = homo_RT[0:3, 3]
                    # how we've moved in the cameras reference frame
                    homo_RT[0:3,3] = np.dot(np.dot(homo_R, init_R), homo_T).T
                    # update our start position by how we've moved
                    homo_RT[0:3,0:3] = np.dot(np.dot(init_R, homo_R.T), init_R)
                    homo_pos = homography.compose_pose(np.dot(start_RT,homo_RT))
                    # print (np.array([homo_pos.pose.position.x, homo_pos.pose.position.y, homo_pos.pose.position.z]) - 
                    #         np.array([vrpn_pos.pose.position.x, vrpn_pos.pose.position.y, vrpn_pos.pose.position.z]))
                    homopospub.publish(homo_pos)
                else:
                    print "No homography matrix :(" 

                prev_img = deepcopy(curr_img) # Updates prev img
            else:
                print "No VRRN :("
