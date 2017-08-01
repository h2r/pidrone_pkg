import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
import rospy
import tf
import copy
from pyquaternion import Quaternion
from camera_class import Camera
import time
from h2rPiCam import streamPi

class Homography:
    """ Runs Homography AR"""
    
    def __init__(self, min_match_count = 10, width = 320, height = 240,
    # camera_matrix = np.array([[ 253.70549591,    0.,          162.39457585], 
    #                         [ 0.,          251.25243215,  126.5400089 ], 
    #                         [   0.,            0., 1.        ]]), 
    camera_matrix = np.array([[ 250.0,    0.,          160.0], 
                            [ 0.,          250.0,  120.0], 
                            [   0.,            0., 1.        ]]), 
    dist_coeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,
                            0.27998478]), 
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50), 
    marker_edge_length = 15.5, kp1=None, des1=None, flann_index_kdtree = 0, 
    index_params = dict(algorithm = 6, table_number = 6,
                        key_size = 12, multi_probe_level = 1), 
    search_params = dict(checks = 50)):

        self.min_match_count = min_match_count
        self.width = width
        self.height = height
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco_dict
        self.marker_edge_length = marker_edge_length
        self.kp1 = kp1
        self.des1 = des1
        self.flann_index_kdtree = flann_index_kdtree
        self.index_params = index_params
        self.search_params = search_params
        self.curr_pos = PoseStamped()
        self.curr_pos.header.frame_id = 'world'
        self.est_RT = None
        self.est_H = np.identity(3)
        self.est_H_raw = np.identity(3)
        self.z_average = 1
        self.est_t = np.array([[0, 0, 0]]).T
        self.est_R = np.identity(3)
        self.est_RT = np.identity(4)
        self.good = []

# Helper for findAruco
    def detectArucoMarker(self, img):
        corners, ids, regjected = cv2.aruco.detectMarkers(img, self.aruco_dict)
        detected_img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        rvecs, tvecs, start = cv2.aruco.estimatePoseSingleMarkers(corners,
        self.marker_edge_length, self.camera_matrix, self.dist_coeffs)
        ret = None
        if rvecs != None:
            rot = cv2.Rodrigues(rvecs)[0]   # get the rotation matrix
            inv_rot = np.transpose(rot)             # invert it
            new_translation = np.dot(inv_rot, np.multiply(tvecs[0][0],-1.0))
            ret = np.identity(3), np.array([[new_translation]])
        return ret

    # Returns the R and t found using the Aruco marker
    def findAruco(self, img):
        Rt = self.detectArucoMarker(img) 
        if Rt is None:
            return False
        else:
            self.est_R, tvecs = Rt
            self.est_t = np.transpose(tvecs[0])     
            self.updatePose()
            return True

    def updateH(self, curr_img, prev_img):
        orb = cv2.ORB_create(nfeatures=500, nlevels=8, scaleFactor=2.0)
        if self.kp1 is None or self.des1 is None:
            self.kp1, self.des1 = orb.detectAndCompute(prev_img,None)
        kp2, des2 = orb.detectAndCompute(curr_img,None)
        print len(kp2)

        self.good = []
        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        if self.des1 is not None and des2 is not None and len(self.des1) > 3 and len(des2) > 3:
            matches = flann.knnMatch(self.des1,des2,k=2)
            # store all the good matches as per Lowe's ratio test.
            for test in matches:
                if len(test) > 1:
                    m, n = test
                    # if m.distance < 0.7*n.distance:
                    #     self.good.append(m)
                    if True:
                        self.good.append(m)

            if len(self.good) > self.min_match_count:
                src_pts = np.float32([self.kp1[m.queryIdx].pt for m in self.good]).reshape(-1,1,2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in self.good]).reshape(-1,1,2)

                H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                self.est_H = H
                # self.est_H /= np.dot(self.est_H, np.array([0,0,1]))[2]
                # self.est_H_raw = np.dot(H, self.est_H_raw) # initial  
                # print np.dot(self.est_H_raw, np.array([0,0,1])),
                # print np.dot(self.est_H, np.array([0, 0, 1]))
                # self.est_H = np.dot(H, self.est_H) # test reversed

                self.kp1 = kp2
                self.des1 = des2

            return True

        else:
            print "Not enough matches are found - %d/%d" % (len(self.good),self.min_match_count)
            return False
   
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

    def get_vel(self, start_RT, timestep):
        retval, Rs, ts, norms = cv2.decomposeHomographyMat(self.est_H,
                self.camera_matrix)
        # print 'Weve got {} options'.format(len(Rs))
        # if s < len(Rs):
        min_index = 0
        min_z_norm = 0
        for i in range(len(Rs)):
            if norms[i][2] < min_z_norm:
                min_z_norm = norms[i][2]
                min_index = i

        T = np.zeros(3)
        T = ts[min_index] * start_RT[2,3]

        return T / timestep



    def get_pose_alt(self, start_RT):
        retval, Rs, ts, norms = cv2.decomposeHomographyMat(self.est_H,
                self.camera_matrix)
        # print 'Weve got {} options'.format(len(Rs))
        # if s < len(Rs):
        min_index = 0
        min_z_norm = 0
        for i in range(len(Rs)):
            if norms[i][2] < min_z_norm:
                min_z_norm = norms[i][2]
                min_index = i

        T = np.zeros(3)
        T = ts[min_index] * start_RT[2,3]
        

        RT = np.identity(4)
        RT[0:3, 0:3] = np.identity(3)
        if np.linalg.norm(T) < 10:
            RT[0:3, 3] = T.T

        est_RT = np.dot(RT, self.est_RT) # comment out for first frame
       
        return est_RT[0:3, 0:3], est_RT[0:3, 3], norms[min_index] # comment out for first frame

        # return Rs[min_index], T, norms[min_index] # comment out for integration
        
        # print 'ROTATION MAGNITUDE', np.linalg.norm(R[0:3,0:3]-np.identity(3))
        # twiddle = np.array([[1,0,0],[0,-1,0],[0,0, -1]])
        # start_T = start_RT[0:3,3]
        # start_R = start_RT[0:3,0:3]
        # start_R_inv = start_R.T
        # final_T = start_T + np.dot(start_R_inv, -1 * T)
        # R_twiddled = np.dot(start_R_inv, twiddle)
        
        # # final_R = np.dot(np.dot(start_R, twiddle), R[0:3,0:3])
        # final_R = R_twiddled
        # RT = np.identity(4)
        # RT[0:3,3] = final_T
        # RT[0:3,0:3] = final_R

        # print s
        # print 'RT\n', RT
        # print 'norm\n', norm

        # pos = self.compose_pose(RT)
        # return pos

first_pos = None

def seed_pos(data):
    global first_pos
    if first_pos is None:
        first_pos = data

if __name__ == '__main__':
    rospy.init_node("homography_class")
    pospub = []
    pospub.append(rospy.Publisher('/pidrone/homo_est1', PoseStamped,
        queue_size=1))
    pospub.append(rospy.Publisher('/pidrone/homo_est2', PoseStamped,
        queue_size=1))
    pospub.append(rospy.Publisher('/pidrone/homo_est3', PoseStamped,
        queue_size=1))
    pospub.append(rospy.Publisher('/pidrone/homo_est4', PoseStamped,
        queue_size=1))
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, seed_pos)
    homography = Homography()
    prev_img = streamPi().next()
    position_flag = False
    position = None
    first_img = None
    global first_pos
    print 'Waiting for first position from optitrak'
    while first_pos is None:
        time.sleep(0.001)
    print "found first pos"
    homography.est_RT = decompose_pose(first_pos) 
    start_RT = decompose_pose(first_pos)
    print "decomposed"
    # cv2.namedWindow('preview')
    for curr_img in streamPi():
        if first_img is None: first_img = curr_img
        elif first_img is not None and not position_flag:
            print 'Starting homography calculations'
            position_flag = homography.updateH(curr_img, prev_img = first_img)   
        elif position_flag:
            homography.updateH(curr_img)
            for i in range(4):
                raw_input('press key to continue')
                position = homography.get_pose_alt(start_RT, i)
                print "################"
                if position is not None:
                    pospub[0].publish(position)
