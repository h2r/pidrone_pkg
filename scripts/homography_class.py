import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
import rospy
import tf
import copy

class Homography:
    """ Runs Homography AR"""
    
    def __init__(self, min_match_count = 10, width = 320, height = 240,
    camera_matrix = np.array([[ 253.70549591,    0.,          162.39457585], 
                            [ 0.,          251.25243215,  126.5400089 ], 
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
        self.est_R = None
        self.est_t = None

    def pickHomography(self, Hs, ts, norms):
        sets = zip(Hs, ts, norms)
        if len(sets) > 1:
            last_two = [s for s in sets if s[2][2] > 0] # filter for norms with positive z

            # pick the winner with the lesser rotation
            # where rotation magnitude is ||R - I||
            if np.linalg.norm(last_two[0][0] - np.identity(3)) < np.linalg.norm(last_two[1][0] - np.identity(3)):
                return last_two[0]
            else:
                return last_two[1]
        else:
            return sets[0]

    def updatePose(self):
        big_mat = np.concatenate((self.est_R, self.est_t), 1)
        big_mat = np.concatenate((big_mat,np.matrix([1,1,1,1])), 0)
        quat_r = tf.transformations.quaternion_from_matrix(big_mat)
        self.curr_pos.header.seq += 1
        self.curr_pos.header.stamp = rospy.Time.now()
        self.curr_pos.pose.position.x = self.est_t[0][0]
        self.curr_pos.pose.position.y = self.est_t[1][0]
        self.curr_pos.pose.position.z = self.est_t[2][0]
        self.curr_pos.pose.orientation.x = quat_r[0]
        self.curr_pos.pose.orientation.y = quat_r[1]
        self.curr_pos.pose.orientation.z = quat_r[2]
        self.curr_pos.pose.orientation.w = quat_r[3]

        return self.curr_pos

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


# Uses homography to get the R and t
    def getRt(self, img2, img1 = None):
        assert img1 is not None or (self.kp1 is not None and self.des1 is not None)

        # Initiate SURF detector
        orb = cv2.ORB_create()
        # find the keypoints and descriptors with SIFT
        if self.kp1 is None:
            self.kp1, self.des1 = orb.detectAndCompute(img1,None)
        kp2, des2 = orb.detectAndCompute(img2,None)

        # Match keypoints
        good = []
        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        if self.des1 is not None and des2 is not None and len(self.des1) > 3 and len(des2) > 3:
            matches = flann.knnMatch(self.des1,des2,k=2)
            # store all the good matches as per Lowe's ratio test.
            for test in matches:
                if len(test) > 1:
                    m, n = test
                    if m.distance < 0.7*n.distance:
                        good.append(m)

        if len(good) > self.min_match_count:
            src_pts = np.float32([self.kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

            # Find Homography
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            retval, Rs, ts, norms = cv2.decomposeHomographyMat(H, self.camera_matrix)

            R, t, norm = self.pickHomography(Rs, ts, norms)
            t[0] *= -1
            R[2][0] *= -1
            R[0][2] *= -1
            self.kp1 = kp2
            self.des1 = des2
            return R, t

        else:
            print "Not enough matches are found - %d/%d" % (len(good),self.min_match_count)
            return None

# Helps updatePose
    def move(self, R0, t0, R, t):
        R_tmp = copy.deepcopy(R0)
        R_tmp[0][2] *= -1 # fix roll problem
        R_tmp[2][0] *= -1
        R_tmp[1][0] *= -1 # fix yaw problem
        R_tmp[0][1] *= -1
        t1 = t0 + np.dot(R_tmp.T, t)*t0[2]
        R1 = np.dot(R ,R0)
        return (R1, t1)

# Updates curr pos based on other params
    def updatePos(self, new_img, curr_img = None):
        Rt = self.getRt(new_img, curr_img)
        if Rt is not None:
            R, t = Rt
            self.est_R, self.est_t = self.move(self.est_R, self.est_t, R,t)
            # convert from R, t to quaternion
            big_mat = np.concatenate((self.est_R, self.est_t), 1)
            big_mat = np.concatenate((big_mat,np.matrix([1,1,1,1])), 0)
            quat_r = tf.transformations.quaternion_from_matrix(big_mat)
            return self.updatePose()
        else:
            return None
