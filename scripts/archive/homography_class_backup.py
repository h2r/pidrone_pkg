import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
import rospy
import tf
import copy
from pyquaternion import Quaternion
from camera_class import Camera
import time

camera = Camera(width=320, height=240)

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
        self.est_H = np.identity(3)
        self.z_average = 1
    def pickHomography(self, Hs, ts, norms):
        sets = zip(Hs, ts, norms)
        max_set = sets[0]
        for s in sets:
            if s[2][2] > max_set[1][1]: max_set = s
        
        return max_set
        # if len(sets) > 1:
        #     last_two = [s for s in sets if s[2][2] > 0] # filter for norms with positive z

        #     # pick the winner with the lesser rotation
        #     # where rotation magnitude is ||R - I||
        #     if np.linalg.norm(last_two[0][0] - np.identity(3)) < np.linalg.norm(last_two[1][0] - np.identity(3)):
        #         return last_two[0]
        #     else:
        #         return last_two[1]
        # else:
        #     return sets[0]

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
        orb = cv2.ORB_create(nfeatures=500, nlevels=8, scaleFactor=2)
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
            self.est_H = H
            # FIRST FRAME
            # self.est_H = np.dot(H, self.est_H)
            R, t, norm = self.pickHomography(Rs, ts, norms)
            t[0] *= -1
            z_sum = 0
            z_num = 0
            for kp in src_pts:
                z_sum +=  np.dot(self.est_H,
                        np.concatenate((kp,np.array([[1]])),1).T)[2]
                z_num += 1
            self.z_average = z_sum / z_num
            print self.z_average
            # self.est_H = self.est_H / z_average
            # R[2][0] *= -1
            # R[0][2] *= -1
            # for kp in src_pts:
                # print np.dot(self.est_H,
                #         np.concatenate((kp,np.array([[1]])),1).T)
            # FIRST FRAME
            # self.kp1 = kp2
            # self.des1 = des2
            return R, t

        else:
            print "Not enough matches are found - %d/%d" % (len(good),self.min_match_count)
            return None

# Helps updatePose
    def move(self, R0, t0, R, t):
        # R_tmp = copy.deepcopy(R0)
        # R[0][2] *= -1 # fix roll problem
        # R[2][0] *= -1
        # R[1][2] *= -1 # fix pitch problem
        # R[2][1] *= -1
        # print R0
        # print t0
        # print R
        # print t
        # t1 = t0 + np.dot(R0.T, t)*t0[2]
        # t1 = t0 + t*t0[2]
        R1 = np.dot(R0 ,R)
        t1 = t0 + np.dot(R0, t)*t0[2]
        # R1 = R0
        return (R1, t1)

# Updates curr pos based on other params
    def updatePos(self, curr_img, prev_img = None):
        Rt = self.getRt(curr_img, prev_img)
        if Rt is not None:
            R, t = Rt
            self.est_R, self.est_t = self.move(self.est_R, self.est_t, R,t)
            return self.updatePose()
        else:
            return None
    
    def compose_pose(self, R, t):
        pos = PoseStamped()
        big_mat = np.concatenate((R, t), 1)
        big_mat = np.concatenate((big_mat,np.matrix([1,1,1,1])), 0)
        quat_r = tf.transformations.quaternion_from_matrix(big_mat)
        pos.header.frame_id = 'world'
        pos.header.seq += 1
        pos.header.stamp = rospy.Time.now()
        pos.pose.position.x = t[0][0]
        pos.pose.position.y = t[1][0]
        pos.pose.position.z = t[2][0]
        pos.pose.orientation.x = quat_r[0]
        pos.pose.orientation.y = quat_r[1]
        pos.pose.orientation.z = quat_r[2]
        pos.pose.orientation.w = quat_r[3]

        return pos
 
    def get_pose_alt(self, start_R, start_t, s):
        retval, Rs, ts, norms = cv2.decomposeHomographyMat(self.est_H \
                /self.z_average, self.camera_matrix)
        # print 'Weve got {} options'.format(len(Rs))
        if s < len(Rs):
            R = Rs[s]
            t = ts[s]
            norm = norms[s]
        else:
            print 'RAN OUT OF AXES'
            R, t, norm = self.pickHomography(Rs, ts, norms)
        # if np.absolute(norm[2]) < np.linalg.norm(norm[0:2]): 
            # print 'NOT USING THIS ONE'

        # print 'ANGLE', np.linalg.norm(R - np.identity(3)),'MAGNITUDE',\
        # np.linalg.norm(t), 'NORM', norm.T
        # print 'TRANSLATION\n', t
        # R[0][2] *= -1
        # R[2][0] *= -1
 
        flip_around_x = np.array([[1, 0, 0], [0,-1,0], [0,0,-1]])
        flip_R = np.dot(R,flip_around_x)

        diff_R = np.dot(start_R, flip_around_x)

        new_R = np.dot(start_R, R)
        
        diff_t = np.dot(diff_R, t)*start_t[2]
        # diff_t[2] *= -1 
        new_t = start_t +  diff_t       
        pos = self.compose_pose(new_R, new_t)
         
        return pos


    def get_pose(self, start_R, start_t, s):
        # print self.est_H
        retval, Rs, ts, norms = \
        cv2.decomposeHomographyMat(self.est_H, self.camera_matrix)
        # print 'Weve got {} options'.format(len(Rs))
        if s < len(Rs):
            R = Rs[s]
            t = ts[s]
            norm = norms[s]
        else:
            print 'RAN OUT OF AXES'
            R, t, norm = self.pickHomography(Rs, ts, norms)
        # if np.absolute(norm[2]) < np.linalg.norm(norm[0:2]): 
            # print 'NOT USING THIS ONE'
        # t[0] *= -1 
        # t[1] *= -1
        # t[2] *= -1
        # R_tmp = copy.deepcopy(R)
        # R_tmp[0][1] *= -1
        # R_tmp[1][0] *= -1
        # R_tmp[0][2] *= -1
        # R_tmp[2][0] *= -1
        # R_tmp[1][2] *= -1
        # R_tmp[2][1] *= -1
   
        # print 'ANGLE', np.linalg.norm(R - np.identity(3)),'MAGNITUDE',\
        # np.linalg.norm(t), 'NORM', norm.T
        # print 'ROTATION\n', R
        # print 'TRANSLATION\n', t
        
        flip_around_x = np.array([[1, 0, 0], [0,-1,0], [0,0,-1]])
        # flipped_R = np.dot(flip_around_x, R)
        # R[1][2] *= -1
        # R[2][1] *= -1
        # R[0][1] *= -1
        # R[1][0] *= -1
        flip_R = np.dot(R, flip_around_x)
        # t = np.dot(flip_around_x, t)
        new_R = np.dot(start_R, R)
        diff_t = np.dot(new_R.T, t)*start_t[2]
        diff_t[2] *= -1
        new_t = start_t +  diff_t       
        pos = self.compose_pose(new_R, new_t)
        return pos


first_pos = None

def seed_pos(data):
    global first_pos
    if first_pos is None:
        first_pos = data


def decompose_pose(p):
    q = Quaternion(p.pose.orientation.w, p.pose.orientation.x,
        p.pose.orientation.y, p.pose.orientation.z)
    t = np.array([[p.pose.position.x, p.pose.position.y,
        p.pose.position.z]]).T
    R = q.rotation_matrix
    return R,t

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
    prev_img = camera.getImage().next()
    position = None
    first_img = None
    global first_pos
    print 'Waiting for first position from optitrak'
    while first_pos is None:
        time.sleep(0.001)
    print "found first pos"
    homography.est_R, homography.est_t = decompose_pose(first_pos) 
    start_R, start_t = decompose_pose(first_pos)
    print "decomposed"
    # cv2.namedWindow('preview')
    for curr_img in camera.getImage():
        # cv2.imshow('preview',curr_img)
        # if cv2.waitKey(1) & 0xFF == ord('n'):
        # print 'Handling new frame'
        if first_img is None: first_img = curr_img
        elif first_img is not None and position is None:
            print 'Starting homopgraphy calculations'
            position = homography.updatePos(curr_img, prev_img = first_img)   
        elif position is not None:
            # position = homography.updatePos(curr_img)
            homography.updatePos(curr_img)
            for i in range(4):
                position = homography.get_pose_alt(start_R, start_t, i)
                # raw_input('press any key (but n) to advance')
                if position is not None:
                    pospub[i].publish(position)
