import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
import rospy
import tf
import copy
from pyquaternion import Quaternion
from camera_class import Camera
from h2rPiCam import streamPi
import time
from homography_class import Homography
from copy import deepcopy

# camera = Camera(width=320, height=240)

homography = Homography()

camera_matrix = np.array([[ 253.70549591,    0.,          162.39457585], 
                        [ 0.,          251.25243215,  126.5400089], 
                        [   0.,            0., 1.        ]])
dist_coeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,
                        0.27998478])
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
marker_edge_length = 15.5
kp1=None
des1=None
flann_index_kdtree = 0
vrpn_pos = None

def detectArucoMarker(img):
    corners, ids, regjected = cv2.aruco.detectMarkers(img, aruco_dict)
    # detected_img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
    rvecs, tvecs, start = cv2.aruco.estimatePoseSingleMarkers(corners,
    marker_edge_length, camera_matrix, dist_coeffs)
    ret = None
    if rvecs != None:
        rot = cv2.Rodrigues(rvecs)[0]   # get the rotation matrix
        ret = rot, tvecs[0][0]
    return ret

def decompose_pose(p):
    q = Quaternion(p.pose.orientation.w, p.pose.orientation.x,
        p.pose.orientation.y, p.pose.orientation.z)
    T = np.identity(4)
    T[0, 3] = p.pose.position.x
    T[1, 3] = p.pose.position.y
    T[2, 3] = p.pose.position.z
    R = q.transformation_matrix
    return np.dot(R, T)


def compose_pose(RT):
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

def arucoRT(img):
    Rt = detectArucoMarker(img)
    if Rt is not None:
        R, t = Rt
        R_inv = R.T
        t_cor = np.dot(R_inv, -1 * t)
        # twiddle = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        twiddle = np.identity(3)
        R_twiddled = np.dot(R_inv, twiddle)
        RT = np.identity(4)
        for i in range(3):
            for j in range(3):
                RT[i, j] = R_twiddled[i, j]
        for i in range(3):
            RT[i, 3] = t_cor[i]
        return RT
    else:
        return None


def vrpn_callback(data):
    global vrpn_pos
    if vrpn_pos is None:
        vrpn_pos = decompose_pose(data)
    

if __name__ == '__main__':
    rospy.init_node("aruco_flight")
    prepospub = rospy.Publisher('/pidrone/aruco_est_pre', PoseStamped, queue_size=1)
    postpospub = rospy.Publisher('/pidrone/aruco_est_post', PoseStamped, queue_size=1)
    homopospub = rospy.Publisher('/pidrone/homo_trans', PoseStamped, queue_size=1)
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_callback)
    prev_img = None
    prev_RT = None
    key = ''
    for curr_img in streamPi():
        curr_img = np.array(curr_img)
        curr_img = cv2.cvtColor(np.array(curr_img), cv2.COLOR_RGB2GRAY)
        if prev_img is None:
            print "first prev"
            prev_img = deepcopy(curr_img)
        else:
            curr_RT = vrpn_pos
            prev_RT = vrpn_pos
            # curr_RT = arucoRT(curr_img)
            # prev_RT = arucoRT(prev_img)
            # cv2.imshow("curr", curr_img)
            # cv2.imshow("prev", prev_img)
            # cv2.waitKey(1)
            # if prev_RT is not None and curr_RT is not None:
            if vrpn_pos is not None:
                prev_pos = compose_pose(prev_RT)
                curr_pos = compose_pose(curr_RT)
                homography.updateHNew(curr_img, prev_img=prev_img)
                key = ''
                init_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                init_R_4 = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1,
                    0], [0, 0, 0, 1]])
                # while key != 'n':
                    # for i in range(4):
                print np.dot(homography.est_H, np.array([0, 0, 1]))
                homography.est_H /= np.dot(homography.est_H, np.array([0,0,1]))[2]
                homo_RTn = homography.get_pose_alt(prev_RT)
                if homo_RTn is not None:


#                             homo_R, homo_T, homo_norm = homo_RTn
#                             homo_T_trans = np.dot(np.dot(homo_R,
#                                 prev_RT[0:3, 0:3]), homo_T)
#                             homo_RT = np.identity(4)
#                             homo_RT[0, 3] = homo_T_trans[0]
#                             homo_RT[1, 3] = homo_T_trans[1]
#                             homo_RT[2, 3] = homo_T_trans[2] + prev_RT[2, 3]
#                             homo_RT[0:3, 0:3] = np.dot(homo_R, prev_RT[0:3, 0:3])
#                             homo_pos = compose_pose(homo_RT)
#                             key = raw_input("press a key")
#                             if key == 'n':
#                                 break
#                             elif key == 'p':
#                                 print "new prev"
#                                 prev_img = deepcopy(curr_img)
#                                 break
#                             print key
#                             homopospub.publish(homo_pos)




                    homo_R, homo_T, homo_norm = homo_RTn
                    homo_T_trans = np.dot(np.dot(homo_R,
                        init_R), homo_T)
                    homo_RT = np.identity(4)
                    homo_RT[0:3,3] = (homo_T_trans.T + prev_RT[0:3,3])
                    # homo_RT[0, 3] = homo_T_trans[0]
                    # homo_RT[1, 3] = homo_T_trans[1]
                    # homo_RT[2, 3] = homo_T_trans[2] + prev_RT[2, 3]
                    homo_RT[0:3, 0:3] = homo_R[0:3, 0:3]
                    homo_pos_pre = compose_pose(deepcopy(homo_RT))

                    #homo_pos_pre = compose_pose(homo_RT.T)
                    #homo_pos_pre = compose_pose(np.dot(homo_RT.T, init_R_4))
                    #homo_pos_pre = compose_pose(np.dot(init_R_4, homo_RT))

                    r, p, y = tf.transformations.euler_from_quaternion(np.array([homo_pos_pre.pose.orientation.x,
                        homo_pos_pre.pose.orientation.y, homo_pos_pre.pose.orientation.z,
                        homo_pos_pre.pose.orientation.w]))

                    q = tf.transformations.quaternion_from_euler(-r, p, y)
                    #q = tf.transformations.quaternion_from_euler(r, p, y)
                    homo_pos_pre.pose.orientation.x = q[0]
                    homo_pos_pre.pose.orientation.y = q[1]
                    homo_pos_pre.pose.orientation.z = q[2]
                    homo_pos_pre.pose.orientation.w = q[3]
                    homo_R_flipped = Quaternion([homo_pos_pre.pose.orientation.w,
                        homo_pos_pre.pose.orientation.x, homo_pos_pre.pose.orientation.y,
                        homo_pos_pre.pose.orientation.z]).rotation_matrix

                    # homo_RT[0:3, 0:3] = np.dot(homo_R, prev_RT[0:3, 0:3])
                    # homo_RT[0:3, 0:3] = np.dot(homo_R_flipped, prev_RT[0:3, 0:3])
                    homo_RT[0:3, 0:3] = homo_R_flipped

                    homo_pos = compose_pose(homo_RT)
                    # key = raw_input("press a key")
                    # if key == 'n':
                    #     break
                    # elif key == 'p':
                    #     print "new prev"
                    #     prev_img = deepcopy(curr_img)
                    #     break
                    # print key
                    homopospub.publish(homo_pos)
                    # print "##############################"
                    # print homo_T
                    # print homo_norm
                else:
                    print "##############################"
                    print "could not find"

                prepospub.publish(prev_pos)
                postpospub.publish(curr_pos)
                prev_img = deepcopy(curr_img) # Updates prev img
            else:
                print "can't see aruco"
