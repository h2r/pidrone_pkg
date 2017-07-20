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
start_RT= None


def vrpn_callback(data):
    global start_RT
    if start_RT is None:
        start_RT = homography.decompose_pose(data)
    

if __name__ == '__main__':
    global start_RT
    rospy.init_node("homography_flight")
    homopospub = rospy.Publisher('/pidrone/homo_pos', PoseStamped, queue_size=1)
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_callback)
    prev_img = None
    init_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    for curr_img in streamPi():
        curr_img = np.array(curr_img)
        curr_img = cv2.cvtColor(np.array(curr_img), cv2.COLOR_RGB2GRAY)
        
        if prev_img is None:
            print "first prev"
            prev_img = deepcopy(curr_img)
        else:
            if start_RT is not None:
                # run homography on a new image and integrate H
                homography.updateH(curr_img, prev_img=prev_img)

                homo_RTn = homography.get_pose_alt(start_RT)
                homo_RT = np.identity(4)
                if homo_RTn is not None:
                    homo_R, homo_T, homo_norm = homo_RTn 
                    
                    # how we've moved in the cameras reference frame
                    homo_RT[0:3,3] = np.dot(np.dot(homo_R, init_R), homo_T).T
                    # update our start position by how we've moved
                    # homo_RT[0:3,3] = homo_T_trans.T + start_RT[0:3,3]
                    # update our rotation by how we've rotated
                    homo_RT[0:3,0:3] = np.dot(np.dot(init_R, homo_R.T), init_R)



                    homo_pos = homography.compose_pose(np.dot(start_RT,homo_RT))
                    homopospub.publish(homo_pos)
                else:
                    print "No homography matrix :(" 

                prev_img = deepcopy(curr_img) # Updates prev img
            else:
                print "No VRRN :("
