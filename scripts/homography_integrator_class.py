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


class HomographyIntegrator():

    def __init__(self, start_RT = np.identity(4)):
        self.homography = Homography()
        self.start_RT = start_RT
        self.R = start_RT[0:3,0:3]
        self.t = start_RT[0:3,3]
        self.prev_img = None

    def step(self, curr_img):
        if self.prev_img is None:
            self.prev_img = deepcopy(curr_img)
            return None
        else:
            self.homography.updateH(curr_img, prev_img=self.prev_img)
            homo_RTn = self.homography.get_pose_alt(start_RT)
            homo_RT = np.identity(4)
            homo_pos = None

            if homo_RTn is not None:
                homo_R, homo_T, homo_norm = homo_RTn 
                
                # how we've moved in the cameras reference frame
                homo_RT[0:3,3] = np.dot(np.dot(homo_R, init_R), homo_T).T
                # update our start position by how we've moved
                # homo_RT[0:3,3] = homo_T_trans.T + start_RT[0:3,3]
                # update our rotation by how we've rotated
                homo_RT[0:3,0:3] = np.dot(np.dot(init_R, homo_R.T), init_R)

                homo_pos = self.homography.compose_pose(np.dot(start_RT,homo_RT))
                
            else: print "No homography matrix :(" 

            self.prev_img = deepcopy(curr_img) # Updates prev img

            return homo_pos

    def fix_rotation(self, R):
        self.R = R


