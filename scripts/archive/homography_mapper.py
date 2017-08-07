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
from find_visible_subset import *

global_pixel_list = []
global_map_max = np.array([10,10])
global_map_min = np.array([-10,-10])
map_size = np.array([400,400]) # height,width cuz numpy
start_RT = np.identity(4)
vrpn_pos = PoseStamped()


def vrpn_callback(data):
    global start_RT
    if start_RT is None:
        start_RT = homography.decompose_pose(data)
    global vrpn_pos
    vrpn_pos = data

def world_to_pixel_map(pt, global_map_min, global_map_max, map_size):
    global_map_range = global_map_max - global_map_min
    pt_pixel_coord = ((pt - global_map_min)/global_map_range * map_size).astype(int)
    return pt_pixel_coord
 
if __name__ == '__main__':
    global start_RT
    rospy.init_node('homography_mapper')
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_callback)
    prev_img = None
    init_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    homography = Homography()

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



                    homo_pos = homography.compose_pose(homo_RT)
                    homo_pos.pose.position.x += vrpn_pos.pose.position.x
                    homo_pos.pose.position.y += vrpn_pos.pose.position.y
                    homo_pos.pose.position.z += vrpn_pos.pose.position.z
                    # print (np.array([homo_pos.pose.position.x, homo_pos.pose.position.y, homo_pos.pose.position.z]) - 
                    #         np.array([vrpn_pos.pose.position.x, vrpn_pos.pose.position.y, vrpn_pos.pose.position.z]))

                    ##################################################
                    #           Reproject to generate map            #
                    ##################################################
                    vrpn_RT = homography.decompose_pose(vrpn_pos)
                    for i in range(len(homography.kp1)):
                        not_a_match = True
                        for match in homography.good:
                            not_a_match and (i !=  match.trainIdx)

                        if not_a_match:
                            kp = homography.kp1[i]
                            des = homography.des1[i]
                            new_kp = pixel_to_global(kp.pt, vrpn_RT,
                                    vrpn_pos.pose.position.z)[0:2].T[0]
                            global_map_max = np.ceil(np.maximum(new_kp,
                                global_map_max))
                            global_map_min = np.floor(np.minimum(new_kp,
                                global_map_min))
                            global_pixel_list.append((new_kp, des))



                else:
                    print "No homography matrix :(" 
                # generate the map image of keypoints    
                pt_map = np.zeros(map_size)
                for pt, des in global_pixel_list:
                    pt_pixel_coord = world_to_pixel_map(pt, global_map_min, global_map_max, map_size)
                    # print 'Displaying', pt, pt_pixel_coord
                    pt_map[pt_pixel_coord[0]][pt_pixel_coord[1]] = 1 # rescale the coordinates and mark as black
               
                map_camera_corners = []
                for corner in find_bounding_box_global(vrpn_RT,
                        vrpn_pos.pose.position.z):
                    corner = corner[0:2].T
                    # print corner, world_to_pixel_map(corner,
                    # global_map_min, global_map_max, map_size)
                    map_camera_corners.append(world_to_pixel_map(corner,
                    global_map_min, global_map_max, map_size))
                
                pt_map = cv2.polylines(pt_map,
                        [np.array(map_camera_corners, dtype='int32')], True, 1)
                # cv2.polylines(pt_map, [np.array([[100,
                #     100],[300,100],[300,300],[100,300]], dtype='int32')], True, 1) 
                cv2.imshow('map_preview',pt_map) # display the map
                cv2.waitKey(1)
                prev_img = deepcopy(curr_img) # Updates prev img
            else:
                print "No VRRN :("
