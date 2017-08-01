import cv2
import numpy as np
import time
import copy
from copy import deepcopy
# import rospy
# import tf
# from pyquaternion import Quaternion
# from camera_class import Camera
# from h2rPiCam import streamPi
# from homography_class import Homography
# from geometry_msgs.msg import PoseStamped


# IZZY's webcam
camera_matrix = np.array([
 [  1.04304578e+03,   0.00000000e+00,   5.46544858e+02],
 [  0.00000000e+00,   1.04432128e+03,  3.46273512e+02],
 [  0.00000000e+00,   0.00000000e+00,  1.00000000e+00]])
dist_coeffs =  np.array([0.11330149, -0.27730294, -0.00748054,  0.00234142,  0.28207847])

# RASPBERRY PI?
# camera_matrix = np.array([[ 253.70549591,    0.,          162.39457585], 
#                         [ 0.,          251.25243215,  126.5400089], 
#                         [   0.,            0., 1.        ]])
# dist_coeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,
#                         0.27998478])

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
marker_edge_length = 15.5
aruco_corners = []
ux = 0.003921216794351938 
uy = 0.003921216794351938
A = np.array([[ux, 0, 0, 0], [0, uy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
RT_intrinsic = np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
camera_corners = [(0,0), (320,0), (320,240),(0,240)]


#########################################
#       QUATERNION
#########################################

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

#########################################
#       ARUCO
#########################################

# returns an R and a t from camera to global if it finds an AR tag in the scene
def detect_aruco_marker(img):
    global aruco_corners
    aruco_corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict)
    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(aruco_corners,
    marker_edge_length, camera_matrix, dist_coeffs)
    ret = None
    if rvecs != None:
        rot = cv2.Rodrigues(rvecs)[0]   # get the rotation matrix
        ret = rot, tvecs[0][0]
    return ret

# takes an R and a t from camera to global (like from aruco, and flips them to global to camera
def invert_RT(R,t):
        R, t 
        R_inv = R.T
        t_cor = np.dot(R_inv, -1 * t)
        RT = np.identity(4)
        RT[0:3, 0:3] = R_inv
        RT[0:3, 3] = t_cor
        return R_inv, t_cor

# takes a frame, runs aruco, and returns an RT from global to camera
def aruco_RT(img):
    Rt = detect_aruco_marker(img)
    if Rt is not None:
        R, t = Rt
        R_inv, t_cor = invert_RT(R,t)
        RT = np.identity(4)
        RT[0:3, 0:3] = R_inv
        RT[0:3, 3] = t_cor
        return RT
    else:
        return None

#########################################
#       PROJECTION
#########################################

# reproject a world coordinate to the pixel coordinate given an R and t from aruco
def aruco_project_world_to_pixel(world_coordinate, R,t, k=camera_matrix):
    RT = np.identity(4)     # construct an [R|t] matrix
    RT[0:3,0:3] = R    
    RT[0:3, 3] = t

    world_coordinate = np.array([[7.75,7.75,0,1]]).T               # homogenous x,y,z,1
    cam_coordinate = np.dot(RT, world_coordinate)[0:3]             # go from world to camera
    cam_coordinate /= cam_coordinate[2]                            # normalize
    pix_coordinate = np.dot(camera_matrix, cam_coordinate)[0:2]    # convert from cm to pixels
    print pix_coordinate.T

# reproject a pixel to the plane given the R and t from aruco
def aruco_reproject_pixel_to_plane(pixel_coords, R,t, k=camera_matrix):
    R,t = invert_RT(R,t)            # flip the R and t (R.T, R.T * -t)
    RT = np.identity(4)
    RT[0:3,0:3] = R
    RT[0:3,3]   = t
    return reproject_pixel_to_plane(pixel_coords, RT, k=camera_matrix)

# reproject a pixel to the plane given an RT from global to camera coords
def reproject_pixel_to_plane(pixel_coords, RT, k=camera_matrix):
    uv_homogenous = np.array([[pixel_coords[0], pixel_coords[1], 1.0]]).T # construct a homogenous matrix u,v,1
    xy_homogenous = np.dot(np.linalg.inv(k), uv_homogenous)               # go from pixel coords to a world coords
    
    R = RT[0:3,0:3]
    t = RT[0:3, 3]
    ray = np.dot(R, xy_homogenous)  # find the ray of that pixel in global coords
    res = -t[2]/ray[2] * ray.T + t  # intersect that ray with the ground plane
    return res

#########################################
#       MAIN
#########################################

if __name__ == '__main__':
    # rospy.init_node("pixel_projector")
    vc = cv2.VideoCapture(0)    # set up the video capture from IZZYs webcam

    # for frame in streamPi():
    while True:
        rval, frame = vc.read()                                     # read a new frame from the webcam.
        if not rval: continue                                       # we failed to get a new frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)             # convert to black and white
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)    # undistort the images

        curr_RT = detect_aruco_marker(frame)
        if curr_RT is not None: 
            for i in range(4):
                corner = np.array([aruco_corners[0][0][i][0], aruco_corners[0][0][i][1]])
                print aruco_reproject_pixel_to_plane(corner, curr_RT[0], curr_RT[1])
                
