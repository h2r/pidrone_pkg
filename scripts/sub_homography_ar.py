#!/usr/bin/env python
import subprocess as sp
import cv2
import numpy as np
import time
import math
import copy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Imu
import rospy
import tf
from os import ftruncate
from pyquaternion import Quaternion
from cv2 import aruco

MIN_MATCH_COUNT = 10

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50) # use the 5x5 dictionary of 50 markers
markerEdgeLength = 15.5 # the length of the edge of a marker. this is in cm for now

# generate aruco markers and save them in the local dir
#for i in range(5):
#    a = aruco.drawMarker(aruco_dict, i, sidePixels = 200, borderBits=1)
#    cv2.imwrite('aruco{}.jpg'.format(i), a)

WIDTH = 320
HEIGHT = 240

cameraMatrix = np.array([[ 253.70549591,    0.,          162.39457585],
 [   0.,          251.25243215,  126.5400089 ],
 [   0.,            0.,            1.        ]])
distCoeffs = np.array([ 0.20462996, -0.41924085,  0.00484044,  0.00776978,  0.27998478])

est_pos = PoseStamped()
est_pos.pose.position.z = 1
est_pos.pose.orientation.w = 1
est_pos.header.frame_id='map'

def flipRoll(R):
    R[0][2] *= -1 
    R[2][0] *= -1 
    #R[0][1] = -R[0][1]
    #R[1][0] = -R[1][0]
    return R

def detectArucoMarker(img):
    corners, ids, regjected = aruco.detectMarkers(img, aruco_dict)      # run aruco on the img
    detected_img = aruco.drawDetectedMarkers(img, corners, ids)         # draw the boxes on the detected markers

    rvecs, tvecs, start = aruco.estimatePoseSingleMarkers(corners, markerEdgeLength, cameraMatrix, distCoeffs)
    ret = None
    sq2 = np.sqrt(2)/2.0
    #r_offset = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    if rvecs != None:
        #rvecs[0][0][2] = rvecs[0][0][2] + 0.5 * np.pi
        rot = cv2.Rodrigues(rvecs)[0]   # get the rotation matrix
        inv_rot = np.transpose(rot)             # invert it
        #new_translation = np.dot(inv_rot, np.multiply(tvecs[0][0],-1.0))     # apply it to the translation matrix
        #t = np.dot(inv_rot, tvecs)
        #t[0][0] = 0
        #t[0][1] = 0
        #inv_rot[0][0] *= -1
        #inv_rot[1][1] *= -1
        
        #print t.shape, tvecs.shape
        ret = np.identity(3),np.array([[[0,0,50]]])
        

    return ret

def pickHomography(Hs, ts, norms):
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

def getRt(img1, img2, kp1, des1):

    # Initiate SURF detector
    surf = cv2.xfeatures2d.SURF_create(upright=True, nOctaves=4,
    hessianThreshold=500)

    # find the keypoints and descriptors with SIFT
    if kp1 is None:
        kp1, des1 = surf.detectAndCompute(img1,None)
        print("redoing frame 1")
    kp2, des2 = surf.detectAndCompute(img2,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    good = []
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    if des1 is not None and des2 is not None and len(des1) > 3 and len(des2) > 3:
        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
    
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        # normalize point positions
        # src_pts = cv2.undistortPoints(src_pts, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
        # dst_pts = cv2.undistortPoints(dst_pts, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)

        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

        retval, Rs, ts, norms = cv2.decomposeHomographyMat(H, cameraMatrix)

        R, t, norm = pickHomography(Rs, ts, norms)
        #t[1] *= -1
        t[0] *= -1
        R[2][0] *= -1
        R[0][2] *= -1
        return R, t, kp2, des2

    else:
        print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
        return None, None, None, None

def move(R0, t0, R, t):
    R_tmp = copy.deepcopy(R0)
    R_tmp[0][2] *= -1
    R_tmp[2][0] *= -1
    R_tmp[1][0] *= -1
    R_tmp[0][1] *= -1
    t1 = t0 + np.dot(R_tmp.T, t)*t0[2]
    
    R1 = np.dot(R ,R0)

    return (R1, t1)

if __name__ == '__main__':
    cmdpub = rospy.Publisher('/pidrone/est_pos', PoseStamped, queue_size=1)
    rospy.init_node('homography_transform', anonymous=False)
    
    raspividcmd = ['raspivid', '-fps', '20', '-t', '0', '-w', str(WIDTH), '-h',
    str(HEIGHT), '-r', '-', '--raw-format', 'yuv', '-o', '/dev/null', '-n',
    '-pf', 'baseline', '-drc', 'off', '-ex', 'fixedfps', '-fl']
    stream = sp.Popen(raspividcmd, stdout = sp.PIPE, universal_newlines = True)
    time.sleep(1)
    curr = None
    prev = None
    test = stream.stdout.read(WIDTH * HEIGHT + (WIDTH * HEIGHT / 2))[0:WIDTH * HEIGHT]
    curr = np.fromstring(test, dtype=np.uint8).reshape(HEIGHT, WIDTH)
    tmp = None
    while tmp is None:
        test = stream.stdout.read(WIDTH * HEIGHT + (WIDTH * HEIGHT / 2))[0:WIDTH * HEIGHT]
        curr = np.fromstring(test, dtype=np.uint8).reshape(HEIGHT, WIDTH)
        tmp = detectArucoMarker(curr) 
        print("looking for Aruco")
    print("found Aruco")
    est_R, tvecs = tmp
    est_t = np.transpose(tvecs[0])     
    
    kp1 = None
    des1 = None
    while True:
        test = stream.stdout.read(WIDTH * HEIGHT + (WIDTH * HEIGHT / 2))[0:WIDTH * HEIGHT]
        curr = np.fromstring(test, dtype=np.uint8).reshape(HEIGHT, WIDTH)
        if prev is not None and curr is not None:
            R, t, kp1, des1 = getRt(prev, curr, kp1, des1)
            if R is not None:
                est_R, est_t = move(est_R, est_t, R,t)
                # convert from R, t to quaternion
                big_mat = np.concatenate((est_R, est_t), 1)
                big_mat = np.concatenate((big_mat,np.matrix([1,1,1,1])), 0)
                quat_r = tf.transformations.quaternion_from_matrix(big_mat)
                
                est_pos.pose.position.x = est_t[0][0]
                est_pos.pose.position.y = est_t[1][0]
                est_pos.pose.position.z = est_t[2][0]
                est_pos.pose.orientation.x = quat_r[0]
                est_pos.pose.orientation.y = quat_r[1]
                est_pos.pose.orientation.z = quat_r[2]
                est_pos.pose.orientation.w = quat_r[3]
            else:
                print("none")
        prev = curr
        print(est_pos)
        cmdpub.publish(est_pos)

