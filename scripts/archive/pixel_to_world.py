from __future__ import division
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from pyquaternion import Quaternion
import cv2
from copy import deepcopy
import rospy
from camera_class import Camera
import time

ux = 0.003921216794351938 
uy = 0.003921216794351938
A = np.array([[ux, 0, 0, 0], [0, uy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
R = None
T = None
R_intrinsic = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
T_intrinsic = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
clicked_coords = None
sd_coords = None
clicked_sd_coords = None
clicked_R = None
clicked_T = None
display_scale = 3.0
d = None

def pixel_to_global(A, R, T, R_intrinsic, T_intrinsic, xy, wh=(320,240)):
    npx = xy[0] - 0.5*wh[0]
    npy = xy[1] - 0.5*wh[1]
    Ax = np.dot(A, np.array([[npx * d, npy * d, d, 1]]).T)
    print 'Ax', Ax
    print 'R', R
    return  np.dot(np.dot(T, R), np.dot(np.dot(T_intrinsic, R_intrinsic), Ax))

def drone_callback(data):
    global R
    global T
    global d
    d = data.pose.position.z
    R = np.identity(4)
    T = np.identity(4)
    T[0, 3] = data.pose.position.x
    T[1, 3] = data.pose.position.y
    T[2, 3] = data.pose.position.z
    q = Quaternion(data.pose.orientation.w, data.pose.orientation.x,
        data.pose.orientation.y, data.pose.orientation.z)
    R = q.transformation_matrix
    # print 'rotation magnitude', np.linalg.norm(R - np.identity(4))

def sd_callback(data):
    global sd_coords
    sd_coords = (data.pose.position.x, data.pose.position.y,
        data.pose.position.z)

def click_callback(event, x, y, flags, param):
    global clicked_coords, clicked_R, clicked_T, clicked_sd_coords, R, T
    if event == cv2.EVENT_RBUTTONUP:
        clicked_coords = (x/display_scale, y/display_scale)
        # clicked_coords = (160, 120)
        print 'clicked coords', clicked_coords
        clicked_R = deepcopy(R)
        clicked_T = deepcopy(T)
        clicked_sd_coords = deepcopy(sd_coords)

if __name__ == '__main__':
    c = Camera()
    rospy.init_node('pixels')
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, drone_callback)
    rospy.Subscriber("/pidrone/sd_pos", PoseStamped, sd_callback)
    with open("output.txt", "a") as output:
        for img in c.getImage():
            while R is None or T is None or sd_coords is None or d is None:
                time.sleep(0.001)
            print 'got new image'
            global clicked_coords
            cv2.namedWindow('preview')
            cv2.setMouseCallback('preview', click_callback)
            big_img = cv2.resize(img, (0,0), fx = display_scale, fy = display_scale)
            cv2.imshow('preview', big_img)

            cv2.waitKey(0)
            while clicked_coords is None:
                time.sleep(0.001)
            dotted_img = cv2.circle(img, (int(clicked_coords[0]),
                int(clicked_coords[1])), 1, (255,255,255), -1)
            # dotted_img = cv2.circle(img, clicked_coords, 10, (255,255,255), -1)
            cv2.imshow('preview', dotted_img)
            cv2.waitKey(0)
            est = pixel_to_global(A, clicked_R, clicked_T, R_intrinsic, T_intrinsic,
                clicked_coords)
            # print clicked_coords
            print 'est', est
            # print "clicked_R", clicked_R
            # to_save = raw_input("press y to save")
            # if to_save == "y":
            #     print 'SAVED'
            #     output.write(str(clicked_coords))
            #     output.write("\n")
            #     output.write(str(clicked_sd_coords))
            #     output.write("\n")
            #     output.write(str(est))
            #     output.write("\n")
            #     output.write("\n")
            #     output.flush()
