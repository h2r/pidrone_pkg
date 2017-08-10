import cv2
import numpy as np
from pidrone_pkg.msg import Mode
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Empty
import rospy
import tf
import copy
from pyquaternion import Quaternion
import time
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import sys

# Needed Variables
bridge = CvBridge()
first_img = None
first_kp = None
first_des = None
orb = cv2.ORB_create(nfeatures=500, nlevels=8, scaleFactor=1.01)
pospub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)

# Configuration Params
index_params = dict(algorithm = 6, table_number = 6,
                        key_size = 12, multi_probe_level = 1)
search_params = dict(checks = 50)
min_match_count = 10
camera_matrix = np.array([[ 250.0, 0., 160.0], 
                            [ 0., 250.0, 120.0], 
                            [   0., 0., 1.]])

z = 0

def height_callback(data):
    global z
    z = data.range

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

def decompose_pose(p):
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

def get_H(curr_img):
    curr_kp, curr_des = orb.detectAndCompute(curr_img,None)

    good = []
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    if first_des is not None and curr_des is not None and len(first_des) > 3 and len(curr_des) > 3:
        matches = flann.knnMatch(first_des,curr_des,k=2)
        # store all the good matches as per Lowe's ratio test.
        for test in matches:
            if len(test) > 1:
                m, n = test
                # if m.distance < 0.7*n.distance:
                #     good.append(m)
                if True:
                    good.append(m)

        if len(good) > min_match_count:
            src_pts = np.float32([first_kp[m.queryIdx].pt for m in good]).reshape(-1,1,2)
            dst_pts = np.float32([curr_kp[m.trainIdx].pt for m in good]).reshape(-1,1,2)

            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

        return H

    else:
        print "Not enough matches are found - %d/%d" % (len(good),min_match_count)
        return None

def get_RT(H):
    retval, Rs, ts, norms = cv2.decomposeHomographyMat(H,
            camera_matrix)
    min_index = 0
    min_z_norm = 0
    for i in range(len(Rs)):
        if norms[i][2] < min_z_norm:
            min_z_norm = norms[i][2]
            min_index = i

    T = ts[min_index] * z

    return Rs[min_index], T, norms[min_index]

def img_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    global first_img
    global first_kp
    global first_des
    constant = 3.0
    if first_img is None:
        first_img = deepcopy(img)
        first_kp, first_des = orb.detectAndCompute(first_img, None)
        print 'Found {} features for the first img!'.format(len(first_kp))
    else:
        H = get_H(img)
        if H is not None:
            R, t, n = get_RT(H)
            RT = np.identity(4)
            RT[0:3, 0:3] = R[0:3, 0:3]
            if np.linalg.norm(t) < 50:
                RT[0:3, 3] = t.T[0]
            pos = compose_pose(RT)
            mode = Mode()
            mode.mode = 5
            mode.x_velocity = pos.pose.position.x * constant
            mode.y_velocity = pos.pose.position.z * constant
            pospub.publish(mode)
        else:
            print "NONE"
        

if __name__ == "__main__":
    rospy.init_node("homography_position")
    rospy.Subscriber("/pidrone/camera", Image, img_callback)
    rospy.Subscriber("/pidrone/infrared", Range, height_callback)
    rospy.spin()
