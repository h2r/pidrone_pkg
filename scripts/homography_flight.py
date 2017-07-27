import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty
import rospy
import tf
import copy
from pyquaternion import Quaternion
from camera_class import Camera
from h2rPiCam import streamPi
import time
from homography_class import Homography
from copy import deepcopy

imu_R = None

homography = Homography()

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
marker_edge_length = 15.5
kp1=None
des1=None
flann_index_kdtree = 0
start_RT= None
vrpn_pos = None

def reset_callback(data):
    global homography
    vrpn_RT = homography.decompose_pose(vrpn_pos)
    homography.est_RT[0:3, 3] = vrpn_RT[0:3, 3] - start_RT[0:3, 3]

def imu_callback(data):
    global imu_R
    imu_R = Quaternion(data.pose.orientation.w, data.pose.orientation.x,
            data.pose.orientation.y, data.pose.orientation.z).rotation_matrix


def vrpn_callback(data):
    global start_RT
    if start_RT is None:
        start_RT = homography.decompose_pose(data)
    global vrpn_pos
    vrpn_pos = data
    

if __name__ == '__main__':
    global start_RT
    global imu_R
    rospy.init_node("homography_flight")
    homopospub = rospy.Publisher('/pidrone/homo_pos', PoseStamped, queue_size=1)
    homovelpub = rospy.Publisher('/pidrone/homo_vel', Marker, queue_size=1)
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_callback)
    rospy.Subscriber("/pidrone/multiwii_attitude", PoseStamped, imu_callback)
    rospy.Subscriber("/pidrone/reset_pos", Empty, reset_callback)
    prev_img = None
    init_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    homo_pos = None
    plane_smooth = 1.0
    z_smooth = 1.0
    prev_time = None

    for curr_img in streamPi():
        curr_img = np.array(curr_img)
        curr_img = cv2.cvtColor(np.array(curr_img), cv2.COLOR_RGB2GRAY)
        # cv2.imshow("curr", curr_img)
        # cv2.waitKey(1)
        
        if prev_img is None:
            print "first prev"
            prev_img = deepcopy(curr_img)
            prev_time = time.time()
        else:
            if start_RT is not None:
                now = time.time()
                # run homography on a new image and integrate H
                homography.updateH(curr_img, prev_img=prev_img)

                homo_RTn = homography.get_pose_alt(start_RT)
                vel = homography.get_vel(start_RT, now - prev_time)
                vel_twist = Marker()
                pt1 = Point()
                pt2 = Point()
                pt2.x = vel[0][0]
                pt2.y = vel[1][0]
                pt2.z = vel[2][0]
                vel_twist.points.append(pt1)
                vel_twist.points.append(pt2)
                homovelpub.publish(vel_twist)
                homo_RT = np.identity(4)
                if homo_RTn is not None:
                    homo_R, homo_T, homo_norm = homo_RTn 
                    
                    # how we've moved in the cameras reference frame
                    homo_RT[0:3,3] = np.dot(np.dot(homo_R, init_R), homo_T).T
                    # update our start position by how we've moved
                    # homo_RT[0:3,3] = homo_T_trans.T + start_RT[0:3,3]
                    # update our rotation by how we've rotated
                    homo_RT[0:3,0:3] = np.dot(np.dot(init_R, homo_R.T), init_R)



                    # if homo_pos is None:
                    #     homo_pos = homography.compose_pose(np.dot(start_RT, homo_RT))
                    # else:
                    #     homo_pos_new = homography.compose_pose(np.dot(start_RT, homo_RT))
                    #     homo_pos.pose.position.x = homo_pos_new.pose.position.x * plane_smooth + homo_pos.pose.position.x * (1 - plane_smooth)
                    #     homo_pos.pose.position.y = homo_pos_new.pose.position.y * plane_smooth + homo_pos.pose.position.y * (1 - plane_smooth)
                    #     homo_pos.pose.position.z = homo_pos_new.pose.position.z * z_smooth + homo_pos.pose.position.z * (1 - z_smooth)
                    homo_pos = homography.compose_pose(np.dot(start_RT,homo_RT))
                    # print (np.array([homo_pos.pose.position.x, homo_pos.pose.position.y, homo_pos.pose.position.z]) - 
                    #         np.array([vrpn_pos.pose.position.x, vrpn_pos.pose.position.y, vrpn_pos.pose.position.z]))
                    homopospub.publish(homo_pos)
                    print homo_pos
                    prev_time = now
                else:
                    print "No homography matrix :(" 

                prev_img = deepcopy(curr_img) # Updates prev img
            else:
                print "No VRRN :("
