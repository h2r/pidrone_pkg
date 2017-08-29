#!/usr/bin/env python
from __future__ import division
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped
import time
import tf
import math
import numpy as np
from copy import deepcopy

prev_pos_time = time.time()

pospub = rospy.Publisher('/pidrone/vrpn_pos', PoseStamped, queue_size=1)
sdpub = rospy.Publisher('/pidrone/sd_pos', PoseStamped, queue_size=1)

def callback_pos(data):
    global prev_pos_time
    
    if time.time() - prev_pos_time > 0:
    # if time.time() - prev_pos_time > 1.0/20.0:
        prev_pos_time = time.time()
        ret = PoseStamped()
        ret.header = data.header
        ret.pose.position.x = data.pose.position.x * 100
        ret.pose.position.y = data.pose.position.y * 100
        ret.pose.position.z = data.pose.position.z * 100
        ret.pose.orientation.x = data.pose.orientation.x
        ret.pose.orientation.y = data.pose.orientation.y
        ret.pose.orientation.z = data.pose.orientation.z
        ret.pose.orientation.w = data.pose.orientation.w
#   offset_rotation = np.array([0.707, 0.707, 0, 0])
        q = data.pose.orientation

#   new_q = tf.transformations.quaternion_multiply(offset_rotation, [q.w, q.x, q.y, q.z])
#   print(new_q)
#   ret.pose.orientation.x = new_q[3]
#   ret.pose.orientation.y = new_q[0]
#   ret.pose.orientation.z = new_q[1]
#   ret.pose.orientation.w = new_q[2]
        pospub.publish(ret)

def callback_sd(data):
    
    ret = PoseStamped()
    ret.header = data.header
    ret.pose.position.x = data.pose.position.x * 100
    ret.pose.position.y = data.pose.position.y * 100
    ret.pose.position.z = data.pose.position.z * 100
    ret.pose.orientation.x = data.pose.orientation.x
    ret.pose.orientation.y = data.pose.orientation.y
    ret.pose.orientation.z = data.pose.orientation.z
    ret.pose.orientation.w = data.pose.orientation.w
#   offset_rotation = np.array([0.707, 0.707, 0, 0])
    q = data.pose.orientation

#   new_q = tf.transformations.quaternion_multiply(offset_rotation, [q.w, q.x, q.y, q.z])
#   print(new_q)
#   ret.pose.orientation.x = new_q[3]
#   ret.pose.orientation.y = new_q[0]
#   ret.pose.orientation.z = new_q[1]
#   ret.pose.orientation.w = new_q[2]
    sdpub.publish(ret)


if __name__ == '__main__':
    rospy.init_node('vrpn_translation')
    rospy.Subscriber("/vrpn_client_node/steve/pose", PoseStamped, callback_pos)
    rospy.Subscriber("/vrpn_client_node/sd/pose", PoseStamped, callback_sd)
    rospy.spin()
