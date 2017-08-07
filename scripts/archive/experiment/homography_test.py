#!/usr/bin/env python
from __future__ import division
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
import time
import tf
import math
import numpy as np
from copy import deepcopy
import cv2

first = True
first_img = None
transform = None

def callback(image):
    global first
    if first:
        first_img = image
    else:
        transform = cv2.estimateRigidTransform(first_img, image, False)
        print(transform)

if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    try:
        rospy.Subscriber("/camera/image_raw", image, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
