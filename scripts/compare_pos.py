from __future__ import division
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped, Transform
from sensor_msgs.msg import Image
import time
import tf
import math
import numpy as np
from copy import deepcopy
import cv2
from cv_bridge import CvBridge
import io
import picamera
import calendar

if __name__ == '__main__':
    rospy.init_node('compare_pos', anonymous=True)
    try:
        rospy.Subscriber("/pidrone/est_pos", PoseStamped, update_est)
        rospy.Subscriber("/pidrone/optitrak_pos", PoseStamped update_ref)
        time.sleep(0.5)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass