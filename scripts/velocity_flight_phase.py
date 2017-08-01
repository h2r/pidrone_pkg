from pid_class import PID
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from h2rPiCam import streamPi
import cv2
import rospy
import numpy as np
from pidrone_pkg.msg import axes_err
from h2rMultiWii import MultiWii
import time
import sys

vrpn_pos = None
smoothed_vel = np.array([0, 0, 0])
alpha = 0.6
ultra_z = 0

def ultra_callback(data):
    global ultra_z
    #if data.range != -1:
        #ultra_z = data.range
    ultra_z = data.range

if __name__ == '__main__':
    rospy.init_node('velocity_flight')
    errpub = rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)
    first = True
    stream = streamPi()
    prev_img = None
    alpha = 0.8
    try:
        error = axes_err()
        while not rospy.is_shutdown():
            curr_img = cv2.cvtColor(np.float32(stream.next()), cv2.COLOR_RGB2GRAY)
            if first:
                prev_img = curr_img
                first = False
            else:
                correlation = cv2.phaseCorrelate(prev_img, curr_img)
                error.x.err = (1 - alpha) * error.x.err + alpha * correlation[0][0]
                error.y.err = (1 - alpha) * error.y.err + alpha * correlation[0][1]
                errpub.publish(error)
                prev_img = curr_img
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise
