from pid_class import PID
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from h2rPiCam import streamPi
import cv2
import rospy
import numpy as np
from pidrone_pkg.msg import RC, axes_err
from h2rMultiWii import MultiWii
import time
import sys

vrpn_pos = None
init_z = None
smoothed_vel = np.array([0, 0, 0])
alpha = 0.6
ultra_z = 0

def vrpn_update_pos(data):
    global vrpn_pos
    global init_z
    vrpn_pos = data
    if init_z is None:
        init_z = vrpn_pos.pose.position.z

def ultra_callback(data):
    global ultra_z
    #if data.range != -1:
        #ultra_z = data.range
    ultra_z = data.range

if __name__ == '__main__':
    rospy.init_node('velocity_flight')
    cmdpub = rospy.Publisher('/pidrone/plane_cmds', RC, queue_size=1)
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_update_pos)
    rospy.Subscriber("/pidrone/infrared", Range, ultra_callback)
    pid = PID()
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
                if np.abs(correlation[0][0] * (300 - ultra_z)) > 500:
                    error.x.err = error.x.err
                else:
                    error.x.err = (1 - alpha) * error.x.err + alpha * correlation[0][0] * (300 - ultra_z)
                if np.abs(correlation[0][1] * (300 - ultra_z)) > 500:
                    error.y.err = error.y.err
                else:
                    error.y.err = (1 - alpha) * error.y.err + alpha * correlation[0][1] * (300 - ultra_z)
                print error.x.err, error.y.err
                cmds = pid.step(error)
                rc = RC()
                rc.roll = cmds[0]
                rc.pitch = cmds[1]
                cmdpub.publish(rc)
                prev_img = curr_img
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise
