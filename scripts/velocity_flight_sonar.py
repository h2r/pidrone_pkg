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
set_z = 20
init_z = 0
smoothed_vel = np.array([0, 0, 0])
alpha = 1.0
ultra_z = 0
pid = PID()
first = True
error = axes_err()
ceiling_height = 300

def ultra_callback(data):
    global ultra_z
    global heightpub
    global pid
    global first
    global init_z
    if data.range != -1:
        ultra_z = data.range
        try:
            if first:
                #board.arm()
                first = False
                init_z = ultra_z
            else:
                error.z.err = init_z - ultra_z + set_z
                cmds = pid.step(error)
                print error
                print cmds
                board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
        except Exception as e:
            board.disarm()
            raise

def plane_callback(data):
    global error
    error.x.err = data.x.err * (ceiling_height - ultra_z)
    error.y.err = data.y.err * (ceiling_height - ultra_z)

if __name__ == '__main__':
    rospy.init_node('velocity_flight_sonar')
    rospy.Subscriber("/pidrone/plane_err", axes_err, plane_callback)
    board = MultiWii("/dev/ttyACM0")
    rospy.Subscriber("/pidrone/infrared", Range, ultra_callback)
    rospy.spin()
    print "Shutdown Recieved"
    board.disarm()
    sys.exit()
