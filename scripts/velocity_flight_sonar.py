from pid_class import PID
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from h2rPiCam import streamPi
import cv2
import rospy
import numpy as np
from pidrone_pkg.msg import RC, axes_err
from std_msgs.msg import Empty
from h2rMultiWii import MultiWii
import time
import sys

vrpn_pos = None
set_z = 35
init_z = 0
smoothed_vel = np.array([0, 0, 0])
alpha = 1.0
ultra_z = 0
cmds_plane = [1500, 1500]
heightpub = rospy.Publisher('/pidrone/height_pos', PoseStamped, queue_size=1)
pid = PID()
first = True

def vrpn_update_pos(data):
    global vrpn_pos
    global init_z
    vrpn_pos = data
    if init_z is None:
        init_z = vrpn_pos.pose.position.z

def ultra_callback(data):
    global ultra_z
    global heightpub
    global pid
    global first
    if data.range != -1:
        ultra_z = data.range
        try:
            if first:
                board.arm()
                first = False
            else:
                error = axes_err()
                print init_z, ultra_z
                error.z.err = init_z - ultra_z + set_z
                cmds_height = pid.step(error)
                print 'range', ultra_z
                cmds = [cmds_plane[0], cmds_plane[1], 0, cmds_height[3]]
                print error
                print cmds
                board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
                height_pos = PoseStamped()
                height_pos.pose.position.z = ultra_z
                heightpub.publish(height_pos)
        except Exception as e:
            board.disarm()
            raise

def plane_callback(data):
    global cmds_plane
    cmds_plane = [data.roll, data.pitch]

if __name__ == '__main__':
    rospy.init_node('velocity_flight_sonar')
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_update_pos)
    rospy.Subscriber("/pidrone/plane_cmds", RC, plane_callback)
    board = MultiWii("/dev/ttyACM0")
    rospy.Subscriber("/pidrone/infrared", Range, ultra_callback)
    rospy.spin()
    print "Shutdown Recieved"
    board.disarm()
    sys.exit()
