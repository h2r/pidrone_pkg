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
init_z = None
smoothed_vel = np.array([0, 0, 0])
alpha = 1.0
ultra_z = 0
cmds_plane = [1500, 1500]

def vrpn_update_pos(data):
    global vrpn_pos
    global init_z
    vrpn_pos = data
    if init_z is None:
        init_z = vrpn_pos.pose.position.z

def ultra_callback(data):
    global ultra_z
    if data.range != -1:
        ultra_z = data.range

def plane_callback(data):
    global cmds_plane
    cmds_plane = [data.roll, data.pitch]

if __name__ == '__main__':
    rospy.init_node('velocity_flight_sonar')
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_update_pos)
    rospy.Subscriber("/pidrone/ultrasonic", Range, ultra_callback)
    rospy.Subscriber("/pidrone/plane_cmds", RC, plane_callback)
    emptypub = rospy.Publisher('/pidrone/empty', Empty, queue_size=1)
    pid = PID()
    first = True
    board = MultiWii("/dev/ttyACM0")
    while vrpn_pos is None:
        if not rospy.is_shutdown():
            print "No VRPN :("
            time.sleep(0.01)
        else:
            print "Shutdown Recieved"
            sys.exit()
    stream = streamPi()
    try:
        while not rospy.is_shutdown():
            if first:
                # board.arm()
                first = False
            else:
                error = axes_err()
                error.z.err = init_z - ultra_z + 40
                cmds_height = pid.step(error)
                print 'vrpn-ultra-error', ultra_z - vrpn_pos.pose.position.z
                cmds = [cmds_plane[0], cmds_plane[1], 0, cmds_height[3]]
                print error
                print cmds
                #board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
                emptypub.publish(Empty())
        print "Shutdown Recieved"
        board.disarm()
        sys.exit()
    except Exception as e:
        board.disarm()
        raise
