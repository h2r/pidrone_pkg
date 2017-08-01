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
import signal

vrpn_pos = None
set_z = 20
init_z = 0
smoothed_vel = np.array([0, 0, 0])
alpha = 1.0
ultra_z = 0
pid = PID()
first = True
error = axes_err()
about_to_land = False
cmds = [1500, 1500, 1500, 900]
# ceiling_height = 300

def idle():
    pass

def takeoff():
    pass

def land():
    global set_z
    global about_to_land
    try:
        for i in range(set_z, 0, -1):
            set_z -= 1
            time.sleep(0.1)
        about_to_land = True
        board.disarm()
    except Exception as e:
        board.disarm()
        raise

def hover():
    pass

def disarm():
    pass

def set_velocity():
    pass

def kill_throttle():
    pass

def ultra_callback(data):
    global ultra_z
    global heightpub
    global pid
    global first
    global init_z
    global cmds
    if data.range != -1:
        ultra_z = data.range
        print 'ultra_z', ultra_z
        try:
            if not about_to_land:
                if first:
                    board.arm()
                    init_z = ultra_z
                    first = False
                else:
                    # att_data = board.getData(MultiWii.ATTITUDE)
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
    error.x.err = data.x.err * ultra_z
    error.y.err = data.y.err * ultra_z

def ctrl_c_handler(signal, frame):
    print "Land Recieved"
    land()
    sys.exit()

if __name__ == '__main__':
    rospy.init_node('velocity_flight_sonar')
    rospy.Subscriber("/pidrone/plane_err", axes_err, plane_callback)
    board = MultiWii("/dev/ttyUSB0")
    rospy.Subscriber("/pidrone/infrared", Range, ultra_callback)
    signal.signal(signal.SIGINT, ctrl_c_handler)
    rospy.spin()
    print "Shutdown Recieved"
    land()
    # board.disarm()
    sys.exit()
