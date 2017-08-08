from pid_class import PID
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
import cv2
import rospy
import numpy as np
from pidrone_pkg.msg import axes_err, Mode
from h2rMultiWii import MultiWii
import time
import sys
import signal

set_z = 20
init_z = 0
smoothed_vel = np.array([0, 0, 0])
alpha = 1.0
ultra_z = 0
pid = PID()
first = True
error = axes_err()
cmds = [1500, 1500, 1500, 900]
current_mode = 4
set_vel_x = 0
set_vel_y = 0
errpub = rospy.Publisher('/pidrone/err', axes_err, queue_size=1)
flow_height_z = 0.000001

mw_angle_comp_x = 0
mw_angle_comp_y = 0
mw_angle_alt_scale = 1.0
mw_angle_coeff = 10.0

def arm():
    global cmds
    global current_mode
    if current_mode == 4:
        current_mode = 0
        cmds = [1500, 1500, 2000, 900]
        time.sleep(1)
        idle()

def idle():
    global cmds
    global current_mode
    if current_mode == 0:
        current_mode = 1
        cmds = [1500, 1500, 1500, 1000]

def takeoff():
    fly(None)

def land():
    global set_z
    global current_mode
    if current_mode == 5:
        try:
            for i in range(set_z, 0, -1):
                set_z -= 1
                time.sleep(0.1)
            disarm()
        except Exception as e:
            disarm()
            raise

def hover():
    pass

def disarm():
    global cmds
    global current_mode
    print "disarming"
    current_mode = 4
    cmds = [1500, 1500, 1000, 900]
    time.sleep(1)
    sys.exit()

def fly(velocity_cmd):
    global cmds
    global current_mode
    global set_vel_x, set_vel_y, set_z
    if current_mode == 1 or current_mode == 5:
        current_mode = 5
        if velocity_cmd is not None:
            set_vel_x = velocity_cmd.x_velocity
            set_vel_y = velocity_cmd.y_velocity
            if set_z + velocity_cmd.z_velocity > 20 and set_z + velocity_cmd.z_velocity < 50:
                set_z += velocity_cmd.z_velocity

def kill_throttle():
    pass

def mode_callback(data):
    global pid
    print "mode_input", data.mode
    if data.mode == 0:
        pid = PID()
        arm()
    elif data.mode == 1:
        idle()
    elif data.mode == 2:
        takeoff()
    elif data.mode == 3:
        land()
    elif data.mode == 4:
        disarm()
    elif data.mode == 5:
        fly(data)

def ultra_callback(data):
    global ultra_z, flow_height_z
    global pid
    global first
    global init_z
    global cmds
    if data.range != -1:
        # scale ultrasonic reading to get z accounting for tilt of the drone
        ultra_z = data.range * mw_angle_alt_scale
        # print 'ultra_z', ultra_z
        try:
            if current_mode == 5:
                if first:
                    #init_z = ultra_z
                    first = False
                else:
                    error.z.err = init_z - ultra_z + set_z
                    # print "setting cmds"
                    cmds = pid.step(error)
        except Exception as e:
            land()
            raise

def vrpn_callback(data):
    global ultra_z, flow_height_z
    global pid
    global first
    global init_z
    global cmds
    # scale ultrasonic reading to get z accounting for tilt of the drone
    ultra_z = data.pose.position.z
    # print 'ultra_z', ultra_z
    try:
        if current_mode == 5:
            if first:
                #init_z = ultra_z
                first = False
            else:
                error.z.err = init_z - ultra_z + set_z
                # print "setting cmds"
                cmds = pid.step(error)
    except Exception as e:
        land()
        raise

def plane_callback(data):
    global error
    global mw_angle_comp_x, mw_angle_comp_y
    global flow_height_z
    global set_vel_x, set_vel_y

    error.x.err = (data.x.err - mw_angle_comp_x) * ultra_z + set_vel_x
    error.y.err = (data.y.err + mw_angle_comp_y) * ultra_z + set_vel_y
    # error.z.err = data.z.err


def ctrl_c_handler(signal, frame):
    print "Land Recieved"
    disarm()

if __name__ == '__main__':
    global mw_angle_comp_x, mw_angle_comp_y, mw_angle_coeff, mw_angle_alt_scale
    rospy.init_node('state_controller')
    rospy.Subscriber("/pidrone/plane_err", axes_err, plane_callback)
    board = MultiWii("/dev/ttyUSB0")
    rospy.Subscriber("/pidrone/infrared", Range, ultra_callback)
#   rospy.Subscriber("/pidrone/vrpn_pos", PoseStamped, vrpn_callback)
    rospy.Subscriber("/pidrone/set_mode", Mode, mode_callback)
    signal.signal(signal.SIGINT, ctrl_c_handler)

    prev_angx = 0
    prev_angy = 0
    prev_angt = time.time()

    while not rospy.is_shutdown():
        # print current_mode, cmds
        errpub.publish(error)
        
        if current_mode != 4:
            # angle compensation calculations
            new_angt = time.time()
            mw_data = board.getData(MultiWii.ATTITUDE)
            new_angx = mw_data['angx']/180.0*np.pi
            new_angy = mw_data['angy']/180.0*np.pi
            mw_angle_comp_x = np.tan((new_angx - prev_angx) * (new_angt - prev_angt)) * mw_angle_coeff
            mw_angle_comp_y = np.tan((new_angy - prev_angy) * (new_angt - prev_angt)) * mw_angle_coeff
            # the ultrasonic reading is scaled by cos(roll) * cos(pitch)
            mw_angle_alt_scale = np.cos(new_angx) * np.cos(new_angy)
            prev_angx = new_angx
            prev_angy = new_angy
            prev_angt = new_angt

        board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
        board.receiveDataPacket()


    print "Shutdown Recieved"
    land()
    # board.disarm()
    sys.exit()
