from sensor_msgs.msg import Imu
import tf
import math
from visualization_msgs.msg import Marker, MarkerArray
from pid_class import PID
from solution_pid_class import student_PID
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
import cv2
import rospy
import numpy as np
from p3_pkg.msg import axes_err, Mode, State
from h2rMultiWii import MultiWii
import time
import sys
import signal
import rospkg
import yaml

# stefie10: High-level comments: 1) Make a class 2) put everything
# inside a main method and 3) no global variables.

# Import yaml and pass to roll_pid, pitch_pid, and throttle_pid

with open("pid_terms.yml", 'r') as stream:
    try:
        yaml_data = yaml.safe_load(stream)
        vx_yaml = yaml_data['vx']
        vy_yaml = yaml_data['vy']
        z_yaml = yaml_data['z']
    except yaml.YAMLError as exc:
        print exc
        print 'Failed to load PID terms! Exiting.'
        sys.exit(1)
        
landing_threshold = 9.
initial_set_z = 0.12
set_z = initial_set_z
smoothed_vel = np.array([0, 0, 0])
alpha = 1.0
ultra_z = 0
pid = PID()
roll_pid = student_PID(vx_yaml)
pitch_pid = student_PID(vy_yaml)
throttle_pid = student_PID(z_yaml)
first = True
error = axes_err()
cmds = [1500, 1500, 1500, 900]
current_mode = 4
set_vel_x = 0
set_vel_y = 0
errpub = rospy.Publisher('/pidrone/err', axes_err, queue_size=1)
modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1)
flow_height_z = 0.000001
reset_pid = True
pid_is = [0,0,0,0]
last_heartbeat = None

cmd_velocity = [0, 0]
cmd_yaw_velocity = 0

mw_angle_comp_x = 0
mw_angle_comp_y = 0
mw_angle_alt_scale = 1.0
mw_angle_coeff = 10.0

flow_x_old = 0.0
flow_y_old = 0.0

def arm():
    global cmds
    global current_mode
    if current_mode == 4:
        current_mode = 0
        cmds = [1500, 1500, 2000, 900]
        # stefie10: I would use rospy.sleep, because that is more
        # likely to play nicely with ros threads.  But really
        time.sleep(1)
        idle()

def idle():
    global cmds
    global current_mode
    if current_mode == 0:
        current_mode = 1
        cmds = [1500, 1500, 1500, 1000]


# stefie10: Remove the method (erase from the file).  We will do this
# sort of thing with Ein.  This whole setup is deeply confusing and
# flawed because it will interact in an unpredictable way with the
# underlying rospy threading module.  It will only work if
# multithreading is happening, and multithreading in Python is deeply
# broken. 
def takeoff():
    global set_z
    global current_mode
    global set_vel_x
    global set_vel_y
    if current_mode == 1:
        current_mode = 2
        try:
            for i in range(40, 60):
                set_vel_x = 0
                set_vel_y = 0
                set_z = i/2.
                rospy.sleep(0.1)
            fly(None)
        except Exception as e:
            disarm()
            raise


def land():
    global set_z, error, landing_threshold, ultra_z
    global current_mode
    # print "LANDING"
    if current_mode == 5 or current_mode == 2:
        current_mode = 3
        try:
            set_z = 0
#            for i in range(int(set_z), 0, -1):
#                set_z -= 1
#                # print i, 'fsjsoidjfdsfjdsijfdsfdsfj'
#                rospy.sleep(0.1)
#            while abs(error.z.err) > landing_threshold:
#                # print "NO", np.absolute(error.z.err), ultra_z, set_z
#                rospy.sleep(0.1)
#            print "I am not high ;)"
#            disarm()
        except Exception as e:
            print e
            print "ERROR"
            disarm()
            raise

def hover():
    pass


def shouldILand():
    if rospy.Time.now() - last_heartbeat > rospy.Duration.from_sec(5):
        return True
    else:
        return False

rospack = rospkg.RosPack()
path = rospack.get_path('p3_pkg')
f = open("%s/params/multiwii.yaml" % path)
means = yaml.load(f)
f.close()
print "means", means
accRawToMss = 9.8 / means["az"]
accZeroX = means["ax"] * accRawToMss
accZeroY = means["ay"] * accRawToMss
accZeroZ = means["az"] * accRawToMss


def publishRos(board, imupub, markerpub, markerarraypub, statepub):
    state = State()
    state.vbat = board.analog['vbat'] * 0.10
    state.amperage = board.analog['amperage']
    statepub.publish(state)
    
    imu = Imu()
    marker = Marker()
    roll = board.attitude['angx']
    pitch = board.attitude['angy']
    yaw = board.attitude['heading']
    quaternion = tf.transformations.quaternion_from_euler(roll * 2 * math.pi / 360, pitch * 2 * math.pi / 360, 0)
    # print(roll, pitch, yaw, quaternion)
    imu.header.frame_id = "base"
    imu.orientation.x = quaternion[0]
    imu.orientation.y = quaternion[1]
    imu.orientation.z = quaternion[2]
    imu.orientation.w = quaternion[3]
    imu.linear_acceleration.x = board.rawIMU['ax'] * accRawToMss - accZeroX
    imu.linear_acceleration.y = board.rawIMU['ay'] * accRawToMss - accZeroY
    imu.linear_acceleration.z = board.rawIMU['az'] * accRawToMss - accZeroZ
    imupub.publish(imu)

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "/base"
    marker.type = Marker.CUBE
    marker.action = 0
    marker.pose.orientation = imu.orientation
    marker.pose.position.x = imu.linear_acceleration.x * 0.1
    marker.pose.position.y = imu.linear_acceleration.y * 0.1
    marker.pose.position.z = imu.linear_acceleration.z * 0.1
    #marker.points.append(Point(0, 0, 0))
    #marker.points.append(Point(imu.linear_acceleration.x * 1,
    #                           imu.linear_acceleration.y * 1,
    #                           imu.linear_acceleration.z * 1))


    marker.id = 0
    marker.lifetime = rospy.Duration(10)
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1
    markerpub.publish(marker)

def disarm():
    global cmds
    global current_mode
    print "disarming"
    current_mode = 4
    cmds = [1500, 1500, 1000, 900]
    rospy.sleep(1)  

    # stefie10: What are the sleeps for?  Almost always a bad idea in
    # something like this.  While you are sleeping you are not
    # handlign messages (singlethreaded), or you are handling messages
    # and data is changing underneath you.

    # stefie10: Why is this working at all?  If you call sys.exit it
    # should exit the program, so how can you disarm and then rearm
    # the drone, which I do all the time?  sys.exit should rarely if
    # ever be called.  Almost always more readable to return out of
    # main or else raise an exception.

    sys.exit()

def fly(velocity_cmd):
    global cmds
    global initial_set_z
    global current_mode
    global set_vel_x, set_vel_y, set_z
    global cmd_velocity
    global cmd_yaw_velocity
    if current_mode == 1 or current_mode == 5 or current_mode == 2:
        current_mode = 5
        #set_z = initial_set_z
        if velocity_cmd is not None:
            set_vel_x = velocity_cmd.x_velocity
            set_vel_y = velocity_cmd.y_velocity
            scalar = 1.
            cmd_velocity = [velocity_cmd.x_i, velocity_cmd.y_i]
            cmd_yaw_velocity = velocity_cmd.yaw_velocity
            if set_z + velocity_cmd.z_velocity > 0.0 and set_z + velocity_cmd.z_velocity < 0.5:
                set_z += velocity_cmd.z_velocity
            print "set_z", set_z, "cmd.z_velocity", velocity_cmd.z_velocity

def kill_throttle():
    pass

def heartbeat_callback(msg):
    global last_heartbeat
    last_heartbeat = rospy.Time.now()

def mode_callback(data):
    global pid, reset_pid, pid_is, set_z, initial_set_z
    global roll_pid, pitch_pid, throttle_pid
    print 'GOT NEW MODE', data.mode
    # stefie10: PLEASE use enums here.  
    if data.mode == 0:
        reset_pid = True
        arm()
    elif data.mode == 1:
        idle()
    elif data.mode == 2:
        if reset_pid:
            reset_pid = False
            roll_pid.reset()
            pitch_pid.reset()
            throttle_pid.reset()
# stefie10: John and I refactored this to a method, but we thought
# that set_z should possibly also be inside the PID controller and it
# calls reset just once.  Then you could simply call pid.reset() and
# eliminate the reset_pid variable completely.
            set_z = initial_set_z
        takeoff()  # stefie10: Remove
    elif data.mode == 4:
        disarm()
    elif data.mode == 3:
        land()
    elif data.mode == 5:        # STATIC FLIGHT
        if reset_pid:
            reset_pid = False
            roll_pid.reset()
            pitch_pid.reset()
            throttle_pid.reset()
            set_z = initial_set_z
        fly(data)
#   elif data.mode == 6:        # DYNAMIC FLIGHT 
#       pid_is = pid.get_is()
#       pid_is[0:3] = 0
#       pid.set_is(pid_is)


def ultra_callback(data):
    global ultra_z, flow_height_z
    global pid
    global first
    global cmds
    global current_mode
    global cmd_velocity
    global cmd_yaw_velocity
    if data.range != -1:
        # scale ultrasonic reading to get z accounting for tilt of the drone
        ultra_z = data.range * mw_angle_alt_scale
        # XXX jgo experimental thrust  compensation, undoing and moving
        #ultra_z = data.range * mw_angle_alt_scale * mw_angle_alt_scale
        # XXX less experimental
        pid.throttle.mw_angle_alt_scale = mw_angle_alt_scale
        
        #print mw_angle_alt_scale, data.range, ultra_z # jgo
        # print 'ultra_z', ultra_z
        try:
            if current_mode == 5 or current_mode == 3 or current_mode == 2:
                if first:
                    first = False
                else:
                    error.z.err = ultra_z - set_z
                    # print "setting cmds"
                    time = rospy.get_time()
                    max_angle = 100
                    cmds[0] = max(min(pitch_pid.step(error.x.err, time), 1500 + max_angle), 1500 - max_angle)
                    cmds[1] = max(min(roll_pid.step(error.y.err, time), 1500 + max_angle), 1500 - max_angle)
                    cmds[3] = throttle_pid.step(error.z.err, time)
        except Exception as e:
            land()
            raise

#def vrpn_callback(data):
#    global ultra_z, flow_height_z
#    global pid
#    global first
#    global cmds
#    global current_mode
#    # scale ultrasonic reading to get z accounting for tilt of the drone
#    ultra_z = data.pose.position.z
#    # print 'ultra_z', ultra_z
#    try:
#        if current_mode == 5:
#            if first:
#                first = False
#            else:
#                error.z.err = ultra_z - set_z
#                # print "setting cmds"
#                cmds = pid.step(error)
#    except Exception as e:
#        land()
#        raise
#
# stefie10: as we discussed, these globals are UGH.  Make this a
# method on a class, replace all the global calls with "self.error",
# "self.mw_angle", etc.
def plane_callback(data):
    global error
    global mw_angle_comp_x, mw_angle_comp_y
    global flow_height_z
    global set_vel_x, set_vel_y
    global flow_x_old, flow_y_old
    #print set_vel_x, set_vel_y
    #error.x.err = (data.x.err - mw_angle_comp_x) * min(ultra_z, 30.) + set_vel_x
    #error.y.err = (data.y.err + mw_angle_comp_y) * min(ultra_z, 30.) + set_vel_y
    error.x.err = (data.x.err - mw_angle_comp_x) * ultra_z + set_vel_x
    error.y.err = (data.y.err + mw_angle_comp_y) * ultra_z + set_vel_y

#    alpha = 0.5
#    error.x.err = error.x.err * alpha + (1. - alpha) * flow_x_old
#    error.y.err = error.y.err * alpha + (1. - alpha) * flow_y_old
#    flow_x_old = error.x.err
#    flow_y_old = error.y.err
    # error.z.err = data.z.err


def ctrl_c_handler(signal, frame):
    print "Land Recieved"
    disarm()



if __name__ == '__main__':
    # stefie10: PUt all this code in a main() method.  The globals
    # should be fields in a class for the controller, as should all
    # the callbacks.
    global mw_angle_comp_x, mw_angle_comp_y, mw_angle_coeff, mw_angle_alt_scale
    global current_mode
    rospy.init_node('state_controller')
    rospy.Subscriber("/pidrone/plane_err", axes_err, plane_callback)
    board = MultiWii("/dev/ttyUSB0")
    rospy.Subscriber("/pidrone/infrared", Range, ultra_callback)
    #rospy.Subscriber("/pidrone/vrpn_pos", PoseStamped, vrpn_callback)
    rospy.Subscriber("/pidrone/set_mode_vel", Mode, mode_callback)
    rospy.Subscriber("/pidrone/heartbeat", String, heartbeat_callback)
    global last_heartbeat
    last_heartbeat = rospy.Time.now()
    signal.signal(signal.SIGINT, ctrl_c_handler)

    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    markerpub = rospy.Publisher('/pidrone/imu_visualization_marker', Marker, queue_size=1, tcp_nodelay=False)
    markerarraypub = rospy.Publisher('/pidrone/imu_visualization_marker_array', MarkerArray, queue_size=1, tcp_nodelay=False)
    statepub = rospy.Publisher('/pidrone/state', State, queue_size=1, tcp_nodelay=False)

    
    prev_angx = 0
    prev_angy = 0
    prev_angt = time.time()
    
    # stefie10: Use labeled constants for the four different modes.  I
    # should never have to remember that mode 0 is disarmed or
    # whatever it really is.  http://wiki.ros.org/msg#Constants.  It
    # is okay if they are integer constants but they should be
    # referenced by name (e.g., Mode.DISARMED) in the code.  I had to
    # inspect the code to figure out what mode was what, and still got
    # it wrong when writing the keyboard controller - it added a whole
    # bunch of extra time and needless errors.  Even if ROS didn't
    # support enums, this code could have used them by defining them
    # as variable constants.

    mode_to_pub = Mode()

    while not rospy.is_shutdown():
        mode_to_pub.mode = current_mode
        modepub.publish(mode_to_pub)
        errpub.publish(error)
        mw_data = board.getData(MultiWii.ATTITUDE)
        analog_data = board.getData(MultiWii.ANALOG)

        publishRos(board, imupub, markerpub, markerarraypub, statepub)        
        
        if current_mode != 4: # stefie10: ENUM ENUM ENUM!
            # angle compensation calculations
            try:
                # stefie10: Put this code inside of a method or several methods. 

                new_angt = time.time()
                new_angx = mw_data['angx']/180.0*np.pi
                new_angy = mw_data['angy']/180.0*np.pi
                mw_angle_comp_x = np.tan((new_angx - prev_angx) * (new_angt - prev_angt)) * mw_angle_coeff
                mw_angle_comp_y = np.tan((new_angy - prev_angy) * (new_angt - prev_angt)) * mw_angle_coeff
                d_theta_x = new_angx - prev_angx
                d_theta_y = new_angy - prev_angy
                dt = new_angt - prev_angt

                angle_mag = np.arccos(np.cos(d_theta_x * dt) * np.cos(d_theta_y * dt))
#               mw_angle_comp_x = np.sin(d_theta_x * dt) * angle_mag * mw_angle_coeff
#               mw_angle_comp_y = (-np.sin(d_theta_y * dt) * np.cos(d_theta_x * dt)) * angle_mag * mw_angle_coeff
                # print mw_angle_comp_x, mw_angle_comp_x_tan
                # the ultrasonic reading is scaled by cos(roll) * cos(pitch)
                #jgo
                #mw_angle_alt_scale = 1.
                mw_angle_alt_scale = np.cos(new_angx) * np.cos(new_angy)
                prev_angx = new_angx
                prev_angy = new_angy
                prev_angt = new_angt

                # stefie10:  Make sure this still exists and works in the refactor!

                if shouldILand():
                    print "Landing because a safety check failed."
                    break

            except:
                print "BOARD ERRORS!!!!!!!!!!!!!!"
                print "BOARD ERRORS!!!!!!!!!!!!!!"
                print "BOARD ERRORS!!!!!!!!!!!!!!"
                print "BOARD ERRORS!!!!!!!!!!!!!!"
                print "BOARD ERRORS!!!!!!!!!!!!!!"
                print "BOARD ERRORS!!!!!!!!!!!!!!"
                sys.exit()
                # stefie10: None of your code will be called after
                # sys.exit, so this is nonsensical.  it is almost
                # always better to raise an exception or break out of
                # the loop to close the board, to try to do a clean
                # exit.
                board.close()

        print cmds
        board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
        board.receiveDataPacket()
        time.sleep(0.01)

    print "Shutdown Recieved"
    board.disarm()


