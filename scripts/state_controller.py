from sensor_msgs.msg import Imu
import tf
import math
from visualization_msgs.msg import Marker, MarkerArray
from pid_class import PID
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
import rospy
import numpy as np
from pidrone_pkg.msg import axes_err, Mode, State
from h2rMultiWii import MultiWii
import time
import signal
import rospkg
import yaml

class StateController(object):

    def __init__(self):
        self.initial_set_z = 30.0
        self.set_z = self.initial_set_z
        self.ultra_z = 0

        self.error = axes_err()
        self.pid = PID()

        self.set_vel_x = 0
        self.set_vel_y = 0

        self.last_heartbeat = None

        self.cmd_velocity = [0, 0]
        self.cmd_yaw_velocity = 0

        self.prev_angx = 0
        self.prev_angy = 0
        self.prev_angt = None

        self.mw_angle_comp_x = 0
        self.mw_angle_comp_y = 0
        self.mw_angle_alt_scale = 1.0
        self.mw_angle_coeff = 10.0

        self.commanded_mode = 4

        self.keep_going = True

    def arm(self, board):
        arm_cmd = [1500, 1500, 2000, 900]
        board.sendCMD(8, MultiWii.SET_RAW_RC, arm_cmd)
        rospy.sleep(1)

    def disarm(self, board):
        disarm_cmd = [1500, 1500, 1000, 900]
        board.sendCMD(8, MultiWii.SET_RAW_RC, disarm_cmd)
        rospy.sleep(1)

    def fly(self, msg):
        if msg is not None:
            self.set_vel_x = msg.x_velocity
            self.set_vel_y = msg.y_velocity

            self.cmd_velocity = [msg.x_i, msg.y_i]
            self.cmd_yaw_velocity = msg.yaw_velocity

            new_set_z = self.set_z + msg.z_velocity
            if 0.0 < new_set_z < 49.0:
                self.set_z = new_set_z

    def heartbeat_callback(self, msg):
        self.last_heartbeat = rospy.Time.now()

    def ultra_callback(self, msg):
        if msg.range != -1:
            # scale ultrasonic reading to get z accounting for tilt of the drone
            self.ultra_z = msg.range * self.mw_angle_alt_scale
            self.error.z.err = self.set_z - self.ultra_z

    def vrpn_callback(self, msg):
        # mocap uses y-axis to represent the drone's z-axis motion
        self.ultra_z = msg.pose.position.y
        self.error.z.err = self.set_z - self.ultra_z

    def plane_callback(self, msg):
        self.error.x.err = (msg.x.err - self.mw_angle_comp_x) * self.ultra_z + self.set_vel_x
        self.error.y.err = (msg.y.err + self.mw_angle_comp_y) * self.ultra_z + self.set_vel_y

    def mode_callback(self, msg):
        self.commanded_mode = msg.mode

    def calc_angle_comp_values(self, mw_data):
        new_angt = time.time()
        new_angx = mw_data['angx'] / 180.0 * np.pi
        new_angy = mw_data['angy'] / 180.0 * np.pi
        self.mw_angle_comp_x = np.tan((new_angx - self.prev_angx) * (new_angt - self.prev_angt)) * self.mw_angle_coeff
        self.mw_angle_comp_y = np.tan((new_angy - self.prev_angy) * (new_angt - self.prev_angt)) * self.mw_angle_coeff

        self.mw_angle_alt_scale = np.cos(new_angx) * np.cos(new_angy)
        # I JUST ADDED THIS IN HAHA
        self.pid.throttle.mw_angle_alt_scale = self.mw_angle_alt_scale

        self.prev_angx = new_angx
        self.prev_angy = new_angy
        self.prev_angt = new_angt

    def shouldIDisarm(self):
        if rospy.Time.now() - self.last_heartbeat > rospy.Duration.from_sec(5):
            return True
        else:
            return False

    def ctrl_c_handler(self, signal, frame):
        print "Caught ctrl-c! About to Disarm!"
        self.keep_going = False

if __name__ == '__main__':

    rospy.init_node('state_controller')

    sc = StateController()
    sc.last_heartbeat = rospy.Time.now()

    ARMED = 0
    DISARMED = 4
    FLYING = 5

    # ROS Setup
    ###########

    # Publishers
    ############
    errpub = rospy.Publisher('/pidrone/err', axes_err, queue_size=1)
    modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1)
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    markerpub = rospy.Publisher('/pidrone/imu_visualization_marker', Marker, queue_size=1, tcp_nodelay=False)
    statepub = rospy.Publisher('/pidrone/state', State, queue_size=1, tcp_nodelay=False)

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/plane_err", axes_err, sc.plane_callback)
    rospy.Subscriber("/pidrone/infrared", Range, sc.ultra_callback)
    rospy.Subscriber("/pidrone/vrpn_pos", PoseStamped, sc.vrpn_callback)
    rospy.Subscriber("/pidrone/set_mode_vel", Mode, sc.mode_callback)
    rospy.Subscriber("/pidrone/heartbeat", String, sc.heartbeat_callback)

    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, sc.ctrl_c_handler)
    board = MultiWii("/dev/ttyUSB0")

    sc.prev_angt = time.time()

    current_mode = DISARMED
    mode_to_pub = Mode()

    while not rospy.is_shutdown() and sc.keep_going:
        mode_to_pub.mode = current_mode
        modepub.publish(mode_to_pub)
        errpub.publish(sc.error)

        mw_data = board.getData(MultiWii.ATTITUDE)
        analog_data = board.getData(MultiWii.ANALOG)

        try:
            if not current_mode == DISARMED:
                sc.calc_angle_comp_values(mw_data)

                if sc.shouldIDisarm():
                    print "Disarming because a safety check failed."
                    break

            fly_cmd = sc.pid.step(sc.error, sc.cmd_velocity, sc.cmd_yaw_velocity)

            if current_mode == DISARMED:
                if sc.commanded_mode == DISARMED:
                    print 'DISARMED -> DISARMED'
                elif sc.commanded_mode == ARMED:
                    sc.arm(board)
                    current_mode = ARMED
                    print 'DISARMED -> ARMED'
                else:
                    print 'Cannot transition from Mode %d to Mode %d' % (current_mode, sc.commanded_mode)

            elif current_mode == ARMED:
                if sc.commanded_mode == ARMED:
                    idle_cmd = [1500, 1500, 1500, 1000]
                    board.sendCMD(8, MultiWii.SET_RAW_RC, idle_cmd)
                    print 'ARMED -> ARMED'
                elif sc.commanded_mode == FLYING:
                    current_mode = FLYING
                    sc.pid.reset(sc)
                    print 'ARMED -> FLYING'
                elif sc.commanded_mode == DISARMED:
                    sc.disarm(board)
                    current_mode = DISARMED
                    print 'ARMED -> DISARMED'
                else:
                    print 'Cannot transition from Mode %d to Mode %d' % (current_mode, sc.commanded_mode)

            elif current_mode == FLYING:
                if sc.commanded_mode == FLYING:
                    r, p, y, t = fly_cmd
                    print 'Fly Commands (r, p, y, t): %d, %d, %d, %d' % (r, p, y, t)
                    board.sendCMD(8, MultiWii.SET_RAW_RC, fly_cmd)
                    print 'FLYING -> FLYING'
                elif sc.commanded_mode == DISARMED:
                    sc.disarm(board)
                    current_mode = DISARMED
                    print 'FLYING -> DISARMED'
                else:
                    print 'Cannot transition from Mode %d to Mode %d' % (current_mode, sc.commanded_mode)
        except:
            print "BOARD ERRORS!!!!!!!!!!!!!!"
            raise
        
        board.receiveDataPacket()

    sc.disarm(board)
    print "Shutdown Recieved"



