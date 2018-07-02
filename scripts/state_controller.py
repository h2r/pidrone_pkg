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
import sys


class StateController(object):

    # Possible modes
    ARMED = 0
    DISARMED = 4
    FLYING = 5

    def __init__(self):
        # Initial setpoint for the z-position of the drone
        self.initial_set_z = 30.0
        # Setpoint for the z-position of the drone
        self.set_z = self.initial_set_z
        # Current z-position of the drone according to sensor data
        self.current_z = 0

        # Tracks x, y velocity error and z-position error
        self.error = axes_err()
        # PID controller
        self.pid = PID()

        # Setpoint for the x-velocity of the drone
        self.set_vel_x = 0
        # Setpoint for the y-velocity of the drone
        self.set_vel_y = 0

        # Time of last heartbeat from the web interface
        self.last_heartbeat = None

        # Commanded x, y velocity of the drone
        self.cmd_velocity = [0, 0]
        # Commanded yaw velocity of the drone
        self.cmd_yaw_velocity = 0

        # Tracks previous drone attitude
        self.prev_angx = 0
        self.prev_angy = 0
        # Time of previous attitude measurement
        self.prev_angt = None

        # Angle compensation values (to account for tilt of drone)
        self.mw_angle_comp_x = 0
        self.mw_angle_comp_y = 0
        self.mw_angle_alt_scale = 1.0
        self.mw_angle_coeff = 10.0

        # Desired and current mode of the drone
        self.commanded_mode = self.DISARMED
        self.current_mode = self.DISARMED

        # Flight controller that receives commands to control motion of the drone
        self.board = MultiWii("/dev/ttyUSB0")

    def arm(self):
        """Arms the drone by sending the arm command to the flight controller"""
        arm_cmd = [1500, 1500, 2000, 900]
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, arm_cmd)
        self.board.receiveDataPacket()
        rospy.sleep(1)

    def disarm(self):
        """Disarms the drone by sending the disarm command to the flight controller"""
        disarm_cmd = [1500, 1500, 1000, 900]
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, disarm_cmd)
        self.board.receiveDataPacket()
        rospy.sleep(1)

    def idle(self):
        """Enables the drone to continue arming until commanded otherwise"""
        idle_cmd = [1500, 1500, 1500, 1000]
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, idle_cmd)
        self.board.receiveDataPacket()

    def fly(self, fly_cmd):
        """Enables flight by sending the calculated flight command to the flight controller"""
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, fly_cmd)
        self.board.receiveDataPacket()

    def update_fly_velocities(self, msg):
        """Updates the desired x, y, yaw velocities and z-position"""
        if msg is not None:
            self.set_vel_x = msg.x_velocity
            self.set_vel_y = msg.y_velocity

            self.cmd_velocity = [msg.x_i, msg.y_i]
            self.cmd_yaw_velocity = msg.yaw_velocity

            new_set_z = self.set_z + msg.z_velocity
            if 0.0 < new_set_z < 49.0:
                self.set_z = new_set_z

    def heartbeat_callback(self, msg):
        """Updates the time of the most recent heartbeat sent from the web interface"""
        self.last_heartbeat = rospy.Time.now()

    def infrared_callback(self, msg):
        """Updates the current z-position of the drone as measured by the infrared sensor"""
        # Scales infrared sensor reading to get z accounting for tilt of the drone
        self.current_z = msg.range * self.mw_angle_alt_scale
        self.error.z.err = self.set_z - self.current_z

    def vrpn_callback(self, msg):
        """Updates the current z-position of the drone as measured by the motion capture rig"""
        # Mocap uses y-axis to represent the drone's z-axis motion
        self.current_z = msg.pose.position.y
        self.error.z.err = self.set_z - self.current_z

    def plane_callback(self, msg):
        """Calculates error for x, y planar motion of the drone"""
        self.error.x.err = (msg.x.err - self.mw_angle_comp_x) * self.current_z + self.set_vel_x
        self.error.y.err = (msg.y.err + self.mw_angle_comp_y) * self.current_z + self.set_vel_y

    def mode_callback(self, msg):
        """Updates commanded mode of the drone and updates targeted velocities if the drone is flying"""
        self.commanded_mode = msg.mode
        
        if self.current_mode == self.FLYING:
            self.update_fly_velocities(msg)

    def calc_angle_comp_values(self, mw_data):
        """Calculates angle compensation values to account for the tilt of the drone"""
        new_angt = time.time()
        new_angx = mw_data['angx'] / 180.0 * np.pi
        new_angy = mw_data['angy'] / 180.0 * np.pi
        self.mw_angle_comp_x = np.tan((new_angx - self.prev_angx) * (new_angt - self.prev_angt)) * self.mw_angle_coeff
        self.mw_angle_comp_y = np.tan((new_angy - self.prev_angy) * (new_angt - self.prev_angt)) * self.mw_angle_coeff

        self.mw_angle_alt_scale = np.cos(new_angx) * np.cos(new_angy)

        self.pid.throttle.mw_angle_alt_scale = self.mw_angle_alt_scale

        self.prev_angx = new_angx
        self.prev_angy = new_angy
        self.prev_angt = new_angt

    def shouldIDisarm(self):
        """Disarms the drone if it has not received a heartbeat in the last five seconds"""
        if rospy.Time.now() - self.last_heartbeat > rospy.Duration.from_sec(5):
            return True
        else:
            return False

    def ctrl_c_handler(self, signal, frame):
        """Disarms the drone and exits the program if ctrl-c is pressed"""
        print "Caught ctrl-c! About to Disarm!"
        self.disarm()
        sys.exit()


if __name__ == '__main__':

    rospy.init_node('state_controller')

    sc = StateController()
    sc.last_heartbeat = rospy.Time.now()

    # ROS Setup
    ###########

    # Publishers
    ############
    errpub = rospy.Publisher('/pidrone/err', axes_err, queue_size=1)
    modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1)
    statepub = rospy.Publisher('/pidrone/state', State, queue_size=1, tcp_nodelay=False)

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/plane_err", axes_err, sc.plane_callback)
    rospy.Subscriber("/pidrone/infrared", Range, sc.infrared_callback)
    rospy.Subscriber("/pidrone/vrpn_pos", PoseStamped, sc.vrpn_callback)
    rospy.Subscriber("/pidrone/set_mode_vel", Mode, sc.mode_callback)
    rospy.Subscriber("/pidrone/heartbeat", String, sc.heartbeat_callback)

    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, sc.ctrl_c_handler)

    sc.prev_angt = time.time()

    mode_to_pub = Mode()
    state_to_pub = State()

    while not rospy.is_shutdown():
        # Publishes current mode message
        mode_to_pub.mode = sc.current_mode
        modepub.publish(mode_to_pub)
        # Publishes current error message
        errpub.publish(sc.error)

        # Obtains data from the flight controller
        mw_data = sc.board.getData(MultiWii.ATTITUDE)
        analog_data = sc.board.getData(MultiWii.ANALOG)

        # Publishes current battery voltage levels to display on the web interface
        state_to_pub.vbat = sc.board.analog['vbat'] * 0.10
        state_to_pub.amperage = sc.board.analog['amperage']
        statepub.publish(state_to_pub)

        try:
            if not sc.current_mode == sc.DISARMED:
                sc.calc_angle_comp_values(mw_data)

                if sc.shouldIDisarm():
                    print "Disarming because a safety check failed."
                    break

            # Uses a PID controller to calculate the flight command: [roll, pitch, yaw, throttle]
            fly_cmd = sc.pid.step(sc.error, sc.cmd_yaw_velocity)

            # Finite state machine implementation of controlling the drone
            if sc.current_mode == sc.DISARMED:
                if sc.commanded_mode == sc.DISARMED:
                    print 'DISARMED -> DISARMED'
                elif sc.commanded_mode == sc.ARMED:
                    sc.arm()
                    sc.current_mode = sc.ARMED
                    print 'DISARMED -> ARMED'
                else:
                    print 'Cannot transition from Mode %d to Mode %d' % (sc.current_mode, sc.commanded_mode)

            elif sc.current_mode == sc.ARMED:
                if sc.commanded_mode == sc.ARMED:
                    sc.idle()
                    print 'ARMED -> ARMED'
                elif sc.commanded_mode == sc.FLYING:
                    sc.current_mode = sc.FLYING
                    sc.pid.reset(sc)
                    print 'ARMED -> FLYING'
                elif sc.commanded_mode == sc.DISARMED:
                    sc.disarm()
                    sc.current_mode = sc.DISARMED
                    print 'ARMED -> DISARMED'
                else:
                    print 'Cannot transition from Mode %d to Mode %d' % (sc.current_mode, sc.commanded_mode)

            elif sc.current_mode == sc.FLYING:
                if sc.commanded_mode == sc.FLYING:
                    sc.fly(fly_cmd)
                    print 'FLYING -> FLYING'
                elif sc.commanded_mode == sc.DISARMED:
                    sc.disarm()
                    sc.current_mode = sc.DISARMED
                    print 'FLYING -> DISARMED'
                else:
                    print 'Cannot transition from Mode %d to Mode %d' % (sc.current_mode, sc.commanded_mode)
        except:
            raise

    sc.disarm()
    print "Shutdown Received"


