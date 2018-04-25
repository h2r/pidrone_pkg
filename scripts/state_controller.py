#!/usr/bin/python
import rospy
from pidrone_pkg.msg import axes_err, Mode, State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from h2rMultiWii import MultiWii
import signal



class Position():
    ''' Struct to store x,y,z'''
    # This should be turned into a data class with the next python 3 release
    def __init__(self, x=0, y=0 ,z=0):
        self.x = x
        self.y = y
        self.z = z


def arm(board):
    ''' Arms the drone by sending the arm command to the flight controller'''

    arm_cmd = [1500, 1500, 2000, 900]
    board.sendCMD(8, MultiWii.SET_RAW_RC, arm_cmd)
    rospy.sleep(1)

def disarm(board):
    ''' Disarms the drone by sending the disarm command to the flight
    controller'''
    disarm_cmd = [1500, 1500, 1000, 900]
    board.sendCMD(8, MultiWii.SET_RAW_RC, disarm_cmd)
    rospy.sleep(1)


def heartbeat_callback(msg):
    ''' Updates last_heartbeat to current time upon receipt of heartbeat from
        basestation 
    '''
    global last_heartbeat
    last_heartbeat = rospy.Time.now()


def commanded_mode_callback(msg):
    ''' Updates the commanded mode for the state machine.
        
        This desired mode will be transitioned to by the state machine if it
        is a valid transition
    '''
    global commanded_mode
    commanded_mode = msg.mode


def commanded_position_callback(msg):
    ''' Updates the commanded position '''
    global commanded_position
    commanded_position.x = msg.pose.position.x
    commanded_position.y = msg.pose.position.y
    commanded_position.z = msg.pose.position.z


def current_position_callback_vrpn(msg):
    ''' Updates the drone's current position based on the Motion Tracker '''
    global current_position
    current_position.x = msg.pose.position.x
    current_position.y = msg.pose.position.y
    current_position.z = msg.pose.position.z


def ctrl_c_handler(signal, frame):
    ''' Catches a ctrl-c, and sets the keep_going flag to false'''
    print('Caught ctrl-c! About to Disarm!')
    global keep_going
    keep_going = False


def calc_error(current_position, commanded_position):
    ''' Returns current_position - commanded_position

    current_position and commanded_position are both Position() objects
    that have .x, .y, and .z fields
    '''
    dx = commanded_position.x - current_position.x
    dy = commanded_position.y - current_position.y
    dz = commanded_position.z - current_position.z
    error = Position(dx, dy, dz)
    return error


def clip(num, lower, upper):
    ''' Clips num to be between lower and upper '''
    return max( min(upper, num), lower)

class PositionController(object):
    '''THIS NEEDS TO MOVE INTO ANOTHER FILE'''
    def __init__(self):
        self.integrated_error_x = 0
        self.integrated_error_y = 0
        self.integrated_error_z = 0

        self.error_last = None


    def step(self, error):
        ''' Give flight controller outputs based on current and previous errors
        '''

        # Negative pitch points nose up
        # Positive pitch points nose down
        # Negative roll rolls left
        # positive roll rolls right

        # with the Motive Tracker setup on 4/25/2018, +Z is toward baxter
        # and +X is left when facing baxter

        # with the drone facing baxter (flight controller pointing toward him),
        # this means +pitch -> +Z, -roll -> +X

        roll_midpoint = 1500
        pitch_midpoint = 1500
        yaw_midpoint = 1500
        thrust_midpoint = 1275


        uc_integrated_error_x = self.integrated_error_x + error.x
        self.integrated_error_x = clip(uc_integrated_error_x, -100, 100)

        uc_integrated_error_y = self.integrated_error_y + error.y
        self.integrated_error_y = clip(uc_integrated_error_y, -100, 100)

        uc_integrated_error_z = self.integrated_error_z + error.z
        self.integrated_error_z = clip(uc_integrated_error_z, -100, 100)

        if self.error_last is not None:
            d_err_x = error.x - self.error_last.x
            d_err_y = error.y - self.error_last.y
            d_err_z = error.z - self.error_last.z
        else:
            d_err_x = 0
            d_err_y = 0
            d_err_z = 0

        self.error_last = error



        kp_x = -10 # negative, because when (cmd - cur) < 0, we want to roll positively
        kp_y = 0 # 
        kp_z = 10


        ki_x = 0
        ki_y = 0
        ki_z = 0

        kd_x = 0
        kd_y = 0
        kd_z = 0

        # roll controls x (for now)
        roll_factor = kp_x*error.x + ki_x*self.integrated_error_x + kd_x*d_err_x

        # Pitch controls z (for now)
        pitch_factor = kp_z*error.z + ki_z*self.integrated_error_z + kd_z*d_err_z

        # Don't care about yaw (for now)
        yaw_factor = 0

        # Thrust controls height rate
        thrust_factor = kp_y*error.y + ki_y*self.integrated_error_y + kd_y*d_err_y

        cmd_r = roll_midpoint + int(clip(100*roll_factor, -100,100))
        cmd_p = pitch_midpoint + int(clip(100*pitch_factor, -100,100))
        cmd_yaw = yaw_midpoint + int(clip(100*yaw_factor, -100,100))
        cmd_t = thrust_midpoint + int(clip(100*thrust_factor, -100,100))

        return [cmd_r, cmd_p, cmd_yaw, cmd_t]
        # Throttle 1250 is roughly the midpoint *with no battery*
        #return [1500, 1500, 1500, 1320]


if __name__ == '__main__':

    rospy.init_node('state_controller')

    global last_heartbeat
    last_heartbeat = rospy.Time.now()

    global commanded_mode
    global commanded_position
    commanded_position = Position()

    global current_position 
    current_position = Position()
    global keep_going
    keep_going = True

    # Verbosity between 0 and 3, 3 is most verbose
    verbose = 3

    position_controller = PositionController()

    DISARMED = 0
    ARMED = 1
    FLYING = 2
    END = 9
    current_mode = DISARMED
    commanded_mode = DISARMED

    # This is a sub-state of flying. This is in case you want to have your
    # controller behave differently in different cases (e.g. takeoff vs steady
    # flight, or velocity vs. position hold, or linearized about different
    # orientations. 
    FLYING_MODE = 0


    # ROS Setup
    ###########
    
    # Subscribers
    #############
    rospy.Subscriber('vrpn_client_node/aaron_pidrone/pose', PoseStamped, current_position_callback_vrpn)
    rospy.Subscriber('/pidrone/heartbeat', String, heartbeat_callback)
    rospy.Subscriber('/pidrone/commanded_position', PoseStamped, commanded_position_callback)
    rospy.Subscriber('/pidrone/commanded_mode', Mode, commanded_mode_callback)

    # Publishers
    ############
    error_publisher = rospy.Publisher('/pidrone/err', axes_err, queue_size=1)

    # This should be changed to a message type that is *actually* a Mode type
    current_mode_publisher = rospy.Publisher('/pidrone/current_mode', Mode, queue_size=1)


    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, ctrl_c_handler)
    board = MultiWii('/dev/ttyUSB0')

    loop_rate = rospy.Rate(100)
    try:
        while keep_going:
            if current_mode == DISARMED:
                if commanded_mode == DISARMED:
                    # Self transition, do nothing
                    if verbose > 3:
                        print('DISARMED -> DISARMED')
                elif commanded_mode == ARMED:
                    arm(board)
                    current_mode = ARMED
                    if verbose >= 3:
                        print('DISARMED -> ARMED')
                elif commanded_mode == END:
                    # Time to end program
                    keep_going = False
                    current_mode = END
                    if verbose >= 3:
                        print('DISARMED -> END')
                else:
                    print('Cannot Transition from Mode %d to Mode %d' % (current_mode, commanded_mode) )
                    
            elif current_mode == ARMED:
                if commanded_mode == ARMED:
                    # Stay in this state, send the idle sequence to prevent
                    # disarming
                    idle_command = [1500, 1500, 1500, 1000]
                    board.sendCMD(8, MultiWii.SET_RAW_RC, idle_command)
                    if verbose > 3:
                        print('ARMED -> ARMED')
                elif commanded_mode == FLYING:
                    current_mode = FLYING
                    if verbose >= 3:
                        print('ARMED -> FLYING')
                elif commanded_mode == DISARMED:
                    disarm(board)
                    current_mode = DISARMED
                    if verbose >= 3:
                        print('FLYING -> DISARMED')
                elif commanded_mode == END:
                    keep_going = False
                    current_mode = END
                    if verbose >= 3:
                        print('FLYING -> END')
                else:
                    print('Cannot Transition from Mode %d to Mode %d' % (current_mode, commanded_mode) )
            elif current_mode == FLYING:
                if commanded_mode == FLYING:
                    # Send command to flight controller based on PID output
                    # at current timestep

                    # Error is a 'position' struct that has .x, .y, and .z fields
                    # giving x, y, and z errors
                    error = calc_error(current_position, commanded_position)
                    fly_commands = position_controller.step(error)
                    r,p,y,t = fly_commands
                    print('Fly Commands (r,p,y,t): %d, %d, %d, %d' % (r,p,y,t) )
                    board.sendCMD(8,MultiWii.SET_RAW_RC, fly_commands)
                    if verbose > 3:
                        print('FLYING -> FLYING')
                    print('Current error (x,y,z): %f, %f, %f' % (error.x, error.y, error.z))
                elif commanded_mode == DISARMED:
                    disarm(board)
                    current_mode = DISARMED
                    if verbose >= 3:
                        print('FLYING -> DISARMED')
                elif commanded_mode == END:
                    keep_going = False
                    current_mode = END
                    if verbose >= 3:
                        print('FLYING -> END')
            elif current_mode == END:
                # IT SHOULD *NOT* BE POSSIBLE TO REACH THIS STATE
                keep_going = False
                print('CURRENT MODE = END, but should not get here')

            mode_to_publish = Mode()
            mode_to_publish.mode = current_mode
            current_mode_publisher.publish(mode_to_publish)

            loop_rate.sleep()

        # end of main loop
        disarm(board)
    except Exception as ex:
        disarm(board)
        raise(ex)




