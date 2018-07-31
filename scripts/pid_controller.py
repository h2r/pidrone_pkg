#!/usr/bin/python
import tf
import sys
import rospy
import signal
import traceback
import numpy as np
from std_msgs.msg import Float32
from pidrone_pkg.msg import Mode, RC
from pid_class import PositionPID, VelocityPID
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

# TODO: figure out safe way to exit program
class ThreeDimVec(object):
    ''' Struct to store x,y,z'''
    # This should be turned into a data class with the next python 3 release
    def __init__(self, x=0, y=0 ,z=0):
        self.x = x
        self.y = y
        self.z = z
    def __str__(self):
        return "x: %f, y: %f, z: %f" % (self.x, self.y, self.z)

    def __mul__(self, other):
        return ThreeDimVec(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __add__(self, other):
        return ThreeDimVec(self.x + other.x, self.y + other.y, self.z + other.z)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return ThreeDimVec(self.x - other.x, self.y - other.y, self.z - other.z)

class Position(ThreeDimVec):
    ''' Struct to store postion x,y,z '''
    def __init__(self, x=0, y=0, z=0):
        super(Position, self).__init__()

class Velocity(ThreeDimVec):
    ''' Struct to store velocity x,y,z '''
    def __init__(self, vx=0, vy=0, vz=0):
        super(Velocity, self).__init__()


class PIDController(object):
    ''' Runs a low-pass pre-filter on desired position or velocity and runs a
    PID controller on the error calculated by the desired position or velocity
    minus the current position or velocity
    '''

    def __init__(self):
        # Initialize the current and desired modes
        self.current_mode = Mode('DISARMED')
        self.desired_mode = Mode('DISARMED')

        # Initialize the control_mode which is a substate of FLYING that
        # determines if the pid is controlling position or velocity. Can be
        # either 'POSITION' or 'VELOCITY'
# TODO is it safe to have this start as velocity or should it be none and then check elsewhere?
        self.control_mode = 'VELOCITY'
        self.previous_control_mode = 'VELOCITY'

        # Initialize the possible control modes to False. These become True only
        # if the associated data is being published
        self.position_control_is_possible = False
        self.velocity_control_is_possible = False

        # Initialize the current and desired positions and orientations
        self.current_position = Position()
        self.desired_position = Position()
        self.filtered_desired_position = Position()
        self.current_orientation = None

        # Initialize the Position PID
        self.position_pid = PositionPID()

        # Initialize the current and desired velocities
        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()
        self.filtered_desired_velocity = Velocity()

        # Initialize the Velocity PID
        self.velocity_pid = VelocityPID()

        # Store the command publisher
        self.cmdpub = None


    # ROS SUBSCRIBER CALLBACK METHODS
    #################################
    def current_pose_callback(self, msg):
        ''' Updates the drone's current position based on the state estimator '''
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        self.current_orientation_quaternion = (msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w)
        #print 'position before rotation:', self.current_position
        #self.world_to_body_frame_rotation()
        #print 'position after rotation:', self.current_position
        # position control is possible because pose data is being published
        self.position_control_is_possible = True

    def desired_pose_callback(self, msg):
        ''' Updates the desired position and control mode '''
        if self.position_control_is_possible:
            self.previous_control_mode = self.control_mode
            self.control_mode = 'POSITION'
            self.desired_position.x = msg.pose.pose.position.x
            self.desired_position.y = msg.pose.pose.position.y
            self.desired_position.z = msg.pose.pose.position.z
            diff_vec = self.desired_position.__sub__(self.current_position)
        else:
           print 'Cannot control position because pose data is not being published'

    def current_twist_callback(self, msg):
        ''' Updates the drone's current velocity based on the state estimator '''
        self.current_velocity.x = msg.twist.twist.linear.x
        self.current_velocity.y = msg.twist.twist.linear.y
        self.current_velocity.z = msg.twist.twist.linear.z
        # velocity control is possible because pose data is being published
        self.velocity_control_is_possible = True

    def desired_twist_callback(self, msg):
        ''' Update the desired velocity and control mode '''
        if self.velocity_control_is_possible:
            self.previous_control_mode = self.control_mode
            self.control_mode = 'VELOCITY'
            self.desired_velocity.x = msg.twist.twist.linear.x
            self.desired_velocity.y = msg.twist.twist.linear.y
            self.desired_velocity.z = msg.twist.twist.linear.z
        else:
            print 'Cannot control velocity because pose data is not being published'

    def current_mode_callback(self, msg):
        ''' Update the current mode '''
        self.current_mode = msg.mode

    def desired_mode_callback(self, msg):
        ''' Update the desired mode '''
        self.desired_mode = msg.mode


    # Step Method
    #############
    def step(self):
        ''' Filters desired_state, then passes off to PID.
            Returns the commands generated by the pid
        '''
        if self.control_mode == 'POSITION':
            # The desired position, after filtering
            self.filtered_desired_position = self.prefilter()
        elif self.control_mode == 'VELOCITY':
            # The desired velocity, after filtering
            self.filtered_desired_velocity = self.prefilter()
        else:
            print 'Internal control_mode error.'
            print 'Stopping controller.'
            sys.exit()

        error = self.calc_error()
        cmds = self.position_pid.step(error)
        return cmds

    # HELPER METHODS
    ################
    def world_to_body_frame_rotation(self):
# TODO uncomment and test:
        current_position_matrix = np.matrix([self.current_position.x,
                                            self.current_position.y,
                                            self.current_position.z])

        r,p,y = tf.transformations.euler_from_quaternion(self.current_orientation_quaternion)
        print 'r,p,y:',r,p,y
        cr,sr,cp,sp,cy,sy = np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
# TODO THIS COULD BE THE TRASNPOSE
        rotation_matrix = np.matrix(
        [   [cy*cp,      -sy*cr + cy*sp*sr,      sy*sr + cy*sp*cr],
            [sy*cp,      cy*cr + sy*sp*sr,       -cy*sr + sy*sp*cr],
            [-sp,        cp*sr,                  cp*cr]
        ])
        # rotation_matrix = rotation_matrix.transpose()

        rotated_current_position = current_position_matrix.dot(rotation_matrix)
        self.current_position = Position(rotated_current_position[0,0], rotated_current_position[0,1], rotated_current_position[0,2])


    def prefilter(self):
        ''' Runs a low-pass filter on the desired position or velocity '''

        # the proportional constant of the filtered_desired_position from the previous step
        c_fdp = .999
        # the proportional constant of the filtered_desired_velocity from the previous step
        c_fdv = .999
        # the proportional constant of the desired_position
        c_dp = .00349
        # the proportional constant of the desired_velocity
        c_dv = .00349

        if self.control_mode == 'POSITION':
            next_filt = c_fdp*self.filtered_desired_position + c_dp*(self.desired_position + self.current_position)
            self.filtered_desired_position = next_filt
        elif self.control_mode == 'VELOCITY':
            next_filt = c_fdv*self.filtered_desired_velocity + c_dv*(self.desired_velocity + self.current_velocity)
            self.filtered_desired_velocity = next_filt
        else:
            print 'Internal control_mode error'
            print 'Internal control_mode error.'
            print 'Stopping controller.'
            sys.exit()

        self.filtered_z_pub.publish(next_filt.z)
        self.des_z_pub.publish(self.desired_position.z)
        # return the newly filtered value
        return next_filt

    def calc_error(self):
        ''' Returns desired_(position or velocity) - current_(position or velocity)

        current_(position or velocity) and desired_(position or velocity) are
        both Position() objects that have .x, .y, and .z fields
        '''
        if self.control_mode == 'POSITION':
            dx = self.desired_position.x - self.current_position.x
            dy = self.desired_position.y - self.current_position.y
            dz = self.desired_position.z - self.current_position.z
        elif self.control_mode == 'VELOCITY':
            dx = self.desired_velocity.x - self.current_velocity.x
            dy = self.desired_velocity.y - self.current_velocity.y
            dz = self.desired_velocity.z - self.current_velocity.z
        else:
            print 'Internal control_mode error'
            print 'Internal control_mode error.'
            print 'Stopping controller.'
            sys.exit()

# TODO CHECK THIS MAG VALUE
        # If the magnitude of the error is greater than 0.05 m, then scale down
        # the error to prevent extreme accelerations
        error = np.array([dx, dy, dz])
        magnitude = np.sqrt(error.dot(error))
        #if magnitude > 0.05:
        #    error = (error / magnitude) * 0.05
        return ThreeDimVec(error[0], error[1], error[2])

    def reset(self):
        ''' Set filtered_desired_position to be current position, set
        filtered_desired_velocity to be zero, and reset both the PositionPID
        and VelocityPID
        '''
        #self.filtered_desired_position = self.current_position
# TODO test this
        self.filtered_desired_velocity = Velocity(0,0,0)
        self.position_pid.reset()
        self.velocity_pid.reset()

    def ctrl_c_handler(self, signal, frame):
        ''' Gracefully handles ctrl-c '''
        print 'Caught ctrl-c\n Stopping Controller'
        sys.exit()

    def publish_cmd(self, cmd):
        """Publish the controls to /pidrone/controller"""
        msg = RC()
        msg.roll = cmd[0]
        msg.pitch = cmd[1]
        msg.yaw = cmd[2]
        msg.throttle = cmd[3]
        self.cmdpub.publish(msg)


if __name__ == '__main__':

    # Verbosity between 0 and 2, 2 is most verbose
    verbose = 2

    # create the PIDController object
    pid_controller = PIDController()


    # ROS Setup
    ###########
    rospy.init_node('pid_controller')

    # Publishers
    ############
    pid_controller.cmdpub = rospy.Publisher('/pidrone/fly_commands', RC, queue_size=1)
    pid_controller.filtered_z_pub = rospy.Publisher('/pidrone/filtered_z', Float32, queue_size=1)
    pid_controller.des_z_pub = rospy.Publisher('/pidrone/des_z', Float32, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber('/pidrone/pose', PoseWithCovarianceStamped, pid_controller.current_pose_callback)
    rospy.Subscriber('/pidrone/twist', TwistWithCovarianceStamped, pid_controller.current_twist_callback)
    rospy.Subscriber('/pidrone/desired_pose', PoseWithCovarianceStamped, pid_controller.desired_pose_callback)
    rospy.Subscriber('/pidrone/desired_twist', TwistWithCovarianceStamped, pid_controller.desired_twist_callback)
    rospy.Subscriber('/pidrone/mode', Mode, pid_controller.current_mode_callback)
    rospy.Subscriber('/pidrone/desired_mode', Mode, pid_controller.desired_mode_callback)


    # Non-ROS Setup
    ###############
    # set up ctrl-c handler
    signal.signal(signal.SIGINT, pid_controller.ctrl_c_handler)
    # set the loop rate (Hz)
    loop_rate = rospy.Rate(100)
    print 'PID Controller Started'
    while not rospy.is_shutdown():
        # if the flying mode changes, reset the pids
        if pid_controller.control_mode != pid_controller.previous_control_mode:
            pid_controller.previous_control_mode = pid_controller.control_mode
            print 'Controlling:', pid_controller.control_mode
            pid_controller.reset()
        # Steps the PID. If we are not flying, this can be used to
        # examine the behavior of the PID based on published values
        fly_command = pid_controller.step()

        # reset the pids before takeoff
        if pid_controller.current_mode == 'ARMED':
            if pid_controller.desired_mode == 'FLYING':
                pid_controller.reset()
        # if the drone is flying, send the fly_command
        elif pid_controller.current_mode == 'FLYING':
            if pid_controller.desired_mode == 'FLYING':
                # Publish the ouput of pid step method
                pid_controller.publish_cmd(fly_command)

        if verbose >= 2:
            if pid_controller.control_mode == 'POSITION':
                print 'current position:', pid_controller.current_position
                print 'desired position:', pid_controller.desired_position
            else:
                print 'current velocity:', pid_controller.current_velocity
                print 'desired velocity:', pid_controller.desired_velocity
        if verbose >= 1:
            print 'r,p,y,t:', fly_command

        loop_rate.sleep()
