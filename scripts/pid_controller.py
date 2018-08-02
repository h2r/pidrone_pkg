#!/usr/bin/python
import tf
import sys
import rospy
import signal
import traceback
import numpy as np
import command_values as cmds
from pidrone_pkg.msg import Mode, RC
from old_pid_class import PID, PIDaxis
from std_msgs.msg import Float32, Empty
from three_dim_vec import Position, Velocity, Error
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped


class PIDController(object):
    ''' Runs a low-pass pre-filter on desired position or velocity and runs a
    PID controller on the error calculated by the desired position or velocity
    minus the current position or velocity
    '''

    def __init__(self):
        # Initialize the current and desired modes
        self.current_mode = Mode('DISARMED')
        self.desired_mode = Mode('DISARMED')

        self.position_control = False

        # store the most recent time stamp of the pose and twist messages
        self.last_pose_time = None
        self.twist_time = None

        # Initialize the current and desired positions and orientations
        self.current_position = Position()
        self.desired_position = Position()
        self.pose_delta_time = None
        self.yaw = 0.0
        self.current_calculated_z_velocity = 0.0

        # Initialize the position error
        self.position_error = Error()

        # Initialize the current and desired velocities
        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        # Initialize the velocity error
        self.velocity_error = Error()

        # Set the distance that a velocity command will move the drone (m)
        self.desired_velocity_travel_distance = 0.5
        self.desired_velocity_travel_time = 0.0
        self.desired_velocity_start_time = None

        # Initialize the primary PID
        self.pid = PID()

        # Initialize the 'position error to velocity error' PIDs:
        # left/right (roll) pid
        self.lr_pid = PIDaxis(0.0500, -0.00000, 0.000, midpoint=0, control_range=(-10.0, 10.0))
        # front/back (pitch) pid
        self.fb_pid = PIDaxis(-0.0500, 0.0000, -0.000, midpoint=0, control_range=(-10.0, 10.0))
        ### # up/down (throttle) pid
        ### self.ud_pid = PIDaxis(-0.0500, 0.0000, -0.000, midpoint=0, control_range=(-10.0, 10.0))

        # initialize variables for yaw velocity PI controller:
        # propotional constant
        self.kp_yaw = 100.0
        # integral constant
        self.ki_yaw = 0.1
        # pi value
        self.kpi_yaw = 20.0
        # max pi value
        self.kpi_max_yaw = 0.01
        # yaw accumulated integral
        self.iacc_yaw = 0.0

        # Store the command publisher
        self.cmdpub = None


    # ROS SUBSCRIBER CALLBACK METHODS
    #################################
    def current_pose_callback(self, msg):
        ''' Updates the drone's current position '''
        current_time = msg.header.stamp.to_sec()
        if self.last_pose_time is not None:
            self.pose_delta_time = current_time - self.last_pose_time
        self.last_pose_time = current_time
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _,_,self.yaw = tf.transformations.euler_from_quaternion(q)
        #print 'position before rotation:', self.current_position
        #self.world_to_body_frame_rotation()
        # I FORGOT ABOUT RADIANS! THIS IS PROBABLY WHY THE VALUES WERE SO OFF!
        #print 'position after rotation:', self.current_position
        # position control is possible because pose data is being published

    def desired_pose_callback(self, msg):
        ''' Updates the desired position '''
        self.desired_position.x = msg.pose.pose.position.x
        self.desired_position.y = msg.pose.pose.position.y
        self.desired_position.z = msg.pose.pose.position.z

    def current_twist_callback(self, msg):
        ''' Updates the drone's current velocity '''
        self.twist_time = msg.header.stamp
        self.current_velocity.x = msg.twist.twist.linear.x
        self.current_velocity.y = msg.twist.twist.linear.y
        self.current_velocity.z = msg.twist.twist.linear.z

    def desired_twist_callback(self, msg):
        ''' Update the desired velocity '''
        self.desired_velocity.x = msg.twist.twist.linear.x
        self.desired_velocity.y = msg.twist.twist.linear.y
        self.desirded_velocity.z = msg.twist.twist.linear.z
        self.calculate_travel_time()

    def current_mode_callback(self, msg):
        ''' Update the current mode '''
        self.current_mode = msg.mode

    def desired_mode_callback(self, msg):
        ''' Update the desired mode '''
        self.desired_mode = msg.mode

#TODO unify comments
    # subscribe /pidrone/toggle_transform
    def toggle_callback(self, msg):
        self.position_control = not self.position_control
        print "Position Control", self.position_control

    # subscribe /pidrone/reset_transform
    def reset_transform_callback(self, msg):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        print "Reset Position"

# TODO ADD YAW
    # Step Method
    #############
    def step(self):
        ''' Filters desired_state, then passes off to PID.
            Returns the commands generated by the pid
        '''
        if self.pose_delta_time is not None:
            self.calc_error()
            if self.desired_velocity.magnitude() > 0:
                self.adjust_desired_velocity()
            return self.pid.step(self.velocity_error)
        else:
            # send idle commands until pose_delta_time is defined
            return cmds.idle_cmd

    # HELPER METHODS
    ################
    def adjust_desired_velocity(self):
        ''' Set the desired velocity back to 0 once the drone has traveled the
        amount of time that causes it to move the specified desired velocity
        travel distance
        '''
        curr_time = rospy.get_time()
        if self.desired_velocity_start_time is not None:
            duration = curr_time - self.desired_velocity_start_time
            if duration > self.desired_velocity_travel_time:
                self.desired_velocity = Velocity(0, 0, 0)
                self.desired_velocity_start_time = None
        else:
            self.desired_velocity_start_time = curr_time

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

#     def calc_z_velocity(self):
#         ''' Caculate the velocity in the z direction from the z position values
#         since these are directly measured
#         '''
# # TODO try using z velocity from camera!
#         self.current_calculated_z_velocity = (self.z - self.previous_z)/(self.pose_delta_time)
    def calc_error(self):
        #TODO ADD COMMENT
        # calculate the velocity error in the x, y, z, and yaw directions
        dvx = self.desired_velocity.x - self.current_velocity.x
        dvy = - (self.desired_velocity.y - self.current_velocity.y)
        # calculate this differently because it is measured by the range sensor
        dz = self.desired_position.z - self.current_position.z
        # TODO dvyaw =
        self.velocity_error = Error(dvx,dvy,dz)
        if self.position_control:
            dx = self.desired_position.x - self.current_position.x
            dy = - (self.desired_position.y - self.current_position.y)
            dz = self.desired_position.z - self.current_position.z
            # TODO this is unused
            self.position_error = Position(dx, dy, dz)
            # calculate a value to add to the velocity error based based on the
            # position error in the x (roll) direction
            #lr_step = self.lr_pid.step(dx, self.pose_delta_time)
            # calculate a value to add to the velocity error based based on the
            # position error in the y (pitch) direction
            #fb_step = self.fb_pid.step(dy, self.pose_delta_time)
            # calculate a value to add to the velocity error based based on the
            # position error in the z (pitch) direction
            # ud_step = self.ud_pid.step(dz, self.pose_delta_time)

            # add the velocity values to their error corresponding components
            self.velocity_error.x #+= lr_step
            self.velocity_error.y #+= fb_step
            # self.velocity_error.z += ud_step


# TODO TEST THIS METHOD AND FIX COMMENT
    def reduce_magnitude(self, error):
        ''' Returns a vector with the same direction but with a reduced
        magnitude to enable small steps when a large error exists. Essentially
        moves the desried position or velocity closer to the current one '''
        error_array = np.array([error.x, error.y, error.z])
        magnitude = np.sqrt(error_array.dot(error_array))
        if magnitude > 0.05:
            error_array = (error_array / magnitude) * 0.05
        return Error(error_array[0], error_array[1], error_array[2])

    def calculate_travel_time(self):
        ''' return the amount of time that desired velocity should be used to
        calculate the error in order to move the drone the specified travel
        distance for a desired velocity
        '''
        return self.velocity_command_travel_distance/ self.desired_velocity.magnitude()

    def reset(self):
        ''' Set filtered_desired_position to be current position, set
        filtered_desired_velocity to be zero, and reset both the PositionPID
        and VelocityPID
        '''
# TODO test this
        # reset position control variables
        self.position_error = Error(0,0,0)
        self.desired_position = Position(self.current_position.x,self.current_position.y,0.3)
        # reset velocity control_variables
        self.velocity_error = Error(0,0,0)
        self.desired_velocity = Velocity(0,0,0)
        # reset the pid
        self.pid.reset()

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
    rospy.Subscriber("/pidrone/reset_transform", Empty, pid_controller.reset_transform_callback)
    rospy.Subscriber("/pidrone/toggle_transform", Empty, pid_controller.toggle_callback)


    # Non-ROS Setup
    ###############
    # set up ctrl-c handler
    signal.signal(signal.SIGINT, pid_controller.ctrl_c_handler)
    # set the loop rate (Hz)
    loop_rate = rospy.Rate(100)
    print 'PID Controller Started'
    while not rospy.is_shutdown():
        # Steps the PID. If we are not flying, this can be used to
        # examine the behavior of the PID based on published values
        fly_command = pid_controller.step()

        # reset the pids after arming
        if pid_controller.current_mode == 'DISARMED':
            if pid_controller.desired_mode == 'ARMED':
                pid_controller.reset()
        # if the drone is flying, send the fly_command
        elif pid_controller.current_mode == 'FLYING':
            if pid_controller.desired_mode == 'FLYING':
                # Publish the ouput of pid step method
                pid_controller.publish_cmd(fly_command)

        if verbose >= 2:
            if pid_controller.position_control:
                print 'current position:', pid_controller.current_position
                print 'desired position:', pid_controller.desired_position
                #print 'filtered position:', pid_controller.filtered_desired_position
                print 'position error:', pid_controller.position_error
            else:
                print 'current velocity:', pid_controller.current_velocity
                print 'desired velocity:', pid_controller.desired_velocity
        if verbose >= 1:
            print 'r,p,y,t:', fly_command

        loop_rate.sleep()
