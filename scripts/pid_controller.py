#!/usr/bin/env python

import tf
import sys
import os
import rospy
import signal
import traceback
import numpy as np
import command_values as cmds
from pid_class import PID, PIDaxis
from geometry_msgs.msg import Pose, Twist
from pidrone_pkg.msg import Mode, RC, State
from std_msgs.msg import Float32, Empty, Bool
from three_dim_vec import Position, Velocity, Error, RPY

# frequency of inner loop velocity controller
INNER_LOOP_FREQUENCY = 100  
INNER_LOOP_PERIOD = 1.0 / INNER_LOOP_FREQUENCY
# frequency of outer loop position controller
OUTER_LOOP_FREQUENCY = 30
OUTER_LOOP_PERIOD = 1.0 / OUTER_LOOP_FREQUENCY

class PIDController(object):
    ''' Controls the flight of the drone by running a PID controller on the
    error calculated by the desired and current velocity and position of the drone
    '''

    def __init__(self):
        # Initialize the current and desired modes
        self.current_mode = Mode('DISARMED')
        self.desired_mode = Mode('DISARMED')

        # Initialize in velocity control
        self.position_control = False
        self.last_position_control = False

        # Initialize the current and desired positions
        self.current_position = Pose().position
        self.desired_position = Pose().position
        self.desired_position.z = 0.3

        # Initialize the current and desired velocities
        self.current_velocity = Twist()
        self.desired_velocity = Twist()

        # Initialize the velocity error
        self.prev_velocity_error = Twist()

        # Initialize the primary PID
        self.pid = PID(period=INNER_LOOP_PERIOD)

        # Initialize the position PIDs:
        # x axis (roll) pid
        self.x_position_pid = PIDaxis(kp=0.15, ki=0.05, kd=0.0, midpoint=0, control_range=(-0.05, 0.05))
        self.x_velocity_from_position = 0
        # y axis (pitch) pid
        self.y_position_pid = PIDaxis(kp=0.15, ki=0.05, kd=0.0, midpoint=0, control_range=(-0.05, 0.05))
        self.y_velocity_from_position = 0
        # timer for outer loop
        self._outer_loop_timer = 0.0

        # Store the command publisher
        self.cmdpub = None



    # ROS SUBSCRIBER CALLBACK METHODS
    #################################
    def current_state_callback(self, state):
        """ Store the drone's current state for calculations """
        # store the positions
        pose = state.pose_with_covariance.pose
        self.current_position.x = pose.position.x
        self.current_position.y = pose.position.y
        self.current_position.z = pose.position.z
        # store the linear velocities
        twist = state.twist_with_covariance.twist
        self.current_velocity.linear.x = twist.linear.x
        self.current_velocity.linear.y = twist.linear.y
        self.current_velocity.linear.z = twist.linear.z

    def absolute_desired_pose_callback(self, msg):
        """ Update desired pose to message values """
        self.desired_position.x = msg.position.x
        self.desired_position.y = msg.position.y
        self.desired_position.z = msg.position.z

    def relative_desired_pose_callback(self, msg):
        """ Update desired pose relative to current values """
        self.desired_position.x = self.current_position.x + msg.position.x
        self.desired_position.y = self.current_position.y + msg.position.y
        self.desired_position.z = self.current_position.z + msg.position.z

    def desired_twist_callback(self, msg):
        """ Update the desired twist """
        self.desired_velocity.linear.x = msg.linear.x
        self.desired_velocity.linear.y = msg.linear.y
        self.desired_velocity.linear.z = msg.linear.z
        self.desired_velocity.angular.z = msg.angular.z # yaw velocity

    def current_mode_callback(self, msg):
        """ Update the current mode """
        self.current_mode = msg.mode

    def desired_mode_callback(self, msg):
        """ Update the desired mode """
        self.desired_mode = msg.mode

    def position_control_callback(self, msg):
        """ Set whether or not position control is enabled """
        self.position_control = msg.data
        if self.position_control:
            self.desired_position.x = self.current_position.x
            self.desired_position.y = self.current_position.y
            # raise the drone since estimateRigidTransform works better when camera is further from surface
            self.desired_position.z = 0.2
        if (self.position_control != self.last_position_control):
            print("Position Control" if self.position_control else "Velocity Control")
            self.last_position_control = self.position_control

    def reset_callback(self, empty):
        """ Reset the desired and current poses of the drone and set
        desired velocities to zero """
        self.desired_position.x = self.current_position.x
        self.desired_position.y = self.current_position.y
        self.desired_velocity.linear.x = 0.0
        self.desired_velocity.linear.y = 0.0

    # Step Method
    #############
    def step(self):
        """ Returns the commands generated by the pid """
        velocity_error_x = self.desired_velocity.linear.x - self.current_velocity.linear.x
        velocity_error_y = self.desired_velocity.linear.y - self.current_velocity.linear.y
        velocity_error_yaw = self.desired_velocity.angular.z
        altitude_error = self.desired_position.z - self.current_position.z
        if self.position_control:
            if self._outer_loop_timer > OUTER_LOOP_PERIOD:
                self._outer_loop_timer = 0.0
                position_error_x = self.desired_position.x - self.current_position.x
                position_error_y = self.desired_position.y - self.current_position.y
                self.x_velocity_from_position = self.x_position_pid.step(position_error_x, OUTER_LOOP_PERIOD)
                self.y_velocity_from_position = self.y_position_pid.step(position_error_y, OUTER_LOOP_PERIOD)
            self._outer_loop_timer += INNER_LOOP_PERIOD
            velocity_error_x += self.x_velocity_from_position
            velocity_error_y += self.y_velocity_from_position
        return self.pid.step(velocity_error_x, velocity_error_y, velocity_error_yaw, altitude_error)

    # HELPER METHODS
    ################
    def reset(self):
        ''' Set desired_position to be current position, set
        filtered_desired_velocity to be zero, and reset both the PositionPID
        and VelocityPID
        '''
        # reset position control variables
        self.desired_position.x = 0.0
        self.desired_position.y = 0.0
        self.current_position.x = 0.0
        self.current_position.y = 0.0
        self.x_velocity_from_position = 0
        self.y_velocity_from_position = 0
        # reset velocity control_variables
        self.prev_velocity_error = Error(0,0,0)
        self.desired_velocity.linear.x = 0.0
        self.desired_velocity.linear.y = 0.0
        self.desired_velocity.angular.z = 0.0
        # reset altitude setpoint
        self.desired_position.z = 0.3
        # reset the pids
        self.pid.reset()
        self.x_position_pid.reset()
        self.y_position_pid.reset()

    def ctrl_c_handler(self, signal, frame):
        """ Gracefully handles ctrl-c """
        print('Caught ctrl-c\n Stopping Controller')
        sys.exit()

    def publish_cmd(self, cmd):
        """Publish the controls to /pidrone/fly_commands """
        msg = RC()
        msg.roll = cmd[0]
        msg.pitch = cmd[1]
        msg.yaw = cmd[2]
        msg.throttle = cmd[3]
        self.cmdpub.publish(msg)


def main(ControllerClass):

    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # create the PIDController object
    pid_controller = ControllerClass()

    # Publishers
    ############
    pid_controller.cmdpub = rospy.Publisher('/pidrone/fly_commands', RC, queue_size=1)
    pid_controller.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
    pid_controller.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/pid_controller', Empty, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber('/pidrone/state', State, pid_controller.current_state_callback)
    rospy.Subscriber('/pidrone/desired/pose/absolute', Pose, pid_controller.absolute_desired_pose_callback)
    rospy.Subscriber('/pidrone/desired/pose/relative', Pose, pid_controller.relative_desired_pose_callback)
    rospy.Subscriber('/pidrone/desired/twist', Twist, pid_controller.desired_twist_callback)
    rospy.Subscriber('/pidrone/mode', Mode, pid_controller.current_mode_callback)
    rospy.Subscriber('/pidrone/desired/mode', Mode, pid_controller.desired_mode_callback)
    rospy.Subscriber('/pidrone/position_control', Bool, pid_controller.position_control_callback)
    rospy.Subscriber('/pidrone/reset_transform', Empty, pid_controller.reset_callback)

    # Non-ROS Setup
    ###############
    # set up ctrl-c handler
    signal.signal(signal.SIGINT, pid_controller.ctrl_c_handler)
    # set the loop rate (Hz)
    loop_rate = rospy.Rate(INNER_LOOP_FREQUENCY)
    print('PID Controller Started')
    while not rospy.is_shutdown():
        fly_command = cmds.disarm_cmd
        pid_controller.heartbeat_pub.publish(Empty())
        if pid_controller.current_mode == 'ARMED':
            pid_controller.position_control_pub.publish(False)
        if pid_controller.current_mode == 'FLYING' and pid_controller.desired_mode == 'FLYING':
            fly_command = pid_controller.step()
        else:
            pid_controller.reset()
        pid_controller.publish_cmd(fly_command)
        loop_rate.sleep()


if __name__ == '__main__':
    main(PIDController)
    