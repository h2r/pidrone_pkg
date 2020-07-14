#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import signal
import command_values as cmds
import yaml
from pid_class import DronePID
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist
from pidrone_pkg.msg import Mode, RC, State
from std_msgs.msg import Float32, Empty, Bool

FREQUENCY = 60  # Controller frequency in Hz
FREQUENCY_DIVISOR = 4  # inner loop runs at FREQUENCY Hz, while outer loop run FREQUENCY_DIVISOR times slower


class PIDController(object):
    ''' Controls the flight of the drone by running a PID controller on the
    error calculated by the desired and current velocity and position of the drone
    '''

    def __init__(self):
        self.current_mode = 'DISARMED'
        self.desired_mode = 'DISARMED'
        self.position_control = False
        self.pid = DronePID(velocity_controller_frequency=FREQUENCY,
                            frequency_divisor=4)
        self.state = None
        self.imu_msg = None
        self.reset()
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('pidrone_pkg')

    # ROS SUBSCRIBER CALLBACK METHODS
    #################################
    def current_state_callback(self, msg):
        """ Store the drone's current state for calculations """
        self.state = msg

    def desired_pose_callback(self, msg):
        self.desired_pose = msg

    def desired_twist_callback(self, msg):
        self.desired_twist = msg

    def current_mode_callback(self, msg):
        """ Update the current mode """
        self.current_mode = msg.mode

    def desired_mode_callback(self, msg):
        """ Update the desired mode """
        self.desired_mode = msg.mode

    def position_control_callback(self, msg):
        """ Set whether or not position control is enabled """
        self.position_control = msg.data
        if msg.data == True:
            self.desired_pose = self.state.pose_with_covariance.pose

    def reset_position_callback(self, empty):
        self.desired_pose = self.state.pose_with_covariance.pose

    def imu_callback(self, msg):
        self.imu_msg = msg

    def trim_right_callback(self, msg):
        self.pid.roll_pid.velocity_pid.increment_k()

    def trim_left_callback(self, msg):
        self.pid.roll_pid.velocity_pid.decrement_k()

    def trim_forward_callback(self, msg):
        self.pid.pitch_pid.velocity_pid.increment_k()

    def trim_backward_callback(self, msg):
        self.pid.pitch_pid.velocity_pid.decrement_k()

    def trim_throttle_up_callback(self, msg):
        self.pid.throttle_pid.increment_k()

    def trim_throttle_down_callback(self, msg):
        self.pid.throttle_pid.decrement_k()

    def trim_yaw_ccw_callback(self, msg):
        self.pid.yaw_pid.velocity_pid.decrement_k()

    def trim_yaw_cw_callback(self, msg):
        self.pid.yaw_pid.velocity_pid.increment_k()

    def trim_save_callback(self, msg):
        with open("%s/params/pid.yaml" % self.path) as f:
            tuning_vals = yaml.load(f)
        tuning_vals["roll"]["velocity"]["k"] = self.pid.roll_pid.velocity_pid.get_k()
        tuning_vals["pitch"]["velocity"]["k"] = self.pid.pitch_pid.velocity_pid.get_k()
        tuning_vals["yaw"]["velocity"]["k"] = self.pid.yaw_pid.velocity_pid.get_k()
        tuning_vals["throttle"]["k"] = self.pid.throttle_pid.get_k()
        with open("%s/params/pid.yaml" % self.path, "w") as f:
            yaml.dump(tuning_vals, f)

    # Helper Methods
    ################
    def reset(self):
        self.position_control = False
        self.pid.reset()
        self.clear_desired_twist()
        self.clear_desired_pose()

    def clear_desired_twist(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.desired_twist = msg

    def clear_desired_pose(self):
        msg = Pose()
        msg.position.x = 0
        msg.position.y = 0
        msg.position.z = 0.3    # want the drone to fly at 0.3 m
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 0
        self.desired_pose = msg

    def ctrl_c_handler(self, signal, frame):
        """ Gracefully handles ctrl-c """
        print('\nCaught ctrl-c'
              '\n INFO: Stopping Controller'
              '\n INFO: Sending DISARM command')
        self.publish_cmd(cmds.disarm_cmd)
        rospy.sleep(1)
        sys.exit()

    def step(self):
        if self.state is None or self.imu_msg is None:
            return cmds.disarm_cmd
        else:
            return self.pid.step(setpoint_twist=self.desired_twist, setpoint_pose=self.desired_pose,
                                 state=self.state, imu=self.imu_msg)

    def publish_cmd(self, cmd):
        """Publish the controls to /pidrone/fly_commands """
        msg = RC()
        msg.roll = cmd[0]
        msg.pitch = cmd[1]
        msg.yaw = cmd[2]
        msg.throttle = cmd[3]
        self.cmdpub.publish(msg)


def main():
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # create the PIDController object
    pid_controller = PIDController()

    # Publishers
    ############
    pid_controller.cmdpub = rospy.Publisher('/pidrone/fly_commands', RC, queue_size=1)
    pid_controller.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/pid_controller', Empty, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber('/pidrone/state', State, pid_controller.current_state_callback)
    rospy.Subscriber('/pidrone/desired/pose', Pose, pid_controller.desired_pose_callback)
    rospy.Subscriber('/pidrone/desired/twist', Twist, pid_controller.desired_twist_callback)
    rospy.Subscriber('/pidrone/mode', Mode, pid_controller.current_mode_callback)
    rospy.Subscriber('/pidrone/desired/mode', Mode, pid_controller.desired_mode_callback)
    rospy.Subscriber('/pidrone/position_control', Bool, pid_controller.position_control_callback)
    rospy.Subscriber('/pidrone/reset_transform', Empty, pid_controller.reset_position_callback)
    rospy.Subscriber('/pidrone/imu', Imu, pid_controller.imu_callback)
    rospy.Subscriber('/pidrone/trim/right', Empty, pid_controller.trim_right_callback)
    rospy.Subscriber('/pidrone/trim/left', Empty, pid_controller.trim_left_callback)
    rospy.Subscriber('/pidrone/trim/front', Empty, pid_controller.trim_forward_callback)
    rospy.Subscriber('/pidrone/trim/back', Empty, pid_controller.trim_backward_callback)
    rospy.Subscriber('/pidrone/trim/throttle/up', Empty, pid_controller.trim_throttle_up_callback)
    rospy.Subscriber('/pidrone/trim/throttle/down', Empty, pid_controller.trim_throttle_down_callback)
    rospy.Subscriber('/pidrone/trim/save', Empty, pid_controller.trim_save_callback)
    rospy.Subscriber('/pidrone/trim/yaw/ccw', Empty, pid_controller.trim_yaw_ccw_callback)
    rospy.Subscriber('/pidrone/trim/yaw/cw', Empty, pid_controller.trim_yaw_cw_callback)

    # Non-ROS Setup
    ###############
    # set up ctrl-c handler
    signal.signal(signal.SIGINT, pid_controller.ctrl_c_handler)
    # set the loop rate (Hz)
    controller_frequency = rospy.Rate(FREQUENCY)
    print('PID Controller Started')
    while not rospy.is_shutdown():
        pid_controller.heartbeat_pub.publish(Empty())

        # if the drone is flying, send the fly_command
        if (pid_controller.current_mode == 'FLYING') and (pid_controller.desired_mode == 'FLYING'):
            # Publish the ouput of pid step method
            fly_command = pid_controller.step()
            pid_controller.publish_cmd(fly_command)
        else:
            pid_controller.reset()

        controller_frequency.sleep()


if __name__ == '__main__':
    main()
