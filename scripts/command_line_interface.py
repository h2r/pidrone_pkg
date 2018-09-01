#!/usr/bin/python
import sys
import os
import rospy
import signal
from three_dim_vec import Position
from multiprocessing import Process
from std_msgs.msg import Empty, Bool
from pidrone_pkg.msg import Mode, State
from geometry_msgs.msg import Pose, Twist



class CommandLineInterface(object):
    """ A text based interface to control the drone from the terminal """

    def __init__(self):
        self.position_control = False

    def ctrl_c_handler(self, signal, frame):
        print('Got ctrl-c!')
        self.publish_desired_mode('DISARMED')
        sys.exit(0)

    def publish_desired_pose(self, x=0, y=0, z=0):
        ''' Publish the desired pose
        x, y, z : the desired position of the drone relative to its current
                  position
        '''
        # create the pose message
        desired_pose_msg = Pose()
        # fill in the message fields:
        desired_pose_msg.position.x = x
        desired_pose_msg.position.y = y
        desired_pose_msg.position.z = z
        # publish desired pose
        self.desired_pose_pub.publish(desired_pose_msg)

    def publish_desired_twist(self, vx=0, vy=0, vz=0, az=0):
        ''' Publish the desired twist
        vx, vy, vz : the desired linear velocities of the drone
        az : the desired angular velocity of the drone about the z axis (yaw velocity)
        '''
        # create the twist message
        desired_twist_msg = Twist()
        # fill in the message fields:
        desired_twist_msg.linear.x = vx
        desired_twist_msg.linear.y = vy
        desired_twist_msg.linear.z = vz
        desired_twist_msg.angular.z = az
        # publish desired twist
        self.desired_twist_pub.publish(desired_twist_msg)

    def publish_desired_mode(self, mode):
        ''' Publish the desired Mode message
        mode : a String that is either 'ARMED', 'DISARMED', or 'FLYING'
        '''
        desired_mode_msg = Mode()
        desired_mode_msg.mode = mode
        self.desired_mode_pub.publish(desired_mode_msg)

    # Keyboard control methods:
    ###########################
    # planar translation in velocity control
    def velocity_move_forward(self):
        self.publish_desired_twist(vy=1)

    def velocity_move_backward(self):
        self.publish_desired_twist(vy=-1)

    def velocity_move_right(self):
        self.publish_desired_twist(vx=1)

    def velocity_move_left(self):
        self.publish_desired_twist(vx=-1)

    # planar translation in position control
    def position_move_forward(self):
        self.publish_desired_pose(y=0.1)

    def position_move_backward(self):
        self.publish_desired_pose(y=-0.1)

    def position_move_right(self):
        self.publish_desired_pose(x=0.1)

    def position_move_left(self):
        self.publish_desired_pose(x=-0.1)

    # vertical translation in both velocity and position control
    def move_up(self):
        self.publish_desired_pose(z=0.05)
    def move_down(self):
        self.publish_desired_pose(z=-0.05)

    # yaw velocity in both velocity and position control
    def yaw_left(self):
        self.publish_desired_twist(az=-100)
    def yaw_right(self):
        self.publish_desired_twist(az=100)


    # ROS Subscriber callback methods:
    ##################################
    def position_control_callback(self, msg):
        self.position_control = msg.data

def publish_heartbeat():
    """ Publish the heartbeat of the command line interface """
    rospy.init_node('command_line_interface_heartbeat')
    heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/command_line_interface', Empty, queue_size=1)
    publish_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        heartbeat_pub.publish(Empty())
        publish_rate.sleep()

def main():

    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # instantiate CommandLineInterface object
    cli = CommandLineInterface()

    # Publishers
    ############
    cli.desired_mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
    cli.desired_pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
    cli.desired_twist_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
    cli.reset_transform_pub = rospy.Publisher('pidrone/reset_transform', Empty, queue_size=1)
    cli.position_control_pub = rospy.Publisher('pidrone/position_control', Bool, queue_size=1)
    cli.map_pub = rospy.Publisher('/pidrone/map', Empty, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber('/pidrone/position_control', Bool, cli.position_control_callback)


    cli.publish_desired_mode('DISARMED')
    signal.signal(signal.SIGINT, lambda x,y: cli.ctrl_c_handler(x,y))
    print('Welcome to the command line interface!')
    print('The commands are as follows:')
    print('; : arm')
    print('t : takeoff')
    print('spacebar : disarm')
    print('i : forward')
    print('k : backward')
    print('j : left')
    print('l : right')
    print('w : up')
    print('s : down')
    print('a : yaw left')
    print('d : yaw right')
    print('r : reset position')
    print('p : enable position control')
    print('v : enable velocity control')

    try:
        while not rospy.is_shutdown():
            raw_entry = raw_input('Type a mode and press enter:\t')
            # (spacebar is pressed)
            if raw_entry == " ":
                desired_mode = 'DISARMED'
                cli.publish_desired_mode(desired_mode)
            else:
                # parse the entry
                entry = raw_entry.strip()
                entry = entry.lower()
                if len(entry) == 0:
                    print 'invalid entry. try again.'
                elif len(entry) == 1:
                    input = entry[0]
                    if 'x' in entry: # This is the 'panic' command
                        desired_mode = 'DISARMED'
                        cli.publish_desired_mode(desired_mode)
                        break
                    elif (input == ';'):
                        desired_mode = 'ARMED'
                        cli.publish_desired_mode(desired_mode)
                    elif (input == 't'):
                        desired_mode = 'FLYING'
                        cli.publish_desired_mode(desired_mode)
                    elif input == 'r':
                        cli.reset_transform_pub.publish(Empty())
                    elif input == 'm':
                        cli.map_pub.publish(Empty())
                    elif input == 'i':
                        if cli.position_control:
                            cli.position_move_forward()
                        else:
                            cli.velocity_move_forward()
                    elif input == 'k':
                        if cli.position_control:
                            cli.position_move_backward()
                        else:
                            cli.velocity_move_backward()
                    elif input == 'j':
                        if cli.position_control:
                            cli.position_move_left()
                        else:
                            cli.velocity_move_left()
                    elif input == 'l':
                        if cli.position_control:
                            cli.position_move_right()
                        else:
                            cli.velocity_move_right()
                    elif input == 'w':
                        cli.move_up()
                    elif input == 's':
                        cli.move_down()
                    elif input == 'a':
                        cli.yaw_left()
                    elif input == 'd':
                        cli.yaw_right()
                    elif input == 'p':
                        cli.position_control_pub.publish(True)
                    elif input == 'v':
                        cli.position_control_pub.publish(False)

                else:
                    # position commands take the form p <x> <y> <z> where x, y, and z are floats
                    # it's a position command
                    if entry[0] == 'p':
                        # turn on position control
                        cli.position_control_pub.publish(True)
                        # set the desired position
                        strs = entry.split()
                        if len(strs) >= 4:
                            x = float(strs[1])
                            y = float(strs[2])
                            z = float(strs[3])
                            cli.publish_desired_pose(x, y, z)

                    # velocity commands take the form v <x> <y> <z> where x, y, and z are floats
                    elif entry[0] == 'v':
                        cli.position_control_pub.publish(False)
                        if len(entry) > 1:
                        # set the desired velocity
                            strs = entry.split()
                            if len(strs) >= 4:
                                vx = float(strs[1])
                                vy = float(strs[2])
                                vz = float(strs[3])
                                cli.publish_desired_twist(vx, vy, vz)
                    else:
                        print 'invalid command. try again'

        cli.publish_desired_mode('DISARMED')
        print 'loop broken due to panic command (x), sending disarm'

    except Exception as ex:
        # Disarm drone on exception
        cli.publish_desired_mode('DISARMED')
        raise

if __name__ == '__main__':
    heartbeat_pub_process = Process(target = publish_heartbeat)
    heartbeat_pub_process.start()
    main()
