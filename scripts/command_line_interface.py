#!/usr/bin/python
import sys
import os
import rospy
import signal
from multiprocessing import Process
from pidrone_pkg.msg import Mode
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Pose, Twist


class CommandLineInterface(object):

    def ctrl_c_handler(self, signal, frame, desired_mode_pub):
        print('Got ctrl-c!')
        self.publish_desired_mode('DISARMED', desired_mode_pub)
        sys.exit(0)

    def publish_desired_pose(self, x, y, z, desired_pose_pub):
        ''' Publish the desired pose
        x, y, z : the desired position coordinates of the drone
        '''
        # create the pose message
        desired_pose_msg = Pose()
        # fill in the message fields:
        desired_pose_msg.position.x = x
        desired_pose_msg.position.y = y
        desired_pose_msg.position.z = z
        # publish desired pose
        desired_pose_pub.publish(desired_pose_msg)

    def publish_desired_twist(self, vx, vy, vz, desired_twist_pub):
        ''' Publish the desired twist
        vx, vy, vz : the desired velocities of the drone
        '''
        # create the twist message
        desired_twist_msg = Twist()
        # fill in the message fields:
        desired_twist_msg.linear.x = vx
        desired_twist_msg.linear.y = vy
        desired_twist_msg.linear.z = vz
        # publish desired twist
        desired_twist_pub.publish(desired_twist_msg)

    def publish_desired_mode(self, mode, desired_mode_pub):
        ''' Publish the desired Mode message
        mode : a String that is either 'ARMED', 'DISARMED', or 'FLYING'
        '''
        desired_mode_msg = Mode()
        desired_mode_msg.mode = mode
        desired_mode_pub.publish(desired_mode_msg)


def publish_heartbeat():
    rospy.init_node('command_line_interface_heartbeat')
    heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/command_line_interface', Empty, queue_size=1)
    while not rospy.is_shutdown():
        heartbeat_pub.publish(Empty())

def main():

    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # instantiate CommandLineInterface object
    cli = CommandLineInterface()

    # Publishers
    ############
    desired_mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
    desired_pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
    desired_twist_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
    reset_transform_pub = rospy.Publisher('pidrone/reset_transform', Empty, queue_size=1)
    position_control_pub = rospy.Publisher('pidrone/position_control', Bool, queue_size=1)
    map_pub = rospy.Publisher('/pidrone/map', Empty, queue_size=1)


    cli.publish_desired_mode('DISARMED', desired_mode_pub)
    signal.signal(signal.SIGINT, lambda x,y: cli.ctrl_c_handler(x,y,desired_mode_pub))
    print('Valid modes are DISARMED, ARMED, and FLYING')
    print('Alternatively, D, A, and F')
    print('To switch between position and velocity control, press \'p\'')
    print('p <x> <y> <z> will command the drone to go to position (x, y, z)')
    print('v <vx> <vy> <vz> will command the travel a set distnace at velocity (vx, vy, vz)')
    try:
        while not rospy.is_shutdown():
            raw_entry = raw_input('Type a mode and press enter:\t')
            entry = raw_entry.strip()
            # make the entry case insensitive
            entry = entry.lower()
            if len(entry) == 0:
                print 'invalid entry. try again.'
            else:
                # position commands take the form p <x> <y> <z> where x, y, and z are floats
                if entry[0] == 'r' and len(entry) == 1:
                    reset_transform_pub.publish(Empty())
                elif entry[0] == 'm' and len(entry) == 1:
                    map_pub.publish(Empty())

                # it's a position command
                elif entry[0] == 'p':
                    # turn on position controll
                    position_control_pub.publish(True)
                    if len(entry) > 1:
                    # set the desired position
                        strs = entry.split()
                        if len(strs) >= 4:
                            x = float(strs[1])
                            y = float(strs[2])
                            z = float(strs[3])
                            cli.publish_desired_pose(x, y, z, desired_pose_pub)

                # velocity commands take the form v <x> <y> <z> where x, y, and z are floats
                elif entry[0] == 'v':
                    position_control_pub.publish(False)
                    if len(entry) > 1:
                    # set the desired velocity
                        strs = entry.split()
                        if len(strs) >= 4:
                            vx = float(strs[1])
                            vy = float(strs[2])
                            vz = float(strs[3])
                            cli.publish_desired_twist(vx, vy, vz, desired_twist_pub)

                # mode commands take the form of a string or letter. possible
                # entries are:
                # disarm : 'DISARMED', 'd', 'D', 'x', 'X'
                # arm : 'ARMED', 'a', 'A'
                # fly : 'FLYING', 'f', 'F'
                else:
                    # It's a mode command
                    if 'x' in entry: # This is the 'panic' command
                        desired_mode = 'DISARMED'
                        cli.publish_desired_mode(desired_mode, desired_mode_pub)
                        break
                    elif (entry == 'DISARMED') or 'd' in entry:
                        desired_mode = 'DISARMED'
                        cli.publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'ARMED') or (entry == 'a'):
                        desired_mode = 'ARMED'
                        cli.publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'FLYING') or (entry == 'f'):
                        desired_mode = 'FLYING'
                        cli.publish_desired_mode(desired_mode, desired_mode_pub)
                    else:
                        print 'invalid command. try again'

    except Exception as ex:
        # Disarm drone on exception
        cli.publish_desired_mode('DISARMED', desired_mode_pub)
        raise

if __name__ == '__main__':
    heartbeat_pub_process = Process(target = publish_heartbeat)
    heartbeat_pub_process.start()
    main()
