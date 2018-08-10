#!/usr/bin/python
import sys
import rospy
import signal
from pidrone_pkg.msg import Mode
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Pose, Twist


def ctrl_c_handler(signal, frame, desired_mode_pub):
    print('Got ctrl-c!')
    publish_desired_mode('DISARMED', desired_mode_pub)
    sys.exit(0)

def publish_desired_pose(x, y, z, desired_pose_pub):
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

def publish_desired_twist(vx, vy, vz, desired_twist_pub):
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

def publish_desired_mode(mode, desired_mode_pub):
    ''' Publish the desired Mode message
    mode : a String that is either 'ARMED', 'DISARMED', or 'FLYING'
    '''
    desired_mode_msg = Mode()
    desired_mode_msg.mode = mode
    desired_mode_pub.publish(desired_mode_msg)

if __name__ == '__main__':
    # ROS Setup
    ###########
    rospy.init_node('aarons_terminal_mode_selector')

    # Publishers
    ############
    desired_mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
    desired_pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
    desired_twist_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
    reset_transform_pub = rospy.Publisher('pidrone/reset_transform', Empty, queue_size=1)
    toggle_transform_pub = rospy.Publisher('pidrone/toggle_transform', Bool, queue_size=1)
    position_control = False

    publish_desired_mode('DISARMED', desired_mode_pub)
    signal.signal(signal.SIGINT, lambda x,y: ctrl_c_handler(x,y,desired_mode_pub))
    print('Valid modes are DISARMED, ARMED, and FLYING')
    print('Alternatively, D, A, and F')
    print('To switch between position and velocity control, press \'p\'')
    print('p <x> <y> <z> will command the drone to go to position (x, y, z)')
    print('v <vx> <vy> <vz> will command the travel a set distnace at velocity (vx, vy, vz)')
    try:
        while not rospy.is_shutdown():
            raw_entry = raw_input('Type a mode and press enter:\t')
            entry = raw_entry.strip()
            entry = entry.lower()
            if len(entry) == 0:
                print 'invalid entry. try again.'
            else:
                # position commands take the form p <x> <y> <z> where x, y, and z are floats
                if entry[0] == 'r' and len(entry) == 1:
                    reset_transform_pub.publish(Empty())

                # it's a position command
                elif entry[0] == 'p':
                    # turn on position controll
                    postion_control = True
                    toggle_transform_pub.publish(position_control)
                    if len(entry) > 1:
                    # set the desired position
                        strs = entry.split()
                        if len(strs) >= 4:
                            x = float(strs[1])
                            y = float(strs[2])
                            z = float(strs[3])
                            publish_desired_pose(x, y, z, desired_pose_pub)

                # veloity commands take the form v <x> <y> <z> where x, y, and z are floats
                elif entry[0] == 'v':
                    position_control = False
                    toggle_transform_pub.publish(position_control)
                    if len(entry) > 1:
                    # set the desired velocity
                        strs = entry.split()
                        if len(strs) >= 4:
                            vx = float(strs[1])
                            vy = float(strs[2])
                            vz = float(strs[3])
                            publish_desired_twist(vx, vy, vz, desired_twist_pub)

                # mode commands take the form of a string or letter. possible
                # entries are:
                # disarm : 'DISARMED', 'D', 'x', 'X'
                # arm : 'ARMED', 'A'
                # fly : 'FLYING', 'F'
                else:
                    # It's a mode command
                    if 'x' in entry: # This is the 'panic' command
                        desired_mode = 'DISARMED'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                        break
                    elif (entry == 'DISARMED') or (entry == 'd'):
                        desired_mode = 'DISARMED'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'ARMED') or (entry == 'a'):
                        desired_mode = 'ARMED'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'FLYING') or (entry == 'f'):
                        desired_mode = 'FLYING'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                    else:
                        print 'invalid command. try again'

    except Exception as ex:
        # Disarm drone on exception
        publish_desired_mode('DISARMED', desired_mode_pub)
        raise
