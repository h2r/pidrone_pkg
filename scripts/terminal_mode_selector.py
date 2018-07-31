#!/usr/bin/python
import sys
import rospy
import signal
from pidrone_pkg.msg import Mode
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped


def ctrl_c_handler(signal, frame, desired_mode_pub):
    print('Got ctrl-c!')
    publish_desired_mode('DISARMED', desired_mode_pub)
    sys.exit(0)

def publish_desired_pose(x, y, z, desired_pose_pub):
    ''' Publish the desired pose
    x, y, z : the desired position coordinates of the drone
    '''
    # create the pose message
    desired_pose_msg = PoseWithCovarianceStamped()
    # fill in the message fields:
    desired_pose_msg.header.stamp = rospy.Time.now()
    desired_pose_msg.header.frame_id = 'World'
    desired_pose_msg.pose.pose.position.x = x
    desired_pose_msg.pose.pose.position.y = y
    desired_pose_msg.pose.pose.position.z = z
    # publish desired pose
    desired_pose_pub.publish(desired_pose_msg)

def publish_desired_twist(vx, vy, vz, desired_twist_pub):
    ''' Publish the desired twist
    vx, vy, vz : the desired velocities of the drone
    '''
    # create the twist message
    desired_twist_msg = TwistWithCovarianceStamped()
    # fill in the message fields:
    desired_twist_msg.header.stamp = rospy.Time.now()
    desired_twist_msg.header.frame_id = 'World'
    desired_twist_msg.twist.twist.linear.x = vx
    desired_twist_msg.twist.twist.linear.y = vy
    desired_twist_msg.twist.twist.linear.z = vz
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
    rospy.init_node('aarons_terminal_mode_selector')
    desired_mode_pub = rospy.Publisher('/pidrone/desired_mode', Mode, queue_size=1)
    desired_pose_pub = rospy.Publisher('/pidrone/desired_pose', PoseWithCovarianceStamped, queue_size=1)
    desired_twist_pub = rospy.Publisher('/pidrone/desired_twist', TwistWithCovarianceStamped, queue_size=1)
    publish_desired_mode('DISARMED', desired_mode_pub)
    signal.signal(signal.SIGINT, lambda x,y: ctrl_c_handler(x,y,desired_mode_pub))
    print('Valid modes are DISARMED, ARMED, and FLYING')
    print('Alternatively, D, A, and F')
    print('To switch between position and velocity control, use the following:')
    print('p <x> <y> <z> will command the drone to go to position (x, y, z)')
    print('v <vx> <vy> <vz> will command the drone to go to velocity (vx, vy, vz)')
    try:
        while not rospy.is_shutdown():
            raw_entry = raw_input('Type a mode and press enter:\t')
            entry = raw_entry.strip()
            if len(entry) == 0:
                print 'invalid entry. try again.'
            else:
                # position commands take the form p <x> <y> <z> where x, y, and z are floats
                if entry[0] == 'p':
                    # It's a position command
                    strs = entry.split()
                    if len(strs) >= 4:
                        x = float(strs[1])
                        y = float(strs[2])
                        z = float(strs[3])
                        publish_desired_pose(x, y, z, desired_pose_pub)

                # position commands take the form v <x> <y> <z> where x, y, and z are floats
                elif entry[0] == 'v':
                    # It's a velocity command
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
                    if 'x' in entry:
                        desired_mode = 'DISARMED'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'DISARMED') or (entry == 'D'):
                        desired_mode = 'DISARMED'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'ARMED') or (entry == 'A'):
                        desired_mode = 'ARMED'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'FLYING') or (entry == 'F'):
                        desired_mode = 'FLYING'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                    elif (entry == 'x') or (entry == 'X'): # This is the 'panic' command
                        desired_mode = 'DISARMED'
                        publish_desired_mode(desired_mode, desired_mode_pub)
                        break
                    else:
                        print 'invalid command. try again'

    except Exception as ex:
        # Disarm drone on exception
        publish_desired_mode('DISARMED', desired_mode_pub)
        raise(ex)
