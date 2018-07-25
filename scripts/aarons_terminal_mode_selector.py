#!/usr/bin/python
import rospy
from pidrone_pkg.msg import Mode
from geometry_msgs.msg import PoseStamped
import signal
import sys

def ctrl_c_handler(signal, frame, modepub):
    print('Got ctrl-c!')
    mode_msg = Mode()
    mode_msg.mode = 0
    modepub.publish(mode_msg)
    sys.exit(0)



if __name__ == '__main__':
    rospy.init_node('aarons_terminal_mode_selector')
    modepub = rospy.Publisher('/pidrone/desired_mode', Mode, queue_size=1)
    commanded_position_pub = rospy.Publisher('/pidrone/desired_position', PoseStamped, queue_size=1)

    cmd_pos_msg = PoseStamped()
    cmd_pos_msg.pose.position.x = 0
    cmd_pos_msg.pose.position.y = 0
    cmd_pos_msg.pose.position.z = 0
    commanded_position_pub.publish(cmd_pos_msg)


    signal.signal(signal.SIGINT, lambda x,y: ctrl_c_handler(x,y,modepub))
    mode_msg = Mode()
    print('Valid modes are DISARMED, ARMED, and FLYING')
    print('Alternatively, D, A, and F')
    print('p <x> <y> <z> will command the drone to go to position (x, y, z)')
    try:
        while not rospy.is_shutdown():
            raw_mode = raw_input('Type a mode and press enter:\t')
            desired_mode = raw_mode.strip()

            # position commands take the form p <x> <y> <z> where x, y, and z are floats
            if desired_mode[0] == 'p':
                # It's a position command
                strs = desired_mode.split()
                if len(strs) >= 4:
                    x = float(strs[1])
                    y = float(strs[2])
                    z = float(strs[3])
                    cmd_pos_msg = PoseStamped()
                    cmd_pos_msg.pose.position.x = x
                    cmd_pos_msg.pose.position.y = y
                    cmd_pos_msg.pose.position.z = z
                    commanded_position_pub.publish(cmd_pos_msg)

            else:
                # It's a mode command
                if ('x' in desired_mode):
                    mode_msg.mode = 'DISARMED'
                elif (desired_mode == 'DISARMED') or (desired_mode == 'D'):
                    mode_msg.mode = 'DISARMED'
                elif (desired_mode == 'ARMED') or (desired_mode == 'A'):
                    mode_msg.mode = 'ARMED'
                elif (desired_mode == 'FLYING') or (desired_mode == 'F'):
                    mode_msg.mode = 'FLYING'
                elif (desired_mode == 'x') or (desired_mode == 'X'): # This is the 'panic' command
                    mode_msg.mode = 'DISARMED'
                    modepub.publish(mode_msg)
                    break
                else:
                    print('%s was not a valid mode' % desired_mode)

                modepub.publish(mode_msg)
    except Exception as ex:
        # Disarm drone on exception
        mode_msg.mode = 'DISARMED'
        modepub.publish(mode_msg)
        raise(ex)
