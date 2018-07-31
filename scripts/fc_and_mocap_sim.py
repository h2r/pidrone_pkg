#!/usr/bin/python
import rospy
from pidrone_pkg.msg import Mode, Battery
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('fake_fc')
    # extra e so its not interfeering
    current_position_pub = rospy.Publisher('vrpn_client_node/aarondrone/pose', PoseStamped, queue_size=1)

    curr_pos_msg = PoseStamped()
    curr_pos_msg.pose.position.x = 0
    curr_pos_msg.pose.position.y = 0
    curr_pos_msg.pose.position.z = 0
    current_position_pub.publish(curr_pos_msg)

    print('Valid modes are DISARMED, ARMED, and FLYING')
    print('Alternatively, D, A, and F')
    print('p <x> <y> <z> will command set the current drone position (x, y, z)')
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
                curr_pos_msg = PoseStamped()
                curr_pos_msg.pose.position.x = x
                curr_pos_msg.pose.position.y = y
                curr_pos_msg.pose.position.z = z
                current_position_pub.publish(curr_pos_msg)

                print 'published'
