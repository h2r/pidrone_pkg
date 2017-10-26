import rospy
from p3_pkg.msg import Mode

if __name__ == '__main__':
    rospy.init_node('terminal_mode_selector')
    modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)
    valid_modes = [0,1,3,4,5]
    mode_msg = Mode()
    print 'Valid modes are', valid_modes
    while not rospy.is_shutdown():
        raw_mode = raw_input('Type a mode number and press enter:\t')
        try:
            des_mode = int(raw_mode.strip())
            if des_mode in valid_modes:
                mode_msg.mode = des_mode
                print 'Sending mode {}!'.format(des_mode)
                modepub.publish(mode_msg)
            else:
                print 'Good job! You entered an integer but we dont support it... yet...'
        except Exception as e:
            print 'Bruh. {} is not an integer'.format(raw_mode)
