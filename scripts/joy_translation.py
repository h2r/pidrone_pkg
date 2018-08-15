import rospy
from pidrone_pkg.msg import Mode
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
import numpy as np
import os

z_total_steps = 24
#z_counter = (z_total_steps / 4) - 1
z_counter = -1
z_step = 5 # cm
scalar = 15
mode = Mode()
mode.mode = 4
modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)
resetpub = rospy.Publisher('/pidrone/reset_transform', Empty, queue_size=1)
togglepub = rospy.Publisher('/pidrone/toggle_transform', Empty, queue_size=1)

def joy_callback(data):
    global scalar
    global modepub
    global mode
    global resetpub
    global z_counter
    global z_step
    global z_total_steps
    print "callback"
    if data.buttons[3] == 1:
        z_counter = (z_counter+1) % z_total_steps
        print 3, "Z Stepping", z_counter
        mode.mode = 5
        mode.x_velocity = 0
        mode.y_velocity = 0
        if z_counter > ((z_total_steps / 2) - 1):
            mode.z_velocity = -z_step
        else:
            mode.z_velocity = z_step
        print "mode", mode
        modepub.publish(mode)
    if data.buttons[7] == 1:
        print "button", 7
        print "mode", mode
    if data.buttons[8] == 1:
        print "botton", 8
        print "mode", mode
    if data.buttons[9] == 1:
        print "button", 9
        print "mode", mode
    if data.buttons[10] == 1:
        print "button", 10
        print "mode", mode

    if data.buttons[4] == 1:
        # disarm
        mode.mode = 4
        print "mode", mode
        modepub.publish(mode)
    elif data.buttons[6] == 1:
        # land
        mode.mode = 3
        print "mode", mode
        modepub.publish(mode)
    elif data.buttons[5] == 1:
        # arm
        mode.mode = 0
        print "mode", mode
        modepub.publish(mode)
    elif data.buttons[7] == 1:
        mode.mode = 5
        mode.x_velocity = -data.axes[2] * scalar
        mode.y_velocity = data.axes[3] * scalar
        mode.z_velocity = data.axes[1]
        mode.yaw_velocity = -200.0 * data.axes[0]
        print "mode", mode
        modepub.publish(mode)
    elif data.buttons[0] == 1:
        # takeoff
        mode.mode = 2
        print "mode", mode
        modepub.publish(mode)

    # should be able to do these at the same time
    if data.buttons[1] == 1:
        print "resetting transform"
        resetpub.publish(Empty())
    if data.buttons[2] == 1:
        print "toggling transform"
        togglepub.publish(Empty())
    
#   if data.buttons[7] == 0:
#       mode.x_velocity = 0
#       mode.y_velocity = 0
#       mode.z_velocity = 0

def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()
    
    
if __name__ == "__main__":
    main()
