import rospy
from pidrone_pkg.msg import Mode
from sensor_msgs.msg import Joy

scalar = 5.0
mode = Mode()
mode.mode = 4
modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)

def joy_callback(data):
    global scalar
    global modepub
    global mode
    if data.buttons[4] == 1:
        mode.mode = 4
    elif data.buttons[5] == 1:
        mode.mode = 0
    elif data.buttons[7] == 1:
        mode.mode = 5
        mode.x_velocity = data.axes[3] * scalar
        mode.y_velocity = data.axes[2] * scalar
        mode.z_velocity = data.axes[1]
    print mode
    modepub.publish(mode)


if __name__ == "__main__":
    rospy.init_node("velocity_joy_node")
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()
