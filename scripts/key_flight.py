#!/usr/bin/env python
import rospy
from pidrone_pkg.msg import RC
from getch import getch

rospy.init_node('key_flight', anonymous=True)

pub = rospy.Publisher('/pidrone/commands', RC, queue_size=10)
msg = RC()
msg.roll = 1500
msg.pitch = 1500
msg.yaw = 1500
msg.throttle = 1000
msg.aux1 = 1500
msg.aux2 = 1500
msg.aux3 = 1500
msg.aux4 = 1500
stop = False
while not rospy.is_shutdown() and not stop:
    print([msg.roll, msg.pitch, msg.yaw, msg.throttle])
    pub.publish(msg)
    key = getch()
    if key == 'w':
        if msg.roll <= 1900:
            msg.roll += 100
    elif key == 's':
        if msg.roll >= 1100:
            msg.roll -= 100
    elif key == 'a':
        if msg.pitch >= 1100:
            msg.pitch -= 100
    elif key == 'd':
        if msg.pitch <= 1900:
            msg.pitch += 100
    elif key == 'q':
        if msg.yaw >= 1100:
            msg.yaw -= 100
    elif key == 'e':
        if msg.yaw <= 1900:
            msg.yaw += 100
    elif key == 'u':
        if msg.throttle <= 1900:
            msg.throttle += 100
    elif key == 'j':
        if msg.throttle >= 1100:
            msg.throttle -= 100
    elif ord(key) == 3:
        print("Recieved ctrl-c... Exiting")
        stop = True
    else:
        print("Could not read key")
