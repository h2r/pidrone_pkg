#!/usr/bin/env python
import rospy
from pidrone_pkg.msg import RC

rospy.init_node('test_pid', anonymous=True)

pub = rospy.Publisher('/pidrone/commands', RC, queue_size=1)
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
    print(msg)
    pub.publish(msg)
