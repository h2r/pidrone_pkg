#!/usr/bin/env python
import rospy
from pidrone_pkg.msg import RC

rospy.init_node('test_pub', anonymous=True)
pub = rospy.Publisher('/pidrone/commands', RC, queue_size=1)

rc = RC()
rc.roll = 1500
rc.pitch = 1500
rc.yaw = 1500
rc.throttle = 1500
rc.aux1 = 1500
rc.aux2 = 1500
rc.aux3 = 1500
rc.aux4 = 1500
while not rospy.is_shutdown():
    pub.publish(rc)
