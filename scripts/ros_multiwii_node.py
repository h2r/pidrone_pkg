#!/usr/bin/env python
import rospy
from pyMultiwii import MultiWii
from sys import stdout
from geometry_msgs.msg import Quaternion, Pose
from pidrone_pkg.msg import RC
import sys
import tf

board = MultiWii("/dev/ttyACM0")

cmds = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

def att_pub():
    pub = rospy.Publisher('/pidrone/orientation', Quaternion, queue_size=1)
    rate = rospy.Rate(300)
    pose = Pose()
    while not rospy.is_shutdown():
        board.sendCMDrecieveATT(16, MultiWii.SET_RAW_RC, cmds)


        # message = "angx = {:+.2f} \t angy = {:+.2f} \t heading = {:+.2f} \t elapsed = {:+.4f} \t".format(float(board.attitude['angx']),float(board.attitude['angy']),float(board.attitude['heading']),float(board.attitude['elapsed']))
        # stdout.write("\r%s" % message )
        # stdout.flush()
        # End of fancy printing

        roll = board.attitude['angx']
        pitch = board.attitude['angy']
        yaw = board.attitude['heading']
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        pub.publish(pose.orientation)
        rate.sleep()

def cmd_call(data):
    print([data.roll, data.pitch, data.yaw, data.throttle, data.aux4])
    if data.aux4 > 1500:
        board.arm()
    elif data.aux4 <= 1400:
        board.disarm()
    cmds[0] = data.roll
    cmds[1] = data.pitch
    cmds[2] = data.yaw
    cmds[3] = data.throttle
    cmds[4] = data.aux1
    cmds[5] = data.aux2
    cmds[6] = data.aux3
    cmds[7] = data.aux4


if __name__ == "__main__":
    rospy.init_node('multiwii', anonymous=True)
    try:
        rospy.Subscriber("/pidrone/commands", RC, cmd_call)
        att_pub()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except Exception,error:
        print "Error on Main: "+str(error)

