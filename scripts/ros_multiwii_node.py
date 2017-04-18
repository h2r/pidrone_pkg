#!/usr/bin/env python
import rospy
from pyMultiwii import MultiWii
from sys import stdout
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from pidrone_pkg.msg import RC
import sys
import tf

board = MultiWii("/dev/ttyACM0")

cmds = [1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500]

def att_pub():
    global cmds
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1)
    rate = rospy.Rate(300)
    imu = Imu()
    board.arm()
    print("armed")
    while not rospy.is_shutdown():
        att_data = board.getData(MultiWii.ATTITUDE)
        print(att_data)
        imu_data = board.getData(MultiWii.RAW_IMU)
#       print cmds[0], cmds[1], cmds[2], cmds[3]
        board.sendCMD(16, MultiWii.SET_RAW_RC, cmds)


        # message = "angx = {:+.2f} \t angy = {:+.2f} \t heading = {:+.2f} \t elapsed = {:+.4f} \t".format(float(board.attitude['angx']),float(board.attitude['angy']),float(board.attitude['heading']),float(board.attitude['elapsed']))
        # stdout.write("\r%s" % message )
        # stdout.flush()
        # End of fancy printing

        roll = board.attitude['angx']
        pitch = board.attitude['angy']
        yaw = board.attitude['heading']
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        imu.orientation.x = quaternion[0]
        imu.orientation.y = quaternion[1]
        imu.orientation.z = quaternion[2]
        imu.orientation.w = quaternion[3]
        imu.angular_velocity.x = board.rawIMU['ax']
        imu.angular_velocity.y = board.rawIMU['ay']
        imu.angular_velocity.z = board.rawIMU['az']
        imupub.publish(imu)
    board.disarm()
    print("disarming")

def cmd_call(data):
    if data.aux4 > 1500:
        board.arm()
    elif data.aux4 <= 1400:
        board.disarm()
    global cmds
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
