from sensor_msgs.msg import Imu
from pidrone_pkg.msg import Battery, FC_command, Mode
from serial import SerialException
import rospkg
import yaml
import rospy
import tf
import numpy as np
import command_values as cmds
from h2rMultiWii import MultiWii
import sys
import signal


class FlightController(object):
    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/battery
    /pidrone/mode

    Subscribers:
    /pidrone/command
    """

    def getBoard(self):
        """Connect to the flight controller board"""
        # (if the flight cotroll usb is unplugged and plugged back in,
        #  it becomes .../USB1)
        try:
            board = MultiWii('/dev/ttyUSB0')
        except SerialException:
            board = MultiWii('/dev/ttyUSB1')
        return board

    def __init__(self, modepub):
        # Connect to the flight controller board
        self.board = self.getBoard()
        # Commands to send to the board
        self.command = cmds.disarm_cmd
        # publisher for mode
        self.modepub = modepub
        # stores the mode from the command message (initialize as disarmed)
        self.new_mode = "DISARMED"

    def command_callback(self, msg):
        """Store the command values from state_controller"""
        r = msg.ctrls.roll
        p = msg.ctrls.pitch
        y = msg.ctrls.yaw
        t = msg.ctrls.throttle
        self.command = [r,p,y,t]
        self.new_mode = msg.mode

    def update_mode(self):
        self.modepub.publish(self.new_mode)

    def ctrl_c_handler(self, signal, frame):
        """Disarms the drone and quits the flight controller node"""
        print "\nCaught ctrl-c! About to Disarm!"
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        self.board.receiveDataPacket()
        rospy.sleep(1)
        self.modepub.publish('DISARMED')
        print "Successfully Disarmed"
        sys.exit()

def main():

    # ROS Setup
    ###########
    rospy.init_node('flight_controller')

# XXX: what do I do about tcp_nodelay???????????
    # Publisher
    ###########
    print 'Publishing:'
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1,
                             tcp_nodelay=False)
    print '/pidrone/imu'
    batpub = rospy.Publisher('/pidrone/battery', Battery, queue_size=1,
                             tcp_nodelay=False)
    print '/pidrone/battery'
    modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1,
                             tcp_nodelay=False)
    print '/pidrone/mode'

    # create the FlightController object
    fc = FlightController(modepub)

    # Subscriber
    ############
    rospy.Subscriber("/pidrone/command", FC_command, fc.command_callback)

    # Messages to Publish
    #####################
    imu_to_pub = Imu()
    bat_to_pub = Battery()

    # Accelerometer parameters
    ##########################
    rospack = rospkg.RosPack()
    path = rospack.get_path('pidrone_pkg')
    with open("%s/params/multiwii.yaml" % path) as f:
        means = yaml.load(f)
    accRawToMss = 9.8 / means["az"]
    accZeroX = means["ax"] * accRawToMss
    accZeroY = means["ay"] * accRawToMss
    accZeroZ = means["az"] * accRawToMss

    signal.signal(signal.SIGINT, fc.ctrl_c_handler)
    r = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():

        # Send current commands to flight controller board
        fc.board.sendCMD(8, MultiWii.SET_RAW_RC, fc.command)
        fc.board.receiveDataPacket()
        fc.update_mode()
        curr_time = rospy.Time.now()

        # Extract Imu Data for Publishing
        # extract roll, pitch, heading
        fc.board.getData(MultiWii.ATTITUDE)
        # extract vbat, amperage
        fc.board.getData(MultiWii.ANALOG)
        # extract lin_acc_x, lin_acc_y, lin_acc_z
        fc.board.getData(MultiWii.RAW_IMU)

        # Calculate values for Imu message
        roll = fc.board.attitude['angx']
        pitch = fc.board.attitude['angy']
        heading = fc.board.attitude['heading']
        lin_acc_x = fc.board.rawIMU['ax'] * accRawToMss - accZeroX
        lin_acc_y = fc.board.rawIMU['ay'] * accRawToMss - accZeroY
        lin_acc_z = fc.board.rawIMU['az'] * accRawToMss - accZeroZ
        quaternion = tf.transformations.quaternion_from_euler(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(heading))

        # Prepare Imu message
# XXX:  should the quaternion be normalized???????
        imu_to_pub.header.frame_id = "base"
        imu_to_pub.header.stamp = curr_time
        imu_to_pub.orientation.x = quaternion[0]
        imu_to_pub.orientation.y = quaternion[1]
        imu_to_pub.orientation.z = quaternion[2]
        imu_to_pub.orientation.w = quaternion[3]
        imu_to_pub.linear_acceleration.x = fc.board.rawIMU['ax'] * accRawToMss - accZeroX
        imu_to_pub.linear_acceleration.y = fc.board.rawIMU['ay'] * accRawToMss - accZeroY
        imu_to_pub.linear_acceleration.z = fc.board.rawIMU['az'] * accRawToMss - accZeroZ

        # Publish Imu message
        imupub.publish(imu_to_pub)

        # Prepare Battery message
        bat_to_pub.vbat = fc.board.analog['vbat'] * 0.10
        bat_to_pub.amperage = fc.board.analog['amperage']

        # Publish Battery message
        batpub.publish(bat_to_pub)

        r.sleep()

    print 'Shutdown received'
    fc.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
    fc.board.receiveDataPacket()

if __name__ == '__main__':
    main()


# DICTIONARY
############
# <variable_name>: <description> , <units>
# roll: roll, degrees
# pitch: pitch, degrees
# heading: yaw heading, degrees
# vbat: battery voltage, volts
# amperage: battery amperage, amps
# linn_acc_x: linear accelleration along x-axtis, m/s^2
# linn_acc_y: linear accelleration along y-axis, m/s^2
# linn_acc_z: linear accelleration along z-axis, m/s^2

# Note, there are no angular velocities to publish
