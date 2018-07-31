#!/usr/bin/python
import tf
import sys
import yaml
import rospy
import rospkg
import signal
import numpy as np
import command_values as cmds
from sensor_msgs.msg import Imu
from h2rMultiWii import MultiWii
from serial import SerialException
from pidrone_pkg.msg import Battery, Mode, RC



class FlightController(object):
    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/battery
    /pidrone/mode

    Subscribers:
    /pidrone/commanded_mode
    /pidrone/fly_commands
    """

    def __init__(self):
        # Connect to the flight controller board
        self.board = self.getBoard()
        # stores the current and previous modes
        self.curr_mode = 'DISARMED'         #initialize as disarmed
        self.prev_mode = 'DISARMED'         #initialize as disarmed
        # store the command to send to the flight controller
        self.command = cmds.disarm_cmd      #initialize as disarmed
        # store the mode publisher
        self.modepub = None

    def getBoard(self):
        """Connect to the flight controller board"""
        # (if the flight cotroll usb is unplugged and plugged back in,
        #  it becomes .../USB1)
        try:
            board = MultiWii('/dev/ttyUSB0')
        except SerialException:
            board = MultiWii('/dev/ttyUSB1')
        return board
# TODO I MAY NEED TO KEEP PUBLISHING THE IDLE COMMAND
    def commanded_mode_callback(self, msg):
        ''' Set the current mode to the commanded mode
        '''
        self.prev_mode = self.curr_mode
        self.curr_mode = msg.mode
        self.update_command()

    def update_command(self):
        ''' Set command values if the mode is ARMED or DISARMED '''
        if self.curr_mode == 'DISARMED':
            self.command = cmds.disarm_cmd
        elif self.curr_mode == 'ARMED':
            if self.prev_mode == 'DISARMED':
                self.command = cmds.arm_cmd
            elif self.prev_mode == 'ARMED':
                self.command = cmds.idle_cmd

    def fly_commands_callback(self, msg):
        ''' Store and send the flight commands if the current mode is FLYING '''
        if self.curr_mode == 'FLYING':
            r = msg.roll
            p = msg.pitch
            y = msg.yaw
            t = msg.throttle
            self.command = [r,p,y,t]

#TODO CHECK IF I NEED SLEEPS HERE
    def send_cmd(self):
        ''' Send commands to the flight controller board '''
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, self.command)
        self.board.receiveDataPacket()
        print 'command sent:', self.command

    def ctrl_c_handler(self, signal, frame):
        ''' Disarm the drone and quits the flight controller node '''
        print "\nCaught ctrl-c! About to Disarm!"
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        self.board.receiveDataPacket()
        rospy.sleep(1)
        self.modepub.publish('DISARMED')
        print "Successfully Disarmed"
        sys.exit()

# TODO THERE MAY BE A HUGE COLLISION BY CALLING GETDATA AND SENDCMD AT THE SAME TIME!
def main():

    # create the FlightController object
    fc = FlightController()

    # ROS Setup
    ###########
    rospy.init_node('flight_controller')

    # Publisher
    ###########
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    batpub = rospy.Publisher('/pidrone/battery', Battery, queue_size=1, tcp_nodelay=False)
    fc.modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1, tcp_nodelay=False)
    print 'Publishing:'
    print '/pidrone/imu'
    print '/pidrone/battery'
    print '/pidrone/mode'

    # Subscriber
    ############
    rospy.Subscriber('/pidrone/commanded_mode', Mode, fc.commanded_mode_callback)
    rospy.Subscriber('/pidrone/fly_commands', RC, fc.fly_commands_callback)

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

        # Send the current command
        fc.send_cmd()
        # publish the current mode of the drone
        fc.modepub.publish(fc.curr_mode)

        # Extract Data for Publishing:
        ##############################
        # extract roll, pitch, heading
        fc.board.getData(MultiWii.ATTITUDE)
        # extract vbat, amperage
        fc.board.getData(MultiWii.ANALOG)
        # extract lin_acc_x, lin_acc_y, lin_acc_z
        fc.board.getData(MultiWii.RAW_IMU)

        # Imu:
        ######
        # Calculate values for Imu message:
        roll = np.deg2rad(fc.board.attitude['angx'])
        pitch = np.deg2rad(fc.board.attitude['angy'])
        heading = np.deg2rad(fc.board.attitude['heading'])
        # transform heading (similar to yaw) to standard math conventions, which
        # means angles are in radians and positive rotation is to the left
        heading = ((np.pi / 2) - heading) % (2 * np.pi)
        lin_acc_x = fc.board.rawIMU['ax'] * accRawToMss - accZeroX
        lin_acc_y = fc.board.rawIMU['ay'] * accRawToMss - accZeroY
        lin_acc_z = fc.board.rawIMU['az'] * accRawToMss - accZeroZ
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        # Prepare Imu message:
# XXX:  should the quaternion be normalized???????
        imu_to_pub.header.frame_id = "body"
        imu_to_pub.header.stamp = rospy.Time.now()
        imu_to_pub.orientation.x = quaternion[0]
        imu_to_pub.orientation.y = quaternion[1]
        imu_to_pub.orientation.z = quaternion[2]
        imu_to_pub.orientation.w = quaternion[3]
        imu_to_pub.linear_acceleration.x = fc.board.rawIMU['ax'] * accRawToMss - accZeroX
        imu_to_pub.linear_acceleration.y = fc.board.rawIMU['ay'] * accRawToMss - accZeroY
        imu_to_pub.linear_acceleration.z = fc.board.rawIMU['az'] * accRawToMss - accZeroZ
        # Publish Imu message
        imupub.publish(imu_to_pub)

        # Battery:
        ##########
        # Prepare Battery message:
        bat_to_pub.vbat = fc.board.analog['vbat'] * 0.10
        bat_to_pub.amperage = fc.board.analog['amperage']
        # Publish Battery message
        batpub.publish(bat_to_pub)

        # sleep for the remainder of the loop time
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
# linn_acc_x: linear accelleration along x-axis, m/s^2
# linn_acc_y: linear accelleration along y-axis, m/s^2
# linn_acc_z: linear accelleration along z-axis, m/s^2

# Note, there are no angular velocities to publish
