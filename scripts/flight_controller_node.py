#!/usr/bin/env python

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
from std_msgs.msg import Header, Empty
from pidrone_pkg.msg import Battery, Mode, RC, State
import os

from fault_protector import FaultProtector


class FlightController(object):
    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/battery
    /pidrone/mode

    Subscribers:
    /pidrone/fly_commands
    /pidrone/desired/mode
    /pidrone/heartbeat/web_interface
    /pidrone/infrared
    /pidrone/state

    INFO: The drone has three modes - 'DISAMRED' 'ARMED' and 'FLYING'
          The valid transitions are:

                DISARMED <---> ARMED ---> FLYING
                    ^                        |
                    |                        |
                    --------------------------


    datasheet for imu on Naze32 Acro: http://invensense.tdk.com/wp-content/uploads/2020/06/PS-MPU-6500A-01-v1.3.pdf
    """

    def __init__(self):
        # Connect to the flight controller board
        self.board = self.get_board()
        # stores the current and previous modes
        self.curr_mode = 'DISARMED'  # initialize as disarmed
        self.prev_mode = 'DISARMED'  # initialize as disarmed
        # store the command to send to the flight controller
        self.command = cmds.disarm_cmd  # initialize as disarmed
        # store the mode publisher
        self.modepub = None
        # store the time for angular velocity calculations
        self.time = rospy.Time.now()

        # Initialize the Imu Message
        ############################
        header = Header()
        header.frame_id = 'Body'
        header.stamp = rospy.Time.now()

        self.imu_message = Imu()
        self.imu_message.header = header

        # Initialize the Battery Message
        ################################
        self.battery_message = Battery()
        self.battery_message.vbat = None
        self.battery_message.amperage = None
        # 3s battery safe discharge is 10.5V (3.5V per cell)
        # but the flight control reads a little lower than actual
        # minimum is 9V (3V per cell)
        self.minimum_voltage = 10

        # Accelerometer parameters
        ##########################
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/multiwii.yaml" % path) as f:
            means = yaml.load(f)
        # 'means' are the average accelerometer readings when the drone is stable
        # create a scalar from 1g to the accelerometer reading for the z-axis
        # since we know that this reading corresponds with accel due to gravity (1g)
        accel_due_to_gravity = 9.8  # m/s^2
        self.accRawToMss = accel_due_to_gravity / means["az"]  # m/s^2/raw
        self.accZeroX = means["ax"] * self.accRawToMss  # raw * m/s^2/raw = m/s^2
        self.accZeroY = means["ay"] * self.accRawToMss  # raw * m/s^2/raw = m/s^2
        self.accZeroZ = means["az"] * self.accRawToMss  # raw * m/s^2/raw = m/s^2
        # Gyroscope parameters
        ######################
        # rad/s/raw (scalars to convert raw gyro to rad per second)
        self.XGyroRawToRs = 0.0010569610567923715  # rad/s/raw
        self.YGyroRawToRs = 0.0010533920049110032  # rad/s/raw
        self.ZGyroRawToRs = 0.0010644278634753999  # rad/s/raw
        self.gyroZeroX = means["gx"] * self.XGyroRawToRs  # raw * rad/s/raw = rad/s
        self.gyroZeroY = means["gy"] * self.YGyroRawToRs  # raw * rad/s/raw = rad/s
        self.gyroZeroZ = means["gz"] * self.ZGyroRawToRs  # raw * rad/s/raw = rad/s
        # Safety
        self.shutdown_reason = ""

    # ROS subscriber callback methods:
    ##################################
    def desired_mode_callback(self, msg):
        """ Set the current mode to the desired mode if the tranistion is valid
        """
        # FLYING to ARMED is not a valid transition
        if (self.curr_mode == 'FLYING') and (msg.mode == 'ARMED'):
            print("INFO: Cannot transition from FLYING to ARMED")
        else:
            self.prev_mode = self.curr_mode
            self.curr_mode = msg.mode
            self.update_command()

    def fly_commands_callback(self, msg):
        """ Store and send the flight commands if the current mode is FLYING """
        if self.curr_mode == 'FLYING':
            r = msg.roll
            p = msg.pitch
            y = msg.yaw
            t = msg.throttle
            self.command = [r, p, y, t]

    # Update methods:
    #################
    def update_imu_message(self):
        """
        Compute the ROS IMU message by reading data from the board.
        """

        # Extract roll, pitch, heading
        self.board.getData(MultiWii.ATTITUDE)
        # Extract lin_acc_x, lin_acc_y, lin_acc_z
        self.board.getData(MultiWii.RAW_IMU)

        # Calculate values to update imu_message:
        roll = np.deg2rad(self.board.attitude['angx'])
        pitch = -np.deg2rad(self.board.attitude['angy'])
        heading = np.deg2rad(self.board.attitude['heading'])
        # Note that at pitch angles near 90 degrees, the roll angle reading can
        # fluctuate a lot
        # Transform heading (yaw) to standard math conventions, which
        # means angles are in radians and positive rotation is CCW
        heading = (-heading) % (2 * np.pi)
        # When first powered up, heading should read near 0
        # get the previous roll, pitch, heading values
        previous_quaternion = self.imu_message.orientation
        quaternion_array = [previous_quaternion.x, previous_quaternion.y, previous_quaternion.z, previous_quaternion.w]
        previous_roll, previous_pitch, previous_heading = tf.transformations.euler_from_quaternion(quaternion_array)

        # Although quaternion_from_euler takes a heading in range [0, 2pi),
        # euler_from_quaternion returns a heading in range [0, pi] or [0, -pi).
        # Thus need to convert the returned heading back into the range [0, 2pi).
        previous_heading = previous_heading % (2 * np.pi)

        # transform euler angles into quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        # Calculate the linear accelerations
        lin_acc_x = self.board.rawIMU['ax'] * self.accRawToMss - self.accZeroX
        lin_acc_y = self.board.rawIMU['ay'] * self.accRawToMss - self.accZeroY
        lin_acc_z = self.board.rawIMU['az'] * self.accRawToMss - self.accZeroZ
        # Calculate the rotational rates
        ang_vel_x = self.board.rawIMU['gx'] * self.XGyroRawToRs - self.gyroZeroX
        ang_vel_y = self.board.rawIMU['gy'] * self.YGyroRawToRs - self.gyroZeroY
        ang_vel_z = self.board.rawIMU['gz'] * self.ZGyroRawToRs - self.gyroZeroZ

        # Rotate the IMU frame to align with our convention for the drone's body
        # frame. IMU accelerometer: x is forward, y is left, z is up. We want: x
        # is right, y is forward, z is up.
        # ACC
        lin_acc_x_drone_body = -lin_acc_y
        lin_acc_y_drone_body = lin_acc_x
        lin_acc_z_drone_body = lin_acc_z
        # GYRO
        ang_vel_x_drone_body = ang_vel_x
        ang_vel_y_drone_body = -ang_vel_y
        ang_vel_z_drone_body = ang_vel_z

        # Account for gravity's affect on linear acceleration values when roll
        # and pitch are nonzero. When the drone is pitched at 90 degrees, for
        # example, the z acceleration reads out as -9.8 m/s^2. This makes sense,
        # since our calibration variable, accZeroZ, zeros the body-frame z-axis
        # acceleration to 0 when the drone is level. However, when it's pitched
        # 90 degrees, the body-frame z-axis is perpendicular to the force of
        # gravity the IMU reads -9.8 m/s^2 along the z-axis.
        g = 9.8
        lin_acc_x_drone_body = lin_acc_x_drone_body + g * np.sin(roll) * np.cos(pitch)
        lin_acc_y_drone_body = lin_acc_y_drone_body + g * np.cos(roll) * (-np.sin(pitch))
        lin_acc_z_drone_body = lin_acc_z_drone_body + g * (1 - np.cos(roll) * np.cos(pitch))

        time = rospy.Time.now()
        self.time = time

        # Update the imu_message:
        # header stamp
        self.imu_message.header.stamp = time
        # orientation
        self.imu_message.orientation.x = quaternion[0]
        self.imu_message.orientation.y = quaternion[1]
        self.imu_message.orientation.z = quaternion[2]
        self.imu_message.orientation.w = quaternion[3]
        # angular velocities
        self.imu_message.angular_velocity.x = ang_vel_x_drone_body
        self.imu_message.angular_velocity.y = ang_vel_y_drone_body
        self.imu_message.angular_velocity.z = ang_vel_z_drone_body
        # linear accelerations
        self.imu_message.linear_acceleration.x = lin_acc_x_drone_body
        self.imu_message.linear_acceleration.y = lin_acc_y_drone_body
        self.imu_message.linear_acceleration.z = lin_acc_z_drone_body

    def update_battery_message(self):
        """
        Compute the ROS battery message by reading data from the board.
        """
        # extract vbat, amperage
        self.board.getData(MultiWii.ANALOG)
        # Update Battery message:
        self.battery_message.vbat = self.board.analog['vbat'] * 0.10
        self.battery_message.amperage = self.board.analog['amperage']

    def update_command(self):
        """ Set command values if the mode is ARMED or DISARMED """
        if self.curr_mode == 'DISARMED':
            self.command = cmds.disarm_cmd
        elif self.curr_mode == 'ARMED':
            if self.prev_mode == 'DISARMED':
                self.command = cmds.arm_cmd
            elif self.prev_mode == 'ARMED':
                self.command = cmds.idle_cmd
            else:
                self.command = cmds.disarm_cmd
                print("INFO: Invalid mode transition from FLYING to ARMED")

    # Helper Methods:
    #################
    def get_board(self):
        """ Connect to the flight controller board """
        # (if the flight control usb is unplugged and plugged back in,
        #  it becomes .../USB1)
        try:
            board = MultiWii('/dev/ttyUSB0')
        except SerialException:
            try:
                board = MultiWii('/dev/ttyUSB1')
            except SerialException:
                print('\nCannot connect to the flight controller board.'
                      'The USB is unplugged. Please check connection.')
                sys.exit()
        return board

    def send_cmd(self):
        """ Send commands to the flight controller board """
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, self.command)
        self.board.receiveDataPacket()

    def ctrl_c_handler(self, signal, frame):
        """ Disarm the drone and quits the flight controller node """
        print("\nCaught ctrl-c! About to Disarm!")
        self.desired_mode_callback(Mode('DISARMED'))
        rospy.sleep(1)
        sys.exit()


def main():
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # create the FlightController and FaultProtector objects
    fc = FlightController()
    fp = FaultProtector()


    # Publishers
    ###########
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    batpub = rospy.Publisher('/pidrone/battery', Battery, queue_size=1, tcp_nodelay=False)
    fc.modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1, tcp_nodelay=False)
    print('Publishing:'
          '\n/pidrone/imu'
          '\n/pidrone/mode'
          '\n/pidrone/battery')

    # Subscribers
    ############
    rospy.Subscriber("/pidrone/desired/mode", Mode, fc.desired_mode_callback)
    rospy.Subscriber('/pidrone/fly_commands', RC, fc.fly_commands_callback)

    signal.signal(signal.SIGINT, fc.ctrl_c_handler)
    # set the loop rate (Hz)
    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            # if the current mode is anything other than disarmed
            # preform as safety check
            if fc.curr_mode != 'DISARMED':
                # Break the loop if a safety check has failed
                if fp.should_i_shutdown(mode=fc.curr_mode,
                                        prev_mode=fc.prev_mode,
                                        battery_voltage=fc.battery_message.vbat,
                                        imu_msg=fc.imu_message):
                    fc.shutdown_reason += fp.get_shutdown_cause()
                    break
            # update and publish flight controller readings
            fc.update_battery_message()
            fc.update_imu_message()
            imupub.publish(fc.imu_message)
            batpub.publish(fc.battery_message)

            # update and send the flight commands to the board
            fc.update_command()
            fc.send_cmd()

            # publish the current mode of the drone
            fc.modepub.publish(fc.curr_mode)

            # sleep for the remainder of the loop time
            rate.sleep()

    except SerialException:
        fc.shutdown_reason += ('\nERROR: Cannot connect to the flight controller board.'
                              '\nThe USB is unplugged. Please check connection.')
    except Exception:
        fc.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        fc.board.receiveDataPacket()
        rospy.sleep(1)
        raise

    finally:
        print(fc.shutdown_reason)
        print('INFO: Sending DISARM command')
        fc.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        fc.board.receiveDataPacket()


if __name__ == '__main__':
    main()
