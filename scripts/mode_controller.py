#!/usr/bin/python
import tf
import sys
import os
import rospy
import signal
import numpy as np
import command_values as cmds
from serial import SerialException
from std_msgs.msg import Empty
from pidrone_pkg.msg import Mode, Battery


class ModeController(object):
    ''' An object used to control the mode of the drone and ensure that mode
    transisitons are safe
    '''

    def __init__(self):

        # Desired, current, and previous modes of the drone
        self.desired_mode = 'DISARMED'
        self.curr_mode = 'DISARMED'
        self.prev_mode = 'DISARMED'

        # Battery values
        self.vbat = None
        self.amperage = None
        # Adjust this based on how low the battery should discharge
        self.minimum_voltage = 9

        # Publisher to send the commanded mode to
        self.cmd_mode_pub = None

    # ROS Callback Methods
    ######################
    def mode_callback(self, msg):
        """Update the current mode of the drone"""
        self.prev_mode = self.curr_mode
        self.curr_mode = msg.mode
        if self.prev_mode != self.curr_mode:
            print self.curr_mode

    def desired_mode_callback(self, msg):
        """Update the current mode of the drone"""
        self.desired_mode = msg.mode

    def battery_callback(self, msg):
        """Update the current vbat and amperage values"""
        self.vbat = msg.vbat
        self.amperage = msg.amperage

    # Heartbeat Callbacks: These update the last time that data was received
    #                       from a node
    def heartbeat_web_interface_callback(self, msg):
        """Update web_interface heartbeat"""
        self.heartbeat_web_interface = rospy.Time.now()

    def heartbeat_command_line_interface_callback(self, msg):
        """Update command_line_interface heartbeat"""
        self.heartbeat_command_line_interface = rospy.Time.now()

    def heartbeat_flight_controller_callback(self, msg):
        """Update the flight_controller heartbeat"""
        self.heartbeat_flight_controller = rospy.Time.now()

    def heartbeat_pid_controller_callback(self, msg):
        """Update pid_controller heartbeat"""
        self.heartbeat_pid_controller = rospy.Time.now()

    def heartbeat_infrared_callback(self, msg):
        """Update ir sensor heartbeat"""
        self.heartbeat_infrared = rospy.Time.now()

    def shouldIDisarm(self):
        """
        Disarm the drone if the battery values are too low or if there is a
        missing heartbeat
        """
        curr_time = rospy.Time.now()
        disarm = False
        if self.vbat != None and self.vbat < self.minimum_voltage:
            print('\nSafety Failure: low battery\n')
            disarm = True
        if ((curr_time - self.heartbeat_web_interface) > rospy.Duration.from_sec(5) and
        (curr_time - self.heartbeat_command_line_interface) > rospy.Duration.from_sec(5)):
            print('\nSafety Failure: user interface heartbeat\n')
            print('Ensure that either the web interface or command line interface is running')
            disarm = True
        if curr_time - self.heartbeat_flight_controller > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving data from flight controller.')
            print('Check the flight_controller node\n')
            disarm = True
        if curr_time - self.heartbeat_pid_controller > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving flight commands.')
            print('Check the pid_controller node\n')
            disarm = True
        if curr_time - self.heartbeat_infrared > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving data from the IR sensor.')
            print('Check the infrared node\n')
            disarm = True

        return disarm

    def ctrl_c_handler(self, signal, frame):
        """Disarms the drone and exits the program if ctrl-c is pressed"""
        print "\nCaught ctrl-c! About to Disarm!"
        self.cmd_mode_pub.publish('DISARMED')
        sys.exit()


def main():
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # Instantiate a ModeController object
    mc = ModeController()
    curr_time = rospy.Time.now()
    mc.heartbeat_infrared = curr_time
    mc.heartbeat_web_interface= curr_time
    mc.heartbeat_pid_controller = curr_time
    mc.heartbeat_flight_controller = curr_time
    mc.heartbeat_command_line_interface = curr_time

    # Publishers
    ############
    mc.cmd_mode_pub = rospy.Publisher('/pidrone/commanded/mode', Mode, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/mode", Mode, mc.mode_callback)
    rospy.Subscriber("/pidrone/desired/mode", Mode, mc.desired_mode_callback)
    rospy.Subscriber("/pidrone/battery", Battery, mc.battery_callback)
    # heartbeat subscribers
    rospy.Subscriber("/pidrone/heartbeat/infrared", Empty, mc.heartbeat_infrared_callback)
    rospy.Subscriber("/pidrone/heartbeat/web_interface", Empty, mc.heartbeat_web_interface_callback)
    rospy.Subscriber("/pidrone/heartbeat/pid_controller", Empty, mc.heartbeat_pid_controller_callback)
    rospy.Subscriber("/pidrone/heartbeat/flight_controller", Empty, mc.heartbeat_flight_controller_callback)
    rospy.Subscriber("/pidrone/heartbeat/command_line_interface", Empty, mc.heartbeat_command_line_interface_callback)


    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, mc.ctrl_c_handler)

    print 'Controlling Mode'
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        try:
            # if the current or desired mode is anything other than disarmed
            # preform as safety check
            if mc.curr_mode != 'DISARMED' or mc.desired_mode != 'DISARMED':
                # Break the loop if a safety check has failed
                if mc.shouldIDisarm():
                    break

            # Finite State Machine
            ######################
            if mc.curr_mode == 'DISARMED':
                if mc.desired_mode == 'DISARMED':
                    mc.cmd_mode_pub.publish('DISARMED')
                elif mc.desired_mode == 'ARMED':
                    print 'sending arm command'
                    mc.cmd_mode_pub.publish('ARMED')
                    rospy.sleep(1)
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

            elif mc.curr_mode == 'ARMED':
                if mc.desired_mode == 'ARMED':
                    mc.cmd_mode_pub.publish('ARMED')
                elif mc.desired_mode == 'FLYING':
                    print 'sending fly command'
                    mc.cmd_mode_pub.publish('FLYING')
                elif mc.desired_mode == 'DISARMED':
                    print 'sending disarm command'
                    mc.cmd_mode_pub.publish('DISARMED')
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

            elif mc.curr_mode == 'FLYING':
                if mc.desired_mode == 'FLYING':
                    mc.cmd_mode_pub.publish('FLYING')
                elif mc.desired_mode == 'DISARMED':
                    print 'sending disarm command'
                    mc.cmd_mode_pub.publish('DISARMED')
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

        except:
                print 'there was an internal error'
                print 'cannot transition to', mc.desired_mode
                sys.exit()
        r.sleep()

    mc.cmd_mode_pub.publish('DISARMED')
    print 'Shutdown Received'
    print 'Sending DISARM command'


if __name__ == '__main__':
    main()
