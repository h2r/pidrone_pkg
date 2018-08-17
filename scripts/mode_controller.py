#!/usr/bin/python
import tf
import sys
import os
import rospy
import signal
import numpy as np
import command_values as cmds
from std_msgs.msg import String
from serial import SerialException
from sensor_msgs.msg import Range, Imu
from pidrone_pkg.msg import Mode, Battery, RC


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
    def heartbeat_callback(self, msg):
        """Update the time of the most recent heartbeat sent from the web interface"""
        self.last_heartbeat = rospy.Time.now()

    def mode_callback(self, msg):
        """Update the current mode of the drone"""
        self.last_mode_time = rospy.Time.now()
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

    def fly_commands_callback(self, msg):
        """Update last_fly_command_time to check if the pid controller is running"""
        self.last_fly_command_time = rospy.Time.now()

    def shouldIDisarm(self):
        """
        Disarm the drone if:
        - the battery values are too low
        - has not received a heartbeat in the last five seconds, or
        - has not received a mode message from the flight controller within
            the last five seconds
        - has not received fly commands from the pid
        """
        curr_time = rospy.Time.now()
        disarm = False
        if mc.vbat != None and mc.vbat < mc.minimum_voltage:
            print '\nSafety Failure: low battery\n'
            disarm = True
        if curr_time - self.last_heartbeat > rospy.Duration.from_sec(5):
            print '\nSafety Failure: no heartbeat\n'
            disarm = True
        if curr_time - self.last_mode_time > rospy.Duration.from_sec(1):
            print '\nSafety Failure: not receiving data from flight controller.'
            print 'Check the flight_controller node\n'
            disarm = True
        if curr_time - self.last_fly_command_time > rospy.Duration.from_sec(1):
            print '\nSafety Failure: not receiving flight commands.'
            print 'Check the pid_controller node\n'
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
    mc.last_heartbeat = curr_time
    mc.last_mode_time = curr_time
    mc.last_fly_command_time = curr_time

    # Publishers
    ############
    mc.cmd_mode_pub = rospy.Publisher('/pidrone/commanded/mode', Mode, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/mode", Mode, mc.mode_callback)
    rospy.Subscriber("/pidrone/desired/mode", Mode, mc.desired_mode_callback)
    rospy.Subscriber("/pidrone/heartbeat", String, mc.heartbeat_callback)
    rospy.Subscriber("/pidrone/battery", Battery, mc.battery_callback)
    rospy.Subscriber("/pidrone/fly_commands", RC, mc.fly_commands_callback)

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
                sys.exit()
        r.sleep()

    mc.cmd_mode_pub.publish('DISARMED')
    print 'Shutdown Received'
    print 'Sending DISARM command'


if __name__ == '__main__':
    main()
