#!/usr/bin/python
import tf
import sys
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
        """Updates the time of the most recent heartbeat sent from the web interface"""
        self.last_heartbeat = rospy.Time.now()

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
        """Updates the current vbat and amperage values"""
        self.vbat = msg.vbat
        self.amperage = msg.amperage

    def shouldIDisarm(self):
        """Disarm the drone if the battery values are too low or
        if it has not received a heartbeat in the last five seconds
        """
        if mc.vbat < mc.minimum_voltage:
            print 'Safety Failure: low battery'
        if rospy.Time.now() - self.last_heartbeat > rospy.Duration.from_sec(5):
            print 'Safety Failure: no heartbeat'
        return (mc.vbat < mc.minimum_voltage or rospy.Time.now() - self.last_heartbeat > rospy.Duration.from_sec(5))

    def ctrl_c_handler(self, signal, frame):
        """Disarms the drone and exits the program if ctrl-c is pressed"""
        print "\nCaught ctrl-c! About to Disarm!"
        self.cmd_mode_pub.publish('DISARMED')
        sys.exit()

if __name__ == '__main__':

    # ROS Setup
    ###########
    rospy.init_node('mode_controller')

    # Instantiate a ModeController object
    mc = ModeController()
    mc.last_heartbeat = rospy.Time.now()

    # Publishers
    ############
    mc.cmd_mode_pub = rospy.Publisher('/pidrone/commanded_mode', Mode, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/heartbeat", String, mc.heartbeat_callback)
    rospy.Subscriber("/pidrone/battery", Battery, mc.battery_callback)
    rospy.Subscriber("/pidrone/mode", Mode, mc.mode_callback)
    rospy.Subscriber("/pidrone/desired_mode", Mode, mc.desired_mode_callback)

    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, mc.ctrl_c_handler)

    # Variables and method used to check connection with flight controller node
    mc.same_cmd_counter = 0
    mc.last_cmd = 'dsm'
    mc.curr_cmd = 'dsm'
    def same_cmd_counter_update(cmd):
        """Count the number of times sc has published the same command. This is
        used to check if the flight_controller is receiving commands"""
        mc.last_cmd = mc.curr_cmd
        mc.curr_cmd = cmd
        if mc.last_cmd == mc.curr_cmd:
            mc.same_cmd_counter = mc.same_cmd_counter + 1

    print 'Controlling Mode'
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        try:
            # Break the loop if the flight controller isn't updating /pidrone/mode
            if mc.same_cmd_counter > 10:
                print ('\n\nThere was an error communicating with the flight controller.'
                       '\nCheck if the flight controller node is active.\n\n')
                break
            # Break the loop if a safety check has failed
            if not mc.curr_mode == 'DISARMED':
                if mc.shouldIDisarm():
                    break

            # Finite State Machine
            ######################
            if mc.curr_mode == 'DISARMED':
                if mc.desired_mode == 'DISARMED':
                    pass
                elif mc.desired_mode == 'ARMED':
                    print 'sending arm command'
                    mc.cmd_mode_pub.publish('ARMED')
                    rospy.sleep(1)
                    same_cmd_counter_update('a')
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
                    same_cmd_counter_update('d')
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
                raise
        r.sleep()

    mc.cmd_mode_pub.publish('DISARMED')
    print 'Shutdown Received'
    print 'Sending DISARM command'
