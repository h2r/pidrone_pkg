from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
from serial import SerialException
import rospy
import numpy as np
from pidrone_pkg.msg import axes_err, Mode, Battery, RC, FC_command
import signal
import sys
import tf
import command_values as cmds

class StateController(object):

    def __init__(self, cmdpub):

        # Desired,current, and previous modes of the drone
        self.desired_mode = 'DISARMED'
        self.curr_mode = 'DISARMED'
        self.prev_mode = 'DISARMED'

        # Battery values
        self.vbat = None
        self.amperage = None
        # Adjust this based on how low the battery should discharge
        self.minimum_voltage = 8

        # Fly command values from controller (initialize with disarm values)
        self.fly_cmd = cmds.disarm_cmd

        # Publisher for flight controller commands
        self.cmdpub = cmdpub

    # Mode Transition Methods
    ###########################
    def arm(self):
        """Arms the drone by publishing the arm command values"""
        self.publish_cmd("ARMED", cmds.arm_cmd)

    def disarm(self):
        """Disarms the drone by publishing the disarm command values"""
        self.publish_cmd("DISARMED", cmds.disarm_cmd)

    def idle(self):
        """Enables the drone to continue arming until commanded otherwise"""
        self.publish_cmd("ARMED", cmds.idle_cmd)

    def fly(self):
        """Enables flight by publishing the calculated flight
        commands to the flight controller
        """
        self.publish_cmd("FLYING", self.fly_cmd)

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

    def controller_callback(self, msg):
        """Update the fly_cmd from the controller"""
        r = msg.roll
        p = msg.pitch
        y = msg.yaw
        t = msg.throttle
        self.fly_cmd = [r,p,y,t]

    # Helper Methods
    ################
    def publish_cmd(self, mode, cmd):
        """Publish the commands to the flight controller"""
        msg = FC_command()
        msg.mode = mode
        msg.ctrls.roll = cmd[0]
        msg.ctrls.pitch = cmd[1]
        msg.ctrls.yaw = cmd[2]
        msg.ctrls.throttle = cmd[3]
        self.cmdpub.publish(msg)

    def shouldIDisarm(self):
        """Disarm the drone if the battery values are too low or
        if it has not received a heartbeat in the last five seconds
        """
        if sc.vbat < sc.minimum_voltage:
            print 'Safety Failure: low battery'
        if rospy.Time.now() - self.last_heartbeat > rospy.Duration.from_sec(5):
            print 'Safety Failure: no heartbeat'
        return (sc.vbat < sc.minimum_voltage or rospy.Time.now() - self.last_heartbeat > rospy.Duration.from_sec(5))

    def ctrl_c_handler(self, signal, frame):
        """Disarms the drone and exits the program if ctrl-c is pressed"""
        print "\nCaught ctrl-c! About to Disarm!"
        self.disarm()
        sys.exit()

if __name__ == '__main__':

    # ROS Setup
    ###########
    rospy.init_node('state_controller')

    # Publishers
    ############
    cmdpub = rospy.Publisher('/pidrone/command', FC_command, queue_size=1)

    # create StateController object
    sc = StateController(cmdpub)
    sc.last_heartbeat = rospy.Time.now()

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/heartbeat", String, sc.heartbeat_callback)
    rospy.Subscriber("/pidrone/battery", Battery, sc.battery_callback)
    rospy.Subscriber("/pidrone/mode", Mode, sc.mode_callback)
    rospy.Subscriber("/pidrone/desired_mode", Mode, sc.desired_mode_callback)
    rospy.Subscriber("/pidrone/controller", RC, sc.controller_callback)

    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, sc.ctrl_c_handler)

    # Variables and method used to check connection with flight controller node
    sc.same_cmd_counter = 0
    sc.last_cmd = 'dsm'
    sc.curr_cmd = 'dsm'
    def same_cmd_counter_update(cmd, ):
        sc.last_cmd = sc.curr_cmd
        sc.curr_cmd = cmd
        if sc.last_cmd == sc.curr_cmd:
            sc.same_cmd_counter = sc.same_cmd_counter + 1

    print 'Controlling State'
    r = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        if sc.same_cmd_counter > 5:
            print ('\n\nThere was an error communicating with the flight controller.'
                   '\nCheck if the flight controller node is active.\n\n')
            break
        if not sc.curr_mode == 'DISARMED':
            if sc.shouldIDisarm():
                break

        # Finite State Machine
        ######################
        if sc.curr_mode == 'DISARMED':
            if sc.desired_mode == 'DISARMED':
                pass
            elif sc.desired_mode == 'ARMED':
                print 'sending ARM command'
                sc.arm()
                rospy.sleep(1)
                same_cmd_counter_update('arm')
            else:
                print 'Cannot transition from Mode %s to Mode %s' % (sc.curr_mode, sc.desired_mode)

        elif sc.curr_mode == 'ARMED':
            if sc.desired_mode == 'ARMED':
                sc.idle()
# XXX: test this sleep value
                rospy.sleep(1)
            elif sc.desired_mode == 'FLYING':
                print 'sending FLYING command'
                sc.fly()
            elif sc.desired_mode == 'DISARMED':
                print 'sending DISARM command'
                sc.disarm()
                rospy.sleep(1)
                same_cmd_counter_update('dsm')
            else:
                print 'Cannot transition from Mode %s to Mode %s' % (sc.curr_mode, sc.desired_mode)

        elif sc.curr_mode == 'FLYING':
            if sc.desired_mode == 'FLYING':
                sc.fly()
                rospy.sleep(0.01)
# XXX: test this sleep value
            elif sc.desired_mode == 'DISARMED':
                print 'sending DISARM command'
                sc.disarm()
                rospy.sleep(1)
            else:
                print 'Cannot transition from Mode %s to Mode %s' % (sc.curr_mode, sc.desired_mode)

        r.sleep()

    sc.disarm()
    print 'Shutdown Received'
    print 'Sending DISARM command'
