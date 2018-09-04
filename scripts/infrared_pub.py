import sys
import os
import rospy
import signal
import Adafruit_ADS1x15
from std_msgs.msg import Empty
from sensor_msgs.msg import Range


class IR(object):
    """A class that reads, analyzes, and publishes IR sensor data.

    Publisher:
    /pidrone/infrared
    """

    def __init__(self):
        self.adc = Adafruit_ADS1x15.ADS1115()
        self.GAIN = 1
        self.distance = 0
        # values used to define the slope and intercept of
        # distance as a function of voltage : d(v) = 1/v * m + b
        self.m = 181818.18181818182 * 1.238
        self.b = -8.3 + 7.5

    def get_range(self):
        """Read the data from the adc and update the distance and
        smoothed_distance values."""
        voltage = self.adc.read_adc(0, self.GAIN)
        if voltage <= 0:
            voltage = 1
            print "ERROR: BAD VOLTAGE!!!"
        self.distance = ((1.0 / voltage) * self.m + self.b) / 100.0 # 100 is for cm -> m

    def publish_range(self, range):
        """Create and publish the Range message to publisher."""
        msg = Range()
        msg.max_range = 0.8
        msg.min_range = 0
        msg.range = range
        msg.header.frame_id = "base"
        msg.header.stamp = rospy.Time.now()
        self.range_pub.publish(msg)

    def ctrl_c_handler(self, signal, frame):
        """Gracefully quit the infrared_pub node"""
        print "\nCaught ctrl-c! Stopping node."
        sys.exit()

def main():
    """Start the ROS node, create the publishers, and continuosly update and
    publish the IR sensor data"""

    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # create IR object
    ir = IR()

    # Publishers
    ############
    ir.range_pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
    ir.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/infrared', Empty, queue_size=1)
    print 'Publishing IR'

    # Non-ROS Setup
    ###############
    # set the while loop frequency
    r = rospy.Rate(100)
    # set up the ctrl-c handler
    signal.signal(signal.SIGINT, ir.ctrl_c_handler)

    while not rospy.is_shutdown():
        ir.heartbeat_pub.publish(Empty())
        ir.get_range()
        ir.publish_range(ir.distance)
        r.sleep()

if __name__ == "__main__":
    main()
