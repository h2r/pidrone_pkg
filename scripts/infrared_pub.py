import rospy
from sensor_msgs.msg import Range
import Adafruit_ADS1x15
import signal
import sys


class IR(object):
    """A class that reads, analyzes, and publishes IR sensor data. The raw data
    is published on the /raw-infrared topic (this will be used by the
    state_estimation node), and the smoothed data is published on the /infrared
    topic (this is the data currently used by sensors and controllers).

    Publisher:
    /pidrone/infrared
    /pidrone/raw_infrared
    """

    def __init__(self):
        self.adc = Adafruit_ADS1x15.ADS1115()
        self.GAIN = 1
        self.distance = 0
        self.smoothed_distance = 0
        # Smoothing values
        # XXX jgo these values underestimate the height on my sensor significantly
        #m = 181818.18181818182
        #m = 2.0 * 181818.18181818182
        #b = -8.3 + 7.5 - 17.0
        self.alpha = 0.2
        self.m = 181818.18181818182 * 1.238 # 1.3 / 1.05
        self.b = -8.3 + 7.5

    def get_range(self):
        """Read the data from the adc and update the distance and
        smoothed_distance values."""
        voltage = self.adc.read_adc(0, self.GAIN)
        if voltage <= 0:
            voltage = 1
            print "ERROR: BAD VOLTAGE!!!"
        self.distance = ((1.0 / voltage) * self.m + self.b) / 100.0
        self.smoothed_distance = (1.0 - self.alpha) * self.smoothed_distance + self.alpha * self.distance
        self.smoothed_distance = min(self.smoothed_distance, 0.55)

    def publish_range(self, range, publisher):
        """Create and publish the Range message to publisher."""
        msg = Range()
        msg.header.stamp = rospy.get_rostime()
        msg.max_range = 0.8
        msg.min_range = 0
        msg.range = range
        msg.header.frame_id = "base"
        publisher.publish(msg)

    def ctrl_c_handler(self, signal, frame):
        """Gracefully quit the infrared_pub node"""
        print "\nCaught ctrl-c! Stopping node."
        sys.exit()

def main():
    """Start the ROS node, create the publishers, and continuosly update and
    publish the IR sensor data"""

    # ROS Setup
    ###########
    rospy.init_node("infrared_pub")

    # Publishers
    ############
    smoothedpub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
    rawpub = rospy.Publisher('/pidrone/raw_infrared', Range, queue_size=1)
    print 'Publishing IR'

    # Non-ROS Setup
    ###############
    # create IR object
    ir = IR()
    # set the while loop frequency
    r = rospy.Rate(100)
    # set up the ctrl-c handler
    signal.signal(signal.SIGINT, ir.ctrl_c_handler)

    while not rospy.is_shutdown():
        ir.get_range()
        ir.publish_range(ir.smoothed_distance, smoothedpub)
        ir.publish_range(ir.distance, rawpub)
        r.sleep()

if __name__ == "__main__":
    main()
