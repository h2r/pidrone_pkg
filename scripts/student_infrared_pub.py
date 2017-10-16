'''
CS1951R - Introduction to Robotics
Brown University
Fall 2017

infrared_node.py
'''

import rospy
from sensor_msgs.msg import Range
import Adafruit_ADS1x15

###############################################################################
# YOUR CODE HERE
###############################################################################
# The Sharp Infrared Rangefinder returns a voltage of roughly 3.1V at 
# 0.08 meters to 0.4V at 0.8 meters. We read in this voltage using the Adafruit
# ADS1015 Analog to Digital Converted (ADC). This device returns a 12bit int
# corresponding to the measured voltage.
#
# You will implmenet the function calc_distance, which takes in this 12bit 
# voltage returns a distance in meters. Note that distance in inversely
# proportional to voltage (d = 1/V) and you will need to both rescale and
# offset your distance (m*x + b).
#
# Measure (with a ruler or tape measure) to estimate your paramters.

def calc_distance(voltage):
    return 0

# Implement exponentional moving average smoothing. The return from this
# function will be passed in again the next time it is called

def exp_smooth(raw_dist, prev_smooth_dist, alpha):
    return raw_dist * alpha + prev_smooth_dist * (1.-alpha)

###############################################################################
# YOUR CODE ABOVE
###############################################################################

def main():
    rospy.init_node("infrared_node")
    r = rospy.Rate(100)
    adc = Adafruit_ADS1x15.ADS1115()
    

    ###############################################################################
    # YOUR CODE HERE
    ###############################################################################
    alpha = 0.3 # feel free to adjust the amount of smoothing

    # (1) initialize a publisher that publishes a Range message to the topic
    # '/pidrone/infrared' with a queue_size of 1

    # (2) instantiate a Range message which you will update and publish in the while
    # loop below. Set the header.frame_id to 'world'

    prev_smooth_dist = None
    while not rospy.is_shutdown():
        voltage = adc.read_adc(0, gain=1)
        raw_dist = calc_distance(voltage) # you implemented this above

        if prev_smooth_dist is None: prev_smooth_dist = raw_dist
        smooth_dist = exp_smooth(raw_dist, prev_smooth_dist, alpha) # this one too!
        prev_smooth_dist = smooth_dist

        # (3) set the timestamp on the Range message using get_rostime. Set the
        # Range to your smoothed distance estimate. Publish the message!

    ###############################################################################
    # YOUR CODE ABOVE
    ###############################################################################
        r.sleep()


    rospy.init_node("infrared_node")
    pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
    rnge = Range()
    rnge.max_range = 0.55
    rnge.min_range = 0.08
    rnge.header.frame_id = "world"
    adc = Adafruit_ADS1x15.ADS1115()
    alpha = 0.3
    r = rospy.Rate(100)

    prev_smooth_dist = None

    while not rospy.is_shutdown():
        voltage = adc.read_adc(0, gain=1)
        raw_dist = calc_distance(voltage)
        if prev_smooth_dist is None: prev_smooth_dist = raw_dist
        smooth_dist = exp_smooth(raw_dist, prev_smooth_dist, alpha)
        prev_smooth_dist = smooth_dist

        rnge.header.stamp = rospy.get_rostime()
        rnge.Range = smooth_dist
        pub.publish(rnge)

        r.sleep()

if __name__ == "__main__":
    main()
