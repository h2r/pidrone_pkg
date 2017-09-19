'''
CS1951R - Introduction to Robotics
Brown University
Fall 2017

infrared_node.py
'''

import rospy
import time
from sensor_msgs.msg import Range
import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()

'''
DO NOT TOUCH CODE ABOVE THIS LINE
'''
# Tune these terms so that distance is in meters
m = pass
b = pass
# distance = (m * raw_value) + b

# Tune this term so that your smoothed_distance does not have spikes but also
# does not have too much latency
alpha = pass
# smoothed_distance = (1. - alpha) * smoothed_distance + alpha * distance

'''
DO NOT TOUCH CODE BELOW THIS LINE
'''
smoothed_distance = 0

GAIN = 1
def get_range():
    global smoothed_distance
    global alpha
    global m
    global b
    global smoothed_distance

    voltage = adc.read_adc(0, gain=GAIN)
    if voltage <= 0:
        voltage = 1
        print "ERROR: BAD VOLTAGE!!!"
    distance = (1.0 / voltage) * m + b
    smoothed_distance = (1.0 - alpha) * smoothed_distance + alpha * distance

    print smoothed_distance
    return smoothed_distance

if __name__ == "__main__":
    rospy.init_node("infrared_pub")
    pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
    rnge = Range()
    rnge.max_range = 0.55
    rnge.min_range = 0.08
    rnge.header.frame_id = "world"
    while not rospy.is_shutdown():
        r = rospy.Rate(100)
        rnge.header.stamp = rospy.get_rostime()
        rnge.range = get_range()
        pub.publish(rnge)
        r.sleep()
