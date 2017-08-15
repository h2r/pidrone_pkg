import rospy
import time
from sensor_msgs.msg import Range
import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
m = 181818.18181818182
b = -8.3 + 7.5
smoothed_distance = 0
alpha = 0.7

alpha_50 = 0.2
alpha_20 = 0.2

def get_range():
    global smoothed_distance

    slew_distance = max( min( smoothed_distance, 50.0 ), 20.0)
    alpha = ( (slew_distance - 20.0) * alpha_50 + (50.0 - slew_distance) * alpha_20 ) / (50.0-20.0)
    print alpha
    #alpha = 1. # TESTING FOR COMPARING ULTRA AND INFRA

    voltage = adc.read_adc(0, gain=GAIN)
    distance = (1.0 / voltage) * m + b
    smoothed_distance = (1.0 - alpha) * smoothed_distance + alpha * distance

    print smoothed_distance, distance, (smoothed_distance - distance)
    return smoothed_distance

if __name__ == "__main__":
    rospy.init_node("infrared_pub")
    pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
    rnge = Range()
    rnge.max_range = 100
    rnge.min_range = 0
    rnge.header.frame_id = "world"
    while not rospy.is_shutdown():
        r = rospy.Rate(100)
        rnge.header.stamp = rospy.get_rostime()
        rnge.range = get_range()
        pub.publish(rnge)
        r.sleep()
