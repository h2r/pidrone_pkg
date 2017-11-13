import rospy
import time
from sensor_msgs.msg import Range
import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
# XXX jgo these values underestimate the height on my sensor significantly
#m = 181818.18181818182
#m = 2.0 * 181818.18181818182
m = 181818.18181818182 * 1.238 # 1.3 / 1.05
b = -8.3 + 7.5
#b = -8.3 + 7.5 - 17.0
smoothed_distance = 0

def get_range():
    global smoothed_distance

    alpha = 0.2
    voltage = adc.read_adc(0, gain=GAIN)
    if voltage <= 0:
        voltage = 1
        print "ERROR: BAD VOLTAGE!!!"
    distance = ((1.0 / voltage) * m + b)/100.0
    smoothed_distance = (1.0 - alpha) * smoothed_distance + alpha * distance
    smoothed_distance = min(smoothed_distance, 0.55)

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
        print rnge
        pub.publish(rnge)
        r.sleep()
