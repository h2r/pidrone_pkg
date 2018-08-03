import rospy
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
alpha = 0.2


def get_range():
    global smoothed_distance

    voltage = adc.read_adc(0, gain=GAIN)
    if voltage <= 0:
        voltage = 1
        print "ERROR: BAD VOLTAGE!!!"
    distance = ((1.0 / voltage) * m + b) / 100.0
    smoothed_distance = (1.0 - alpha) * smoothed_distance + alpha * distance
    smoothed_distance = min(smoothed_distance, 0.55)

    return distance, smoothed_distance


def main():
    rospy.init_node("infrared_pub")
    raw_pub = rospy.Publisher('/pidrone/infrared_raw', Range, queue_size=1)
    pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
    # Raw range message for data logging and analysis purposes
    raw_rnge = Range()
    rnge = Range()
    raw_rnge.max_range = 100
    raw_rnge.min_range = 0
    raw_rnge.header.frame_id = 'base'
    rnge.max_range = 100
    rnge.min_range = 0
    rnge.header.frame_id = 'base'
    r = rospy.Rate(100)
    print "publishing IR"
    while not rospy.is_shutdown():
        r = rospy.Rate(100)
        curr_time = rospy.Time.now()
        raw_rnge.header.stamp = curr_time
        rnge.header.stamp = curr_time
        raw_rnge.range, rnge.range = get_range()
        print rnge
        raw_pub.publish(raw_rnge)
        pub.publish(rnge)
        r.sleep()


if __name__ == "__main__":
    main()
