import rospy
import time
from sensor_msgs.msg import Range
import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
m = 181818.18181818182
b = -8.3 + 4.0
smoothed_distance = 0
alpha = 0.4

def get_range():
    global smoothed_distance
    voltage = adc.read_adc(0, gain=GAIN)
    distance = (1.0 / voltage) * m + b
    smoothed_distance = (1.0 - alpha) * smoothed_distance + alpha * distance

    print smoothed_distance, distance, (smoothed_distance - distance)
    return smoothed_distance

if __name__ == "__main__":
    rospy.init_node("infrared_pub")
    pub = rospy.Publisher('/pidrone/infrared', Range, queue_size=1)
    rnge = Range()
    rnge.header.frame_id = "world"
    while not rospy.is_shutdown():
        rnge.range = get_range()
        pub.publish(rnge)
        time.sleep(0.001)
