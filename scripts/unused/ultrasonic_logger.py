import rospy
import numpy as np
from sensor_msgs.msg import Range
infra = np.array([[0,0]],dtype='float')
ultra = np.array([[0,0]],dtype='float')
start = None

def ultra_callback(data):
    global ultra,start
    new = np.array([[(data.header.stamp-start).to_sec(), data.range]])
    ultra = np.concatenate((ultra,new), axis=0)

def infra_callback(data):
    global infra,start
    new = np.array([[(data.header.stamp-start).to_sec(), data.range]])
    infra = np.concatenate((infra,new), axis=0)


if __name__ == '__main__':
    global infra,ultra,start
    rospy.init_node('ultrasonic_logger')
    start = rospy.get_rostime()
    rospy.Subscriber('/pidrone/ultrasonic', Range, ultra_callback)
    rospy.Subscriber('/pidrone/infrared', Range, infra_callback)
    rospy.sleep(60)
    print np.shape(infra), np.shape(ultra)
    np.savetxt('test_data/infrared_flying.txt', infra, fmt='%f')
    np.savetxt('test_data/ultrasonic_flying.txt', ultra, fmt='%f')
    print 'SAVED'
