import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import time
import sys

write_file = open(sys.argv[1], 'a')

def write_to_file(data, name):
    global write_file
    write_file.write("vrpn, " + str(data.header.stamp.to_sec()) + ", " +
    str(data.pose.position.x) + ", " + str(data.pose.position.y) + ", " +
    str(data.pose.position.z) + ", " + str(data.pose.orientation.x) + ", " +
    str(data.pose.orientation.y) + ", " + str(data.pose.orientation.z) + ", " +
    str(data.pose.orientation.w) + "\n")
    write_file.flush()

def vrpn_callback(data):
    write_to_file(data, "vrpn")

def test_callback(data):
    write_to_file(data, "homo")

if __name__ == "__main__":
    rospy.init_node("dynamic_test")
    rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_callback)
    rospy.Subscriber("/pidrone/homo_pos", PoseStamped, test_callback)
    while not rospy.is_shutdown():
        time.sleep(0.001)
    write_file.close()

