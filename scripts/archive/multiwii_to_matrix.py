import numpy as np
from h2rMultiWii import MultiWii  
import tf
from geometry_msgs.msg import PoseStamped
import rospy
rospy.init_node("multiwii_attitude")

board = MultiWii("/dev/ttyACM0")

while True:
