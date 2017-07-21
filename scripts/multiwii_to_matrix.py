import numpy as np
from h2rMultiWii import MultiWii  
import tf
from geometry_msgs.msg import PoseStamped
import rospy
rospy.init_node("multiwii_attitude")

board = MultiWii("/dev/ttyACM0")
attpub = rospy.Publisher('/pidrone/multiwii_attitude', PoseStamped, queue_size=1)
pos = PoseStamped()
pos.header.frame_id = 'world'

while True:
    data = board.getData(MultiWii.ATTITUDE)
    y = data['heading']/180.0*np.pi
    r = data['angx']/180.0*np.pi
    p = data['angy']/180.0*np.pi
    q = np.array(tf.transformations.quaternion_from_euler(-p, r, -y))
    pos.pose.orientation.x = q[0]
    pos.pose.orientation.y = q[1]
    pos.pose.orientation.z = q[2]
    pos.pose.orientation.w = q[3]
    attpub.publish(pos)
