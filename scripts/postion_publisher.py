import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("position_publisher")
    r = rospy.Rate(100)
    pub = rospy.Publisher("/pidrone/pos_setpoint", PoseStamped, queue_size=1)
    rospy.sleep(1)
    pos = PoseStamped()
    pos.header.stamp = rospy.get_rostime()
    pos.pose.position.y = -2
    print pos
    pub.publish(pos)
