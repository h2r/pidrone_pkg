import rospy
from std_msgs.msg import Empty
from pidrone_pkg.msg import Mode

'''
The purpose of this script is to automate the process of commanding the drone to
arm itself, take off, reset the position hold transform image, and toggle into
position hold. Originally used to overcome latency in sending commands over the
Javascript web interface over the RLAB network.

USE CAUTION WHEN USING THIS SCRIPT! It arms the drone and commands it to take
off automatically!

Note 2018-06-29: It appears that this automation script may not actually solve
the apparent network latency issue. Perhaps there is no network issue and the
latency we are experiencing is instead a result of the Pi being too hot and
clocking down?
'''

class AutomatePositionHold(object):
    
    def __init__(self):
        rospy.init_node('automate_position_hold')
        self.resetpub = rospy.Publisher('/pidrone/reset_transform', Empty, queue_size=1)
        self.togglepub = rospy.Publisher('/pidrone/toggle_transform', Empty, queue_size=1)
        self.modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)

    def start_automation(self):
        rospy.sleep(5.0)
        self.publish_arm()
        rospy.sleep(5.0)
        self.publish_takeoff()
        rospy.sleep(5.0)
        self.publish_reset()
        rospy.sleep(5.0)
        self.publish_toggle()
    
    def publish_arm(self):
        print 'Publishing arm'
        mode_msg = Mode()
        mode_msg.mode = 0
        mode_msg.x_velocity = 0
        mode_msg.y_velocity = 0
        mode_msg.z_velocity = 0
        mode_msg.yaw_velocity = 0
        self.modepub.publish(mode_msg)
    
    def publish_takeoff(self):
        print 'Publishing take-off'
        mode_msg = Mode()
        mode_msg.mode = 5
        mode_msg.x_velocity = 0
        mode_msg.y_velocity = 0
        mode_msg.z_velocity = 0
        mode_msg.yaw_velocity = 0
        self.modepub.publish(mode_msg)
        
    def publish_reset(self):
        print 'Publishing reset'
        self.resetpub.publish(Empty())
        
    def publish_toggle(self):
        print 'Publishing toggle'
        self.togglepub.publish(Empty())

if __name__ == '__main__':
    automate_position_hold = AutomatePositionHold()
    automate_position_hold.start_automation()