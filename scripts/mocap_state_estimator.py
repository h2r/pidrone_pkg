from pidrone_pkg.msg import State
import rospy

class MocapStateEstimator(StateEstimator)


    def __init__(self, rigid_body_name):
        """ A constructor for MocapStateEstimator
        Input: rigid_body_name- the name of the rigid body that gets set in motive
        """
        # initialize the parent class
        StateEstimator.init(self)
        # store the topic that the motion capture publishes to
        mocap_topic = 'vrpn_client_node' + rigid_body_name + '/pose'

    def vrpn_callback(self, msg):
        ''' Updates the drones state based on the Motion Tracker '''
        global current_position
        # y and z axes are switched in mocap, and positive x is flipped
        self.pose.header.stamp = rospy.get_rostime()
        self.pose.pose.position.x = - msg.pose.position.x
        self.pose.pose.position.y = msg.pose.position.z
        self.pose.pose.position.z = msg.pose.position.y

        self.pose.pose.orientation.

    def
        self.publish_state
    # TODO publish here

# TODO can this be simplified in parent class?
    def get_state_estimator(self):
        return self

    def update_pose(self):
        # This is taken care of by the vrpn_callback
        pass

    def update_twist(self):
        # Mocap does not publish twist messages
        pass

    def setup_ROS(self):
        """ Inherit the setup_ROS method and add mocap Subscriber """
        StateEstimator.setup_ROS(self)
        rospy.Subscriber(mocap_topic, PoseStamped, vrpn_callback)


if __name__ == '__main__':
    StateEstimator.main()
