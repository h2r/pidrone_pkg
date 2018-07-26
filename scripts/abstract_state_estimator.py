import rospy
from pidrone_pkg.msg import State
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

class StateEstimator(object):

    def __init__(self):
        """ A constructor for StateEstimator """
        self.pose = PoseWithCovarianceStamped()
        self.twist = TwistWithCovarianceStamped()
        self.setup_ROS()

    def setup_ROS(self):
        """ Initialize the state estimator node """
        rospy.init_node('state_estimator')
        self.statepub = rospy.Publisher('/pidrone/state', State, queue_size=1, tcp_nodelay=False)

    def publish_state(self):
        """ Publish the State message """
        state = State()
        state.pose_with_covariance_stamped = self.pose
        state.twist_with_covariance_stamped = self.twist
        self.statepub.publish(state)
