import abc
import sys
import signal
import rospy
from pidrone_pkg.msg import State
from geometry_msgs import PoseWithCovarianceStamped, TwistWithCovarianceStamped

class StateEstimator(abc.ABC):

    def __init__(self):
    """ A constructor for StateEstimator """
        self.pose = PoseWithCovarianceStamped()
        self.twist = TwistWithCovarianceStamped()

    @abstractmethod
    def get_state_estimator(self):
        """ return an instance of a class that inherits the StateEstimator class
        """
        pass

    @abstractmethod
    def update_pose(self):
        """ update the current pose of the drone """
        pass

    @abstractmethod
    def update_twist(self):
        """ update the current twist of the drone """
        pass

    def setup_ROS(self):
        """ Initialize the state estimator node """
        rospy.init_node('state_estimator')
        self.statepub = rospy.Publisher('/pidrone/State', State, queue_size=1, tcp_nodelay=False)

    def publish_state(self):
        """ Publish the State message """
        state = State()
        state.pose_with_covariance_stamped = self.pose
        state.twist_with_covariance_stamped = self.twist
        self.statepub.publish(state)

    def ctrl_c_handler(self, signal, frame):
        """ Gracefully handle ctrl-c """
        print "\nStopping State Estimation Node"
        sys.exit()

def main():
    state_estimator = get_state_estimator()
    state_estimator.setup_ROS()
    # gracefully handle Ctrl-c
    signal.signal(signal.SIGINT, state_estimator.ctrl_c_handler)
    # set the loop rate (Hz)
    loop_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        state_estimator.update_pose()
        state_estimator.update_twist()
        state_estimator.publish_state()
        loop_rate.sleep()
    print 'Shutdown received'
