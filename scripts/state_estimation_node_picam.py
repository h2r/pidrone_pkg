#!/usr/bin/python
import sys
import rospy
import signal
from pidrone_pkg.msg import Flow, Phase
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

class PiCameraStateEstimator(object):
    ''' A class that subscribes to data from the picamera that is published by
    the camera_controller node
    '''

    def __init__(self):
        ''' A constructor for PiCameraStateEstimator
        '''
        # store publishers
        self.posepub = None
        self.twistpub = None

        # check if the camera flow data is publishing
        self.received_flow_data = False

    def anaylze_flow_callback(self, msg):
        ''' Publish the twist of the drone from the flow values '''
        self.received_flow_data = True
        twist_to_pub = TwistWithCovarianceStamped()
        # update the header fields
        twist_to_pub.header.stamp = rospy.Time.now()
        twist_to_pub.header.frame_id = 'Body'
        # update the linear and yaw velocities
        twist_to_pub.twist.twist.linear.x = msg.x_velocity
        twist_to_pub.twist.twist.linear.y = msg.y_velocity
        twist_to_pub.twist.twist.linear.z = msg.z_velocity
        twist_to_pub.twist.twist.angular.z = msg.yaw_velocity
        # publish the twist message
        self.twistpub.publish(twist_to_pub)

    def analyze_phase_callback(self, msg):
        ''' Publish the pose of the drone from the phase values '''
        pose_to_pub = PoseWithCovarianceStamped()
        pose_to_pub.header.stamp = msg.header.stamp
        pose_to_pub.header.frame_id = 'Body'
        pose_to_pub.pose.pose.position.x = - msg.x_position
        pose_to_pub.pose.pose.position.y = msg.y_position
        pose_to_pub.pose.pose.position.z = msg.z_position
        # fill in the orientation quaternion
        pose_to_pub.pose.pose.orientation.x = 0
        pose_to_pub.pose.pose.orientation.y = 0
        pose_to_pub.pose.pose.orientation.z = msg.yaw_angle
        pose_to_pub.pose.pose.orientation.w = 1

        self.posepub.publish(pose_to_pub)

    def ctrl_c_handler(self, signal, frame):
        """ Stop subscribing to and publishing the mocap data """
        print "\nCaught ctrl-c. Stopping node."
        sys.exit()

if __name__ == '__main__':

    # Instantiate a PiCameraStateEstimator object
    state_estimator = PiCameraStateEstimator()

    # ROS setup
    ###########
    # Initialize the state estimator node
    rospy.init_node('state_estimator')

    # Publishers
    ############
    state_estimator.posepub = rospy.Publisher('/pidrone/pose', PoseWithCovarianceStamped, queue_size=1, tcp_nodelay=False)
    state_estimator.twistpub = rospy.Publisher('/pidrone/twist', TwistWithCovarianceStamped, queue_size=1, tcp_nodelay=False)

    # Subscribers
    #############
    rospy.Subscriber('/picamera/flow', Flow, state_estimator.anaylze_flow_callback)
    rospy.Subscriber('/picamera/phase', Phase, state_estimator.analyze_phase_callback)

    # set up ctrl-c handler
    signal.signal(signal.SIGINT, state_estimator.ctrl_c_handler)
    print 'waiting for flow data'
    while state_estimator.received_flow_data == False:
        pass
    # print the topics that are being published to
    print 'Publishing to:'
    print '/pidrone/pose'
    print '/pidrone/twist'

    # keep the node running for the callback methods
    while not rospy.is_shutdown():
        pass
