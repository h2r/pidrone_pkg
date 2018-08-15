#!/usr/bin/python
import sys
import os
import rospy
import signal
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, AccelWithCovarianceStamped

class Mocap(object):
    ''' A class that subscribes to data from mocap, does the required axes
    transformations, and then publishes the data
    '''

    def __init__(self, rigid_body_name):
        ''' A constructor for Mocap

        rigid_body_name : the name of the rigid body set in motive
        '''
        # store the topics that the motion capture publishes to
        self.mocap_pose_topic = 'vrpn_client_node/' + rigid_body_name + '/pose'
        self.mocap_twist_topic = 'vrpn_client_node/' + rigid_body_name + '/twist'
        self.mocap_accel_topic = 'vrpn_client_node/' + rigid_body_name + '/accel'

        # store publishers
        self.posepub = None
        self.twistpub = None
        self.accelpub = None

        # check if the mocap data is streaming
        self.received_data = False

    def pose_callback(self, msg):
        ''' Publish the pose of the drone based on the Motion Tracker data '''
        # NOTICE: the positions and orientations are modified because the axes
        # of the mocap are set up as right-handed y-up. This means that the y
        # and z axes are switched, and positive x is to the left.
        pose_to_pub = PoseWithCovarianceStamped()
        pose_to_pub.header.stamp = msg.header.stamp
        pose_to_pub.header.frame_id = 'World'
        pose_to_pub.pose.pose.position.x = - msg.pose.position.x
        pose_to_pub.pose.pose.position.y = msg.pose.position.z
        pose_to_pub.pose.pose.position.z = msg.pose.position.y
        pose_to_pub.pose.pose.orientation.x = - msg.pose.orientation.x
        pose_to_pub.pose.pose.orientation.y = msg.pose.orientation.z
        pose_to_pub.pose.pose.orientation.z = msg.pose.orientation.y
        self.posepub.publish(pose_to_pub)

        self.received_data = True

    def twist_callback(self, msg):
        ''' Publish the twist of the drone based on the Motion Tracker data '''
        # NOTICE: the velocities are modified because the axes
        # of the mocap are set up as right-handed y-up. This means that the y
        # and z axes are switched, and positive x is to the left.
        twist_to_pub = TwistWithCovarianceStamped()
        twist_to_pub.header.stamp = msg.header.stamp
        twist_to_pub.header.frame_id = 'World'
        twist_to_pub.twist.twist.linear.x = - msg.twist.linear.x
        twist_to_pub.twist.twist.linear.y = msg.twist.linear.z
        twist_to_pub.twist.twist.linear.z = msg.twist.linear.y
        twist_to_pub.twist.twist.angular.x = - msg.twist.angular.x
        twist_to_pub.twist.twist.angular.y = msg.twist.angular.z
        twist_to_pub.twist.twist.angular.z = msg.twist.angular.y
        self.twistpub.publish(twist_to_pub)

    def accel_callback(self, msg):
        ''' Publish the acceleration of the drone based on the Motion Tracker data '''
        # NOTICE: the accelerations are modified because the axes
        # of the mocap are set up as right-handed y-up. This means that the y
        # and z axes are switched, and positive x is to the left.
        accel_to_pub = AccelWithCovarianceStamped()
        accel_to_pub.header.stamp
        accel_to_pub.header.frame_id = 'World'
        accel_to_pub.accel.accel.linear.x = - msg.accel.linear.x
        accel_to_pub.accel.accel.linear.y = msg.accel.linear.z
        accel_to_pub.accel.accel.linear.z = msg.accel.linear.y
        accel_to_pub.accel.accel.angular.x = - msg.accel.angular.x
        accel_to_pub.accel.accel.angular.y = msg.accel.angular.z
        accel_to_pub.accel.accel.angular.z = msg.accel.angular.y
        self.accelpub.publish(accel_to_pub)

    def ctrl_c_handler(self, signal, frame):
        """ Stop subscribing to and publishing the mocap data """
        print "\nCaught ctrl-c. Stopping node."
        sys.exit()
        
    
def main():
    # Instantiate a Mocap object
    rigid_body_name = raw_input('Enter the name of the rigid body: ')
    mc = Mocap(rigid_body_name)

    # ROS setup
    ###########
    # Initialize the mocap node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # Publishers
    ############
    mc.posepub = rospy.Publisher('/pidrone/pose', PoseWithCovarianceStamped, queue_size=1, tcp_nodelay=False)
    mc.twistpub = rospy.Publisher('/pidrone/twist', TwistWithCovarianceStamped, queue_size=1, tcp_nodelay=False)
    mc.accelpub = rospy.Publisher('/pidrone/accel', AccelWithCovarianceStamped, queue_size=1, tcp_nodelay=False)

    # Subscribers
    #############
    rospy.Subscriber(str(mc.mocap_pose_topic), PoseStamped, mc.pose_callback)
    rospy.Subscriber(str(mc.mocap_twist_topic), TwistStamped, mc.twist_callback)
    rospy.Subscriber(str(mc.mocap_accel_topic), AccelStamped, mc.accel_callback)

    # wait for data from the mocap
    # set up ctrl-c handler
    signal.signal(signal.SIGINT, mc.ctrl_c_handler)
    print 'waiting for mocap data'
    while not mc.received_data:
        pass

    # print the topics that are being published to
    print 'Publishing to:'
    print mc.mocap_pose_topic
    print mc.mocap_twist_topic
    print mc.mocap_accel_topic

    try:
        # Keep the node running for the callback methods
        rospy.spin()
    finally:
        # Upon termination of this script, print out a message
        print '{} node terminating.'.format(node_name)
        

if __name__ == '__main__':
    main()
