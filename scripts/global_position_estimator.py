import rospy

# first, refactor flow_pub_transform so that it computes the features
# explicitly, and then calls cv2.estimateRigidTransform using those
# features.  Verify position hold still works.  It should be faster. 
#

# second, compute the local pose for each feature.  Camera frame.  For
# each feature, you'll have an x,y,z in the camera frame (where the
# camera is zero, zero,zero).  Verify it's right by publishing the
# pose as a ROS marker array.  Visualize it in rviz.

# third, make a feature map.  So given the global true pose of the
# robot from the mocap, store the global x,y,z location of each
# feature by transforming above to the global frame using the mocap.

# fourth, write out the math and agree how it will work.

# publish the features in the camera frame in ros

# this node subscribes and does an update given the map and each feature.

# this node publishes a TF transform from /map to /base_link

# Use meters (not centimers) and radians.  


def main():
    rospy.init_node('global_position_estimator')

if __name__ == '__main__':
    main()
