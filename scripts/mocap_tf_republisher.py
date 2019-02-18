#!/usr/bin/python
import rospy
import tf
import os
import sys
import signal
import numpy as np

def transform_drone_axes(translation, rotation):
    broadcaster = tf.TransformBroadcaster()
    new_x = translation[0]
    new_y = -translation[2]
    new_z = translation[1]
    new_rot = [rotation[0], -rotation[2], rotation[1], rotation[3]]
    # Quaternion that rotates about the x axis
    #ninety_deg_rot_about_y = tf.transformations.quaternion_about_axis(-np.pi/2.0, (0, 1, 0))
    #ninety_deg_rot_about_x = tf.transformations.quaternion_about_axis(-np.pi/2.0, (1, 0, 0))
    #intermediate_rot = tf.transformations.quaternion_multiply(ninety_deg_rot_about_y, rotation)
    #new_rot = tf.transformations.quaternion_multiply(ninety_deg_rot_about_y, rotation)

    broadcaster.sendTransform((new_x, new_y, new_z),
                              #translation,
                              #rotation,
                              new_rot,
                              rospy.Time.now(),
                              "irosdrone_z_up_frame",
                              "world")

def ctrl_c_handler(signal, frame):
    print "\nCaught Ctrl-C. Stopping node."
    sys.exit()

def main():
    # Initialize the mocap republisher node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    signal.signal(signal.SIGINT, ctrl_c_handler)

    listener = tf.TransformListener()

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (translation, rotation) = listener.lookupTransform("world", "irosdrone", rospy.Time(0))
            transform_drone_axes(translation, rotation)
        except tf.LookupException as e:
            continue

if __name__ == '__main__':
    main()
