#!/usr/bin/env python
from __future__ import division
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped, Transform
from sensor_msgs.msg import Image
import time
import tf
import math
import numpy as np
from copy import deepcopy
import cv2
from cv_bridge import CvBridge

first = True
first_img = None
transform = None
prev = None
ref_set = []
summed_transformation = Transform()
summed_transformation.translation.y = 1

cmdpub = rospy.Publisher('/pidrone/error', Transform, queue_size=1)

def compareToRefSet(image):
    new_img = bridge.imgmsg_to_cv2(image, "bgr8")
    for r in ref_set:
        affine = cv2.estimateRigidTransform(r['img'], new_img, False)
        transform = affineToTransform(affine)
        # offset the transform by the tranform of the ref image
        for i in range(5):
            transform[i] += (r['tf'])[i]

def affineToTransform(affine, summed_transform):
    transformation = Transform()
    if affine is not None:
        scalex = np.linalg.norm(affine[:, 0])
        scalez = np.linalg.norm(affine[:, 1])
        # calc translation 
        transformation.translation.y = 1/((scalex + scalez) / 2)
        transformation.translation.x = affine[0, 2]#* summed_transform.translation.y
        transformation.translation.z = affine[1, 2]#* summed_transform.translation.y
        # calc rotation
        affine[:, 0] /= scalex
        affine[:, 1] /= scalez
        yaw = math.atan2(affine[1, 0], affine[0, 0])
        rotation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        transformation.rotation.x = rotation[0]
        transformation.rotation.y = rotation[1]
        transformation.rotation.z = rotation[2]
        transformation.rotation.w = rotation[3]
        return transformation
    else: return None

def transformToDist(tf):
    return math.sqrt(tf.translation.x^2 + tf.translation.y^2 + tf.translation.z^2)

def callback(image):
    bridge = CvBridge()
    global first
    if first:
        print("firsted")
        global first_image
        first_img = bridge.imgmsg_to_cv2(image, "bgr8")
        global prev
        prev = first_img
        global first
        ref_set.append({'img': image, 'tf': [0,0,0,0,0]})
        first = False
    else:
        transformation = Transform()
        global first_img
        global prev
        curr = bridge.imgmsg_to_cv2(image, "bgr8")
        #new = first_img[:, 60:300]
        test1 = deepcopy(first_img)
        test2 = deepcopy(first_img)
        test1 = test1[30:209, 40:279]
        test2 = test2[35:214, 40:279]
        affine = cv2.estimateRigidTransform(test1, test2, False)
        print(affine)
        if affine is not None:
            transform = affineToTransform(affine, summed_transformation)
# Summing
            summed_transformation.translation.x += transform.translation.x
            summed_transformation.translation.y *= transform.translation.y
            summed_transformation.translation.z += transform.translation.z
            summed_transformation.rotation.x += transform.rotation.x
            summed_transformation.rotation.y += transform.rotation.y
            summed_transformation.rotation.z += transform.rotation.z
            summed_transformation.rotation.w += transform.rotation.w
            #print(summed_transformation)
            cmdpub.publish(summed_transformation)
            global prev
            prev = curr
        else:
            print("skipped")

if __name__ == '__main__':
    rospy.init_node('rigid_transform', anonymous=True)
    try:
        print("started")
        rospy.Subscriber("/camera/image_raw", Image, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
