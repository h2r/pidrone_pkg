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
import io
import picamera
import calendar

first = True
first_img = None
transform = None
prev = None
ref_set = []
est_pos = PoseStamped()
est_pos.position.y = 1

cmdpub = rospy.Publisher('/pidrone/est_pos', PoseStamped, queue_size=1)

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
        # offset camera center -> translate affected by scale
        t_offsetx = 320*(1-scalex)/2
        t_offsetz = 240*(1-scalez)/2
        # calc translation 
        transformation.translation.y = 1/((scalex + scalez) / 2)
        transformation.translation.x = int(affine[0, 2] - t_offsetx) #* summed_transform.translation.y
        transformation.translation.z = int(affine[1, 2] - t_offsetz) #* summed_transform.translation.y
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
    now = rospy.get_time()
    print now - image.header.stamp.secs + ((10 ** -9) * image.header.stamp.nsecs)
    bridge = CvBridge() # IZZY: Why do you re-init this every time
    curr = bridge.imgmsg_to_cv2(image, "bgr8")
    global first
    if first:
        print("firsted")
        first_img = curr
        global prev
        prev = first_img
        global first
        ref_set.append({'img': image, 'tf': [0,0,0,0,0]})
        first = False
    else:
        transformation = Transform()
        global first_img
        global prev
        test1 = deepcopy(first_img)
        test2 = deepcopy(first_img)
        test2 = test2[30:210,40:280]
        test2 = cv2.resize(test2, (320, 240))
#       cv2.waitKey(0)
        affine = cv2.estimateRigidTransform(prev, curr, False)
        if affine is not None:
            transform = affineToTransform(affine, est_pos)
            #print transform
# Summing
            est_pos.pose.position.x += transform.translation.x
            est_pos.pose.position.y *= transform.translation.y
            est_pos.pose.position.z += transform.translation.z
#           est_pos.pose.orientation.x += transform.rotation.x
#           est_pos.pose.orientation.y += transform.rotation.y
#           est_pos.pose.orientation.z += transform.rotation.z
#           est_pos.pose.orientation.w += transform.rotation.w
            est_pos.pose.orientation.x = 0
            est_pos.pose.orientation.y = 0
            est_pos.pose.orientation.z = 0
            est_pos.pose.orientation.w = 1
            # print(est_pos)
            cmdpub.publish(est_pos)
            global prev
            prev = curr
        else:
            print("skipped")

if __name__ == '__main__':
    rospy.init_node('rigid_transform', anonymous=True)
    try:
        print("started")
        stream = io.BytesIO()
        prev = None
        curr = None
        i = 0
        with picamera.PiCamera() as camera:
            stream = io.BytesIO()
            camera.start_preview()
            time.sleep(2)
            print(rospy.get_time())
            for i, filename in enumerate(camera.capture_continuous(stream,
                format='jpeg', use_video_port=True)):
                stream.truncate()
                data = np.fromstring(stream.getvalue(), dtype=np.uint8)
                prev = curr
                curr = cv2.imdecode(data, 1)
                stream.seek(0)
                if i == 59:
                    break
            print(rospy.get_time())
            camera.stop_preview()
        #rospy.Subscriber("/camera/image_raw", Image, callback)
        #rospy.spin()


    except rospy.ROSInterruptException:
        pass
