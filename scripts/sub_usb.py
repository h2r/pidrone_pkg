#!/usr/bin/env python
import subprocess as sp
import cv2
import numpy as np
import time
import math
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Imu
import rospy
import tf

WIDTH = 480
HEIGHT = 360

est_pos = PoseStamped()
est_pos.pose.position.y = 1

summed_transform = [0, 1, 0, 0] # x, y, z, yaw

orientation = [0, 0, 0, 1]

video_capture = cv2.VideoCapture(0)
video_capture.set(3, WIDTH)
video_capture.set(4, HEIGHT)
video_capture.set(4, HEIGHT)


# rotate vector v1 by quaternion q1 
def qv_mult(q1, v1):
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )

def affineToTransform(affine, summed_transform):
    transformation = [0, 0, 0, 0]
    if affine is not None:
        scalex = np.linalg.norm(affine[:, 0])
        scalez = np.linalg.norm(affine[:, 1])
        # print 'scalex {} \t scalez {} \t diff {}'.format(scalex,scalez, scalex
        # - scalez)
        # offset camera center -> translate affected by scale
        t_offsetx = WIDTH*(1-scalex)/(2*scalex)
        t_offsetz = HEIGHT*(1-scalez)/(2*scalez)
        # calc translation 
        transformation[1] = 1/((scalex + scalez) / 2)
        transformation[0] = int(affine[0, 2] - t_offsetx) * summed_transform[1] / WIDTH
        transformation[2] = int(affine[1, 2] - t_offsetz) * summed_transform[1] / HEIGHT
        # calc rotation
        affine[:, 0] /= scalex
        affine[:, 1] /= scalez
        transformation[3] = math.atan2(affine[1, 0], affine[0, 0])
        # account for camera tilt
        thing = qv_mult(orientation, [0, 0, summed_transform[1]])
        summed_transform[0] += transformation[0]
        summed_transform[1] *= transformation[1]
        # print transformation[0]
        summed_transform[2] += transformation[2]
        summed_transform[3] += transformation[3]
        # print(thing[1], thing[0])
    else:
        print("skipped")

def imucall(data):
    global orientation
    orientation = [data.orientation.x, data.orientation.y, data.orientation.z,
    data.orientation.w]

if '__name__ == r__main__':
    rospy.init_node('sub')
    imu = rospy.Subscriber('/pidrone/imu', Imu, imucall)
    cmdpub = rospy.Publisher('/pidrone/est_pos', PoseStamped, queue_size=1)
    curr = None
    prev = None
    init = None
    first = True
    i = 0
    while True:
        t = time.time()
        ret, frame = video_capture.read()
        # print 'Video capture:\t', time.time() - t
        if ret:
            t = time.time()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # print 'Color convert:\t', time.time() - t
            if first:
                init = frame
                first = False
            else:
                curr = frame
            if prev is not None and curr is not None:
                t = time.time()
                affine = cv2.estimateRigidTransform(prev, curr, False)
                # print 'estimateRigidTransform:\t', time.time() - t
                t = time.time()
                affineToTransform(affine, summed_transform)
                # print 'affineToTransform', time.time() - t
#           cv2.imshow('prev', prev)
#           cv2.imshow('curr', curr)
#           cv2.waitKey(0)
            i += 1
            prev = curr
            est_pos.pose.position.x = summed_transform[0]
            est_pos.pose.position.y = summed_transform[1]
            est_pos.pose.position.z = summed_transform[2]
            rotation = tf.transformations.quaternion_from_euler(0, 0,
            summed_transform[3])
#       est_pos.orientation.x += rotation[0]
#       est_pos.orientation.y += rotation[1]
#       est_pos.orientation.z += rotation[2]
            est_pos.pose.orientation.w = 1
            print(est_pos)
            cmdpub.publish(est_pos)

