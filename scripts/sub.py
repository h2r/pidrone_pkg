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
from os import ftruncate

WIDTH = 320
HEIGHT = 240

est_pos = PoseStamped()
est_pos.header.frame_id='world'
est_pos.pose.position.y = 1

summed_transform = [0, 0, 1, 0] # x, y, z, yaw

orientation = [0, 0, 0, 1]

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
        # print(affine)
        scalex = np.linalg.norm(affine[:, 0])
        scaley = np.linalg.norm(affine[:, 1])
        # print 'scalex {} \t scaley {} \t diff {}'.format(scalex,scaley, scalex
        # - scaley)
        # offset camera center -> translate affected by scale
        t_offsetx = WIDTH*(1-scalex)/(2*scalex)
        t_offsety = HEIGHT*(1-scaley)/(2*scaley)
        # calc translation 
        transformation[0] = int(affine[0, 2] - t_offsetx) * summed_transform[2]
        transformation[1] = int(affine[1, 2] - t_offsety) * summed_transform[2]
        transformation[2] = 1/((scalex + scaley) / 2)
        # calc rotation
        affine[:, 0] /= scalex
        affine[:, 1] /= scaley
        transformation[3] = math.atan2(affine[1, 0], affine[0, 0])
        # account for camera tilt
        thing = qv_mult(orientation, [0, 0, summed_transform[2]])
        summed_transform[0] += transformation[0]
        summed_transform[1] += transformation[1]
        # print transformation[0]
        summed_transform[2] *= transformation[2]
        summed_transform[3] += transformation[3]
        # print(thing[1], thing[0])
    else:
        print("skipped")

def imucall(data):
    global orientation
    orientation = [data.orientation.x, data.orientation.y, data.orientation.z,
    data.orientation.w]

if __name__ == '__main__':
    imu = rospy.Subscriber('/pidrone/imu', Imu, imucall)
    cmdpub = rospy.Publisher('/pidrone/est_pos', PoseStamped, queue_size=1)
    rospy.init_node('rigid_transform', anonymous=True)
    raspividcmd = ['raspivid', '-fps', '20', '-t', '0', '-w', str(WIDTH), '-h',
    str(HEIGHT), '-r', '-', '--raw-format', 'yuv', '-o', '/dev/null', '-n',
    '-pf', 'baseline', '-drc', 'off', '-ex', 'fixedfps', '-fl']
    stream = sp.Popen(raspividcmd, stdout = sp.PIPE, universal_newlines = True)
#   i = 0
    time.sleep(1)
    curr = None
    prev = None
#   init = None
#   first = True
#   stream.stdout.read(WIDTH * HEIGHT * 3) # EATING
    while True:
        test = stream.stdout.read(WIDTH * HEIGHT + (WIDTH * HEIGHT / 2))[0:WIDTH * HEIGHT]
        # test = stream.stdout.read(WIDTH * HEIGHT * 3)[WIDTH*HEIGHT * 3/2:WIDTH * HEIGHT * 5/2]
#       if first:
#           init = np.fromstring(test, dtype=np.uint8).reshape(HEIGHT, WIDTH)
#           first = False
#       else:
        curr = np.fromstring(test, dtype=np.uint8).reshape(HEIGHT, WIDTH)
        if prev is not None and curr is not None:
#           cv2.imshow('curr', curr)
#           cv2.waitKey(1)
            affine = cv2.estimateRigidTransform(prev, curr, False)
            affineToTransform(affine, summed_transform)
#       i += 1
        prev = curr
        est_pos.pose.position.x = summed_transform[0]
        est_pos.pose.position.y = summed_transform[1]
        est_pos.pose.position.z = summed_transform[2]
        est_pos.pose.orientation.x = 0
        est_pos.pose.orientation.y = 0
        est_pos.pose.orientation.z = 0
        est_pos.pose.orientation.w = summed_transform[3]
        cmdpub.publish(est_pos)
