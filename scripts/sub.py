#!/usr/bin/env python
import subprocess as sp
import cv2
import numpy as np
import time
import math
from geometry_msgs.msg import Pose, PoseStamped
import rospy
import tf

WIDTH = 320
HEIGHT = 240

est_pos = Pose()
est_pos.position.y = 1

summed_transform = [0, 1, 0, 0] # x, y, z, yaw

def affineToTransform(affine, summed_transform):
    transformation = [0, 0, 0, 0]
    if affine is not None:
        scalex = np.linalg.norm(affine[:, 0])
        scalez = np.linalg.norm(affine[:, 1])
        # offset camera center -> translate affected by scale
        t_offsetx = WIDTH*(1-scalex)/2
        t_offsetz = HEIGHT*(1-scalez)/2
        # calc translation 
        transformation[1] = 1/((scalex + scalez) / 2)
        transformation[0] = int(affine[0, 2] - t_offsetx) * summed_transform[1]
        transformation[2] = int(affine[1, 2] - t_offsetz) * summed_transform[1]
        # calc rotation
        affine[:, 0] /= scalex
        affine[:, 1] /= scalez
        transformation[3] = math.atan2(affine[1, 0], affine[0, 0])
        summed_transform[0] += transformation[0]
        summed_transform[1] *= transformation[1]
        summed_transform[2] += transformation[2]
        summed_transform[3] += transformation[3]
    else:
        print("skipped")

if __name__ == '__main__':
    cmdpub = rospy.Publisher('/pidrone/est_pos', Pose, queue_size=1)
    rospy.init_node('rigid_transform', anonymous=True)
    raspividcmd = ['raspivid', '-fps', '20', '-t', '0', '-w', str(WIDTH), '-h',
    str(HEIGHT), '-r', '-', '--raw-format', 'yuv', '-o', '/dev/null', '-n', '-pf', 'baseline']
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
            cv2.imshow('curr', curr)
            cv2.waitKey(1)
            affine = cv2.estimateRigidTransform(prev, curr, False)
            affineToTransform(affine, summed_transform)
#       i += 1
        prev = curr
        est_pos.position.x = summed_transform[0]
        est_pos.position.y = summed_transform[1]
        est_pos.position.z = summed_transform[2]
        rotation = tf.transformations.quaternion_from_euler(0, 0,
        summed_transform[3])
        est_pos.orientation.x += rotation[0]
        est_pos.orientation.y += rotation[1]
        est_pos.orientation.z += rotation[2]
        est_pos.orientation.w = 1
#       print(est_pos)
        cmdpub.publish(est_pos)

