import subprocess as sp
import cv2
import numpy as np
import time
import math

summed_transform = [0, 1, 0, 0] # x, y, z, yaw

def affineToTransform(affine, summed_transform):
    transformation = [0, 0, 0, 0]
    if affine is not None:
        scalex = np.linalg.norm(affine[:, 0])
        scalez = np.linalg.norm(affine[:, 1])
        # offset camera center -> translate affected by scale
        t_offsetx = 640*(1-scalex)/2
        t_offsetz = 480*(1-scalez)/2
        # calc translation 
        transformation[1] = 1/((scalex + scalez) / 2)
        transformation[0] = int(affine[0, 2] - t_offsetx) #* summed_transform.translation.y
        transformation[2] = int(affine[1, 2] - t_offsetz) #* summed_transform.translation.y
        # calc rotation
        affine[:, 0] /= scalex
        affine[:, 1] /= scalez
        transformation[3] = math.atan2(affine[1, 0], affine[0, 0])
        summed_transform[0] += transformation[0]
        summed_transform[1] *= transformation[1]
        summed_transform[2] += transformation[2]
        summed_transform[3] += transformation[3]

raspividcmd = ['raspivid', '-fps', '30', '-t', '0', '-w', '640', '-h', '480',
'--raw-format', 'rgb', '-r', '-', '-o', '/dev/null']
stream = sp.Popen(raspividcmd, stdout = sp.PIPE, universal_newlines = True)
i = 0
time.sleep(1)
curr = None
prev = None
while True:
    test = stream.stdout.read(640 * 480)
    stream.stdout.read(640 * 240)
    curr = np.fromstring(test, dtype=np.uint8).reshape(480, 640)
    if prev is not None and curr is not None:
        affine = cv2.estimateRigidTransform(prev, curr, False)
        affineToTransform(affine, summed_transform)
    else:
        print(curr, prev)
    prev = curr
    print(summed_transform)
