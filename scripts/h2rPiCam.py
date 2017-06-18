#!/usr/bin/env python

import cv2
import numpy as np
import subprocess as sp
import time


import time
import picamera
import numpy as np


def streamPi():
    print "making cameraa"
    with picamera.PiCamera() as camera:
        while True:
            camera.resolution = (100, 100)
            camera.framerate = 24
            output = np.empty((112 * 128 * 3,), dtype=np.uint8)
            camera.capture(output, 'rgb')
            output = output.reshape((112, 128, 3))
            output = output[:100, :100, :]
            cv2.imshow('curr', output)
            cv2.waitKey(1)



def stream():
    WIDTH = 320
    HEIGHT = 240

    raspividcmd = ['raspivid', '-fps', '20', '-t', '0', '-w', str(WIDTH), '-h',
                   str(HEIGHT), '-r', '-', '--raw-format', 'gray', '-o', '/dev/null', '-n',
                   '-drc', 'off', '-ex', 'fixedfps', '-fl', '-awb', 'auto', '-ifx', 'none', '-md', '7', '-mm', 'average',]
    print "cmd: ", " ".join(raspividcmd)

    stream = None


    try:
        stream = sp.Popen(raspividcmd, stdout = sp.PIPE, universal_newlines = True)
        while stream.returncode == None:
            
            #test = stream.stdout.read(WIDTH * HEIGHT + (WIDTH * HEIGHT / 2))[0:WIDTH * HEIGHT]
            #test = stream.stdout.read(WIDTH * HEIGHT * 3)
            test = stream.stdout.read(WIDTH * HEIGHT)
            
            print "test: %d '%s'" % (len(test), test[0])
            curr = np.fromstring(test, dtype=np.uint8).reshape(HEIGHT, WIDTH)
            print "curr: ", curr[0]
            cv2.imshow('curr', curr[0:WIDTH*HEIGHT])
            cv2.waitKey(1)
            stream.poll()
            
    finally:
        if stream != None:
            stream.terminate()
            stream.kill()



def main():
    stream()
    #streamPi()


if __name__ == "__main__":
        main()
