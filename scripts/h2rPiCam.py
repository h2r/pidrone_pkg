#!/usr/bin/env python

import cv2
import numpy as np
import subprocess as sp
import time


def stream():
    WIDTH = 320
    HEIGHT = 240

    raspividcmd = ['raspivid', '-fps', '20', '-t', '0', '-w', str(WIDTH), '-h',
                   str(HEIGHT), '-r', '-', '--raw-format', 'yuv', '-o', '/dev/null', '-n','-qp', '10', 
                   '-pf', 'baseline', '-drc', 'off', '-ex', 'fixedfps', '-fl', '-awb', 'fluorescent', '-ifx', 'none', '-md', '7']
    print "cmd: ", " ".join(raspividcmd)

    stream = None


    try:
        stream = sp.Popen(raspividcmd, stdout = sp.PIPE, universal_newlines = True)
        while stream.returncode == None:
            #test = stream.stdout.read(WIDTH * HEIGHT + (WIDTH * HEIGHT / 2))[0:WIDTH * HEIGHT]
            test = stream.stdout.read(WIDTH * HEIGHT * 3)

            curr = np.fromstring(test, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
            print "shape", curr.shape
            cv2.imshow('curr', curr[0:WIDTH*HEIGHT])
            cv2.waitKey(1)
            stream.poll()
            
    finally:
        if stream != None:
            stream.terminate()
            stream.kill()



def main():

    stream()


if __name__ == "__main__":
        main()
