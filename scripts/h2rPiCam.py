#!/usr/bin/env python

import cv2
import numpy as np
import subprocess as sp
import time


import time
import picamera
import numpy as np
import io




import collections
class SplitFrames(object):
    def __init__(self, width, height, buffersize=50):
        self.stream = io.BytesIO()
        self.count = 0
        self.width = width
        self.height = height
        self.images = collections.deque(maxlen=buffersize)

    def write(self, buf):
        #self.stream.write(buf)

        #output = np.empty((self.width * self.height * 3,), dtype=np.uint8)
        output = np.fromstring(buf, dtype=np.uint8)
        output = output.reshape((self.height, self.width, 3))
        bgr = output[...,::-1]

        
        gray_image = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        cv2.imshow('color', bgr)
        #cv2.imshow('gray', gray_image)
        #cv2.waitKey(1)
        self.count += 1
        self.images.append((time.time(), bgr))
        

def streamPi():
    width = 320
    height = 240
    try:
        output = SplitFrames(width, height)
        with picamera.PiCamera(resolution=(width,height), framerate=120) as camera:
            time.sleep(2)
            start = time.time()
            #camera.iso = 100
            #time.sleep(2)

            #camera.shutter_speed = camera.exposure_speed
            #camera.exposure_mode = 'off'
            g = camera.awb_gains
            print "gain", g
            print "analog gain", camera.analog_gain
            print "awb", camera.awb_mode

            #camera.awb_mode = 'off'
            #camera.awb_gains = g

            from cv_bridge import CvBridge, CvBridgeError
            import rospy
            rospy.init_node('h2rPiCam', anonymous=False)
            from sensor_msgs.msg import Image
               
              
            image_pub = rospy.Publisher("/pidrone/picamera/image",Image)
            bridge = CvBridge()
            print "start recording"
            camera.start_recording(output, format='rgb')
            rate = rospy.Rate(30)
            rate.sleep()
            last_ts = None
            while not rospy.is_shutdown():
                camera.wait_recording(0)
                if len(output.images) == 0:
                    continue
                ts, image = output.images[-1]
                if ts == last_ts:
                    continue
                #cv2.imshow('color', image)
                #cv2.waitKey(1)
                image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
                image_pub.publish(image_message)
                rate.sleep()

            camera.stop_recording()
            print "stop recording"
                
    finally:
        finish = time.time()
    print('Sent %d images in %d seconds at %.2ffps' % (
        output.count, finish-start, output.count / (finish-start)))
    for ts, bgr in output.images:
        cv2.imshow('color', bgr)
        #cv2.imshow('gray', gray_image)
        cv2.waitKey(1)

                                                                                                                                                                                                                
def streamPiStill():
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
    HEIGHT = 256

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
    #stream()
    streamPi()
    #streamPiStill()


if __name__ == "__main__":
        main()
