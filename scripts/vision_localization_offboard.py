"""
offboard_vision

Run this file for SLAM or localization offboard (run it on the pi)
"""


import numpy as np
import picamera
import picamera.array
from analyze_flow import AnalyzeFlow
from sensor_msgs.msg import Image, Range, CameraInfo
import rospy
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import camera_info_manager
import argparse
import cv2
from std_msgs.msg import Empty

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240


class CameraTransmitter(picamera.array.PiMotionAnalysis):

    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.prev_img = None
        self.prev_time = None
        self.prev_rostime = None

    def write(self, data):
        self.prev_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        self.prev_rostime = rospy.Time.now()
        self.prev_time = self.prev_rostime.to_sec()


class DataWriter(picamera.array.PiMotionAnalysis):
    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge

        self.detector = cv2.ORB(nfeatures=20, scoreType=cv2.ORB_FAST_SCORE)
        
        self.z = 0.0

        self.prev_img = None
        self.prev_time = None
        self.prev_rostime = None

        self.data = []
        self.lock = True

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()
        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime

        kp, des = self.detector.detectAndCompute(curr_img, None)

        if not self.lock:
            self.data.append([[k.pt for k in kp], des, self.z])

    def ir_callback(self, data):
        self.z = data.range

    def put_to_file(self, data):
        if self.lock:
            print "unlocking"
            self.lock = False
        else:
            print self.data[0:5]
            self.lock = True
            self.fp = open('flight_data.txt', "w")
            self.fp.write(str(self.data))
            self.fp.close()
            print "closed file"


def main():
    parser = argparse.ArgumentParser(description='Run SLAM or localization offboard')

    parser.add_argument('--offline', action='store_true',
                        help=('Writes flight data to a text file'))
    args = parser.parse_args()

    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")

    try:
        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
            with AnalyzeFlow(camera) as flow_analyzer:
                flow_analyzer.setup(camera.resolution)
                camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                print "Starting Flow"

                if args.offline:
                    phase_analyzer = DataWriter(camera, bridge)

                    rospy.Subscriber('/pidrone/infrared', Range, phase_analyzer.ir_callback)
                    rospy.Subscriber('/pidrone/reset_transform', Empty, phase_analyzer.put_to_file)

                    camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
                    last_time = None
                    while not rospy.is_shutdown():
                        camera.wait_recording(1 / 40.0)

                        if phase_analyzer.prev_img is not None and phase_analyzer.prev_time != last_time:
                            image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
                            image_message.header.stamp = phase_analyzer.prev_rostime
                            # print "stamp", image_message.header.stamp
                            last_time = phase_analyzer.prev_rostime
                            image_pub.publish(image_message)
                            camera_info_pub.publish(cim.getCameraInfo())
                else:
                    camera_transmitter = CameraTransmitter(camera, bridge)
                    camera.start_recording(camera_transmitter, format='bgr', splitter_port=2)
                    last_time = None
                    while not rospy.is_shutdown():
                        camera.wait_recording(1 / 40.0)

                        if camera_transmitter.prev_img is not None and camera_transmitter.prev_time != last_time:
                            image_message = bridge.cv2_to_imgmsg(camera_transmitter.prev_img, encoding="bgr8")
                            image_message.header.stamp = camera_transmitter.prev_rostime
                            # print "stamp", image_message.header.stamp
                            last_time = camera_transmitter.prev_rostime
                            image_pub.publish(image_message)
                            camera_info_pub.publish(cim.getCameraInfo())
                    
                camera.stop_recording(splitter_port=1)
                camera.stop_recording(splitter_port=2)
        print "Shutdown Received"
    except Exception:
        print "Camera Error!!"
        raise


if __name__ == '__main__':
    main()