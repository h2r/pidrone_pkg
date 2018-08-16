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


def main():
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
                camera_transmitter = CameraTransmitter(camera, bridge)

                camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                print "Starting Flow"
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