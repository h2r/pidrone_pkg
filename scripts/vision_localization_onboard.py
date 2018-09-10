#!/usr/bin/env python
"""
vision_localization.py

This file can run SLAM or localization on board, offline or online
"""


from onboard_localization import *
from onboard_slam import *
from MATL import *
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import camera_info_manager
import rospy
from sensor_msgs.msg import Image, Range, CameraInfo
from analyze_flow import AnalyzeFlow
from std_msgs.msg import Empty
from pidrone_pkg.msg import State
import argparse


def main():
    parser = argparse.ArgumentParser(description='Estimate the drone\'s pose with SLAM or localization')
    # Arguments to determine if the throttle command is being used. E.g.:
    #   rosrun topic_tools throttle messages /pidrone/infrared 40.0
    parser.add_argument('--SLAM', action='store_true',
                        help=('Do you want to do SLAM?'))
    parser.add_argument('--offline', action='store_true',
                        help=('Run offline?'))
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

                if args.SLAM:
                    if args.offline:
                        phase_analyzer = MATL(camera, bridge)
                        rospy.Subscriber('/pidrone/map', Empty, phase_analyzer.map_callback)
                    else:
                        phase_analyzer = SLAM(camera, bridge)
                else:
                    phase_analyzer = Localizer(camera, bridge)

                rospy.Subscriber('/pidrone/reset_transform', Empty, phase_analyzer.reset_callback)
                rospy.Subscriber('/pidrone/state', State, phase_analyzer.state_callback)

                camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                print "Starting Flow"
                camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
                last_time = None
                while not rospy.is_shutdown():
                    camera.wait_recording(1 / 100.0)

                    if phase_analyzer.prev_img is not None and phase_analyzer.prev_time != last_time:
                        image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
                        image_message.header.stamp = phase_analyzer.prev_rostime
                        last_time = phase_analyzer.prev_rostime
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
