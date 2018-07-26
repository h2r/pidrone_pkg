import numpy as np
import picamera
import picamera.array
import cv2
import rospy
import time
import sys
from h2rMultiWii import MultiWii
from picam_flow import AnalyzeFlow
from pidrone_pkg.msg import axes_err

if __name__ == '__main__':
    rospy.init_node('velocity_flight_flow')
    velpub= rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)
    try:
        velocity = axes_err()
        with picamera.PiCamera(framerate=90) as camera:
            with AnalyzeFlow(camera) as flow_analyzer:
                camera.resolution = (320, 240)
                flow_analyzer.setup(camera.resolution)
                camera.start_recording(
                      '/dev/null', format='h264', motion_output=flow_analyzer)

                while not rospy.is_shutdown():
                    velocity.x.err = flow_analyzer.x_motion 
                    velocity.y.err = flow_analyzer.y_motion
                    velocity.z.err = flow_analyzer.z_motion
                    velpub.publish(velocity)

                # camera.wait_recording(30)
                camera.stop_recording()
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise 
