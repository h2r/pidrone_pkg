import numpy as np
import picamera
import picamera.array
import cv2
import rospy
import time
import sys
from h2rMultiWii import MultiWii
from picam_flow_class import AnalyzeFlow
from pidrone_pkg.msg import axes_err
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

if __name__ == '__main__':
    rospy.init_node('flow_pub')
    velpub= rospy.Publisher('/pidrone/plane_err', axes_err, queue_size=1)
    imgpub = rospy.Publisher("/pidrone/images", Image, queue_size=1)
    try:
        velocity = axes_err()
        bridge = CvBridge()
        with picamera.PiCamera(framerate=90) as camera:
            with AnalyzeFlow(camera) as flow_analyzer:
                camera.resolution = (320, 240)
                flow_analyzer.setup(camera.resolution)
                camera.start_recording("/dev/null", format='h264', motion_output=flow_analyzer)

                while not rospy.is_shutdown():
                    image = np.empty((240 * 320 * 3,), dtype=np.uint8)
                    camera.capture(image, 'bgr', use_video_port=True)
                    image = image.reshape((240, 320, 3))
                    cv_image = bridge.cv2_to_imgmsg(image, "bgr8")
                    imgpub.publish(cv_image)
                    velocity.x.err = flow_analyzer.x_motion 
                    velocity.y.err = flow_analyzer.y_motion
                    velocity.z.err = flow_analyzer.z_motion
                    camera.wait_recording(1/100.0)
                    velpub.publish(velocity)

                camera.stop_recording()
        print "Shutdown Recieved"
        sys.exit()
    except Exception as e:
        raise 
