import sys
import os
import rospy
import picamera
from sensor_msgs.msg import Image
from analyze_flow import AnalyzeFlow
from analyze_phase import AnalyzePhase
from cv_bridge import CvBridge


def main():
    ''' Initialize a ROS node and start the flow and phase analyzers '''
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # create a publisher to publish the raw image so that the web interface can
    # display the camera feed
    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)

    print "Vision started"

    try:
        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            # instantiate phase_analyzer object
            with AnalyzePhase(camera) as phase_analyzer:
                # instantiate flow_analyzer object
                with AnalyzeFlow(camera) as flow_analyzer:
                    # run the setup functions for each of the image callback classes
                    flow_analyzer.setup(camera.resolution)
                    phase_analyzer.setup()

                    # start the recordings for the image and the motion vectors
                    camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                    camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
                    # nonblocking wait
                    while not rospy.is_shutdown():
                        camera.wait_recording(1/100.0)
                        # publish the raw image
                        if phase_analyzer.previous_image is not None:
                            image_message = bridge.cv2_to_imgmsg(phase_analyzer.previous_image, encoding="bgr8")
                            image_pub.publish(image_message)
                # safely shutdown the camera recording for flow_analyzer
                camera.stop_recording(splitter_port=1)
            # safely shutdown the camera recording for phase_analyzer
            camera.stop_recording(splitter_port=2)

        print "Shutdown Received"
        sys.exit()

    except Exception as e:
        print "Camera Error!"
        raise


if __name__ == '__main__':
    main()
