import sys
import rospy
import picamera
from analyze_flow import AnalyzeFlow
from analyze_pose import AnalyzePose

def main():
    rospy.init_node('camera_controller')
    try:
        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with AnalyzePose(camera) as pose_analyzer:
                with AnalyzeFlow(camera) as flow_analyzer:
                    # run the setup functions for each of the image callback classes
                    flow_analyzer.setup(camera.resolution)
                    pose_analyzer.setup(camera.resolution)
                    # start the recordings for the image and the motion vectors
                    camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                    camera.start_recording(pos_analyzer, format='bgr', splitter_port=2)
                     # nonblocking wait
                    while not rospy.is_shutdown(): camera.wait_recording(1/100.0)

            camera.stop_recording(splitter_port=1)  # stop recording both the flow
            camera.stop_recording(splitter_port=2)  # and the images

        print "Shutdown Recieved"
        sys.exit()

    except Exception as e:
        raise

if __name__ == '__main__':
    main()
