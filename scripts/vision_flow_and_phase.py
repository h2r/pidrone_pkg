import sys
import rospy
import picamera
from analyze_twist import AnalyzeTwist
from analyze_pose import AnalyzePose

def main():
    ''' Start a ros node and start the twist and pose analyzers '''
    rospy.init_node('camera_controller')
    print 'Vision started'
    try:
        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with AnalyzePose(camera) as pose_analyzer:
                with AnalyzeTwist(camera) as twist_analyzer:
                    # run the setup functions for each of the image callback classes
                    twist_analyzer.setup(camera.resolution)
                    pose_analyzer.setup(camera.resolution)
                    # start the recordings for the image and the motion vectors
                    camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=twist_analyzer)
                    camera.start_recording(pose_analyzer, format='bgr', splitter_port=2)
                     # nonblocking wait
                    while not rospy.is_shutdown(): camera.wait_recording(1/100.0)

                camera.stop_recording(splitter_port=1)  # stop recording both the flow
            camera.stop_recording(splitter_port=2)      # and the images

        print "Shutdown Recieved"
        sys.exit()

    except Exception as e:
        raise

if __name__ == '__main__':
    main()
