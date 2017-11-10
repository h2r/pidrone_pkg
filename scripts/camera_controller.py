import picamera
import rospy
import sys
from picam_flow_class import AnalyzeFlow
from picam_pos_class import AnalyzePos
            
def main():
    rospy.init_node('camera_controller')                 
    try:
        flow_msg = axes_err()
        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with AnalyzePos(camera) as pos_analyzer:
                with AnalyzeFlow(camera) as flow_analyzer:
                    # run the setup functions for each of the image callback classes
                    flow_analyzer.setup(camera.resolution)
                    pos_analyzer.setup(camera.resolution)
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
