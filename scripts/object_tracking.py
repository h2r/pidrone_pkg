from __future__ import division
import cv2
from pidrone_pkg.msg import State
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from analyze_flow import AnalyzeFlow
from cv_bridge import CvBridge
import rospy
import picamera
import picamera.array
import numpy as np

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.])


class ObjectTracker(picamera.array.PiMotionAnalysis):
    def __init__(self, camera):
        picamera.array.PiMotionAnalysis.__init__(self, camera)

        self.obj_pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)

        self.detector = cv2.ORB(nfeatures=150, scoreType=cv2.ORB_FAST_SCORE)

        self.curr_obj_coordinates = None
        self.error = None
        self.track_object = False

        self.prev_img = None

        self.alpha = 0.70
        self.z = 0.16

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))

        # start object tracking
        if self.track_object:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and len(curr_kp) > 0:
                kp_x = []
                kp_y = []

                for i in range(len(curr_kp)):
                    kp_x.append(curr_kp[i].pt[0])
                    kp_y.append(curr_kp[i].pt[1])

                obj_coordinates, density = find_densest_point(np.array(kp_x), np.array(kp_y))

                if self.curr_obj_coordinates is not None:
                    self.curr_obj_coordinates = self.alpha*self.curr_obj_coordinates + (1-self.alpha)*obj_coordinates
                else:
                    self.curr_obj_coordinates = obj_coordinates

                self.error = (np.subtract(self.curr_obj_coordinates, CAMERA_CENTER) * self.z) / 290. * np.array((1, -1))
                print "ERROR: ", self.error
                print "density: ", density

                obj_pose = Pose()
                obj_pose.position.x = self.error[0]
                obj_pose.position.y = self.error[1]
                self.obj_pose_pub.publish(obj_pose)
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

        self.prev_img = curr_img

    def reset_callback(self, data):
        if not self.track_object:
            print "Start tracking object"
        else:
            print "Stop tracking object"

        self.track_object = not self.track_object
        self.curr_obj_coordinates = None

    def state_callback(self, data):
        self.z = data.pose_with_covariance.pose.position.z


def find_densest_point(x, y, r=50, max_density_point=None, max_density=None, iteration=0):

    while iteration < 2:

        points_x = x
        points_y = y
        iteration += 1

        num_points = len(points_x)

        z = np.array([complex(points_x[i], points_y[i]) for i in range(num_points)])
        distances = abs(z[..., np.newaxis]-z)

        densities = np.exp((-distances ** 2) / (r**2)).sum(axis=1)
        max_density_index = densities.argmax()
        max_density_point = np.array([points_x[max_density_index], points_y[max_density_index]])
        max_density = densities.max()

        local_r = 50.0

        mask = distances[max_density_index] <= local_r
        l_points_x = points_x[mask]
        l_points_y = points_y[mask]

        return find_densest_point(l_points_x, l_points_y, 50, max_density_point, max_density, iteration)

    return max_density_point, max_density


def main():
    rospy.init_node("object_tracking")

    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)

    try:

        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with ObjectTracker(camera) as tracker:

                rospy.Subscriber("/pidrone/reset_transform", Empty, tracker.reset_callback)
                rospy.Subscriber("/pidrone/state", State, tracker.state_callback)

                with AnalyzeFlow(camera) as flow_analyzer:
                    # run the setup functions for each of the image callback classes
                    flow_analyzer.setup(camera.resolution)

                    # start the recordings for the image and the motion vectors
                    camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                    camera.start_recording(tracker, format='bgr', splitter_port=2)

                    while not rospy.is_shutdown():
                        camera.wait_recording(1 / 100.0)

                        if tracker.prev_img is not None:
                            image_message = bridge.cv2_to_imgmsg(tracker.prev_img, encoding="bgr8")
                            image_pub.publish(image_message)

                camera.stop_recording(splitter_port=1)
            camera.stop_recording(splitter_port=2)
        print "Shutdown Received"
    except Exception:
        print "Camera Error!!"
        raise


if __name__ == "__main__":
    main()
