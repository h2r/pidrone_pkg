from __future__ import division
import numpy as np
import cv2
from pidrone_pkg.msg import Mode
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Empty
import rospy
from cv_bridge import CvBridge
import sys
import math
from pid_class import PIDaxis
import signal

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.])


class AnalyzePhase:
    def __init__(self):

        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.infrared_callback)
        rospy.Subscriber("/pidrone/picamera/image_raw", Image, self.image_callback)

        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.bridge = CvBridge()

        self.detector = cv2.ORB(nfeatures=200, scoreType=cv2.ORB_FAST_SCORE)  # FAST_SCORE is a little faster to compute

        self.prev_img = None
        self.prev_time = None
        self.prev_rostime = None
        self.prev_obj_coordinates = None
        self.curr_obj_coordinates = None

        self.track_object = False

        self.mode = Mode()
        self.mode.mode = 5

        self.lr_pid = PIDaxis(10., 0.0, 0.0, midpoint=0, control_range=(-5.0, 5.0))
        self.fb_pid = PIDaxis(-10., 0.0, 0.0, midpoint=0, control_range=(-5.0, 5.0))

        self.alpha = 0.70
        self.z = 0.16

    def image_callback(self, data):
        curr_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        # start object tracking
        if self.track_object:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and len(curr_kp) > 0:
                kp_x = []
                kp_y = []

                for i in range(len(curr_kp)):
                    kp_x.append(curr_kp[i].pt[0])
                    kp_y.append(curr_kp[i].pt[1])

                obj_coordinates, density = find_densest_point(kp_x, kp_y)

                if self.curr_obj_coordinates is not None:
                    self.curr_obj_coordinates[0] = self.alpha*self.curr_obj_coordinates[0] + (1-self.alpha)*obj_coordinates[0]
                    self.curr_obj_coordinates[1] = self.alpha*self.curr_obj_coordinates[1] + (1-self.alpha)*obj_coordinates[1]

                    error = (np.subtract(self.curr_obj_coordinates, CAMERA_CENTER) * self.z) / 290.
                    print "ERROR: ", error

                    self.mode.x_velocity = self.lr_pid.step(error[0], curr_time - self.prev_time)
                    print "CMD VELOCITY X: ", self.mode.x_velocity
                    self.mode.y_velocity = self.fb_pid.step(error[1], curr_time-self.prev_time)
                    print "CMD VELOCITY Y: ", self.mode.y_velocity
                    self.mode.yaw_velocity = 0
                    self.pospub.publish(self.mode)

                else:
                    self.curr_obj_coordinates = obj_coordinates

                print "object coordinates: ", self.curr_obj_coordinates
                print "density: ", density

            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.prev_obj_coordinates = self.curr_obj_coordinates

    # this happens when you press r
    def reset_callback(self, data):
        if not self.track_object:
            print "Start tracking object"
        else:
            print "Stop tracking object"

        self.track_object = not self.track_object
        self.lr_pid._i = 0
        self.fb_pid._i = 0

    # this is the default mode ie velocity control
    def mode_callback(self, data):
        self.mode.mode = data.mode
        print "VELOCITY"
        # TODO scale is not consistent, check index.html and pid_class.py
        data.z_velocity = data.z_velocity * 100
        self.pospub.publish(data)

    def infrared_callback(self, data):
        self.z = data.range


def find_densest_point(x, y, r=50, max_density_point=None, max_density=None, iteration=0):

    while iteration < 2:

        points_x = x
        points_y = y
        iteration = iteration + 1
        max_density_index = None

        max_density = -float("inf")

        num_points = len(points_x)

        distances = [[0 for i in range(num_points)] for i in range(num_points)]

        for i in range(num_points):
            for j in range(i, num_points):
                d = calc_distance(points_x[i], points_y[i], points_x[j], points_y[j])
                distances[i][j] = d
                distances[j][i] = d

        for i in range(num_points):
            density = 0
            for j in range(num_points):
                density = density + math.exp((-distances[i][j] ** 2) / (r ** 2))
            if density > max_density:
                max_density = density
                max_density_index = i

        max_density_point = [points_x[max_density_index], points_y[max_density_index]]

        local_r = 50.0
        l_points_x = []
        l_points_y = []

        for i in range(num_points):
            if distances[max_density_index][i] <= local_r:
                l_points_x.append(points_x[i])
                l_points_y.append(points_y[i])

        return find_densest_point(l_points_x, l_points_y, 50, max_density_point, max_density, iteration)

    return max_density_point, max_density


def calc_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def ctrl_c_handler(signal, frame):
    print "Caught ctrl-c! About to Disarm!"
    sys.exit(0)


def main():
    rospy.init_node('object_tracking')

    signal.signal(signal.SIGINT, ctrl_c_handler)

    phase_analyzer = AnalyzePhase()
    print "Start"

    rospy.spin()

if __name__ == '__main__':
    main()
