from __future__ import division
import cv2
from pidrone_pkg.msg import State
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Pose
import rospy
import math
from cv_bridge import CvBridge


class ObjectTracker:
    def __init__(self):
        self.bridge = CvBridge()

        self.obj_pos_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)

        self.detector = cv2.ORB(nfeatures=200, scoreType=cv2.ORB_FAST_SCORE)

        self.curr_obj_coordinates = None

        self.track_object = False

        self.alpha = 0.70
        self.z = 0.16

    def image_callback(self, data):
        curr_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

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
                else:
                    self.curr_obj_coordinates = obj_coordinates

                self.curr_obj_coordinates = self.curr_obj_coordinates * self.z / 290.
                print "object coordinates: ", self.curr_obj_coordinates
                print "density: ", density

                obj_pose = Pose()
                obj_pose.position.x = self.curr_obj_coordinates[0]
                obj_pose.position.y = self.curr_obj_coordinates[1]
                self.obj_pose_pub.publish(obj_pose)
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

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


def main():
    rospy.init_node("object_tracking")
    tracker = ObjectTracker()
    rospy.Subscriber("/pidrone/reset_transform", Empty, tracker.reset_callback)
    rospy.Subscriber("/pidrone/picamera/image_raw", Image, tracker.image_callback)
    rospy.Subscriber("/pidrone/state", State, tracker.state_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
