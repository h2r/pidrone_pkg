import numpy as np
import picamera
import picamera.array
import cv2
from picam_flow_class import AnalyzeFlow
from pidrone_pkg.msg import axes_err, Mode, ERR
from sensor_msgs.msg import Image, Range, CameraInfo
from std_msgs.msg import Empty
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
import sys
from pid_class import PIDaxis
import camera_info_manager
import math
from geometry_msgs.msg import Twist


MIN_MATCH_COUNT = 10
MAX_MISSING_COUNT = 20
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
MAP_WIDTH = 1024    # in pixel
MAP_HEIGHT = 768
MAP_REAL_WIDTH = 0.7    # in meter
MAP_REAL_HEIGHT = 0.54
METER_TO_PIXEL = (MAP_WIDTH / MAP_REAL_WIDTH + MAP_HEIGHT / MAP_REAL_HEIGHT) / 2.
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)


class AnalyzePhase(picamera.array.PiMotionAnalysis):
    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.br = tf.TransformBroadcaster()

        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber('/pidrone/angle', Twist, self.angle_callback)
        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.lr_pid = PIDaxis(10.0, 0.000, 1.0, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(10.0, 0.000, 1.0, midpoint=0, control_range=(-10.0, 10.0))

        self.detector = cv2.ORB(nfeatures=120, scoreType=cv2.ORB_FAST_SCORE)  # FAST_SCORE is a little faster to compute
        map_imgs = [cv2.imread('map1.jpg'), cv2.imread('map2.jpg'), cv2.imread('map3.jpg'), cv2.imread('map4.jpg')]
        map_detector = cv2.ORB(nfeatures=500, scoreType=cv2.ORB_FAST_SCORE)
        # TODO use particle filter, so we can use more features which makes GridAdaptedFeatureDetector work better
        # map_detector = cv2.GridAdaptedFeatureDetector(self.detector, maxTotalKeypoints=500)  # find features evenly
        self.map_index = 0
        self.map_features = []
        # save features for each map in an array
        for map_img in map_imgs:
            map_kp = map_detector.detect(map_img, None)
            map_kp, map_des = self.detector.compute(map_img, map_kp)
            if map_kp is None or map_des is None:
                print "Map cannot be detect and compute !"
                sys.exit()
            else:
                self.map_features.append((map_kp, map_des))
        self.map_kp, self.map_des = self.map_features[0]
        self.prev_img = None
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
        self.prev_kp = None
        self.prev_des = None
        self.first = True
        self.prev_time = None
        self.prev_rostime = None
        self.pos = [0, 0]
        self.z = 0.075
        self.angle_x = 0.0  # the hz of state_controller is different
        self.angle_y = 0.0
        self.smoothed_yaw = 0.0
        self.yaw_observed = 0.0
        self.iacc_yaw = 0.0
        self.transforming = False
        self.target_pos = [0, 0]
        self.map_counter = 0
        self.max_map_counter = 0
        self.map_missing_counter = 0
        self.mode = Mode()
        self.mode.mode = 5
        # constant
        self.kp_yaw = 100.0
        self.ki_yaw = 0.1
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.2  # blend position with first frame and int

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        if self.transforming:
            # cannot find matched features in current map, then change map
            if self.map_missing_counter > MAX_MISSING_COUNT:
                if self.map_index == len(self.map_features) - 1:
                    self.map_index = 0
                else:
                    self.map_index += 1
                self.map_kp, self.map_des = self.map_features[self.map_index]
                print 'change to map', self.map_index

            if self.first:
                pos, yaw = self.feature_locate(curr_kp, curr_des)
                if pos is not None and yaw is not None:
                    self.map_missing_counter = 0

                    self.pos = pos
                    self.yaw_observed = yaw
                    self.smoothed_yaw = yaw
                    self.target_pos = pos

                    self.first = False
                    image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
                    self.first_image_pub.publish(image_message)
                    print "start and set pose", pos
                else:
                    self.map_missing_counter += 1
                    print 'start or reset failed'
            else:
                pos, yaw = self.feature_locate(curr_kp, curr_des)

                if pos is not None and yaw is not None:
                    self.map_counter += 1
                    self.max_map_counter = max(self.map_counter, self.max_map_counter)
                    self.map_missing_counter = 0

                    self.pos = [self.hybrid_alpha * pos[0] + (1.0 - self.hybrid_alpha) * self.pos[0],
                                self.hybrid_alpha * pos[1] + (1.0 - self.hybrid_alpha) * self.pos[1]]
                    err_x = self.target_pos[0] - self.pos[0]
                    err_y = self.target_pos[1] - self.pos[1]
                    self.mode.x_velocity = self.lr_pid.step(err_x, curr_time - self.prev_time)
                    self.mode.y_velocity = self.fb_pid.step(err_y, curr_time - self.prev_time)

                    self.yaw_observed = -yaw    # take the opposite
                    self.smoothed_yaw = (1.0 - self.alpha_yaw) * self.smoothed_yaw + self.alpha_yaw * self.yaw_observed
                    yaw = self.smoothed_yaw
                    self.iacc_yaw += yaw * self.ki_yaw
                    self.mode.yaw_velocity = yaw * self.kp_yaw + self.iacc_yaw

                    self.pospub.publish(self.mode)
                    print "MAP", self.max_map_counter, self.map_counter, "\nPOS", self.pos, "\nTARGET", self.target_pos
                else:
                    # TODO or hover ?
                    self.map_counter = 0
                    self.map_missing_counter += 1

                    err_x = self.target_pos[0] - self.pos[0]
                    err_y = self.target_pos[1] - self.pos[1]
                    self.mode.x_velocity = self.lr_pid.step(err_x, curr_time - self.prev_time) / 10.  # less power
                    self.mode.y_velocity = self.fb_pid.step(err_y, curr_time - self.prev_time) / 10.  # less power

                    yaw = self.smoothed_yaw
                    self.iacc_yaw += yaw * self.ki_yaw
                    self.mode.yaw_velocity = (yaw * self.kp_yaw + self.iacc_yaw) / 10.  # less power

                    print "LOST !", "\nPOS", self.pos, "\nTARGET", self.target_pos

        self.prev_img = curr_img
        self.prev_kp = curr_kp
        self.prev_des = curr_des
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.yaw_observed),
                              rospy.Time.now(),
                              "base",
                              "world")

    def feature_transform(self, kp1, des1, kp2, des2):
        transform = None

        if des1 is not None and des2 is not None:
            matches = self.matcher.knnMatch(des1, des2, k=2)

            good = []
            for match in matches:
                if len(match) > 1 and match[0].distance < 0.7 * match[1].distance:
                    good.append(match[0])

            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            # estimateRigidTransform needs at least three pairs
            if src_pts is not None and dst_pts is not None and len(src_pts) > 3 and len(dst_pts) > 3:
                transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)

        return transform

    def feature_locate(self, kp, des):
        """
        find features from current image,
        match features with map,
        compute the location.
        :param kp: train kp
        :param des: train des
        :return: global x, y, and yaw
        """
        pos = None
        yaw = None

        if des is not None:
            matches = self.matcher.knnMatch(des, self.map_des, k=2)

            # take from opencv tutorial
            good = []
            for match in matches:
                if len(match) > 1 and match[0].distance < 0.7 * match[1].distance:
                    good.append(match[0])

            if len(good) > MIN_MATCH_COUNT:
                src_pts = np.float32([kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([self.map_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)

                if transform is not None:
                    transformed_center = cv2.transform(CAMERA_CENTER, transform)    # get local pixel
                    transformed_center = [transformed_center[0][0][0] / float(METER_TO_PIXEL),  # map to local pose
                                          (MAP_HEIGHT - transformed_center[0][0][1]) / float(METER_TO_PIXEL)]
                    transformed_center = self.local_to_global(transformed_center)  # get the global pose
                    yaw = np.arctan2(transform[1, 0], transform[0, 0])  # get global heading

                    # correct the pose if the drone is not level
                    z = math.sqrt(self.z ** 2 / (1 + math.tan(self.angle_x) ** 2 + math.tan(self.angle_y) ** 2))
                    offset_x = np.tan(self.angle_x) * z
                    offset_y = np.tan(self.angle_y) * z
                    global_offset_x = math.cos(yaw) * offset_x + math.sin(yaw) * offset_y
                    global_offset_y = math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
                    pos = [transformed_center[0] + global_offset_x, transformed_center[1] + global_offset_y]
            # else:
            #     print "Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT)

        return pos, yaw

    def local_to_global(self, pos):
        if self.map_index == 1:
            pos[1] += MAP_REAL_HEIGHT
        elif self.map_index == 2:
            pos[0] += MAP_REAL_WIDTH
            pos[1] += MAP_REAL_HEIGHT
        elif self.map_index == 3:
            pos[0] += MAP_REAL_WIDTH

        return pos

    def angle_callback(self, data):
        self.angle_x = data.angular.x
        self.angle_y = data.angular.y

    def range_callback(self, data):
        if data.range != -1:
            self.z = data.range

    def reset_callback(self, data):
        print "Resetting position"
        self.first = True
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        self.iacc_yaw = 0.0
        self.map_counter = 0
        self.max_map_counter = 0
        self.map_missing_counter = 0
        self.map_index = 0
        self.map_kp, self.map_des = self.map_features[0]

    def toggle_callback(self, data):
        self.transforming = not self.transforming
        print "Position hold", "enabled." if self.transforming else "disabled."

    def mode_callback(self, data):
        if not self.transforming or data.mode == 4 or data.mode == 3:
            print "VELOCITY"
            # TODO scale is not consistent, check index.html and pid_class.py
            data.z_velocity = data.z_velocity * 100
            self.pospub.publish(data)
        else:
            self.target_pos[0] += data.x_velocity / 100.
            self.target_pos[1] += data.y_velocity / 100.
            print "POSITION", self.target_pos


def main():
    rospy.init_node('flow_pub')

    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")

    try:
        velocity = axes_err()
        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
            with AnalyzeFlow(camera) as flow_analyzer:
                flow_analyzer.setup(camera.resolution)
                phase_analyzer = AnalyzePhase(camera, bridge)

                camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                print "Starting Flow"
                camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
                last_time = None
                while not rospy.is_shutdown():
                    camera.wait_recording(1 / 100.0)

                    if phase_analyzer.prev_img is not None and phase_analyzer.prev_time != last_time:
                        image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
                        image_message.header.stamp = phase_analyzer.prev_rostime
                        # print "stamp", image_message.header.stamp
                        last_time = phase_analyzer.prev_rostime
                        image_pub.publish(image_message)
                        camera_info_pub.publish(cim.getCameraInfo())

                camera.stop_recording(splitter_port=1)
                camera.stop_recording(splitter_port=2)
        print "Shutdown Received"
        sys.exit()
    except Exception as e:
        raise


if __name__ == '__main__':
    main()
