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
from geometry_msgs.msg import TwistStamped
from global_position_estimator_distance import LocalizationParticleFilter, create_map, PROB_THRESHOLD


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# keep the ratio between pixel and meter is around 1024 (lower is better) for 0.7 meters
MAP_PIXEL_WIDTH = 2048    # in pixel
MAP_PIXEL_HEIGHT = 1616
MAP_REAL_WIDTH = 1.4    # in meter
MAP_REAL_HEIGHT = 1.07
# assume a pixel in x and y has the same length
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
MAX_BAD_COUNT = -10
NUM_PARTICLE = 30


class AnalyzePhase(picamera.array.PiMotionAnalysis):
    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.br = tf.TransformBroadcaster()

        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber('/pidrone/angle', TwistStamped, self.angle_callback)
        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.lr_pid = PIDaxis(10.0, 0.000, 1.0, midpoint=0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDaxis(10.0, 0.000, 1.0, midpoint=0, control_range=(-10.0, 10.0))

        self.detector = cv2.ORB(nfeatures=100, scoreType=cv2.ORB_FAST_SCORE)  # FAST_SCORE is a little faster to compute
        map_grid_kp, map_grid_des = create_map('map.jpg')
        self.estimator = LocalizationParticleFilter(map_grid_kp, map_grid_des)

        self.first_locate = True
        self.first_hold = True
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.locate_position = False
        self.prev_time = None
        self.prev_rostime = None
        self.pos = [0, 0, 0]
        self.yaw = 0.0
        self.z = 0.075
        self.iacc_yaw = 0.0
        self.hold_position = False
        self.target_pos = [0, 0, 0]
        self.target_yaw = 0.0
        self.map_counter = 0
        self.max_map_counter = 0
        self.mode = Mode()
        self.mode.mode = 5
        # constant
        self.kp_yaw = 70.0
        self.ki_yaw = 0.1
        self.alpha_yaw = 0.5  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.5  # blend position with first frame and int
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.2  # blend position with first frame and int
        # angle
        self.angle_x = 0.0  # the hz of state_controller is different
        self.angle_y = 0.0

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        # start MCL localization
        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and curr_kp is not None:
                # generate particles for the first time
                if self.first_locate:
                    particle = self.estimator.initialize_particles(NUM_PARTICLE, curr_kp, curr_des)
                    self.first_locate = False
                    self.pos = particle.position[0:3]
                    self.yaw = particle.position[3]
                    print 'first', particle
                else:
                    # get a estimate velocity over time
                    particle = self.estimator.update(self.z, self.angle_x, self.angle_y, self.prev_kp, self.prev_des,
                                                     curr_kp, curr_des)

                    # update position
                    self.pos = [self.hybrid_alpha * particle.position[0] + (1.0 - self.hybrid_alpha) * self.pos[0],
                                self.hybrid_alpha * particle.position[1] + (1.0 - self.hybrid_alpha) * self.pos[1],
                                self.z]
                    self.yaw = self.alpha_yaw * particle.position[3] + (1.0 - self.alpha_yaw) * self.yaw
                    # print 'particle', particle
                    print 'pose', self.pos, self.yaw

                    # if all particles are not good estimations
                    if is_almost_equal(particle.weight, PROB_THRESHOLD):
                        self.map_counter = self.map_counter - 1
                    elif self.map_counter <= 0:
                        self.map_counter = 1
                    else:
                        self.map_counter = min(self.map_counter + 1, -MAX_BAD_COUNT)
                    print 'count', self.map_counter

                    # if no particles are good estimations, we should restart
                    if self.map_counter < MAX_BAD_COUNT:
                        self.first_locate = True
                        self.map_counter = 0
                        print 'Restart localization'
                    else:
                        if self.hold_position:
                            if self.first_hold:
                                self.target_pos = self.pos
                                self.target_yaw = 0  # rotate is not implement
                                self.first_hold = False
                                image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
                                self.first_image_pub.publish(image_message)
                            else:
                                err_x = self.target_pos[0] - self.pos[0]
                                err_y = self.target_pos[1] - self.pos[1]
                                self.mode.x_velocity = self.lr_pid.step(err_x, curr_time - self.prev_time)
                                self.mode.y_velocity = self.fb_pid.step(err_y, curr_time - self.prev_time)
                                err_yaw = self.target_yaw - self.yaw
                                self.iacc_yaw += err_yaw * self.ki_yaw
                                self.mode.yaw_velocity = err_yaw * self.kp_yaw + self.iacc_yaw
                                self.pospub.publish(self.mode)
                            print 'target', self.target_pos, self.target_yaw
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

            self.prev_kp = curr_kp
            self.prev_des = curr_des

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                              rospy.Time.now(),
                              "base",
                              "world")

    # the angle is just estimate
    def angle_callback(self, data):
        self.angle_x = data.twist.angular.x
        self.angle_y = data.twist.angular.y

    def range_callback(self, data):
        if data.range != -1:
            self.z = data.range

    def reset_callback(self, data):
        print "Start localization"
        self.locate_position = True
        self.first_locate = True
        self.hold_position = False
        self.map_counter = 0
        self.max_map_counter = 0

    def toggle_callback(self, data):
        self.hold_position = not self.hold_position
        self.first_hold = True
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        self.iacc_yaw = 0.0
        print "Position hold", "enabled." if self.hold_position else "disabled."

    def mode_callback(self, data):
        if not self.hold_position or data.mode == 4 or data.mode == 3:
            print "VELOCITY"
            # TODO scale is not consistent, check index.html and pid_class.py
            data.z_velocity = data.z_velocity * 100
            self.pospub.publish(data)
        else:
            self.target_pos[0] += data.x_velocity / 100.
            self.target_pos[1] += data.y_velocity / 100.
            print "Target position", self.target_pos


def is_almost_equal(x,y, epsilon=1*10**(-8)):
    return abs(x-y) <= epsilon


def main():
    rospy.init_node('localization')

    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")

    try:
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
