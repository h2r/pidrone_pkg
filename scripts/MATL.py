"""""
MATL.py

Runs "MATL" for the PiDrone, MATL being "Mapping and then Localization" (as opposed to "Simultaneous Mapping and 
Localization" for SLAM). To use, run the script and press 'm' to begin mapping. The image data are saved and then run
through FastSLAM once you land and press 'm' a second time. Once the map is complete, press 'r' to start localization
and fly around!
"""""

import numpy as np
import picamera
import picamera.array
import cv2
from picam_flow_class import AnalyzeFlow
from pidrone_pkg.msg import Mode
from sensor_msgs.msg import Image, Range, CameraInfo
from std_msgs.msg import Empty
import rospy
import tf
from cv_bridge import CvBridge
import sys
from pid_class import PIDaxis
import camera_info_manager
from geometry_msgs.msg import TwistStamped
from slam_helper import FastSLAM
from MATL_helper import PROB_THRESHOLD, LocalizationParticleFilter

# ---------- camera parameters DO NOT EDIT ----------- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
# --------------------- #

# ---------- SLAM parameters ----------- #
MAX_BAD_COUNT = -1000
SLAM_PARTICLE = 40
LOCALIZATION_PARTICLE = 40
NUM_FEATURES = 50
# --------------------- #


class AnalyzePhase(picamera.array.PiMotionAnalysis):
    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.br = tf.TransformBroadcaster()

        # bind method calls to subscribed topics
        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber('/pidrone/angle', TwistStamped, self.angle_callback)
        rospy.Subscriber('/pidrone/map', Empty, self.map_callback)

        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.lr_pid = PIDaxis(10.0, 0.000, 0.0, midpoint=0, control_range=(-5.0, 5.0))
        self.fb_pid = PIDaxis(10.0, 0.000, 0.0, midpoint=0, control_range=(-5.0, 5.0))

        self.detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
        self.SLAM_estimator = FastSLAM()
        self.localization_estimator = None

        self.angle_x = 0.0
        self.angle_y = 0.0

        self.first_locate = True

        self.map_built = False
        self.build_map = False

        self.first_hold = True
        self.hold_position = False
        self.compute_map = False

        self.state = "START"

        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None

        self.prev_rostime = None
        self.pos = [0, 0, 0, 0]
        self.target_pos = [0, 0, 0, 0]
        self.z = 0.0

        self.map_counter = 0

        self.mode = Mode()
        self.mode.mode = 5

        self.map_data = []
        self.z_data = []
        self.fp = open("/home/pi/ws/src/pidrone_pkg/scripts/map.txt", 'w')

        # constant
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        # The standby state does not require kp and des
        if self.state == "STANDBY" and self.compute_map:

            # create a SLAM object and the denominator for progress percentage
            self.SLAM_estimator.generate_particles(SLAM_PARTICLE)
            denominator = len(self.map_data)

            # run stored map data through fastSLAM to produce a map
            for i in range(1, len(self.map_data)):
                self.SLAM_estimator.run(self.z_data[i-1], self.map_data[i-1][0], self.map_data[i-1][1], self.map_data[i][0],
                                        self.map_data[i][1])
                # make a nice-looking progress bar
                percent = int(round(float(i) / denominator, 2) * 100)
                sys.stdout.write("\rIncorporated %d%% of data into map!" % percent)
                sys.stdout.flush()

            # find the particle with the best map
            particle_weights = [p.weight for p in self.SLAM_estimator.particles]
            best_index = particle_weights.index(max(particle_weights))
            map = self.SLAM_estimator.particles[best_index].landmarks
            self.compute_map = False

            # initialize a localization object and sort the kp and des into grids
            map_kp = [[lm.x, lm.y] for lm in map]
            map_des = [[d for d in lm.des] for lm in map]
            self.fp.write(str(map_kp))
            self.fp.write('\n')
            self.fp.write(str(map_des))
            self.fp.write('\n')
            self.localization_estimator = LocalizationParticleFilter()
            self.localization_estimator.create_map(map_kp, map_des)

            print '\n'
            print "The map is completed with ", len(map), "landmarks!"
            print self.state
        else:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and curr_kp is not None:

                if self.state == "MAPPING":
                    self.map_data.append((curr_kp, curr_des))
                    self.z_data.append(self.z)
                elif self.state == "LOCALIZING":
                        if self.first_locate:
                            particle = self.localization_estimator.initialize_particles(LOCALIZATION_PARTICLE, curr_kp,
                                                                                        curr_des)
                            self.first_locate = False
                            self.pos = [particle.x(), particle.y(), particle.z(), particle.yaw()]
                            print 'first', particle
                            self.fp.write(str([k.pt for k in curr_kp]))
                            self.fp.write('\n')
                            self.fp.write(str([[d for d in des] for des in curr_des]))
                        else:
                            particle = self.localization_estimator.update(self.z, self.angle_x, self.angle_y,
                                                                        self.prev_kp, self.prev_des, curr_kp, curr_des)

                            # update position
                            self.pos = [self.hybrid_alpha * particle.x() + (1.0 - self.hybrid_alpha) * self.pos[0],
                                        self.hybrid_alpha * particle.y() + (1.0 - self.hybrid_alpha) * self.pos[1],
                                        self.z,
                                        self.alpha_yaw * particle.yaw() + (1.0 - self.alpha_yaw) * self.pos[3]]
                            print '--pose', self.pos[0], self.pos[1], self.pos[3]

                            # if average particle weight is close to initial weight
                            if is_almost_equal(particle.weight(), PROB_THRESHOLD):
                                self.map_counter = self.map_counter - 1
                            elif self.map_counter <= 0:
                                self.map_counter = 1
                            else:
                                self.map_counter = min(self.map_counter + 1, -MAX_BAD_COUNT)

                            # if it's been a while without a significant average weight
                            if self.map_counter < MAX_BAD_COUNT:
                                self.first_locate = True
                                self.fb_pid._i = 0
                                self.lr_pid._i = 0
                                self.map_counter = 0
                                self.mode.x_velocity = 0
                                self.mode.y_velocity = 0
                                self.mode.yaw_velocity = 0
                                self.pospub.publish(self.mode)
                                print 'Restart localization'
                            else:
                                if self.hold_position:
                                    if self.first_hold:
                                        self.target_pos = self.pos
                                        self.first_hold = False
                                        image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
                                        self.first_image_pub.publish(image_message)
                                    else:
                                        err_x = self.target_pos[0] - self.pos[0]
                                        err_y = self.target_pos[1] - self.pos[1]
                                        self.mode.x_velocity = self.lr_pid.step(err_x, curr_time - self.prev_time)
                                        self.mode.y_velocity = self.fb_pid.step(err_y, curr_time - self.prev_time)
                                        self.mode.yaw_velocity = 0
                                        self.pospub.publish(self.mode)
                                    print '--target', self.target_pos[0], self.target_pos[1], self.target_pos[3]

                            print 'count', self.map_counter

                self.prev_kp = curr_kp
                self.prev_des = curr_des

            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.pos[3]),
                              rospy.Time.now(),
                              "base",
                              "world")

    def reset_callback(self, data):
        # TODO should be able to restart localization during the flight?
        """
        start localization when '/pidrone/reset_transform' is published to (press 'r')
        """
        self.first_locate = True
        self.hold_position = False
        self.map_counter = 0

        if self.state == "LOCALIZING":
            self.state = "STANDBY"
        elif self.state == "STANDBY":
            self.state = "LOCALIZING"
        else:
            print "ILLEGAL STATE"

        print self.state

    def map_callback(self, data):
        """
        gets called when you press 'm' in the web interface (publish to /pidrone/map)
        the first time you press, it will start saving the kp and des, and the second you press, it will
        start building the map, and from then on it starts alternating
        """
        if self.state == "START":
            self.state = "MAPPING"
        elif self.state == "MAPPING":
            self.state = "STANDBY"
            self.compute_map = True
        elif self.state == "STANDBY":
            self.state = "MAPPING"
        else:
            print "ILLEGAL STATE"

        print self.state

    def toggle_callback(self, data):
        """
        toggle position hold when '/pidrone/toggle_transform' is published to (press 'p')
        """
        self.hold_position = not self.hold_position
        self.first_hold = True
        self.fb_pid._i = 0
        self.lr_pid._i = 0
        print "Position hold", "enabled." if self.hold_position else "disabled."

    def angle_callback(self, data):
        """
        updates angle data when '/pidrone/angle' is published to
        """
        self.angle_x = data.twist.angular.x
        self.angle_y = data.twist.angular.y

    def range_callback(self, data):
        """
        update z when '/pidrone/infrared' is published to
        """
        if data.range != -1:
            self.z = data.range

    def mode_callback(self, data):
        """
        update the mode whenever '/pidrone/set_mode' is published to, velocity mode is default
        """
        self.mode.mode = data.mode
        if not self.hold_position or data.mode == 4 or data.mode == 3:
            print "VELOCITY"
            data.z_velocity = data.z_velocity * 100
            self.pospub.publish(data)
        else:
            self.target_pos[0] += data.x_velocity / 100.
            self.target_pos[1] += data.y_velocity / 100.
            print "Target position", self.target_pos


def is_almost_equal(x, y):
    epsilon = 1 * 10 ** (-8)
    return abs(x - y) <= epsilon


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
