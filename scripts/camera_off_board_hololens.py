import numpy as np
import cv2
from pidrone_pkg.msg import axes_err, Mode, ERR
from sensor_msgs.msg import Image, Range, CameraInfo, CompressedImage
from std_msgs.msg import Empty, Float32MultiArray
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from pid_class import PIDaxis
from geometry_msgs.msg import TwistStamped, Point
from global_position_estimator_distance import LocalizationParticleFilter, create_map, PROB_THRESHOLD


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# keep the ratio between pixel and meter is around 1024 (lower is better) for 0.7 meters
# MAP_PIXEL_WIDTH = 1700    # in pixel
# MAP_PIXEL_HEIGHT = 1213
# MAP_REAL_WIDTH = 1.4    # in meter
# MAP_REAL_HEIGHT = 1.07
MAP_PIXEL_WIDTH = 3215
MAP_PIXEL_HEIGHT = 3600
MAP_REAL_WIDTH = 2.0
MAP_REAL_HEIGHT = 2.29
# assume a pixel in x and y has the same length
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
MAX_BAD_COUNT = -50
NUM_PARTICLE = 50
INIZ_Z = 0.45
CELL_DISTANCE = 0.5

class AnalyzePhase:

    def __init__(self):
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        # self.lr_pid = PIDaxis(11.0, 0.001, 0.001, midpoint=0, control_range=(-5.0, 5.0))
        # self.fb_pid = PIDaxis(11.0, 0.001, 0.001, midpoint=0, control_range=(-5.0, 5.0))
        self.lr_pid = PIDaxis(12.0, 0.002, 0.0015, midpoint=0, control_range=(-5.0, 5.0))
        self.fb_pid = PIDaxis(12.0, 0.002, 0.0015, midpoint=0, control_range=(-5.0, 5.0))

        self.detector = cv2.ORB(nfeatures=200, scoreType=cv2.ORB_FAST_SCORE)  # FAST_SCORE is a little faster to compute
        map_grid_kp, map_grid_des = create_map('big_map.jpg')
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
        self.target_pos = [0, 0, INIZ_Z]
        self.target_point = Point()
        self.target_yaw = 0.0
        self.map_counter = 0
        self.max_map_counter = 0
        self.mode = Mode()
        self.mode.mode = 5
        # constant
        self.kp_yaw = 50.0
        self.ki_yaw = 0.1
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.35  # blend position with first frame and int
        # angle
        self.angle_x = 0.0  # the hz of state_controller is different
        self.angle_y = 0.0
        # TODO: current z position is controlled by state_controller
        self.increment_z = 0
        # path from MDP
        self.path = []
        self.path_index = -1

        rospy.Subscriber("/pidrone/set_mode", Mode, self.mode_callback)
        rospy.Subscriber('/pidrone/path', Float32MultiArray, self.path_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber('/pidrone/angle', TwistStamped, self.angle_callback)
        rospy.Subscriber('/pidrone/picamera/image_raw', Image, self.image_callback)
        self.pospub = rospy.Publisher('/pidrone/set_mode_vel', Mode, queue_size=1)
        self.target_pos_pub = rospy.Publisher('/pidrone/drone_position', Point, queue_size=1)
        # self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)
        self.captured_image_pub = rospy.Publisher("/pidrone/picamera/captured_image", CompressedImage, queue_size=1, latch=True)

    def image_callback(self, data):
        curr_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        curr_rostime = rospy.Time.now()
        curr_time = curr_rostime.to_sec()

        # process path from MDP
        if len(self.path) != 0 and is_close_to(self.pos, self.target_pos, distance=0.2):
            if self.path[self.path_index + 1] == -1:   # take photo
                if is_close_to(self.pos, self.target_pos, distance=0.1):
                    # image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
                    # self.first_image_pub.publish(image_message)
                    image_message = self.bridge.cv2_to_compressed_imgmsg(curr_img)
                    self.captured_image_pub.publish(image_message)
                    self.path_index += 1
            else:                                      # take action
                if self.increment_z == 0:
                    self.target_pos[0] += self.path[self.path_index + 1]
                    self.target_pos[1] += self.path[self.path_index + 2]
                    self.increment_z = self.path[self.path_index + 3] * 100.
                    self.target_pos[2] += self.path[self.path_index + 3]
                    self.path_index += 3
            if self.path_index == len(self.path) - 1:  # reset path
                self.path = []
                self.path_index = -1

        # start MC localization
        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and curr_kp is not None:
                # generate particles for the first time
                if self.first_locate:
                    particle = self.estimator.initialize_particles(NUM_PARTICLE, curr_kp, curr_des)
                    self.first_locate = False
                    self.pos = particle.position[0:3]
                    self.pos[1] += 0.05  # camera offset
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
                    print '--pose', str(round(self.pos[0], 3)), str(round(self.pos[1], 3)), str(round(self.pos[2], 3)), self.yaw

                    # if all particles are not good estimations
                    if is_almost_equal(particle.weight, PROB_THRESHOLD):
                        self.map_counter = self.map_counter - 1
                    elif self.map_counter <= 0:
                        self.map_counter = 1
                    else:
                        self.map_counter = min(self.map_counter + 1, -MAX_BAD_COUNT)

                    # if no particles are good estimations, we should restart
                    if self.map_counter < MAX_BAD_COUNT:
                        self.first_locate = True
                        self.fb_pid._i = 0
                        self.lr_pid._i = 0
                        self.iacc_yaw = 0.0
                        self.map_counter = 0
                        self.mode.x_velocity = 0
                        self.mode.y_velocity = 0
                        self.mode.z_velocity = 0
                        self.mode.yaw_velocity = 0
                        self.pospub.publish(self.mode)
                        print 'Restart localization'
                    else:
                        if self.hold_position:
                            if self.first_hold:
                                self.target_pos[0] = (int)(self.pos[0] / CELL_DISTANCE) * CELL_DISTANCE + CELL_DISTANCE / 2.
                                self.target_pos[1] = (int)(self.pos[1] / CELL_DISTANCE) * CELL_DISTANCE + CELL_DISTANCE / 2. 
                                self.target_pos[2] = INIZ_Z
                                self.target_yaw = 0  # rotate is not implement
                                self.first_hold = False
                                # image_message = self.bridge.cv2_to_imgmsg(curr_img, encoding="bgr8")
                                # self.first_image_pub.publish(image_message)
                            else:
                                err_x = self.target_pos[0] - self.pos[0]
                                err_y = self.target_pos[1] - self.pos[1]
                                self.mode.x_velocity = self.lr_pid.step(err_x, curr_time - self.prev_time)
                                self.mode.y_velocity = self.fb_pid.step(err_y, curr_time - self.prev_time)
                                self.mode.z_velocity = self.increment_z
                                self.increment_z = 0
                                err_yaw = self.target_yaw - self.yaw
                                self.iacc_yaw += err_yaw * self.ki_yaw
                                self.mode.yaw_velocity = err_yaw * self.kp_yaw + self.iacc_yaw
                                self.pospub.publish(self.mode)
                            self.target_point.x = self.target_pos[0]
                            self.target_point.y = self.target_pos[1]
                            self.target_point.z = self.target_pos[2]
                            self.target_pos_pub.publish(self.target_point)
                            print '--target', self.target_pos[0], self.target_pos[1], self.target_pos[2], self.target_yaw

                    print 'count', self.map_counter
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

            self.prev_kp = curr_kp
            self.prev_des = curr_des

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
            data.z_velocity = data.z_velocity * 100.
            self.pospub.publish(data)
        else:
            self.target_pos[0] += data.x_velocity / 100.
            self.target_pos[1] += data.y_velocity / 100.
            # TODO scale is not consistent, check index.html and pid_class.py
            if self.increment_z == 0:
                self.target_pos[2] += data.z_velocity
                self.increment_z = data.z_velocity * 100.
            print "Target position", self.target_pos

    def path_callback(self, data):
        self.path = data.data
        print(self.path)

def is_almost_equal(x,y, epsilon=1*10**(-8)):
    return abs(x-y) <= epsilon

def is_close_to(current_pos, target_pos, distance=0.1):
    return distance > abs(current_pos[0] - target_pos[0]) + abs(current_pos[1] - target_pos[1]) + abs(current_pos[2] - target_pos[2])    

def main():
    rospy.init_node('camera_off_board')

    phase_analyzer = AnalyzePhase()
    print "Start"

    rospy.spin()


if __name__ == '__main__':
    main()