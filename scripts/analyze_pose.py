import cv2
import rospy
import picamera
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
from three_dim_vec import ZeroOrderState
from geometry_msgs.msg import PoseWithCovarianceStamped


class AnalyzePos(picamera.array.PiMotionAnalysis):

    def setup(self, camera_wh):
        self.position_control = False
        self.has_first = False
        self.first_image = None
        self.previous_image = None
        # current position
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # initial z value used for motion calculations
        self.z = 0.075
        # value used for complimentary filter blending
        self.alpha = 0.1
        # counter for the number of consecutive images that contain the first image
        self.first_counter = 0
        # pose message
        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.frame_id = 'Body'

        # ROS Setup
        ###########
        # Publisher
        self.pose_pub = rospy.Publisher('/pidrone/pose', PoseWithCovarianceStamped, queue_size=1)
        # Subscribers
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)
        rospy.Subscriber("/pidrone/command/javascript", Mode, self.javascript_callback)

    def write(self, data):

        if self.position_control:
            image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))

            if not self.has_first:
                self.has_first = True
                self.first_image = image
                self.previous_image = image
                print "Capture the first image"
            else:
                transform_first = cv2.estimateRigidTransform(self.first_image, image, False)
                ### delta_time = current_time - self.previous_time

                # first image lost or not
                if transform_first is not None:
                    translation_first, yaw_first = self.translation_and_yaw(transform_first)
                    self.x = -((1 - self.alpha) * self.x + self.alpha * translation_first[0])
                    self.y = (1 - self.alpha) * self.y + self.alpha * translation_first[1]
                    self.yaw = (1 - self.alpha) * self.yaw + self.alpha * yaw_first
                    ### self.mode_to_pub.x_velocity = self.pid_first_x.step(self.target_x - self.x, delta_time)
                    ### self.mode_to_pub.y_velocity = self.pid_first_y.step(self.target_y - self.y, delta_time)
                    ### self.mode_to_pub.yaw_velocity = self.pid_yaw.step(self.target_yaw - self.yaw, delta_time)
                    self.counter += 1
                    print "count:", self.counter
                else:
                    transform_previous = cv2.estimateRigidTransform(self.previous_image, image, False)

                    # previous image lost or not
                    if transform_previous is not None:
                        translation_previous, yaw_previous = self.translation_and_yaw(transform_previous)
                        self.x += -(translation_previous[0])
                        self.y += translation_previous[1]
                        self.yaw += yaw_previous
                        print "Lost the first image !"
                    else:
                    ###    self.mode_to_pub.x_velocity = 0
                    ###    self.mode_to_pub.y_velocity = 0
                    ###    self.mode_to_pub.yaw_velocity = 0
                        print "Lost the previous image !!!!!"

                    self.counter = 0

                self.pose_msg.pose.pose.x = self.x
                self.pose_msg.pose.pose.y = self.y
                self.pose_msg.pose.pose.z = self.z
                # convert the yaw to a quaternion
                q = tf.transformations.quaternion_from_euler(0,0,self.yaw)
                # this only requires the z and w fields of the quaternion
                self.pose_msg.pose.orientation.z = q[2]
                self.pose_msg.pose.orientation.w = q[3]
                self.posepub.publish(self.pose)

            self.previous_image = image

            #print self.x, self.y, self.yaw


    # normalize image
    def translation_and_yaw(self, transform):
        translation_x_y = [float(transform[0, 2]) / 320 * self.z, float(transform[1, 2]) / 240 * self.z]

        yaw_scale = np.sqrt(transform[0, 0]**2 + transform[1, 0]**2)
        yaw_y_x = [float(transform[1, 0]) / yaw_scale, float(transform[0, 0]) / yaw_scale]
        yaw = np.arctan2(yaw_y_x[0], yaw_y_x[1])

        return translation_x_y, yaw

    # subscribe /pidrone/infrared
    def range_callback(self, data):
        if data.range != -1:
            # convert and store the range reading in meters
            self.z = data.range * 100

    # subscribe /pidrone/reset_transform
    def reset_callback(self, data):
        self.has_first = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.counter = 0
        print "Reset Transform"

    # subscribe /pidrone/toggle_transform
    def toggle_callback(self, data):
        self.position_control = not self.position_control
        print "Position Control", self.position_control
