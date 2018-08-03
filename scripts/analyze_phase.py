import tf
import cv2
import rospy
import picamera
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
from pidrone_pkg.msg import Phase


class AnalyzePhase(picamera.array.PiMotionAnalysis):
    ''' A class that uses OpenCV's estimateRigidTransform method to calculate
    the change in position of the drone.
    For more info, visit:
    https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform
    '''

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
        # phase message
        self.phase_msg = Phase()
        self.phase_msg.header.frame_id = 'Body'

        # ROS Setup
        ###########
        # Publisher
        self.phasepub = rospy.Publisher('/picamera/phase', Phase, queue_size=1)
        # Subscribers
        rospy.Subscriber("/pidrone/infrared", Range, self.range_callback)
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Empty, self.toggle_callback)

    def write(self, data):
        ''' A method that is called everytime an image is taken '''

        if self.position_control:
            image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))

            if not self.has_first:
                self.has_first = True
                self.first_image = image
                self.previous_image = image
                print "\nCapture the first image"
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
                        self.reset_callback(data)
                        self.counter = 0

            self.previous_image = image

        self.publish_phase()

            #print self.x, self.y, self.yaw


    # normalize image
    def translation_and_yaw(self, transform):
        translation_x_y = [float(transform[0, 2]) / 320 * self.z, float(transform[1, 2]) / 240 * self.z]

        yaw_scale = np.sqrt(transform[0, 0]**2 + transform[1, 0]**2)
        yaw_y_x = [float(transform[1, 0]) / yaw_scale, float(transform[0, 0]) / yaw_scale]
        yaw = np.arctan2(yaw_y_x[0], yaw_y_x[1])

        return translation_x_y, yaw

    # publish to /picamera/phase
    def publish_phase(self):
        ''' Publish the phase values '''
        self.phase_msg.header.stamp = rospy.Time.now()
        self.phase_msg.x_position = self.x
        self.phase_msg.y_position = self.y
        # convert z back to meters
        self.phase_msg.z_position = self.z / 100
        self.phase_msg.yaw_angle = self.yaw
        # convert the yaw to a quaternion
        self.phasepub.publish(self.phase_msg)

    # subscribe /pidrone/infrared
    def range_callback(self, data):
        ''' Store the range value '''
        if data.range != -1:
            # convert and store the range reading in cm
            self.z = data.range * 100

    # subscribe /pidrone/reset_transform
    def reset_callback(self, data):
        ''' Reset the current phase '''
        self.has_first = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.counter = 0
        print "Reset Transform"

    # subscribe /pidrone/toggle_transform
    def toggle_callback(self, data):
        ''' Toggle whether the phase is calculated and published '''
        self.position_control = not self.position_control
        print "Position Control", self.position_control
