import tf
import cv2
import rospy
import picamera
import numpy as np
from sensor_msgs.msg import Image
from pidrone_pkg.msg import State
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import PoseStamped


class AnalyzePhase(picamera.array.PiMotionAnalysis):
    """
    A class that uses OpenCV's estimateRigidTransform method to calculate
    the change in position of the drone.
    For more info, visit:
    https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform

    Publisher:
    /pidrone/picamera/pose
    /pidrone/picamera/lost

    Subscribers:
    /pidrone/reset_transform
    /pidrone/position_control
    """

    def setup(self):

        # initialize the Pose data
        self.pose_msg = PoseStamped()
        self.altitude = 0.0
        self.x_position_from_state = 0.0
        self.y_position_from_state = 0.0

        # position hold is initialized as False
        self.position_control = False
        self.first_image = None
        self.previous_image = None

        # first image vars
        self.first = True
        self.new_first = True

        # ROS Setup
        ###########
        # Publisher
        self.posepub = rospy.Publisher('/pidrone/picamera/pose', PoseStamped, queue_size=1)

        # Subscribers
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/position_control", Bool, self.position_control_callback)
        rospy.Subscriber("/pidrone/state", State, self.state_callback)

    def write(self, data):
        ''' A method that is called everytime an image is taken '''
        image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
        if self.first_image is None:
            self.first_image = image
            self.previous_image = image
            self.new_first = True
        else:
            self.new_first = False
            # try to estimate the transformations from the first image
            transform_first = cv2.estimateRigidTransform(self.first_image, image, True)
            if transform_first is not None:
                # calculate the x,y and translations, and yaw rotation from the transformation
                translation_first, yaw_first = self.translation_and_yaw(transform_first)
                self.pose_msg.pose.position.x = translation_first[0]*self.altitude
                self.pose_msg.pose.position.y = translation_first[1]*self.altitude
                self.publish_pose(yaw_first)
            else:
                # try to estimate the transformation from the previous image
                transform_previous = cv2.estimateRigidTransform(self.previous_image, image, True)
                # if the previous image was visible (the transformation was succesful)
                # calculate the position by adding the displacement from the previous image
                # to the current position estimate
                if transform_previous is not None:
                    int_displacement, yaw_previous = self.translation_and_yaw(transform_previous)
                    self.pose_msg.pose.position.x = self.x_position_from_state + (int_displacement[0]*self.altitude)
                    self.pose_msg.pose.position.y = self.y_position_from_state + (int_displacement[1]*self.altitude)
                    self.publish_pose(yaw_previous)
            self.previous_image = image

    # normalize image
    def translation_and_yaw(self, transform):
        translation_x_y = [0 - float(transform[0, 2]) / 320,
                            float(transform[1, 2]) / 240]

        # yaw can be up to ~ 20 deg
        yaw_scale = np.sqrt(transform[0, 0]**2 + transform[1, 0]**2)
        yaw_y_x = [float(transform[1, 0]) / yaw_scale, float(transform[0, 0]) / yaw_scale]
        yaw = np.arctan2(yaw_y_x[0], yaw_y_x[1])

        return translation_x_y, yaw

    # subscribe /pidrone/reset_transform
    def reset_callback(self, msg):
        """ Reset the current position and orientation """
        print("Resetting Phase")
        # reset position control variables
        self.first_image = None
        # reset the pose values
        self.pose_msg = PoseStamped()

    # subscribe /pidrone/position_control
    def position_control_callback(self, msg):
        ''' Set whether the pose is calculated and published '''
        self.position_control = msg.data

    def state_callback(self, msg):
        """
        Store z position (altitude) reading from State, along with most recent
        x and y position estimate
        """
        self.altitude = msg.pose_with_covariance.pose.position.z
        self.x_position_from_state = msg.pose_with_covariance.pose.position.x
        self.y_position_from_state = msg.pose_with_covariance.pose.position.y

    def publish_pose(self, yaw):
        _,_,z,w = tf.transformations.quaternion_from_euler(0,0,yaw)
        self.pose_msg.pose.orientation.z = z
        self.pose_msg.pose.orientation.w = w
        self.pose_msg.header.stamp = rospy.Time.now()
        self.posepub.publish(self.pose_msg)
