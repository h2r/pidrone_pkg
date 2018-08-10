import tf
import cv2
import rospy
import picamera
import numpy as np
from pidrone_pkg.msg import State
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, Bool



class AnalyzePose(picamera.array.PiMotionAnalysis):
    """
    A class that uses OpenCV's estimateRigidTransform method to calculate
    the change in position of the drone.
    For more info, visit:
    https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform

    Publisher:
    /pidrone/picamera/pose
    /pidrone/picamera/transforming_on_first_image

    Subscribers:
    /pidrone/reset_transform
    /pidrone/position_control
    /pidrone/state
    """

    def setup(self, camera_wh):

        # initialize the Pose data
        self.pose = Pose()

        # position hold is initialized as False
        self.position_control = False
        self.transforming_on_first_image = False
        self.first_image = None
        self.previous_image = None

        # first image vars
        self.first = True
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None

        # ROS Setup
        ###########
        # Publisher
        self.posepub = rospy.Publisher('/pidrone/picamera/pose', Pose, queue_size=1)
        self.transforming_on_first_image_pub = rospy.Publisher('/pidrone/picamera/transforming_on_first_image', \
                Bool, queue_size=1, latch = True)
        # Subscribers
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/position_control", Bool, self.position_control_callback)

    def write(self, data):
        ''' A method that is called everytime an image is taken '''

        # Run the following only if position control is enabled to prevent
        # wasting computation resources on unused position data
        if self.position_control:
            image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
            # if there is no first image stored, tell the user to capture an image
            if self.first:
                self.first = False
                print "Capturing a new first image"
                self.first_image = image
                self.previous_image = image
                self.last_first_time = rospy.get_time()
            # if a first image has been stored
            else:
                # try to estimate the transformations from the first image
                transform_first = cv2.estimateRigidTransform(self.first_image, image, False)

                # if the first image was visible (the transformation was succesful) :
                if transform_first is not None:
                    self.transforming_on_first_image = True
                    # calculate the x,y, and yaw translations from the transformation
                    translation_first, yaw_first = self.translation_and_yaw(transform_first)
                    # use an EMA filter to smoothe the position and yaw values
                    self.pose.position.x = translation_first[0]
                    self.pose.position.y = translation_first[1]
                    _,_,z,w = tf.transformations.quaternion_from_euler(0,0,yaw_first)
                    self.pose.orientation.z = z
                    self.pose.orientation.w = w
                    # update first image data
                    self.first_image_counter += 1
                    self.max_first_counter = max(self.max_first_counter, self.first_image_counter)
                    self.last_first_time = rospy.get_time()
                    print "count:", self.first_image_counter
                # else the first image was not visible (the transformation was not succesful) :
                else:
                    self.transforming_on_first_image = False
                    # try to estimate the transformation from the previous image
                    transform_previous = cv2.estimateRigidTransform(self.previous_image, image, False)

                    # if the previous image was visible (the transformation was succesful)
                    # calculate the position by integrating
                    if transform_previous is not None:
                        if self.last_first_time is None:
                            self.last_first_time = rospy.get_time()
                        time_since_first = rospy.get_time() - self.last_first_time
                        print "integrated", time_since_first
                        print "max_first_counter: ", self.max_first_counter
                        int_displacement, yaw_previous = self.translation_and_yaw(transform_previous)
                        self.pose.position.x = int_displacement[0]
                        self.pose.position.y = int_displacement[1]
                        _,_,z,w = tf.transformations.quaternion_from_euler(0,0,yaw_previous)
                        self.pose.orientation.z = z
                        self.pose.orientation.w = w
                        print "Lost the first image !"
                    # if the previous image wasn't visible (the transformation was not
                    # succesful), reset the pose and print lost
                    else:
                        print "Lost!"
                        #self.reset_callback()

            self.previous_image = image

        self.posepub.publish(self.pose)
        self.transforming_on_first_image_pub.publish(self.transforming_on_first_image)

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
    def reset_callback(self, msg=Empty()):
        """ Reset the current position and orientation """
        print "Resetting Phase"

        # reset position control variables
        self.transforming_on_first_image = False
        self.first = True

        # reset first image vars
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None

        # reset the pose values
        self.pose = Pose()

    # subscribe /pidrone/position_control
    def position_control_callback(self, msg):
        ''' Set whether the pose is calculated and published '''
        self.position_control = msg.data
        print "Position Control", self.position_control
