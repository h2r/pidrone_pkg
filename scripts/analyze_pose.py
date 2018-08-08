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
    /pidrone/picamera/has_first_image

    Subscribers:
    /pidrone/reset_transform
    /pidrone/toggle_transform
    /pidrone/state
    """

    def setup(self, camera_wh):

        # initialize the Pose data
        self.pose = Pose()
        self.pose.position.x = 0
        self.pose.position.y = 0
        self.pose.position.z = 0.075 # the offset height of the drone before takeoff

        # store the yaw
        self.yaw = 0

        # position hold is initialized as False
        self.position_control = False
        self.has_first = False
        self.first_image = None
        self.previous_image = None
        # value used for EMA filter
        self.alpha = 0.1

        # counter for the number of consecutive images that contain the first image
        self.first_image_counter = 0

        # ROS Setup
        ###########
        # Publisher
        self.posepub = rospy.Publisher('/pidrone/picamera/pose', Pose, queue_size=1)
        self.has_first_image_pub = rospy.Publisher('/pidrone/picamera/pose', Bool, queue_size=1, latch = True)
        # Subscribers
        rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback)
        rospy.Subscriber("/pidrone/toggle_transform", Bool, self.toggle_callback)
        rospy.Subscriber("/pidrone/state", State, self.state_callback)

    def write(self, data):
        ''' A method that is called everytime an image is taken '''

        # Run the following only if position control is enabled to prevent
        # wasting computation resources on unused position data
        if self.position_control:
            image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
            # if there is no first image stored, tell the user to capture an image
            if not self.has_first:
                self.has_first = True
                self.first_image = image
                self.previous_image = image
                print "\nCapture the first image by pressing 'r' "
            # if a first image has been stored
            else:
                # try to estimate the transformations from the first image
                transform_first = cv2.estimateRigidTransform(self.first_image, image, False)

                # if the first image was visible (the transformation was succesful) :
                if transform_first is not None:
                    # calculate the x,y, and yaw translations from the transformation
                    translation_first, yaw_first = self.translation_and_yaw(transform_first)
                    # use an EMA filter to smoothe the position and yaw values
# TODO CHECK SIGNS AND CHECK THAT ESTIMATERBD IS IN METERS!
# TODO MOVE ALL OF THIS TO THE STATE ESTIMATOR AND CALL IT EMA_STATE_ESTIMATOR
                    self.pose.position.x = (1.0 - self.alpha) * self.pose.position.x +
                        self.alpha * translation_first[0] * self.pose.position.z
                    self.pose.position.y = (1.0 - self.alpha) * self.pose.position.y +
                        self.alpha * translation_first[1] * self.pose.position.z
                    self.yaw = (1.0 - self.alpha) * self.yaw + self.alpha * yaw_first
                    # update first image data
                    self.first_image_counter += 1
                    self.max_first_counter = self.first_image_counter
                    self.last_first_time = rospy.Time.now()
                    print "count:", self.first_image_counter
                # else the first image was not visible (the transformation was not succesful) :
                else:
                    # try to estimate the transformation from the previous image
                    transform_previous = cv2.estimateRigidTransform(self.previous_image, image, False)

                    # if the previous image was visible (the transformation was succesful)
                    # calculate the position by integrating
                    if transform_previous is not None:
                        time_since_first = rospy.Time.now() - self.last_first_time
                        print "integrated", time_since_first
                        print "max_first_counter: ", self.max_first_counter
                        int_displacement, yaw = self.translation_and_yaw(transform_previous)
# TODO SHOULD X BE NEGATIVE?
                        self.pose.position.x += int_displacement[0] * self.pose.position.z
                        self.pose.position.y += int_displacement[1] * self.pose.position.z
# TODO do something with yaw
                        yaw += yaw_previous
                        self.pose.orientation = tf.quaternion_from_euler(0,0,yaw)
                        print "Lost the first image !"
                    # if the previous image wasn't visible (the transformation was not
                    # succesful), reset the pose and print lost
                    else:
                        self.reset_pose()
                        print "Lost!"
                        self.reset_callback(data)
                    # reset the first image counter
                    self.first_image_counter = 0

            self.previous_image = image
# TODO In flowpub they do the pid step here and this might affect the stepper because of dt, so if it doesn't work try that
# TODO also, I'm not using cvc for now, but go back and use it if it doesn't work
        self.posepub.publish(self.pose)
        self.has_first_image_pub.publish(self.has_first_image)


    # normalize image
    def translation_and_yaw(self, transform):
        translation_x_y = [float(transform[0, 2]) / 320 * self.pose.position.z,
                            float(transform[1, 2]) / 240 * self.pose.position.z]

        yaw_scale = np.sqrt(transform[0, 0]**2 + transform[1, 0]**2)
        yaw_y_x = [float(transform[1, 0]) / yaw_scale, float(transform[0, 0]) / yaw_scale]
        yaw = np.arctan2(yaw_y_x[0], yaw_y_x[1])

        return translation_x_y, yaw


    def state_callback(self, state):
        ''' Store the altitude from the state estimator '''
        # the altitude is the z position from the state estimator (meters)
        altitude = state.pose_with_covariance.pose.pose.position.z
        if altitude > 0:
            # store the altitude for use in computations
            self.pose.position.z = altitude

    # subscribe /pidrone/reset_transform
    def reset_callback(self):
        ''' Reset the current position '''
        self.has_first = False
        self.pose.postion.x = 0
        self.pose.position.y = 0
        self.yaw = 0
        self.first_image_counter = 0
        print "Reset Transform"

    # subscribe /pidrone/toggle_transform
    def toggle_callback(self, data):
        ''' Toggle whether the phase is calculated and published '''
        self.position_control = data
        print "Position Control", self.position_control
