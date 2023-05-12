#!/usr/bin/env python3


import tf
import time
import cv2
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from pidrone_pkg.msg import State
from sensor_msgs.msg import Range
from cv_bridge import CvBridge


class RigidTransformNode(object):
    """
    A class that uses OpenCV's estimateRigidTransform method to calculate
    the change in position of the drone.
    For more info, visit:
    https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform

    Publisher:
    ~pose

    Subscribers:
    ~reset_transform
    ~position_control
    """
    def __init__(self, node_name):
        # initialize the DTROS parent class
        rospy.init_node(node_name)

        camera_wh = (320, 240)

        self.bridge = CvBridge()

        # initialize the Pose data
        self.pose_msg = PoseStamped()
        self.altitude = 0.0
        self.x_position_from_state = 0.0
        self.y_position_from_state = 0.0

        # position hold is initialized as False
        self.position_control = False
        self.first_image = None
        self.previous_image = None

        # used as a safety check for position control
        self.consecutive_lost_counter = 0
        self.lost = False

        # first image vars
        self.first = True
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None

        # ROS Setup
        ###########
        # Publisher
        self._posepub = rospy.Publisher('/pidrone/picamera/pose', PoseStamped, queue_size=1)
        self._lostpub = rospy.Publisher('/pidrone/picamera/lost', Bool, queue_size=1)
        
        # Subscribers
        self._rtsub = rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback, queue_size=1)
        self._pcsub = rospy.Subscriber("/pidrone/position_control", Bool, self.position_control_callback, queue_size=1)
        self._stsub = rospy.Subscriber("/pidrone/state", State, self.state_callback, queue_size=1)
        self._isub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback, queue_size=1)
        self._sub_alt = rospy.Subscriber('/pidrone/range', Range, self.altitude_cb, queue_size=1)

        self.altitude = 0.03 # initialize to a bit off the ground
        self.altitude_ts = rospy.Time.now()
        

    def altitude_cb(self, msg):
        """
        The altitude of the robot
        Args:
            msg:  the message publishing the altitude

        """
        self.altitude = msg.range
        self.altitude_ts = msg.header.stamp


    def image_callback(self, msg):
        ''' A method that is called everytime an image is taken '''

        #print("image cb: " + msg.format)
        #jpg = np.frombuffer(msg.data, np.uint8)
        #data = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
        
        #image = np.reshape(np.frombuffer(data, dtype=np.uint8), (240, 320, 3))

        
        #image = self.bridge.compressed_imgmsg_to_cv2(msg)
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="mono8")
        dimage = image.copy()

        #print(image.shape)
        #cv2.waitKey(0)
        #time.sleep(10)
        # Run the following only if position control is enabled to prevent
        # wasting computation resources on unused position data

        duration_from_last_altitude = rospy.Time.now() - self.altitude_ts
        if duration_from_last_altitude.to_sec() > 10:
            rospy.logwarn("No altitude received for {:10.4f} seconds.".format(duration_from_last_altitude.to_sec()))
        
        if self.position_control:
            

            # if there is no first image stored, tell the user to capture an image
            if self.first:
                self.first = False
                print("Capturing a new first image")
                self.first_image = image
                self.first_points = cv2.goodFeaturesToTrack(self.first_image, maxCorners=10, qualityLevel=0.01, minDistance=8)
                # for point in self.first_points:
                #     x = int(point[0,0])
                #     y = int(point[0,1])
                #     print((x,y))
                #     cv2.circle(dimage, center=(x,y), radius=2, color=(255,0,0), thickness=10)
                # cv2.imshow("image", dimage)
                # cv2.waitKey(10)
                self.previous_image = image
                self.last_first_time = rospy.get_time()
            # if a first image has been stored
            else:
                # try to estimate the transformations from the first image
                #print(self.first_image)

                nextPts, status, err = cv2.calcOpticalFlowPyrLK(self.first_image, image, self.first_points, None)

                transform_first, inliers = cv2.estimateAffinePartial2D(self.first_points, nextPts, False)
                #print(transform_first)

                #cv2.estimateAffinePartial2D(self.first_image, image, False)

                # if the first image was visible (the transformation was succesful) :
                if transform_first is not None:
                    self.lost = False
                    # calculate the x,y, and yaw translations from the transformation
                    translation_first, yaw_first = self.translation_and_yaw(transform_first)
                    # use an EMA filter to smooth the position and yaw values
                    self.pose_msg.pose.position.x = translation_first[0]*self.altitude
                    self.pose_msg.pose.position.y = translation_first[1]*self.altitude
                    # With just a yaw, the x and y components of the
                    # quaternion are 0
                    _,_,z,w = tf.transformations.quaternion_from_euler(0,0,yaw_first)
                    self.pose_msg.pose.orientation.z = z
                    self.pose_msg.pose.orientation.w = w
                    # update first image data
                    self.first_image_counter += 1
                    self.max_first_counter = max(self.max_first_counter, self.first_image_counter)
                    self.last_first_time = rospy.get_time()
                    print(("count:", self.first_image_counter))
                # else the first image was not visible (the transformation was not succesful) :
                else:
                    # try to estimate the transformation from the previous image
                    transform_previous, inliers = cv2.estimateAffinePartial2D(self.previous_image, image, False)

                    # if the previous image was visible (the transformation was succesful)
                    # calculate the position by integrating
                    if transform_previous is not None:
                        self.lost = False
                        if self.last_first_time is None:
                            self.last_first_time = rospy.get_time()
                        time_since_first = rospy.get_time() - self.last_first_time
                        print(("integrated", time_since_first))
                        print(("max_first_counter: ", self.max_first_counter))
                        int_displacement, yaw_previous = self.translation_and_yaw(transform_previous)
                        self.pose_msg.pose.position.x = self.x_position_from_state + (int_displacement[0]*self.altitude)
                        self.pose_msg.pose.position.y = self.y_position_from_state + (int_displacement[1]*self.altitude)
                        _,_,z,w = tf.transformations.quaternion_from_euler(0,0,yaw_previous)
                        self.pose_msg.pose.orientation.z = z
                        self.pose_msg.pose.orientation.w = w
                        print("Lost the first image !")
                    # if the previous image wasn't visible (the transformation was not
                    # succesful), reset the pose and print lost
                    else:
                        print("Lost!")
                        if self.lost:
                            self.consecutive_lost_counter += 1
                        else:
                            self.lost = True

                self.previous_image = image

        # if the camera is lost over ten times in a row, then publish lost
        # to disable position control
        if self.lost:
            if self.consecutive_lost_counter >= 10:
                self._lostpub.publish(True)
                self.consecutive_lost_counter = 0
        else:
            self.consecutive_lost_counter = 0
            self._lostpub.publish(False)

        # publish the pose message
        self.pose_msg.header.stamp = rospy.Time.now()
        self._posepub.publish(self.pose_msg)

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
        self.first = True

        # reset first image vars
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None

        # reset the pose values
        self.pose_msg = PoseStamped()

        self._lostpub.publish(False)
        print("done")

    # subscribe /pidrone/position_control
    def position_control_callback(self, msg):
        ''' Set whether the pose is calculated and published '''
        self.position_control = msg.data
        print(("Position Control", self.position_control))

    def state_callback(self, msg):
        """
        Store z position (altitude) reading from State, along with most recent
        x and y position estimate
        """
        self.altitude = msg.pose_with_covariance.pose.position.z
        self.x_position_from_state = msg.pose_with_covariance.pose.position.x
        self.y_position_from_state = msg.pose_with_covariance.pose.position.y
        
    
def main():
    rigid_transform_node = RigidTransformNode("rigid_transform_node")
    rospy.spin()

if __name__ == '__main__':
    main()
