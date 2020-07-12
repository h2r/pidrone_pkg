import sys
import os
import rospy
import signal
import numpy as np
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
from pidrone_pkg.msg import State


class SensorValidator(object):
    """A class that validates IR sensor and Camera sensor inputs while the
       drone is flying.

       The validation is based on the assumption that the sensor measurements
       should be nonhomogeneous over a set of consecutive samples. The size of
       the set is a parameter that is tuned to be as small as possible without
       causing interuptions when the sensors are operating correctly.

    Subscriber:
    /pidrone/picamera/twist
    /pidrone/infrared
    """

    def __init__(self):
        # variables to store whether the sensors are erroring
        self.error_ir = True
        self.error_camera = True
        # the number of sufficient readings for which a nonhomgeneity is
        # guarenteed under correct sensor operation
        # TODO: experiment with numbers. Use probability?
        # TODO: check if the slow takeoffs messes this up if the drone is on
        # the ground, then the range value won't change
        self.CUTOFF_IR = 50
        self.CUTOFF_CAMERA = 10
        
        # create the numpy arrays to store previous sensor values
        self.history_ir = np.zeros(self.CUTOFF_IR, dtype=float)
        self.history_camera = np.zeros((2, self.CUTOFF_CAMERA), dtype=float)
        # iterators to move through the array and replace the oldest values
        self.i_ir = 0
        self.i_camera = 0
        self.i_camera_zero = 0
        # strings to identify the source of a system shutdown signal
        self.shutdown_string_ir = ("Safety Failure: The range measurements are not changing. "
                                   "\nPlease verify that the IR sensor is functioning properly.")
        self.shutdown_string_camera = ("Safety Failure: The velocity measurements are not changing."
                                       "\nPlease verify that the Camera sensor is functioning properly.")
        # store the height of the drone so that safety logic can distinguish
        # when the drone is on the ground vs in the air
        self.altitude = 0.0
        # the height of the drone when it is considered to be in the air
        self.flying_altitude = 0.2

        # node_name = os.path.splitext(os.path.basename(__file__))[0]
        # rospy.init_node(node_name)
        # Subscribers
        ############
        rospy.Subscriber('/pidrone/picamera/twist', TwistStamped, self.camera_callback)
        rospy.Subscriber('/pidrone/infrared', Range, self.infrared_callback)
        rospy.Subscriber('/pidrone/state', State, self.state_callback)


    def get_errors(self):
        if self.error_ir:
            return self.shutdown_string_ir
        elif self.error_camera:
            return self.shutdown_string_camera
        else:
            return None

    def increment_iterator(self, iterator, array_size):
        return (iterator + 1) % array_size

    def state_callback(self, msg):
        self.altitude = msg.pose_with_covariance.pose.position.z

    def infrared_callback(self, msg):
        self.history_ir[self.i_ir] = msg.range
        self.i_ir = self.increment_iterator(self.i_ir, self.CUTOFF_IR)
        if np.all(self.history_ir == self.history_ir[0]):
            self.error_ir = True
        else:
            self.error_ir = False

    def camera_callback(self, msg):
        # x and y must be compared individually because if the camera cuts out
        # mid-flight, then the previous values for each are held
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        self.history_camera[0, self.i_camera] = x
        self.history_camera[1, self.i_camera] = y
        self.i_camera = self.increment_iterator(self.i_camera, self.CUTOFF_CAMERA)
        # check for repeated measurements of non-zero values
        if not ((abs(x) == 0.0) or (abs(y) == 0.0)):
            if (np.all(self.history_camera[0, :] == x) or
                    np.all(self.history_camera[1, :] == y)):
                self.error_camera = True
            else:
                self.error_camera = False
