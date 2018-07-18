from pid_class import PID
from pidrone_pkg.msg import RC, axes_err, Velocity, Mode, RC
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Range, Imu
import command_values
import rospy
import signal
import sys
import time
import tf
import numpy as np

class PIDController(object):

    def __init__(self, ctrlpub):
        # Initial setpoint for the z-position of the drone
        self.initial_set_z = 30.0
        # Setpoint for the z-position of the drone
        self.set_z = self.initial_set_z
        # Current z-position of the drone according to sensor data
        self.current_z = 0

        # Tracks x, y velocity error and z-position error
        self.error = axes_err()
        # PID controller
        self.pid = PID()

        # Setpoint for the x-velocity of the drone
        self.set_vel_x = 0
        # Setpoint for the y-velocity of the drone
        self.set_vel_y = 0

        # Commanded yaw velocity of the drone
        self.cmd_yaw_velocity = 0

        # Tracks previous drone attitude
        self.prev_angx = 0
        self.prev_angy = 0
        # Time of previous attitude measurement
        self.prev_angt = None

        # Angle compensation values (to account for tilt of drone)
        self.mw_angle_comp_x = 0
        self.mw_angle_comp_y = 0
        self.mw_angle_alt_scale = 1.0
        self.mw_angle_coeff = 10.0
        self.angle = TwistStamped()

        # Control Publisher
        self.ctrlpub = ctrlpub

        # Control values (initialize ot disarmed values)
        self.ctrl_vals = command_values.disarm_cmd


        # Store the current and previous mode of the drone
        self.prev_mode = "DISARMED"
        self.curr_mode = "DISARMED"


    def mode_callback(self, msg):
        """Update the prev and curr mode variables"""
        self.prev_mode = self.curr_mode
        self.curr_mode = msg.mode

    def publish_ctrl(self, ctrl):
        """Publish the controls to /pidrone/controller"""
        msg = RC()
        msg.roll = ctrl[0]
        msg.pitch = ctrl[1]
        msg.yaw = ctrl[2]
        msg.throttle = ctrl[3]
        self.ctrlpub.publish(msg)

    def update_fly_velocities(self, msg):
        """Updates the desired x, y, yaw velocities and z-position"""
        if msg is not None:
            self.set_vel_x = msg.x_velocity
            self.set_vel_y = msg.y_velocity

            self.cmd_yaw_velocity = msg.yaw_velocity

            new_set_z = self.set_z + msg.z_velocity
            if 0.0 < new_set_z < 49.0:
                self.set_z = new_set_z

    def infrared_callback(self, msg):
        """Updates the current z-position of the drone as measured by the infrared sensor"""
        # Scales infrared sensor reading to get z accounting for tilt of the drone
        self.current_z = msg.range * self.mw_angle_alt_scale * 100
        self.error.z.err = self.set_z - self.current_z

    def vrpn_callback(self, msg):
        """Updates the current z-position of the drone as measured by the motion capture rig"""
        # Mocap uses y-axis to represent the drone's z-axis motion
        self.current_z = msg.pose.position.y * 100
        self.error.z.err = self.set_z - self.current_z

    def plane_callback(self, msg):
        """Calculates error for x, y planar motion of the drone"""
        self.error.x.err = (msg.twist.linear.x - self.mw_angle_comp_x) * self.current_z + self.set_vel_x
        self.error.y.err = (msg.twist.linear.y + self.mw_angle_comp_y) * self.current_z + self.set_vel_y

    def imu_callback(self, msg):
        """Calculates angle compensation values to account for the tilt of the drone"""
        # Convert the quaternion to euler angles
        # store the orientation quaternion
        oq = msg.orientation
        orientation_list = [oq.x, oq.y, oq.z, oq.w]
        (new_angx, new_angy, new_angz) = tf.transformations.euler_from_quaternion(orientation_list)
        new_angt = time.time()

        # Passes angle of drone and correct velocity of drone
        self.angle.twist.angular.x = new_angx
        self.angle.twist.angular.y = new_angy
        self.angle.header.stamp = rospy.get_rostime()

        dt = new_angt - self.prev_angt
        self.mw_angle_comp_x = np.tan((new_angx - self.prev_angx) * dt) * self.mw_angle_coeff
        self.mw_angle_comp_y = np.tan((new_angy - self.prev_angy) * dt) * self.mw_angle_coeff

        self.mw_angle_alt_scale = np.cos(new_angx) * np.cos(new_angy)
        self.pid.throttle.mw_angle_alt_scale = self.mw_angle_alt_scale

        self.prev_angx = new_angx
        self.prev_angy = new_angy
        self.prev_angt = new_angt

    def set_vel_callback(self,msg):
        """Update the current fly velocities if the drone mode is FLYING"""
        if self.curr_mode == 'FLYING':
            self.update_fly_velocities(msg)

    def ctrl_c_handler(self, signal, frame):
        """Stops the controller"""
        print "Caught ctrl-c! Stopping PIDController!"
        sys.exit()

def main():

    # ROS Setup
    ###########
    rospy.init_node('pid_controller')

    # Publishers
    ############
    errpub = rospy.Publisher('/pidrone/err', axes_err, queue_size=1)
    anglepub = rospy.Publisher('/pidrone/angle', TwistStamped, queue_size=1)
    ctrlpub = rospy.Publisher('/pidrone/controller', RC, queue_size=1)

    # create PIDController object
    pid_ctrlr = PIDController(ctrlpub)

    # Subscribers
    #############
    rospy.Subscriber("/pidrone/plane_err", TwistStamped, pid_ctrlr.plane_callback)
    rospy.Subscriber("/pidrone/infrared", Range, pid_ctrlr.infrared_callback)
    rospy.Subscriber("/pidrone/vrpn_pos", PoseStamped, pid_ctrlr.vrpn_callback)
    rospy.Subscriber("/pidrone/set_vel", Velocity, pid_ctrlr.set_vel_callback)
    rospy.Subscriber("/pidrone/imu", Imu, pid_ctrlr.imu_callback)
    rospy.Subscriber("/pidrone/mode", Mode, pid_ctrlr.mode_callback)

    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, pid_ctrlr.ctrl_c_handler)

    pid_ctrlr.prev_angt = time.time()

    print 'PID controller started'
    while not rospy.is_shutdown():
        # Publishes current error message
        errpub.publish(pid_ctrlr.error)
        if not pid_ctrlr.curr_mode == 'DISARMED':
            anglepub.publish(pid_ctrlr.angle)

        # Use a PID controller to calculate the flight command: [roll, pitch, yaw, throttle]
        fly_ctrl = pid_ctrlr.pid.step(pid_ctrlr.error, pid_ctrlr.cmd_yaw_velocity)

        if pid_ctrlr.curr_mode == 'FLYING':
            if pid_ctrlr.prev_mode == 'FLYING':
                pid_ctrlr.publish_ctrl(fly_ctrl)
            if pid_ctrlr.prev_mode == 'ARMED':
                pid_ctrlr.pid.reset(pid_ctrlr)

    print 'Stopping PID controller'


if __name__ == '__main__':
    main()
