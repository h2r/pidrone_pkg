import rospy
import rospkg
import yaml
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped, PoseStamped
from pidrone_pkg.msg import RC, State
from std_msgs.msg import Header, Empty, Bool


class FaultProtector(object):
    """A class that ...
    """

    def __init__(self):
        # instance vars
        self.shutdown = False
        self.shutdown_cause = ""
        # heartbeats
        curr_time = rospy.Time.now()
        self.heartbeat_infrared = curr_time
        self.heartbeat_camera = curr_time
        self.heartbeat_web_interface = curr_time
        self.heartbeat_pid_controller = curr_time
        self.heartbeat_flight_controller = curr_time
        self.heartbeat_state_estimator = curr_time
        self.heartbeat_fly_commands = curr_time
        self.heartbeat_camera_pose = curr_time
        # callbacks variables
        self.altitude = 0.0

        # import safe values
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/thresholds.yaml" % path) as f:
            self.thresholds = yaml.load(f)
        # import fault strings
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/shutdown_strings.yaml" % path) as f:
            self.strs = yaml.load(f)

        # Subscribers
        ############
        # heartbeat subscribers
        rospy.Subscriber("/pidrone/heartbeat/web_interface", Empty, self.heartbeat_web_interface_callback)
        rospy.Subscriber("/pidrone/heartbeat/pid_controller", Empty, self.heartbeat_pid_controller_callback)
        rospy.Subscriber("/pidrone/state", State, self.heartbeat_state_estimator_callback)
        rospy.Subscriber('/pidrone/picamera/twist', TwistStamped, self.heartbeat_camera_callback)
        rospy.Subscriber('/pidrone/infrared', Range, self.heartbeat_infrared_callback)
        rospy.Subscriber('/pidrone/fly_commands', RC, self.heartbeat_fly_commands_callback)
        rospy.Subscriber('/pidrone/picamera/pose', PoseStamped, self.heartbeat_pose_callback)

        # Publishers
        ############
        self.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)


    # Heartbeat Callbacks: These update the last time that data was received
    #                       from a node
    def heartbeat_web_interface_callback(self, msg):
        """Update web_interface heartbeat"""
        self.heartbeat_web_interface = rospy.Time.now()

    def heartbeat_pid_controller_callback(self, msg):
        """Update pid_controller heartbeat"""
        self.heartbeat_pid_controller = rospy.Time.now()

    def heartbeat_state_estimator_callback(self, msg):
        """Update state_estimator heartbeat and the current altitude"""
        self.heartbeat_state_estimator = rospy.Time.now()
        self.altitude = msg.pose_with_covariance.pose.position.z

    def heartbeat_infrared_callback(self, msg):
        """Update ir sensor heartbeat"""
        self.heartbeat_infrared = rospy.Time.now()

    def heartbeat_camera_callback(self, msg):
        """Update camera sensor heartbeat"""
        self.heartbeat_camera = rospy.Time.now()

    def heartbeat_pose_callback(self, msg):
        """turn off position control if the camera is lost (unrelated to disarming) """
        self.heartbeat_camera_pose = rospy.Time.now()

    def heartbeat_fly_commands_callback(self, msg):
        """Update camera sensor heartbeat"""
        self.heartbeat_fly_commands = rospy.Time.now()

    def check_heartbeats(self, mode, prev_mode):
        curr_time = rospy.Time.now()
        if curr_time - self.heartbeat_web_interface > rospy.Duration.from_sec(self.thresholds["heartbeat"]["web_interface"]):
            self.shutdown_cause += self.strs["web_interface"]
            self.shutdown = True
        if curr_time - self.heartbeat_pid_controller > rospy.Duration.from_sec(self.thresholds["heartbeat"]["pid_controller"]):
            self.shutdown_cause += self.strs["pid"]
            self.shutdown = True
        if curr_time - self.heartbeat_infrared > rospy.Duration.from_sec(self.thresholds["heartbeat"]["infrared"]):
            self.shutdown_cause += self.strs["infrared"]
            self.shutdown = True
        if curr_time - self.heartbeat_camera > rospy.Duration.from_sec(self.thresholds["heartbeat"]["camera"]):
            self.shutdown_cause += self.strs["camera"]
            self.shutdown = True
        if curr_time - self.heartbeat_state_estimator > rospy.Duration.from_sec(self.thresholds["heartbeat"]["state_estimator"]):
            self.shutdown = True
            self.shutdown_cause += self.strs["state_estimator"]
        if mode == "FLYING" and prev_mode == "FLYING":
            if curr_time - self.heartbeat_fly_commands > rospy.Duration.from_sec(self.thresholds["heartbeat"]["fly_commands"]):
                self.shutdown = True
                self.shutdown_cause += self.strs["fly_commands"]
        # turn off position_control if the camera is lost (unrelated to disarming)
        if curr_time - self.heartbeat_camera_pose > rospy.Duration.from_sec(self.thresholds["position_control"]):
            self.position_control_pub.publish(False)

    def check_accelerations(self, imu_msg):
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z
        if abs(ax) > self.thresholds["acceleration"]["x"]:
            self.shutdown = True
            self.shutdown_cause += self.strs["acc"]["x"] + self.strs["change_cutoff"] + self.strs["prev_value"] + str(ax)
        if abs(ay) > self.thresholds["acceleration"]["y"]:
            self.shutdown = True
            self.shutdown_cause += self.strs["acc"]["y"] + self.strs["change_cutoff"] + self.strs["prev_value"] + str(ay)
        if abs(az) > self.thresholds["acceleration"]["z"]:
            self.shutdown = True
            self.shutdown_cause += self.strs["acc"]["z"] + self.strs["change_cutoff"] + self.strs["prev_value"] + str(az)

    def check_angular_rates(self, imu_msg):
        roll_rate = imu_msg.angular_velocity.x
        pitch_rate = imu_msg.angular_velocity.y
        yaw_rate = imu_msg.angular_velocity.z
        if abs(roll_rate) > self.thresholds["angular_velocity"]["x"]:
            self.shutdown = True
            self.shutdown_cause += self.strs["gyro"]["x"] + self.strs["change_cutoff"] + self.strs["prev_value"] + str(roll_rate)
        if abs(pitch_rate) > self.thresholds["angular_velocity"]["y"]:
            self.shutdown = True
            self.shutdown_cause += self.strs["gyro"]["y"] + self.strs["change_cutoff"] + self.strs["prev_value"] + str(pitch_rate)
        if abs(yaw_rate) > self.thresholds["angular_velocity"]["z"]:
            self.shutdown = True
            self.shutdown_cause += self.strs["gyro"]["z"] + self.strs["change_cutoff"] + self.strs["prev_value"] + str(yaw_rate)

    def check_altitude(self):
        if self.altitude > self.thresholds["altitude"]:
            self.shutdown = True
            self.shutdown_cause += self.strs["altitude"] + self.strs["change_cutoff"] + self.strs["prev_value"] + str(self.altitude)

    def check_battery(self, battery_voltage):
        if battery_voltage is not None and battery_voltage < self.thresholds["battery"]["min_flying_voltage"]:
            self.shutdown_cause += self.strs["battery"] + self.strs["prev_value"] + str(battery_voltage)
            self.shutdown = True

    def should_i_shutdown(self, mode, prev_mode, battery_voltage, imu_msg):
        self.check_battery(battery_voltage)
        self.check_heartbeats(mode, prev_mode)
        self.check_accelerations(imu_msg)
        self.check_angular_rates(imu_msg)
        self.check_altitude()
        return self.shutdown

    def get_shutdown_cause(self):
        return self.shutdown_cause
