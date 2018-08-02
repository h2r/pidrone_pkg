from __future__ import division
import rospy
from std_msgs.msg import Float32


class PositionPID(object):
    '''A PID class for controlling position'''
    def __init__(self):
        self.integrated_error_x = 0
        self.integrated_error_y = 0
        self.integrated_error_z = 0

        self.cur_filt_d_err_z = 0
        self.prev_d_err_z = 0
        self.error_last = None
        self.time = None

        self.kd_pub = rospy.Publisher('/pidrone/position/kd', Float32, queue_size=1)
        self.kp_pub = rospy.Publisher('/pidrone/position/kp', Float32, queue_size=1)
        self.ki_pub = rospy.Publisher('/pidrone/position/ki', Float32, queue_size=1)
        self.zerr_pub = rospy.Publisher('/pidrone/position/zerr', Float32, queue_size=1)

    def clip(self, num, lower, upper):
        ''' Clips num to be between lower and upper '''
        return max( min(upper, num), lower)

    def step(self, error):
        ''' Give flight controller outputs based on current and previous errors
        '''

        # Negative pitch points nose up
        # Positive pitch points nose down
        # Negative roll rolls left
        # positive roll rolls right

        # with the Motive Tracker setup on 4/25/2018, +Z is toward baxter
        # and +X is left when facing baxter

        # with the drone facing baxter (flight controller pointing toward him),
        # this means +pitch -> +Z, -roll -> +X

        if self.time is None:
            time_elapsed = 1
        else:
            time_elapsed = rospy.get_time() - self.time

        roll_midpoint = 1555
        pitch_midpoint = 1531
        yaw_midpoint = 1500
        thrust_midpoint = 1320

        if self.error_last is not None:
            d_err_x = error.x - self.error_last.x
            d_err_y = error.y - self.error_last.y
            d_err_z = error.z - self.error_last.z
        else:
            d_err_x = 0
            d_err_y = 0
            d_err_z = 0


        c_cur_filt = .2
        c_cmd = .6
        cur_filt_d_err_z = c_cur_filt*self.cur_filt_d_err_z + c_cmd*(d_err_z + self.prev_d_err_z)

        self.cur_filt_d_err_z = cur_filt_d_err_z
        self.prev_d_err_z = d_err_z

        d_err_z = cur_filt_d_err_z

        self.error_last = error

        kp_x = 0
        kp_y = 0
        kp_z = 0


        ki_x = 0
        ki_y = 0
        ki_z = 0

        kd_x = 0
        kd_y = 0
        kd_z = 0

        # Limit integrated windup to +/- .1, 10% of the thrust
        uc_integrated_error_x = self.integrated_error_x + ki_x*error.x*time_elapsed
        self.integrated_error_x = self.clip(uc_integrated_error_x, -.1, .1)

        uc_integrated_error_y = self.integrated_error_y + ki_y*error.y*time_elapsed
        self.integrated_error_y = self.clip(uc_integrated_error_y, -.1, .1)

        uc_integrated_error_z = self.integrated_error_z + ki_z*error.z*time_elapsed
        self.integrated_error_z = self.clip(uc_integrated_error_z, -.1, .1)


        # roll controls x (for now)
        roll_factor = (kp_x*error.x + self.integrated_error_x + (kd_x*d_err_x)/time_elapsed)

        # Pitch controls y (for now)
        pitch_factor = (kp_y*error.y + self.integrated_error_y + (kd_y*d_err_y)/time_elapsed)

        # Don't care about yaw (for now)
        yaw_factor = 0

        # Thrust controls z
        thrust_factor = (kp_z*error.z + self.integrated_error_z + (kd_z*d_err_z)/time_elapsed)
        self.zerr_pub.publish(error.z)
        self.kp_pub.publish(kp_z*error.z)
        self.ki_pub.publish(ki_z*self.integrated_error_z)
        self.kd_pub.publish(kd_z*d_err_z)

        cmd_r = roll_midpoint + int(self.clip(100*roll_factor, -100,100))
        cmd_p = pitch_midpoint + int(self.clip(100*pitch_factor, -100,100))
        cmd_yaw = yaw_midpoint + int(self.clip(100*yaw_factor, -100,100))
        cmd_t = int(self.clip(thrust_midpoint + 100*thrust_factor, 1200,2000))
        return [cmd_r, cmd_p, cmd_yaw, cmd_t]

    def reset(self):
        ''' Set integrated error to 0, and prev error (for D term) to 0 '''
        self.time = None
        self.integrated_error_x = 0
        self.integrated_error_y = 0
        self.integrated_error_z = 0
        self.error_last = None


class VelocityPID(object):
    '''A PID class for controlling velocity'''
    def __init__(self):
        self.integrated_error_x = 0
        self.integrated_error_y = 0
        self.integrated_error_z = 0

        self.cur_filt_d_err_z = 0
        self.prev_d_err_z = 0
        self.error_last = None

        self.kd_pub = rospy.Publisher('/pidrone/velocity/kd', Float32, queue_size=1)
        self.kp_pub = rospy.Publisher('/pidrone/velocity/kp', Float32, queue_size=1)
        self.ki_pub = rospy.Publisher('/pidrone/velocity/ki', Float32, queue_size=1)
        self.zerr_pub = rospy.Publisher('/pidrone/velocity/zerr', Float32, queue_size=1)

    def clip(self, num, lower, upper):
        ''' Clips num to be between lower and upper '''
        return max( min(upper, num), lower)

    def step(self, error):
        ''' Give flight controller outputs based on current and previous errors
        '''

        # Negative pitch points nose up
        # Positive pitch points nose down
        # Negative roll rolls left
        # positive roll rolls right

        # with the Motive Tracker setup on 4/25/2018, +Z is toward baxter
        # and +X is left when facing baxter

        # with the drone facing baxter (flight controller pointing toward him),
        # this means +pitch -> +Z, -roll -> +X

        roll_midpoint = 1520
        pitch_midpoint = 1600
        yaw_midpoint = 1500
        thrust_midpoint = 1380



        if self.error_last is not None:
            d_err_x = error.x - self.error_last.x
            d_err_y = error.y - self.error_last.y
            d_err_z = error.z - self.error_last.z
        else:
            d_err_x = 0
            d_err_y = 0
            d_err_z = 0


        c_cur_filt = .2
        c_cmd = .6
        cur_filt_d_err_z = c_cur_filt*self.cur_filt_d_err_z + c_cmd*(d_err_z + self.prev_d_err_z)

        self.cur_filt_d_err_z = cur_filt_d_err_z
        self.prev_d_err_z = d_err_z

        d_err_z = cur_filt_d_err_z

        self.error_last = error

        kp_x = .6
        kp_y = .6
        kp_z = 10


        ki_x = 0
        ki_y = 0
        ki_z = .0005

        kd_x = 2
        kd_y = 2
        kd_z = 180

        # Limit integrated windup to +/- .1, 10% of the thrust
        uc_integrated_error_x = self.integrated_error_x + ki_x*error.x
        self.integrated_error_x = self.clip(uc_integrated_error_x, -.1, .1)

        uc_integrated_error_y = self.integrated_error_y + ki_y*error.y
        self.integrated_error_y = self.clip(uc_integrated_error_y, -.1, .1)

        uc_integrated_error_z = self.integrated_error_z + ki_z*error.z
        self.integrated_error_z = self.clip(uc_integrated_error_z, -.1, .1)


        # roll controls x (for now)
        roll_factor = (kp_x*error.x + self.integrated_error_x + kd_x*d_err_x)

        # Pitch controls y (for now)
        pitch_factor = kp_y*error.y + ki_y*self.integrated_error_y + kd_y*d_err_y

        # Don't care about yaw (for now)
        yaw_factor = 0

        # Thrust controls z
        thrust_factor = kp_z*error.z + self.integrated_error_z + kd_z*d_err_z
        self.zerr_pub.publish(error.z)
        self.kp_pub.publish(kp_z*error.z)
        self.ki_pub.publish(ki_z*self.integrated_error_z)
        self.kd_pub.publish(kd_z*d_err_z)

        cmd_r = roll_midpoint + int(self.clip(100*roll_factor, -100,100))
        cmd_p = pitch_midpoint + int(self.clip(100*pitch_factor, -100,100))
        cmd_yaw = yaw_midpoint + int(self.clip(100*yaw_factor, -100,100))
        cmd_t = int(self.clip(thrust_midpoint + 100*thrust_factor, 1200,2000))
        return [cmd_r, cmd_p, cmd_yaw, cmd_t]

    def reset(self):
        ''' Set integrated error to 0, and prev error (for D term) to 0 '''
        self.integrated_error_x = 0
        self.integrated_error_y = 0
        self.integrated_error_z = 0
        self.error_last = None
