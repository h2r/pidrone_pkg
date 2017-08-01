#!/usr/bin/env python
from __future__ import division
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped
import time
import tf
import math
import numpy as np
from copy import deepcopy

cmdpub = rospy.Publisher('/pidrone/commands_old', RC, queue_size=1)

def calc_thrust_and_theta(Fx, Fy, Fz):
    theta = math.atan2(math.sqrt(Fx**2 + Fz**2), Fy)
    thrust = Fy/math.cos(theta)
    return (thrust, theta)

def calc_roll_pitch_from_theta(Fx, Fz, theta):
    y_axis = np.array([0, 1, 0])

    # XXX jgo:
    #f_hat = np.array([Fx, theta, Fz])/math.sqrt(Fx**2 + Fz**2)
    f_hat = np.array([Fx, 0, Fz])/math.sqrt(Fx**2 + Fz**2)

    r_hat = np.cross(y_axis, f_hat)
    dScale = np.linalg.norm(r_hat)

    # XXX jgo:
    #quat = np.array([r_hat[0], r_hat[1], r_hat[2],
        #theta])/math.sqrt(r_hat[0]**2 + r_hat[1]**2 + r_hat[2]**2 + theta**2)
    sScale = math.sin(theta * 0.5) / dScale
    quat = np.array([r_hat[0] * sScale, r_hat[1] * sScale, r_hat[2] * sScale,
        math.cos(theta * 0.5)] )

    return tf.transformations.euler_from_quaternion(quat)

millis = lambda: int(round(time.time() * 1000))

kp = {
	'lr': -1.1,
	'fb': -1.1,
        
	'yaw': 		500.0,
	'alt': 	10,
        'alt_above': 0
}

ki = {
	'lr': 	-0.00025,
	'fb':	-0.00025,
	'yaw': 		0.0,
	'alt': 		0.04
} 
kd = {
	'lr': 	-300,
	'fb': 	-300,
	'yaw':	0.0,
	'alt':  700
}

pos_time = rospy.Time()

# these positions are global (ie what comes out of the motion tracker)
sp_global  = Pose() # set point
pos_global = Pose() # set point
pos_global.position.x = 1
old_pos_global = Pose() # set point

sp_global.position.x = 0
sp_global.position.y = 0
sp_global.position.z = 0
sp_global.orientation.x = 0
sp_global.orientation.y = 0
sp_global.orientation.z = 0
sp_global.orientation.w = 1

pos_global.position.x = 0
pos_global.position.y = 0
pos_global.position.z = 0
pos_global.orientation.x = 0
pos_global.orientation.y = 0
pos_global.orientation.z = 0
pos_global.orientation.w = 1

# these positions are definted relative to the orientation of the quad
sp 	= {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0} # set point
pos = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0} # current position

output = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}

old_err   = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
err   = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Pterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Iterm = {'fb': 0.0, 'lr': 0.0, 'alt': 50.0, 'yaw': 0.0}
Dterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
old_old_Dterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
old_Dterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
new_Dterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}

def pid():
    rc = RC()
    time_prev = millis()
    while not rospy.is_shutdown():
        global sp_global
        global pos_global
        calced_yaw = -calc_yaw_from_quat((pos_global.orientation.x, pos_global.orientation.y,
            pos_global.orientation.z, pos_global.orientation.w))
        
        time.sleep(0.001)
        time_elapsed = millis() - time_prev
        time_prev = millis() 

         
# Throwaway variables
        sp_global_yaw = sp_global.orientation.w
        #(ahh, pls, sp_global_yaw) = tf.transformations.euler_from_quaternion([sp_global.orientation.x, sp_global.orientation.y, sp_global.orientation.z, sp_global.orientation.w])
        (work, now, pos_global_yaw) = tf.transformations.euler_from_quaternion([pos_global.orientation.x, pos_global.orientation.y, pos_global.orientation.z, pos_global.orientation.w])
#       (pos_global_pitch, pos_global_yaw, pos_global_roll) = (0, calced_yaw, 0)

        # convert to the quad's frame of reference from the global
        global sp
        global pos
	# XXX jgo: is this supposed to be multiplication of the vector ~_global
	# (in the xz plane) by the rotation matrix induced by ~_global_yaw?  if
	# so, could you be missing a minus sign in front of one of the sin functions?

# The bottom left sin is negative so that we get the negative rotation, since
# we are converting to relative coordinates later
#       sp['fb'] = math.cos(pos_global_yaw) * sp_global.position.y + math.sin(pos_global_yaw) * sp_global.position.x
#       sp['lr'] = -math.sin(pos_global_yaw) * sp_global.position.y + math.cos(pos_global_yaw) * sp_global.position.x

#       pos['fb'] = math.cos(pos_global_yaw) * pos_global.position.y + math.sin(pos_global_yaw) * pos_global.position.x
#       pos['lr'] = -math.sin(pos_global_yaw) * pos_global.position.y + math.cos(pos_global_yaw) * pos_global.position.x

        error = {'x': sp_global.position.x - pos_global.position.x, 'y':
        sp_global.position.y - pos_global.position.y}

        #sp['fb'] = math.cos(pos_global_yaw) * err['y'] + math.sin(pos_global_yaw) * err['x']
        #sp['lr'] = - math.cos(pos_global_yaw) * err['x'] + math.sin(pos_global_yaw) * err['y']
        #print sp['fb'], sp['lr']
        pos['fb'] = 0
        pos['lr'] = 0

        #sp['yaw'] = sp_global_yaw - pos_global_yaw
        pos['yaw'] = 0.0

        sp['alt'] = sp_global.position.z - pos_global.position.z
        pos['alt'] = 0.0
	
        # DIFFERENCE OF ANGLES
        err_norm = np.sqrt(error['x']**2 + error['y']**2)
        err_angle = -math.atan2(error['x'], error['y'])
        diff_angle = pos_global_yaw - err_angle
        sp['fb'] = np.cos(diff_angle) * err_norm
        sp['lr'] = np.sin(diff_angle) * err_norm

        # XXX jgo: also it seems like you are setting "sp" yaw and alt relative
	# to the values held by "pos", and setting "pos" values to 0,
	# indicating that both are in the reference frame of "pos". Yet, the fb
	# and lr values of "sp" and "pos" are both set relative to their own
	# yaw values. Do you perhaps mean to use pos_global_yaw rather than
	# sp_global_yaw, and maybe -pos_global_yaw in both cases rather than
	# pos_global_yaw? (this sign matter interacts with whether and where a minus sign
	# should go in front of a sin term above). my suspicion is reinforced,
	# figuring that the output feeds to the roll and pitch of the aircraft in its current
	# position, it seems like you want the fb and lr of both "sp" and "pos"
	# represented in the frame of "pos".  

	# XXX jgo: my apologies if I'm misinterpreting this.


        global old_err
        global old_pos_global
        if pos_global != old_pos_global:
            for key in sp.keys(): 
                err[key] = sp[key] - pos[key] # update the error
                #if key == 'yaw':
                    # print(sp[key], pos[key])

                # calc the PID components of each axis
                Pterm[key] = err[key]
                # XXX jgo: this is a very literal interpretation of the I term
                # which might be difficult to tune for reasons which we can
                # discuss.  it is more typical to take the integral over a finite
                # interval in the past. This can be implemented with a ring buffer,
                # or quickly approximated with an exponential moving average.
                Iterm[key] += err[key] * time_elapsed
                if key == 'alt' and Iterm[key] < 0:
                    Iterm[key] = 0
                # XXX jgo: the sign of Dterm * kd should act as a viscosity or
                # resistance term.  if our error goes from 5 to 4, 
                # then Dterm ~ 4 - 5 = -1; so it looks like kd should be a positive number.
                if key != 'alt':
                    old_old_Dterm[key] = old_Dterm[key]
                    old_Dterm[key] = new_Dterm[key]
                    new_Dterm[key] = (err[key] - old_err[key])/time_elapsed
                    Dterm[key] = (new_Dterm[key] * 8.0 + old_Dterm[key] * 5.0 +
                    old_old_Dterm[key] * 2.0)/15.0
                else:
                    Dterm[key] = (err[key] - old_err[key])/time_elapsed
                # XXX jgo: definitely get something working with I and D terms equal to 0 before using these

                old_err[key] = err[key]
                if key == 'alt' and Pterm[key] < 0:
                    output[key] = Pterm[key] * kp['alt_above'] + Iterm[key] * ki[key] + Dterm[key] * kd[key]
                elif key == 'alt':
                    output[key] = Pterm[key] * kp[key] + Iterm[key] * ki[key] + Dterm[key] * kd[key]
                else:
                    output[key] = Pterm[key] * kp[key] + Iterm[key] * ki[key] + Dterm[key] * kd[key]
            old_pos_global = deepcopy(pos_global)
    
        # calculate the thrust and desired angle
        # (thrust, theta) = calc_thrust_and_theta(output['lr'], output['alt'], output['fb'])
        # and use that to calculate roll pitch yaw
#       (pitch, yaw, roll) = calc_roll_pitch_from_theta(output['lr'], output['fb'], theta)
        rc.roll = max(1000, min(1500 + output['lr'], 2000))
        rc.pitch = max(1000, min(1500 + output['fb'], 2000))
        rc.yaw = max(1000, min(1500 + output['yaw'], 2000))
        rc.throttle = max(1150, min(1200 + output['alt'], 2000))
        rc.aux1 = 1800
        rc.aux2 = 1500
        rc.aux3 = 1500
        rc.aux4 = 1500
        #print Pterm['alt'] * kp['alt'], Dterm['alt'] * kd['alt'], Iterm['alt'] * ki['alt']
        print rc.roll, rc.pitch, rc.yaw, rc.throttle
        #print rc.roll > 1500, rc.pitch > 1500
        #print(str(roll) + "\t" + str(pitch) + "\t" + str(yaw))
        # print pos_global
        # print 'TIME ELAPSED:', rospy.Time.now().to_sec() - pos_time.to_sec()
        cmdpub.publish(rc)

# rotate vector v1 by quaternion q1 
def qv_mult(q1, v1):
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]
    

def calc_yaw_from_quat(q):
    v = qv_mult(q,(1,0,0))
    yaw = math.atan2(v[2], v[0])
    return yaw


def update_sp(data):
    global sp_global
    print 'UPDATING SET POINT'
    sp_global = data.pose
    sp_global.orientation.w = data.pose.orientation.w

def update_pos(data):
    global pos_global
    global pos_time
    pos_global = data.pose
    pos_time = data.header.stamp

if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    try:
        rospy.Subscriber("/pidrone/est_pos", PoseStamped, update_pos)
        rospy.Subscriber("/pidrone/target_pos", PoseStamped, update_sp)
        time.sleep(0.5)
        global sp_global
        global pos_global
        sp_global.position.x = 0
        sp_global.position.y = 0
        sp_global.position.z = 80
        sp_global.orientation.w = 1
        time.sleep(0.1)
        pid()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


