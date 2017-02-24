#!/usr/bin/env python
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped
import time
import tf
import math

millis = lambda: int(round(time.time() * 1000))

kp = {
	'lr': 	0,
	'fb': 	100,
	'yaw': 		0,
	'alt': 		000
}

ki = {
	'lr': 	0.0,
	'fb':	0.0,
	'yaw': 		0.0,
	'alt': 		0.0
} 
kd = {
	'lr': 	0.0,
	'fb': 	0.0,
	'yaw': 		0.0,
	'alt': 		0.0
}


# these positions are global (ie what comes out of the motion tracker)
sp_global  = Pose() # set point
pos_global = Pose() # set point

sp_global.position.x = -0.04
sp_global.position.y = 0.6
sp_global.position.z = 0.185
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

err   = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Pterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Iterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Dterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}

def pid():
    cmdpub = rospy.Publisher('/pidrone/commands', RC, queue_size=1)
    rc = RC()
    time_prev = millis()
    while not rospy.is_shutdown():
        global sp_global
        global pos_global
        time.sleep(0.05)
        time_elapsed = millis() - time_prev
        time_prev = millis() 
        (sp_global_roll, sp_global_pitch, sp_global_yaw) = tf.transformations.euler_from_quaternion([sp_global.orientation.x, sp_global.orientation.y, sp_global.orientation.z, sp_global.orientation.w])

        (pos_global_roll, pos_global_pitch, pos_global_yaw) = tf.transformations.euler_from_quaternion([pos_global.orientation.x, pos_global.orientation.y, pos_global.orientation.z, pos_global.orientation.w])

        print((pos_global_roll, pos_global_yaw, pos_global_pitch))

        # convert to the quad's frame of reference from the global
        global sp
        global pos
        sp['fb'] = math.cos(sp_global_yaw) * sp_global.position.z + math.sin(sp_global_yaw) * sp_global.position.x
        sp['lr'] = math.sin(sp_global_yaw) * sp_global.position.z + math.cos(sp_global_yaw) * sp_global.position.x

        pos['fb'] = math.cos(pos_global_yaw) * pos_global.position.z + math.sin(pos_global_yaw) * pos_global.position.x
        pos['lr'] = math.sin(pos_global_yaw) * pos_global.position.z + math.cos(pos_global_yaw) * pos_global.position.x

        sp['yaw'] = sp_global_yaw - pos_global_yaw
        pos['yaw'] = 0.0

        sp['alt'] = sp_global.position.y - pos_global.position.y
        pos['alt'] = 0.0


        old_err = err
        for key in sp.keys(): 
            err[key] = sp[key] - pos[key] # update the error

            # calc the PID components of each axis
            Pterm[key] = err[key]
            Iterm[key] += err[key] * time_elapsed
            Dterm[key] = (err[key] - old_err[key])/time_elapsed

            output[key] = Pterm[key] * kp[key] + Iterm[key] * ki[key] + Dterm[key] * kd[key]
        rc.roll = max(1000, min(1500 + output['lr'], 2000))
        rc.pitch = max(1000, min(1500 + output['fb'], 2000))
        rc.yaw = max(1000, min(1500 + output['yaw'], 2000))
        rc.throttle = max(1000, min(1500 + output['alt'], 2000))
        rc.aux1 = 1500
        rc.aux2 = 1500
        rc.aux3 = 1500
        rc.aux4 = 1500
        cmdpub.publish(rc)

def update_sp(data):
    global sp_global
    sp_global = data.pose
    sp_global.position.y -= 0.2

def update_pos(data):
    global pos_global
    pos_global = data.pose

if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    try:
        rospy.Subscriber("/vrpn_client_node/wand/pose", PoseStamped, update_sp)
        rospy.Subscriber("/vrpn_client_node/drone/pose", PoseStamped, update_pos)
        pid()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

