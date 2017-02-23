#!/usr/bin/env python
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped
import time
import tf

millis = lambda: int(round(time.time() * 1000))

kp = {
	'roll': 	0.1,
	'pitch': 	0.1,
	'yaw': 		0.1,
	'alt': 		0.1
}

ki = {
	'roll': 	0.0,
	'pitch':	0.0,
	'yaw': 		0.0,
	'alt': 		0.0
}

kd = {
	'roll': 	0.0,
	'pitch': 	0.0,
	'yaw': 		0.0,
	'alt': 		0.0
}


# these positions are global (ie what comes out of the motion tracker)
sp_global  = Pose() # set point
pos_global = Pose() # set point

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

err   = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Pterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Iterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Dterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}

time_prev = millis()

def pid():
    cmdpub = rospy.Publisher('/pidrone/commands', RC, queue_size=1)
    rc = RC()
    while not rospy.is_shutdown():
        time_elapsed = millis() - time_prev
        time_prev = millis()

        (sp_global_roll, sp_global_pitch, sp_global_yaw) = tf.transformations.euler_from_quaternion([sp_global.orientation.x, sp_global.orientation.y, sp_global.orientation.z, sp_global.orientation.w])

        (pos_global_roll, pos_global_pitch, pos_global_yaw) = tf.transformations.euler_from_quaternion([pos_global.orientation.x, pos_global.orientation.y, pos_global.orientation.z, pos_global.orientation.w])

        # convert to the quad's frame of reference from the global
        sp['fb'] = math.cos(sp_global_yaw) * sp_global['z'] + math.sin(sp_global_yaw) * sp_global['x']
        sp['lr'] = math.sin(sp_global_yaw) * sp_global['z'] + math.cos(sp_global_yaw) * sp_global['x']

        pos['fb'] = math.cos(pos_global_yaw) * pos_global['z'] + math.sin(pos_global_yaw) * pos_global['x']
        pos['lr'] = math.sin(pos_global_yaw) * pos_global['z'] + math.cos(pos_global_yaw) * pos_global['x']

        sp = sp_global_yaw - pos_global_yaw
        pos['yaw'] = 0.0

        sp = sp_global.position.y - pos_global.position.y
        pos['alt'] = 0.0


        old_err = err
        for key in sp.keys(): 
            err[key] = sp[key] - pos[key] # update the error

            # calc the PID components of each axis
            Pterm[key] = err[key]
            Iterm[key] += err[key] * time_elapsed
            Dterm[key] = (err[key] - old_err[key])/time_elapsed

            output[key] = Pterm[key] * kp[key] + Iterm[key] * kI[key] + Dterm[key] * kd[key]
        rc.roll = max(1000, min(1500 + output['lr'], 2000))
        rc.pitch = max(1000, min(1500 + output['fb'], 2000))
        rc.yaw = max(1000, min(1500 + output['yaw'], 2000))
        rc.throttle = max(1000, min(1500 + output['alt'], 2000))
        rc.aux1 = 1500
        rc.aux2 = 1500
        rc.aux3 = 1500
        rc.aux4 = 1500

def update_sp(data):
    sp_global = data

def update_pos(data):
    pos_global = data.pose

if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    try:
        rospy.Subscriber("/pidrone/target_position", Pose, update_sp)
        rospy.Subscriber("vrpn", Pose, update_pos)
        pid()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except Exception,error:
        print "Error on Main: "+str(error)

