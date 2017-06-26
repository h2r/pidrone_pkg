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

cmdpub = rospy.Publisher('/pidrone/target_pos', PoseStamped, queue_size=1)
pos = None # the current position of the drone
initial_pos = PoseStamped() # the starting position the drone
sp = PoseStamped() # the current set point (in global coords)
commands = [ # the sequence of commands to send the drone [x,y,z,yaw]
[0,0,0,0],
[0,0,80,0],
[40,40,80,0],
[-40,40,80,0],
[-40,-40,80,0],
[0,0,80,0],
[0,0,40,0],
[0,0,100,0],
[0,0,60,0],
[0,0,20,0]]
#[0,0,80,np.pi],
#[0,0,80,np.pi/2.0],
#[0,0,80,-np.pi/4.0]]
MSE = [0,0,0] # mean squared error [plane, alt, yaw]
num_readings = 0 # number of pos readings to calculate error
def run_test():
    for cmd in commands:
        print cmd
        sp.pose.position.x = cmd[0] + initial_pos.pose.position.x
        sp.pose.position.y = cmd[1] + initial_pos.pose.position.y
        sp.pose.position.z = cmd[2] + initial_pos.pose.position.z
        sp.pose.orientation.w = cmd[3]
        cmdpub.publish(sp)
        time.sleep(2) # give the drone two seconds to get to its next pos        
    print 'FINISHED'
    print 'PLANE\t', MSE[0]/num_readings
    print 'ALT\t', MSE[1]/num_readings
    print 'YAW\t', MSE[2]/num_readings
    sp.pose.position = initial_pos.pose.position
    cmdpub.publish(sp)
    
def update_pos(data):
    global pos
    global initial_pos
    global MSE
    global num_readings
    if pos is None:
        initial_pos = data
        pos = data
    else:
        pos = data
        err_plane = (sp.pose.position.x - pos.pose.position.x)**2 + \
        (sp.pose.position.y - pos.pose.position.y)**2
        err_alt = (sp.pose.position.z - pos.pose.position.z)**2
        _,_,pos_yaw = tf.transformations.euler_from_quaternion([pos.pose.orientation.x,
        pos.pose.orientation.y, pos.pose.orientation.z,
        pos.pose.orientation.w])
        err_yaw = (sp.pose.orientation.w - pos_yaw)**2
        MSE[0] += err_plane
        MSE[1] += err_alt
        MSE[2] += err_yaw
        num_readings += 1
if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    try:
        rospy.Subscriber("/pidrone/est_pos", PoseStamped, update_pos)
        print 'Waiting for the mo cap'
        while pos is None:
            time.sleep(0.01)
        run_test()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


