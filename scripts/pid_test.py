#!/usr/bin/env python
from __future__ import division
import rospy
from pidrone_pkg.msg import RC
from geometry_msgs.msg import Pose, PoseStamped
import time
import tf
import math
import numpy as np
import sys
from copy import deepcopy

to_exit = False
cmdpub = rospy.Publisher('/pidrone/target_pos', PoseStamped, queue_size=1)
pos = None # the current position of the drone
initial_pos = PoseStamped() # the starting position the drone
sp = PoseStamped() # the current set point (in global coords)
# commands = [ # the sequence of commands to send the drone [x,y,z,yaw]
# [0,0,0,0],
# [0,0,80,0],
# [40,40,80,0],
# [-40,40,80,0],
# [-40,-40,80,0],
# [0,0,80,0],
# [0,0,40,0],
# [0,0,100,0],
# [0,0,60,0],
# [0,0,20,0]]
# [0,0,80,np.pi],
# [0,0,80,np.pi/2.0],
# [0,0,80,-np.pi/4.0]]
# commands = [[0, 0, 0, 0],
#             [-50, -50, 100, np.pi/4],
#             [-50, 50, 100, 3 * np.pi / 4],
#             [50, 50, 100, -3 * np.pi/4],
#             [50, -50, 100, -1 * np.pi/4],
#             [-50, -50, 100, 0],
#             [-50, -50, 200, 0],
#             [-50, 50, 200, 0],
#             [50, 50, 200, 0],
#             [50, -50, 200, 0],
#             [-50, -50, 200, 0],
#             [0, 0, 0, 0]]
# commands = [
#         [0,0,0,0],
#         [0,0,80,0],
#         [0,-40,80,0],
#         [40,-40,80,0],
#         [40,40,80,0],
#         [40,-40,80,0],
#         [-40,-40,80,0],
#         [-40,40,80,0],
#         [-40,-40,80,0],
#         [0,-40,80,0],
#         [0,0,80,0],
#         [0,0,0,0]]


# commands = [
#         [0,0,0,0],
#         [0,0,80,0],
#         [0,-50,80,0],
#         [0,50,80,0],
#         [0,-50,80,0],
#         [0,50,80,0],
#         [0,0,80,0],
#         [0,0,0,0]]

# commands = [
#         [0,0,0,0],
#         [0,0,80,0],
#         [-50,0,80,np.pi/2],
#         [50,0,80, -3 * np.pi/4],
#         [-50,0,80,np.pi/4],
#         [50,0,80,np.pi/2],
#         [0,0,80,0],
#         [0,0,0,0]]

commands = [
        [0,0,0,0],
        [0,0,120,0],
        # [50,0,120,0],
        # [-50,0,120,0],
        # [50,0,120,0],
        # [-50,0,120,0],
        # [0,0,120,0],
        [0,0,0,0]]

# commands = [
#         [0, 0, 0, 0],
#         [0, 0, 80, 0],
#         [0, 0, 180, 0],
#         [0, 0, 80, 0],
#         [0, 0, 180, 0],
#         [0, 0, 0, 0]]

MSE = [0,0,0] # mean squared error [plane, alt, yaw]
num_readings = 0 # number of pos readings to calculate error
# start_time = rospy.Time()
test_started = False
test_completed = False
def quat_to_rpy(q):
    """ takes in a quaternion (like from a pose message) and returns (roll, pitch, yaw) """
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


def gen_setpoints(start, end, rate=None, num_steps=None, step_size=None):
    # assert not (num_steps is not None and step_size is not None)
    # assert not (num_steps is None and step_size is None)
    diff = Pose()
    diff.position.x = end.pose.position.x - start.pose.position.x
    diff.position.y = end.pose.position.y - start.pose.position.y
    diff.position.z = end.pose.position.z - start.pose.position.z
    print start.pose.orientation,  "\n", end.pose.orientation
    diff.orientation.w = end.pose.orientation.w - start.pose.orientation.w 
    curr_pos = deepcopy(start)
    curr_pos.header.frame_id = 'world'
    curr_yaw = start.pose.orientation.w
    pos_list = [deepcopy(start)]
   
    global sp
    freq = 120
    des_time = 1.0/float(freq)

    if rate is not None:
        time_plane = np.sqrt(diff.position.x**2 + diff.position.y**2)/rate[0]
        time_alt = np.absolute(diff.position.z)/rate[1]
        time_max = np.ceil(max(time_plane, time_alt))
        num_steps = np.ceil(freq * time_max)
       
    if num_steps is not None:
        num_steps = int(num_steps)
        for s in range(num_steps):
            start_time = time.time()
            curr_pos.pose.position.x += diff.position.x/float(num_steps)
            curr_pos.pose.position.y += diff.position.y/float(num_steps)
            curr_pos.pose.position.z += diff.position.z/float(num_steps)
            curr_yaw += diff.orientation.w / float(num_steps)
            orientation_array = tf.transformations.quaternion_from_euler(0,0,curr_yaw)
            # print orientation_array
            curr_pos.pose.orientation.x = orientation_array[0]
            curr_pos.pose.orientation.y = orientation_array[1]
            curr_pos.pose.orientation.z = orientation_array[2]
            curr_pos.pose.orientation.w = orientation_array[3]
            sp = curr_pos
            #print(curr_pos)
            cmdpub.publish(curr_pos)
            time_elapsed = time.time() - start_time
            if des_time - time_elapsed > 0: time.sleep(des_time - time_elapsed)


    # if step_size is not None:
    #     num_steps_plane = np.sqrt(diff.position.x**2 + diff.position.y**2)/step_size[0]
    #     num_steps_alt = np.absolute(diff.position.z)/step_size[1]
    #     num_steps = np.ceil(max(num_steps_plane, num_steps_alt))
 
    # if num_steps is not None:
    #     num_steps = int(num_steps)
    #     for s in range(num_steps):
    #         curr_pos.pose.position.x += diff.position.x/float(num_steps)
    #         curr_pos.pose.position.y += diff.position.y/float(num_steps)
    #         curr_pos.pose.position.z += diff.position.z/float(num_steps)
    #         curr_yaw += diff.orientation.w / float(num_steps)
    #         orientation_array = tf.transformations.quaternion_from_euler(0,0,curr_yaw)
    #         curr_pos.pose.orientation.x = orientation_array[0]
    #         curr_pos.pose.orientation.y = orientation_array[1]
    #         curr_pos.pose.orientation.z = orientation_array[2]
    #         curr_pos.pose.orientation.w = orientation_array[3]
    #         pos_list.append(deepcopy(curr_pos))
    # else:
    #     return None
    
    return pos_list


def run_test():
    old_cmd = None
    for cmd in commands:
        # print cmd
        new_cmd = PoseStamped()
        new_cmd.pose.position.x = cmd[0] + initial_pos.pose.position.x
        new_cmd.pose.position.y = cmd[1] + initial_pos.pose.position.y
        new_cmd.pose.position.z = cmd[2] + initial_pos.pose.position.z
        new_cmd.pose.orientation.w = cmd[3]
        if old_cmd is not None:
            gen_setpoints(old_cmd, new_cmd, rate = (45,25))
            time.sleep(10)
        old_cmd = deepcopy(new_cmd)

def update_pos(data):
    global pos
    global initial_pos
    global MSE
    global num_readings
    # global start_time
    global sp
    global test_started
    global test_completed
    if pos is None:
        initial_pos = data
        pos = data
    
    if test_started and not test_completed:
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
 
    elif test_completed:
        pos = data
        print 'FINISHED'
        print 'PLANE\t', MSE[0]/num_readings
        print 'ALT\t', MSE[1]/num_readings
        print 'YAW\t', MSE[2]/num_readings
        # land()
        global to_exit
        to_exit = True
 
    # elif rospy.Time.now().to_sec() - start_time.to_sec() > 20:
    #     pos = data
    #     print 'FINISHED'
    #     print 'PLANE\t', MSE[0]/num_readings
    #     print 'ALT\t', MSE[1]/num_readings
    #     print 'YAW\t', MSE[2]/num_readings
    #     # land()
    #     global to_exit
    #     to_exit = True
    #     sys.exit(1)
    # elif rospy.Time.now().to_sec() - start_time.to_sec() > 10:
    #     pos = data
    #     err_plane = (sp.pose.position.x - pos.pose.position.x)**2 + \
    #     (sp.pose.position.y - pos.pose.position.y)**2
    #     err_alt = (sp.pose.position.z - pos.pose.position.z)**2
    #     _,_,pos_yaw = tf.transformations.euler_from_quaternion([pos.pose.orientation.x,
    #     pos.pose.orientation.y, pos.pose.orientation.z,
    #     pos.pose.orientation.w])
    #     err_yaw = (sp.pose.orientation.w - pos_yaw)**2
    #     MSE[0] += err_plane
    #     MSE[1] += err_alt
    #     MSE[2] += err_yaw
    #     num_readings += 1
    
if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    start_time = rospy.Time.now()
    try:
        rospy.Subscriber("/pidrone/est_pos", PoseStamped, update_pos)
        print 'Waiting for the mo cap'
        while pos is None:
            time.sleep(0.01)
        test_started = True
        run_test()
        test_completed = True
        while True:
            if to_exit:
                sys.exit(1)
            else:
                time.sleep(0.001)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

