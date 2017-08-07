import rospy
from pidrone_pkg.msg import Mode
from geometry_msgs.msg import PoseStamped
import numpy as np
from copy import deepcopy

mode = Mode()
mode.mode = 5
modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)
max_accel = np.array([1., 1., 1, 1]) # cm/s/s
movements = []
pub_frequency = 10.

def sp_callback(data):
    global movements
    movement = np.array([data.pose.position.x, 
                         data.pose.position.y,
                         0,
                         0])
    movements.append(movement)
    

if __name__ == "__main__":
    rospy.init_node("acceleration_controller")
    rospy.Subscriber("/pidrone/pos_setpoint", PoseStamped, sp_callback)
    global max_accel, movements, pub_frequency
    while not rospy.is_shutdown():
        if len(movements) > 0:
            # movement is the desired diplacement vector
            movement = movements.pop(0)
            movement /= 2.
            travel_time = np.max(np.sqrt(2. * np.abs(movement)/max_accel)) 
            # the desired acceleration in each axes to ramp smoothly
            adjusted_accel = (2. * movement) / (travel_time ** 2)
            # the number of velocity points to ramp up 
            num_points = int(np.floor(pub_frequency * travel_time))
            # generate the list of desired velocities to ramp up
            command_list = []
            for i in range(1, num_points+1):
                command_list.append((i / pub_frequency) * adjusted_accel)
            
            command_list_r = deepcopy(command_list)
            command_list_r.reverse()
            command_list += command_list_r # mirror to ramp back down  
            command_list.append(np.array([0., 0., 0., 0.]))
            
            for cmd in command_list:
                mode.x_velocity = cmd[0]
                mode.y_velocity = cmd[1]
                mode.z_velocity = cmd[2]
                mode.yaw_velocity = cmd[3]
                print mode
                modepub.publish(mode)
                rospy.sleep(1. / pub_frequency)
