import rospy
from geometry_msgs.msg import PoseStamped
from pidrone_pkg.msg import axes_err, Mode
import numpy as np
from copy import deepcopy 
vrpn_prev_time = 0
vrpn_prev = np.array([0,0,0], dtype='float')
vrpn_curr = np.array([0,0,0], dtype='float')
vrpn_speed = np.array([0,0,0], dtype='float')
flow_speed = np.array([0,0,0], dtype='float')

def vrpn_callback(data):
    global vrpn_curr, vrpn_speed, vrpn_prev_pos, vrpn_prev_time
    dt = data.header.stamp.to_sec() - vrpn_prev_time
    vrpn_prev = deepcopy(vrpn_curr)
    vrpn_curr[0] = data.pose.position.x
    vrpn_curr[1] = data.pose.position.y
    vrpn_curr[2] = data.pose.position.z
    vrpn_speed = (vrpn_curr - vrpn_prev) / dt
    vrpn_prev_time = data.header.stamp.to_sec()

def flow_callback(data):
    global flow_speed, vrpn_curr
    flow_speed[0] = data.x.err * vrpn_curr[2]
    flow_speed[1] = data.y.err * vrpn_curr[2]
    flow_speed[2] = data.z.err * vrpn_curr[2]

if __name__ == "__main__":
    global flow_speed, vrpn_speed
    rospy.init_node("flow_coefficient_tester")
    rospy.Subscriber("/pidrone/plane_err", axes_err, flow_callback)
    rospy.Subscriber("/pidrone/vrpn_pos", PoseStamped, vrpn_callback)
    coeffpub = rospy.Publisher("/pidrone/flow_coeff", axes_err, queue_size=1)
    r = rospy.Rate(100)
    counter = 0
    tot = np.array([0,0,0], dtype='float')
    coeff = axes_err()
    while not rospy.is_shutdown():
        c = vrpn_speed/flow_speed
        for i in range(3):
            if np.absolute(c[i]) > 5000.0: c[i] = 0
        (coeff.x.err, coeff.y.err, coeff.z.err) = c
        coeffpub.publish(coeff) 
        r.sleep()
