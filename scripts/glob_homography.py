# from camera_class import Camera
from pidrone_pkg.msg import RC, axes_err
# from homography_class import Homography
from pid_class import PID
import rospy
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from h2rMultiWii import MultiWii  
import time
import numpy as np
import tf
from std_msgs.msg import Empty

# camera = Camera()
# homography = Homography()
pid = PID()
board = MultiWii("/dev/ttyACM0")
ready_to_fly = False
pos = None
errpub = rospy.Publisher('/pidrone/error', axes_err, queue_size=1)
cmdpub = rospy.Publisher('/pidrone/commands_new', RC, queue_size=1)
sp = None

def reset_callback(data):
    print "RESETTING"
    global sp
    global pos
    global pid
    # pid.throttle._i = 0
    # pid.roll._i = 0
    # pid.pitch._i = 0
    # pid.yaw._i = 0
    sp = deepcopy(pos)
    pid.update_setpoint(sp)

def publish_att(attpub, att_pos):
    data = board.getData(MultiWii.ATTITUDE)
    y = data['heading']/180.0*np.pi
    r = data['angx']/180.0*np.pi
    p = data['angy']/180.0*np.pi
    # print -p, r, -y
    q = np.array(tf.transformations.quaternion_from_euler(-p, r, -y))
    att_pos.pose.orientation.x = q[0]
    att_pos.pose.orientation.y = q[1]
    att_pos.pose.orientation.z = q[2]
    att_pos.pose.orientation.w = q[3]
    attpub.publish(att_pos)


def vrpn_update_pos(data):
    global ready_to_fly
    global pos
    global board
    global pid
    try:
        pos = data
        error = axes_err()
        if ready_to_fly:
            cmds = pid.step(pos, error)
            errpub.publish(error)
            # print error
            rc = RC()
            rc.roll = cmds[0]
            rc.pitch = cmds[1]
            rc.yaw = cmds[2]
            rc.throttle = cmds[3]
            cmdpub.publish(rc)
            print cmds
            board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
    except Exception as e:
        print("Caught exception in callback", e)
        board.disarm()
        ready_to_fly = False

if __name__ == '__main__':
    try:
        rospy.init_node("glob_homography")
        rospy.Subscriber("/pidrone/target_pos", PoseStamped, pid.update_setpoint)
        # rospy.Subscriber("/pidrone/reset_pos", Empty, reset_callback)
        rospy.Subscriber("/pidrone/homo_pos", PoseStamped, vrpn_update_pos)
        attpub = rospy.Publisher('/pidrone/multiwii_attitude', PoseStamped, queue_size=1)
        att_pos = PoseStamped()
        att_pos.header.frame_id = 'world'
        # curr_img = None
        # prev_img = camera.getImage().next()
        # aruco_found = False
        # position = None
        # start_time = rospy.Time()
        # for curr_img in camera.getImage():
        #     if not aruco_found:
        #         aruco_found = homography.findAruco(curr_img)
        #         prev_image = deepcopy(curr_img)
        #         print("Looking for ARuco")
        #     else:
        #         if position is None:
        #             position = homography.updatePos(curr_img, prev_img)
        #             pid.update_setpoint(deepcopy(position))
        #             if flying:
                        # board.arm()
        #         else:
        #             position = homography.updatePos(curr_img)
        #             cmds = pid.step(position)
        #             print position, cmds
        #             board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
        #             print 'TIME ELAPSED:', rospy.Time.now().to_sec() - \
        #             start_time.to_sec()
            # start_time = rospy.Time.now()
        
        board.arm()
        print 'Waiting for position estimate'
        while pos is None:
            pass
            time.sleep(0.001)
        print 'Received position. Starting'
        # sp = PoseStamped()
        # sp.pose.position.x = 0
        # sp.pose.position.y = 0
        # sp.pose.position.z = 10
        # sp.pose.orientation.w = 1
        global sp
        sp = deepcopy(pos)
        sp.pose.position.z += 10
        # sp.pose.position.y += 20
        # sp.pose.orientation.x = 0
        # sp.pose.orientation.y = 0
        # sp.pose.orientation.z = 0
        # sp.pose.orientation.w = 1
        pid.update_setpoint(sp)
        # while not rospy.is_shutdown():
        ready_to_fly = True
        while not rospy.is_shutdown():
            publish_att(attpub, att_pos)
        ready_to_fly = False
        # while ready_to_fly:
            
        #     time.sleep(0.0001)
        print("CONTROL DISARM")
        board.disarm()
        # camera.stream.terminate()
    except:
        print("ERROR DISARM")
        board.disarm()
        # camera.stream.terminate()
       
