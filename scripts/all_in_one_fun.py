from h2rPiCam import streamPi
from pidrone_pkg.msg import RC, ERR, axes_err
from homography_integrator_class import HomographyIntegrator
from pid_class import PID
import rospy
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from h2rMultiWii import MultiWii  
import time
homo = HomographyIntegrator()
pid = PID()
board = MultiWii("/dev/ttyACM0")
ready_to_fly = False
pos = None

if __name__ == '__main__':
    try:
        rospy.init_node("glob_homography")
        rospy.Subscriber("/pidrone/target_pos", PoseStamped, pid.update_setpoint)
        rospy.Subscriber("/pidrone/est_pos", PoseStamped, vrpn_update_pos)
        curr_img = None
        position = None
        for curr_img in streamPi():
            if position is None:
                position = homography.updatePos(curr_img, prev_img)
                pid.update_setpoint(deepcopy(position))
                board.arm()
                flying = True
            else:

        	    data = board.getData(MultiWii.ATTITUDE)
			    y = data['heading']/180.0*np.pi
			    r = data['angx']/180.0*np.pi
			    p = data['angy']/180.0*np.pi
			    q = np.array(tf.transformations.quaternion_from_euler(-p, r, -y))

			    homo.fix_position(tf.transformations.matrix_from_quaternion)
                position = homo.step(curr_img)

                cmds = pid.step(position)
                print position, cmds
                board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
        
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
        sp = deepcopy(pos)
        sp.pose.position.z += 5
        # sp.pose.position.y += 20
        # sp.pose.orientation.x = 0
        # sp.pose.orientation.y = 0
        # sp.pose.orientation.z = 0
        # sp.pose.orientation.w = 1
        pid.update_setpoint(sp)
        # while not rospy.is_shutdown():
        ready_to_fly = True
        rospy.spin()
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
        raise