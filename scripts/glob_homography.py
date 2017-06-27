from camera_class import Camera
from homography_class import Homography
from pid_class import PID
import rospy
from geometry_msgs.msg import PoseStamped
from copy import deepcopy

camera = Camera()
homography = Homography()
pid = PID()

if __name__ == '__main__':
    rospy.init_node("glob_homography")
    rospy.Subscriber("/pidrone/target_pos", PoseStamped, pid.update_setpoint)
    curr_img = None
    prev_img = camera.getImage().next()
    aruco_found = False
    position = None
    for curr_img in camera.getImage():
        if not aruco_found:
            aruco_found = homography.findAruco(curr_img)
            prev_image = deepcopy(curr_img)
            print("Looking for ARuco")
        else:
            if position is None:
                position = homography.updatePos(curr_img, prev_img)
                print(position)
            else:
                position = homography.updatePos(curr_img)
                print(position)
