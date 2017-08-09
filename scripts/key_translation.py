import rospy
from pidrone_pkg.msg import Mode





def main():
    rospy.init_node("key_translation")
    import sys, tty, termios
    fd = sys.stdin.fileno()
    tty.setraw(sys.stdin.fileno())
    mode = Mode()
    mode.mode = 4
    modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown(): 
        ch = sys.stdin.read(1)
        if ch == "j":
            mode.mode = 4
        elif ch == "k":
            mode.mode = 0

        elif ch == "a":
            mode.mode = 5
            mode.x_velocity = 10
        elif ch == "d":
            mode.mode = 5
            mode.x_velocity = -10
        else:
            pass
        
        modepub.publish(mode)
        rate.sleep()


        

                                                    
if __name__ == "__main__":
    main()
