import rospy
from pidrone_pkg.msg import Mode
import getch
import time


def main():
    rospy.init_node("key_translation")
    import sys, tty, termios
    fd = sys.stdin.fileno()
    #attr = termios.tcgetattr(sys.stdin.fileno())
    #tty.setraw(sys.stdin.fileno())
    mode = Mode()
    mode.mode = 4
    modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)
    rate = rospy.Rate(10)
    msg = """
Commands: 
j:  arm
k:  disarm
t:  takeoff
q:  quit
"""
    try:
        while not rospy.is_shutdown():
            print msg
            ch = getch.getch()
            if ch == "k":
                mode.mode = 4
            elif ch == "j":
                mode.mode = 0
            elif ch == "a":
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 5
            elif ch == "d":
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 5
            elif ch == "0":
                mode.mode = 0
            elif ch == "1":
                mode.mode = 1
            elif ch == "2":
                mode.mode = 2
            elif ch == "3":
                mode.mode = 3
            elif ch == "4":
                mode.mode = 4
            elif ch == "5":
                mode.mode = 5
            elif ch == "t":
                mode.mode = 2
            elif ch == "y":
                mode.mode = 1
            elif ch == "q":
                break
            elif ord(ch) == 3:
                break
            else:
                print "unknown character: '%d'" % ord(ch)
                pass

            modepub.publish(mode)
            rate.sleep()

    finally:
        mode.mode = 4
        modepub.publish(mode)
        time.sleep(0.25)

        
if __name__ == "__main__":
    main()
