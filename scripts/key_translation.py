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
l:  land
q:  quit
"""
    try:
        print msg
        while not rospy.is_shutdown():
            ch = getch.getch(0.1)
            if ch == None:
                continue

            if ch == "k":
                # disarm
                print "disarming"
                mode.mode = 4
                modepub.publish(mode)
            elif ch == "j":
                # arm
                mode.mode = 0
                modepub.publish(mode)
            elif ch == "l":
                # land
                mode.mode = 3
                modepub.publish(mode)
            elif ch == "a":
                mode.mode = 5
                mode.x_velocity = -1
                mode.y_velocity = 0
                mode.z_velocity = 0
                modepub.publish(mode)
            elif ch == "d":
                mode.mode = 5
                mode.x_velocity = 1
                mode.y_velocity = 0
                mode.z_velocity = 0
                modepub.publish(mode)
            elif ch == "w":
                mode.mode = 5
                mode.x_velocity = -1
                mode.y_velocity = 1
                mode.z_velocity = 0
                modepub.publish(mode)
            elif ch == "s":
                mode.mode = 5
                mode.x_velocity = 1
                mode.y_velocity = 1
                mode.z_velocity = 0
                modepub.publish(mode)
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
                modepub.publish(mode)
            elif ch == "y":
                mode.mode = 1
            elif ch == "q":
                break
            elif ord(ch) == 3: # Ctrl-C
                break            
            else:
                print "unknown character: '%d'" % ord(ch)
                pass
            print msg
            rate.sleep()
    finally:
        print "sending disarm"
        mode.mode = 4
        modepub.publish(mode)
        time.sleep(0.25)

        
if __name__ == "__main__":
    main()
