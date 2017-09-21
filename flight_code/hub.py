'''
CS1951R - Introduction to Robotics
Brown University
Fall 2017

hub.py
'''

if __name == "__main__":
    rospy.init_node("hub")
    board = MultiWii("/dev/ttyUSB0")
