import rospy
import drone_MDP
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
import time
import load_drone_draggn
from i_draggn import ProgArgNet

ACTION_TABLE = {'forward': (0, 0.5, 0), 'back': (0, -0.5, 0), 'left': (-0.5, 0, 0), 'right': (0.5, 0, 0), 'up': (0, 0, 0.15), 'down': (0, 0, -0.15), 'take_photo': (-10,)}

command = ""
prev_command = ""
# hololens
redBox_pos = (-1, -1, -1)
greenBox_pos = (-1, -1, -1)
blueBox_pos = (-1, -1, -1)
drone_pos = None
drone_path = Float32MultiArray()

def drone_callback(data):
    """
    Assume the space is 2 x 1.5 x 0.6 meters, a 4 x 3 x 2 grid
    """
    global drone_pos

    z = 0
    if data.z > 0.3:
        z = 1
    drone_pos = (int(data.x / 0.5), int(data.y / 0.5), z)

def box_callback(data):
    global redBox_pos
    global greenBox_pos
    global blueBox_pos

    positions = (data.data).split()
    redBox_pos = (int(positions[0]), int(positions[1]), int(positions[2]))
    greenBox_pos = (int(positions[3]), int(positions[4]), int(positions[5]))
    blueBox_pos = (int(positions[6]), int(positions[7]), int(positions[8]))

def command_callback(data):
    global command

    command = str(data.data)

rospy.init_node("path_pub")
pub = rospy.Publisher('/pidrone/path', Float32MultiArray, queue_size=1)
rospy.Subscriber("/pidrone/drone_position", Point, drone_callback)
rospy.Subscriber("/hololens/box_position", String, box_callback)
rospy.Subscriber("/hololens/language_command", String, command_callback)

time.sleep(3)
while True:

    if command != prev_command:
        print(drone_pos)
        print(command)
        prev_command = command
        prog, arg = load_drone_draggn.run(command)
        # Action
        if prog in ACTION_TABLE:
            arg_1 = prog
            arg_2 = int(arg)
            print("start action")
            print(prog, arg)
            if arg_1 == 'take_photo':
                drone_path.data = [ACTION_TABLE[arg_1][0]]
            else:
                move = ACTION_TABLE[arg_1]
                drone_path.data = [move[0] * arg_2, move[1] * arg_2, move[2] * arg_2]
            pub.publish(drone_path)
        # Goal
        else:
            if prog == 'agent_in_room':
                block_color = 'None'
                room_color = str(arg)
            elif prog == 'photo_in_drone':
                block_color = str(arg)
                room_color = 'None'
            else:
                arg_1, arg_2 = arg.split("_")
                block_color = arg_1
                room_color = arg_2
            print("start goal")
            print(prog, arg)
            drone_MDP.run(block_color, room_color, redBox_pos, greenBox_pos, blueBox_pos, drone_pos, pub, drone_path)            
    time.sleep(1)


