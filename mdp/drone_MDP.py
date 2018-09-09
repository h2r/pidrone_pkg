# from simple_rl.agents import QLearningAgent, RandomAgent, DoubleQAgent
# from simple_rl.run_experiments import run_single_agent_on_mdp, run_agents_on_mdp
from simple_rl.mdp.MDPClass import MDP
from simple_rl.planning.ValueIterationClass import ValueIteration
from drone_class import DroneTask, DroneRoom, DroneDoor, DroneBlock
from drone_class import DroneState

import random
import os
import sys
from collections import defaultdict
parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
sys.path.insert(0, parent_dir)

class DroneMDP(MDP):

    # don't need rotation in MDP
    ACTIONS = ["forward", "back", "left", "right", "up", "down", "take_photo"]
    ACTION_TABLE = {'forward': (0, 0.5, 0), 'back': (0, -0.5, 0), 'left': (-0.5, 0, 0), 'right': (0.5, 0, 0), 'up': (0, 0, 0.15), 'down': (0, 0, -0.15), 'take_photo': (-10,)}

    def __init__(self, init_location, task, rooms, blocks=None, doors=None, random_init_location=False, gamma=0.99):
        """
        :param init_location: Initial drone location
        :param task: The given task for this MDP
        :param rooms: List of rooms
        :param blocks: List of blocks
        :param doors: List of doors
        :param random_init_location: random initialize drone location
        :param gamma: gamma factor
        """
        blocks_location = [(block.x, block.y, block.z) for block in blocks]
        legal_location = [cell for room in rooms for cell in room.cells_in_room if cell not in blocks_location]
        doors_location = [(door.x, door.y, z) for door in doors for z in range(door.height)]
        legal_location.extend(doors_location)
        self.legal_location = set(legal_location)
        self.doors_location = set(doors_location)
        self.task = task
        self.rooms = rooms
        self.blocks = blocks

        if random_init_location:
            init_location = random.choice(legal_location)

        init_state = DroneState(init_location, DroneBlock('start', -1, -1, -1, 'start'))
        MDP.__init__(self, self.ACTIONS, self._transition_func, self._reward_func, init_state=init_state, gamma=gamma)

    def __str__(self):
        return "DroneMDP " + str(self.task)

    def _transition_func(self, state, action):
        """
        :param state: The state
        :param action: The action
        :return: The next state you get if in state and perform action
        """
        new_x, new_y, new_z, take_photo = self._move(state, action)
        next_state = state.copy()

        # the action is taking photo
        if take_photo:
            # scan cells right one cell below the drone
            above_a_block = False
            for block in self.blocks:
                if block.x == new_x and block.y == new_y and block.z == new_z-1:
                    next_state.photo_block = block
                    above_a_block = True
            # if not above_a_block:
            #     next_state.photo_block = DroneBlock('', new_x, new_y, new_z-1, '')
        # # the action is taking photo
        # if take_photo and state.photo_block.color != self.task.block_color:
        #     # scan cells right one cell below the drone
        #     for block in self.blocks:
        #         if block.color == self.task.block_color and block.x == new_x and block.y == new_y and block.z == new_z-1:
        #             next_state.photo_block = block
        # update x, y, z if it is legal
        elif (new_x, new_y, new_z) in self.legal_location:
            next_state.x = new_x
            next_state.y = new_y
            next_state.z = new_z

        # make terminal
        if self._is_terminal(next_state):
            next_state.set_terminal(True)

        return next_state

    def _move(self, state, action):
        """
        Perform one action, assume the heading is 0 when drone facing to x coordinate
        """
        dx, dy, dz = 0, 0, 0
        take_photo = False
        if action == self.ACTIONS[0]:
            dy = 1
        elif action == self.ACTIONS[1]:
            dy = -1
        elif action == self.ACTIONS[2]:
            dx = -1
        elif action == self.ACTIONS[3]:
            dx = 1
        elif action == self.ACTIONS[4]:
            dz = 1
        elif action == self.ACTIONS[5]:
            dz = -1
        elif action == self.ACTIONS[6]:
            take_photo = True

        new_x = state.x + dx
        new_y = state.y + dy
        new_z = state.z + dz

        return new_x, new_y, new_z, take_photo

    def _reward_func(self, state, action):
        """
        :param state: The state you are in before performing the action
        :param action: The action you would like to perform in the state
        :return: A double indicating how much reward to assign to that state.
                 1000.0 for the terminal state.  -1.0 for every other state.
        """
        if self._is_terminal(state):
            return 0.0
        next_state = self.transition_func(state, action)
        # if self._is_terminal(next_state):
        #     return 1000.0
        # elif action == self.ACTIONS[6] and (next_state.photo_block.color != self.task.block_color or state.photo_block.color == self.task.block_color):
        #     return -1000.0
        # elif action == self.ACTIONS[6] and next_state.photo_block.color == self.task.block_color:
        #     return -0.5
        # else:
        #     return -1.0
        return 1000.0 if self._is_terminal(next_state) else -1.0

    def _is_terminal(self, state):
        """
        :param state: The state we want to check is terminal
        :return: A boolean indicating whether the state is terminal or not
        """
        is_target_block = False
        is_goal_room = False

        # have photo of the target block
        if self.task.block_name is None:
            if self.task.block_color == 'None':
                is_target_block = True
            if self.task.block_color == state.photo_block.color:
                is_target_block = True
        else:
            if self.task.block_name == state.photo_block.name:
                is_target_block = True
        # in the goal room
        if is_target_block:
            if self.task.goal_room_color == 'None':
                is_goal_room = True
            if self.task.goal_room_name is None:
                for room in self.rooms:
                    if room.color == self.task.goal_room_color and room.in_room(state.x, state.y, state.z):
                        is_goal_room = True
            else:
                for room in self.rooms:
                    if room.name == self.task.goal_room_name and room.in_room(state.x, state.y, state.z):
                        is_goal_room = True

        return is_target_block and is_goal_room

    def visualize_agent(self, agent):
        # import airsim, time

        cur_state = self.get_init_state()
        reward = 0
        done = False

        # # connect to the AirSim simulator
        # velocity = 0.5  # set velocity
        # scale = 0.9  # to make the drone move a little more in AirSim so looks better
        # client = airsim.MultirotorClient()
        # client.confirmConnection()
        # client.reset()
        # client.enableApiControl(True)
        # client.takeoffAsync().join()
        # time.sleep(1)
        # # go to the initial place
        # client.moveToPositionAsync(0, 0, 0, velocity).join()
        # time.sleep(1)

        while not done:
            action = agent.act(cur_state, reward)
            print('start from\t' + str((cur_state.x, cur_state.y, cur_state.z)) + '\t' + action)

            prev_state = cur_state
            reward, cur_state = self.execute_agent_action(action)

            if prev_state != cur_state:
                # # AirSim flips x and y
                # client.moveToPositionAsync(cur_state.y * scale, cur_state.x * scale, -cur_state.z * scale, velocity, timeout_sec=3).join()
                # time.sleep(0.5)
                print('arrive at\t' + str((cur_state.x, cur_state.y, cur_state.z)))

            if cur_state.is_terminal():
                print('Task Complete !')
                done = True
                
        # client.enableApiControl(False)

    def visualize_policy(self, policy):
        # import airsim, time, pyautogui

        cur_state = self.get_init_state()
        done = False

        # # connect to the AirSim simulator
        # velocity = 0.5  # set velocity
        # scale = 1  # to make the drone move a little more in AirSim so looks better
        # client = airsim.MultirotorClient()
        # client.confirmConnection()
        # client.reset()
        # client.enableApiControl(True)
        # client.takeoffAsync(timeout_sec=3).join()
        # time.sleep(1)
        # # go to the initial place
        # client.moveToPositionAsync(0, 0, 0, velocity).join()
        # time.sleep(1)

        while not done:
            action = policy[cur_state]
            print('start from\t' + str((cur_state.x, cur_state.y, cur_state.z)) + '\t' + action)

            prev_state = cur_state
            reward, cur_state = self.execute_agent_action(action)

            if prev_state != cur_state:
                # AirSim flips x and y
                # client.moveToPositionAsync(cur_state.y * scale, cur_state.x * scale, -cur_state.z * scale, velocity, timeout_sec=3).join()
                print('arrive at\t' + str((cur_state.x, cur_state.y, cur_state.z)))
                # Take photo
                if action == self.ACTIONS[-1]:
                    # pyautogui.keyDown('c')
                    print('take photo')
                # time.sleep(1)

            if cur_state.is_terminal():
                print('Task Complete !')
                done = True
                
        # client.enableApiControl(False)

    def send_path(self, policy, pub, drone_path):
        """
        Assume the space is 2 x 2 x 0.6 meters, a 4 x 4 x 2 grid
        Send a list of path, -1 stands for take_photo
        """

        path = []
        cur_state = self.get_init_state()
        done = False

        while not done:
            action = policy[cur_state]
            print('start from\t' + str((cur_state.x, cur_state.y, cur_state.z)) + '\t' + action)

            prev_state = cur_state
            reward, cur_state = self.execute_agent_action(action)

            if prev_state != cur_state:
                path.extend(list(self.ACTION_TABLE[action]))

                if action == self.ACTIONS[-1]:
                    print('take photo')
                else:
                    print('arrive at\t' + str((cur_state.x, cur_state.y, cur_state.z)))

            if cur_state.is_terminal():
                print('Task Complete !')
                done = True
                drone_path.data = path
                pub.publish(drone_path)

# Without drone
def main():
    height = 2  # vertical space
    task = DroneTask("red", "None")
    room1 = DroneRoom("room1", [(x, y, z) for x in range(0, 4) for y in range(0, 1) for z in range(height)], "red")
    room2 = DroneRoom("room2", [(x, y, z) for x in range(0, 2) for y in range(2, 3) for z in range(height)], color="green")
    room3 = DroneRoom("room3", [(x, y, z) for x in range(3, 4) for y in range(2, 3) for z in range(height)], color="blue")
    block1 = DroneBlock("block1", 0, 2, 0, color="red")
    block2 = DroneBlock("block2", 2, 0, -1, color="green")
    block3 = DroneBlock("block3", 3, 2, 0, color="blue")
    rooms = [room1, room2, room3]
    blocks = [block1, block2, block3]
    doors = [DroneDoor(1, 1, height), DroneDoor(3, 1, height)]
    mdp = DroneMDP((0, 0, 0), task, rooms=rooms, blocks=blocks, doors=doors)

    # print("Start Q learning")
    # ql_agent = QLearningAgent(actions=mdp.get_actions())
    # # run_agents_on_mdp([ql_agent], mdp, instances=2, episodes=2500, steps=100, reset_at_terminal=True, verbose=True)
    # run_single_agent_on_mdp(ql_agent, mdp, episodes=2000, steps=200)
    print("Start Value Iteration")
    vi = ValueIteration(mdp)
    vi.run_vi()
    action_seq, state_seq = vi.plan(mdp.init_state)
    policy = defaultdict()
    for i in range(len(action_seq)):
        policy[state_seq[i]] = action_seq[i]
    print("Start AirSim")
    # mdp.visualize_agent(ql_agent)
    mdp.visualize_policy(policy)

# With drone
def run(task_block, task_room, red_pos, green_pos, blue_pos, drone_pos, pub, drone_path):
    """
    Assume the block is on the floor of each cell
    Get initial pos of drone from caller
    """
    height = 2  # vertical space
    task = DroneTask(task_block, task_room)
    room1 = DroneRoom("room1", [(x, y, z) for x in range(4) for y in range(1) for z in range(height)], "red")
    room2 = DroneRoom("room2", [(x, y, z) for x in range(0, 2) for y in range(2, 4) for z in range(height)], color="green")
    room3 = DroneRoom("room3", [(x, y, z) for x in range(3, 4) for y in range(2, 4) for z in range(height)], color="blue")
    block1 = DroneBlock("block1", red_pos[0], red_pos[1], red_pos[2] - 1, color="red")
    block2 = DroneBlock("block2", green_pos[0], green_pos[1], green_pos[2] - 1, color="green")
    block3 = DroneBlock("block3", blue_pos[0], blue_pos[1], blue_pos[2] - 1, color="blue")
    rooms = [room1, room2, room3]
    blocks = [block1, block2, block3]
    doors = [DroneDoor(1, 1, height), DroneDoor(3, 1, height)]
    mdp = DroneMDP(drone_pos, task, rooms=rooms, blocks=blocks, doors=doors)

    print("Start Value Iteration")
    vi = ValueIteration(mdp)
    vi.run_vi()
    action_seq, state_seq = vi.plan(mdp.init_state)
    policy = defaultdict()
    for i in range(len(action_seq)):
        policy[state_seq[i]] = action_seq[i]
    print("Start Flying")
    mdp.send_path(policy, pub, drone_path)


if __name__ == "__main__":
    main()
