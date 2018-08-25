from simple_rl.mdp.StateClass import State


class DroneState(State):

    def __init__(self, location, photo_block=None):
        """
        :param location: A tuple, the coordinate (x,y,z) of drone
        :param photo_block: A DroneBlock
        """
        self.x = location[0]
        self.y = location[1]
        self.z = location[2]
        self.photo_block = photo_block
        State.__init__(self, data=[location, photo_block])

    def __hash__(self):
        return hash((self.x, self.y, self.z, self.photo_block))

    def __eq__(self, other):
        return isinstance(other, DroneState) and self.x == other.x and self.y == other.y and self.z == other.z \
               and self.photo_block == other.photo_block

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        str_builder = "DRONE: " + str((self.x, self.y, self.z)) + "\n"
        str_builder += "\nThe photo of block: " + str(self.photo_block) + "\n"
        return str_builder

    def copy(self):
        return DroneState((self.x, self.y, self.z), self.photo_block.copy())


class DroneRoom:

    def __init__(self, name, cells_in_room, color=''):
        """
        :param name: The name of the room
        :param cells_in_room: List of (x,y,z), reachable cells in this room
        :param color: The color of the room
        """
        self.name = name
        self.cells_in_room = cells_in_room
        self.color = color

    def __hash__(self):
        """
        blocks and doors are not used for hashing
        """
        return hash((self.name, self.color, tuple(self.cells_in_room)))

    def __eq__(self, other):
        """
        blocks and doors are not used for equality test
        """
        return isinstance(other, DroneRoom) and self.name == other.name and self.color == other.color and list_equal(
            self.cells_in_room, other.cells_in_room)

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        return "ROOM[" + self.name + ", " + "".join(str(cell) for cell in self.cells_in_room) + ", " + self.color + "]"

    def copy(self):
        return DroneRoom(self.name, self.cells_in_room[:], color=self.color)

    def in_room(self, x, y, z):
        return (x, y, z) in self.cells_in_room


class DroneBlock:

    def __init__(self, name, x, y, z, color=''):
        """
        :param name: The name of this block
        :param x: The x coordinate of this block
        :param y: The y coordinate of this block
        :param z: The z coordinate of this block
        :param color: The color of this block
        """
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.color = color

    def __hash__(self):
        return hash((self.name, self.x, self.y, self.z, self.color))

    def __eq__(self, other):
        return isinstance(other, DroneBlock) and self.name == other.name and self.x == other.x and self.y == other.y \
               and self.z == other.z and self.color == other.color

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        return "Block[" + self.name + ", " + str((self.x, self.y, self.z)) + ", " + self.color + "]"

    def name(self):
        return self.name

    def copy_with_name(self, new_name):
        return DroneBlock(new_name, self.x, self.y, self.z, self.color)

    def copy(self):
        return DroneBlock(self.name, self.x, self.y, self.z, self.color)


class DroneDoor:

    def __init__(self, x, y, height):
        """
        The z coordinate is ignored
        :param x: The x coordinate of this door
        :param y: The y coordinate of this door
        :param height: The height of this door
        """
        self.x = x
        self.y = y
        self.height = height

    def __hash__(self):
        return hash((self.x, self.y, self.height))

    def __eq__(self, other):
        return isinstance(other, DroneDoor) and self.x == other.x and self.y == other.y and self.height == other.height

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        return "Door[" + str((self.x, self.y, self.height)) + "]"

    def copy(self):
        return DroneDoor(self.x, self.y, self.height)

# TODO: delete the name of block
class DroneTask:

    def __init__(self, block_color, goal_room_color, block_name=None, goal_room_name=None):
        """
        We assume we have the images of all passed cells, because the drone can record the video. We don't need the take photo in MDP 
        You can choose which attributes you would like to have represent the blocks and the rooms
        Task: Take the photo of a block, then, go to a room
        """
        self.block_color = block_color
        self.goal_room_color = goal_room_color
        self.block_name = block_name
        self.goal_room_name = goal_room_name

    def __hash__(self):
        return hash((self.block_color, self.goal_room_color, self.block_name, self.goal_room_name))

    def __eq__(self, other):
        return isinstance(other, DroneTask) and self.block_color == other.block_color \
               and self.goal_room_color == other.goal_room_color and self.block_name == other.block_name \
               and self.goal_room_name == other.goal_room_name

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        if self.goal_room_name is None and self.block_name is None:
            return "Take photo of " + self.block_color + " block, go to the " + self.goal_room_color + " room"
        elif self.block_name is None and self.goal_room_name is not None:
            return "Take photo of " + self.block_color + " block, go to the room named " + self.goal_room_name
        elif self.goal_room_name is None and self.block_name is not None:
            return "Take photo of the block named " + self.block_name + ", go to the " + self.goal_room_color + " room"
        else:
            return "Take photo of the block named " + self.block_name + ", go to the room named " + self.goal_room_name


def list_equal(list1, list2):
    """
    Return True if two lists are equal
    """
    return len(list1) == len(list2) and set(list1) == set(list2)
