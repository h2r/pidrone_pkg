from pyMultiwii import MultiWii

board = MultiWii("/dev/ttyACM0")

board.arm()
board.disarm()

def g():
    return board.getData(MultiWii.ATTITUDE)

