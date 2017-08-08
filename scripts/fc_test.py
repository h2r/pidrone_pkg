from h2rMultiWii import MultiWii

board = MultiWii("/dev/ttyUSB0")

board.arm()
board.disarm()

def g():
    return board.getData(MultiWii.ATTITUDE)

