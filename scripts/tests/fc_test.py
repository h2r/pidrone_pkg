from h2rMultiWii import MultiWii
import time

board = MultiWii("/dev/ttyUSB0")

def g():
    return board.getData(MultiWii.ATTITUDE)

board.arm()
board.disarm()
print g()
