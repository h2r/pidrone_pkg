from h2rMultiWii import MultiWii
import time

board = MultiWii("/dev/ttyUSB0")

def g():
    return board.getData(MultiWii.ATTITUDE)

def f():
    print "arming"
    for i in range(1000):
        board.sendCMD(8, MultiWii.SET_RAW_RC, [1500, 1500, 2000, 900])
        time.sleep(0.001)
    for i in range(9000):
        board.sendCMD(8, MultiWii.SET_RAW_RC, [1500, 1000, 1500, 1500])
#   for i in range(3000):
#       board.sendCMD(8, MultiWii.SET_RAW_RC, [1000, 1500, 1500, 1500])
#   for i in range(3000):
#       board.sendCMD(8, MultiWii.SET_RAW_RC, [2000, 1500, 1500, 1500])
#   for i in range(3000):
#       board.sendCMD(8, MultiWii.SET_RAW_RC, [1500, 1000, 1500, 1500])
#   for i in range(3000):
#       board.sendCMD(8, MultiWii.SET_RAW_RC, [1000, 2000, 1500, 1500])
    print "disarming"
    for i in range(1000):
        board.sendCMD(8, MultiWii.SET_RAW_RC, [1500, 1500, 1000, 900])
        time.sleep(0.001)

f()
