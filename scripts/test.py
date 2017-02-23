from pyMultiwii import MultiWii

board = MultiWii("/dev/ttyACM0")

board.arm()
board.disarm()
