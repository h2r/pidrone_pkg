

from h2rMultiWii import MultiWii
import time

def main():
    board = MultiWii("/dev/ttyUSB0")
    board.sendCMD(0, MultiWii.PID, [])
    board.receiveDataPacket()
    print(board.pid)
    time.sleep(2)
    print("Done!")
    
if __name__ == "__main__":
    main()
