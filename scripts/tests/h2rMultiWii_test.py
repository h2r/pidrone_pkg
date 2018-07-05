from h2rMultiWii import MultiWii
import time

def main():
    board = MultiWii("/dev/ttyUSB0")
    # mw_data = board.getData(MultiWii.ATTITUDE)
    # print mw_data
    # cmds = [1500, 1500, 1000, 900]
    # board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
    # time.sleep(1)

    # cmds = [1500, 1500, 1000, 1000]
    # board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
    # time.sleep(1)


    # cmds = [1500, 1500, 1500, 1500]
    # board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
    # time.sleep(1)
    print "Calibrate ACC... make sure we are level and still."
    board.sendCMD(0, MultiWii.ACC_CALIBRATION, [])
    board.receiveDataPacket()

    time.sleep(2)


    
if __name__ == "__main__":
    main()
