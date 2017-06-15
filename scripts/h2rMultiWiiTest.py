from h2rMultiWii import MultiWii
import time

print "making board."
board = MultiWii("/dev/ttyACM0")

#print "arming."
#board.arm()
#print "armed."

#for i in range(1000):
# print "Ident: "
# print board.getData(MultiWii.IDENT)
# print "Attitude: "
# print board.getData(MultiWii.ATTITUDE)

# print "RC: "
# print board.getData(MultiWii.RC)
# print board.getData(MultiWii.RC)

# print "sending RC"
# board.sendCMD(16,MultiWii.SET_RAW_RC, [1500,1500,2000,1000, 1500, 1500, 1500, 1500])

#print "RC2"

#print board.getData(MultiWii.RC)
#print board.getData(MultiWii.STATUS)
#while True:
#print board.getData(MultiWii.RAW_IMU)

#board.sendCMD(16,MultiWii.SET_RAW_RC, [1500,1500,2000,1000, 1500, 1500, 1500, 1500])    
time.sleep(0.1)





#    print "IMU: "
#    print board.getData(MultiWii.RAW_IMU)
#    print "Status: "
#    print board.getData(MultiWii.STATUS)

    #print "POS_EST: "
    #print board.getData(MultiWii.POS_EST)

#    time.sleep(0.1)

#print board.getData(MultiWii.RC)
# while True:
#     print "RC"
#     print board.getData(MultiWii.RC)
#     time.sleep(0.001)



#print "box ids"
#print board.getData(MultiWii.BOXIDS)

#print "box values"
#print board.getData(MultiWii.BOX)

#print "box names"
#print board.getData(MultiWii.BOXNAMES)

#print "Analog"
#print board.getData(MultiWii.ANALOG)

def pulseMotor():
    print board.getData(MultiWii.MOTOR)
    print "arm"
    for i in range(10):
        board.sendCMD(16,MultiWii.SET_RAW_RC, [1500, 1500, 1500, 988, 
                                               2000, 1500, 1500, 1500])
        time.sleep(0.01)

    time.sleep(1)
    print board.getData(MultiWii.MOTOR)
    print "power up"
    for i in range(10):

        board.sendCMD(16,MultiWii.SET_RAW_RC, [1500, 1500, 1500, 2000, 
                                               2000, 1500, 1500, 1500])
        time.sleep(0.01)

    time.sleep(2)
    print board.getData(MultiWii.MOTOR)
    print "power down"
    for i in range(20):
        board.sendCMD(16,MultiWii.SET_RAW_RC, [1500, 1500, 1500, 988, 
                                               1500, 1500, 1500, 1500])
        time.sleep(0.01)

    time.sleep(1)
    print board.getData(MultiWii.MOTOR)

print board.getData(MultiWii.IDENT)
print board.getData(MultiWii.MOTOR)
print board.getData(MultiWii.RAW_IMU)

#print board.receiveDataPacket()
pulseMotor()
#print "disarm"
#board.disarm()
board.ser.close()




# while True:
#     print "RC"
#     print board.getData(MultiWii.RC)
#     time.sleep(0.001)

