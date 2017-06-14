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


#board.sendCMD(16,MultiWii.SET_RAW_RC, [1500,1500,2000,1000, 1500, 1500, 1500, 1500])    
#time.sleep(0.1)





#    print "IMU: "
#    print board.getData(MultiWii.RAW_IMU)
#    print "Status: "
#    print board.getData(MultiWii.STATUS)

    #print "POS_EST: "
    #print board.getData(MultiWii.POS_EST)

#    time.sleep(0.1)

#print board.getData(MultiWii.RC)

#print "RC"
#print board.getData(MultiWii.RC)


#print "box ids"
#print board.getData(MultiWii.BOXIDS)

#print "box values"
#print board.getData(MultiWii.BOX)

#print "box names"
#print board.getData(MultiWii.BOXNAMES)

#print "Analog"
#print board.getData(MultiWii.ANALOG)


#board.sendCMD(16,MultiWii.SET_RAW_RC, [1500,1500,2000,1000, 1500, 1500, 1500, 1500])

#time.sleep(1)

#print "receive."
#print board.receiveDataPacket()


#print "disarm"
#board.disarm()
#board.ser.close()



