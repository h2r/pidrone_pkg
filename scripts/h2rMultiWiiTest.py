from h2rMultiWii import MultiWii
import time

print "making board."
board = MultiWii("/dev/ttyACM0")

#print "arming."
#board.arm()

#print "armed."

#for i in range(1000):
#    print "Ident: "
#    print board.getData(MultiWii.IDENT)
print "Attitude: "
print board.getData(MultiWii.ATTITUDE)

print "RC: "
print board.getData(MultiWii.RC)
print board.getData(MultiWii.RC)

#    print "IMU: "
#    print board.getData(MultiWii.RAW_IMU)
#    print "Status: "
#    print board.getData(MultiWii.STATUS)

    #print "POS_EST: "
    #print board.getData(MultiWii.POS_EST)

#    time.sleep(0.1)


    

#print "disarm"
#board.disarm()
board.ser.close()



