#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2014 Altax.net"

__license__ = "GPL"
__version__ = "1.5"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import serial, time, struct


class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    POS_EST = 123
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    DEBUG = 254


    """Class initialization"""
    def __init__(self, serPort):

        """Global variables of data"""
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'elapsed':0,'timestamp':0}
        self.posest = {'x':0,'y':0,'z':0,'elapsed':0,'timestamp':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}

        self.ident = {"version":"", "multitype":"", "msp_version":"", "capability":""}
        self.analog = {}
        self.boxids = []
        self.box = []
        self.temp = ();
        self.temp2 = ();
        self.elapsed = 0
        self.PRINT = 1

        self.ser = serial.Serial(baudrate=115200)
        self.ser.port = serPort
        #self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        """Time to wait until the board becomes operational"""
        wakeup = 2
        try:
            self.ser.open()
            if self.PRINT:
                print "Waking up board on "+self.ser.port+"..."
            for i in range(1,wakeup):
                if self.PRINT:
                    print wakeup-i
                    time.sleep(1)
                else:
                    time.sleep(1)
            #self.sendCMD(0,MultiWii.IDENT,[])
        except Exception, error:
            print "\n\nError opening "+self.ser.port+" port.\n"+str(error)+"\n\n"
            raise

    """Function for sending a command to the board"""
    def sendCMD(self, data_length, code, data):
        checksum = 0
        total_data = ['$', 'M', '<', data_length, code] + data
        for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ ord(i)
        total_data.append(checksum)
        try:
            b = None
            data = struct.pack('<3c2B%dhB' % len(data), *total_data)
            b = self.ser.write(data)
            assert b == len(data)
            self.ser.flush()
        except Exception, error:
            print "\n\nError in sendCMD."
            print "("+str(error)+")\n\n"
            raise
            pass


    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,2000,1000, 1500, 1500, 1500, 1500]
            self.sendCMD(16,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1000, 1500, 1500, 1500, 1500]
            self.sendCMD(16,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    """Function to receive a data packet from the board"""
    def getData(self, cmd):
        try:
            self.sendCMD(0,cmd,[])
            return self.receiveDataPacket()
        except Exception, error:
            print error
            raise
            pass

    def receiveDataPacket(self):
        start = time.time()
        while True:
            header = self.ser.read()
            if header == '$':
                header = header+self.ser.read(2)
                break
            time.sleep(0.01)
        datalengthraw = self.ser.read()
        try:
            datalength = struct.unpack('<b', datalengthraw)[0]
        except:
            print "data length raw", datalengthraw, len(datalengthraw)
            raise
        #print "datalength", datalength
        code = struct.unpack('<b', self.ser.read())[0]
        #print "code", code
        data = self.ser.read(datalength)
        #print "data", len(data)

        #self.ser.flushInput()
        #self.ser.flushOutput()
        elapsed = time.time() - start
        if code == MultiWii.ATTITUDE:
            temp = struct.unpack('<'+'hhh',data)
            #print "temp", temp
            self.attitude["cmd"] = code
            self.attitude['angx']=float(temp[0]/10.0)
            self.attitude['angy']=float(temp[1]/10.0)
            self.attitude['heading']=float(temp[2])
            self.attitude['elapsed']=round(elapsed,3)
            self.attitude['timestamp']="%0.2f" % (time.time(),) 
            return self.attitude
        elif code == MultiWii.BOXIDS:
            temp = struct.unpack('<'+'b'*datalength,data)
            #print "temp", temp
            self.boxids = temp
            return self.boxids
        elif code == MultiWii.BOX:
            print "datalength", datalength
            assert datalength % 2 == 0
            temp = struct.unpack('<'+'H'*(datalength/2), data)
            #print "temp", temp
            self.box = temp
            return self.box
        elif code == MultiWii.ANALOG:
            temp = struct.unpack('<'+'bHHH', data)
            self.analog["vbat"] = temp[0]
            self.analog["intPowerMEterSum"] = temp[1]
            self.analog["rssi"] = temp[2]
            self.analog["amperage"] = temp[3]
            return self.analog
        elif code == MultiWii.BOXNAMES:
            print "datalength", datalength
            assert datalength % 2 == 0
            temp = struct.unpack('<'+'s' * datalength, data)
            temp = "".join(temp)[:-1].split(";")
            #print "temp", temp
            self.boxnames = temp
            return self.boxnames
        elif code == MultiWii.IDENT:
            temp = struct.unpack('<'+'BBBI',data)
            self.ident["cmd"] = code
            self.ident["version"] = temp[0]
            self.ident["multitype"] = temp[1]
            self.ident["msp_version"] = temp[2]
            self.ident["capability"] = temp[3]
            return self.ident
        elif code == MultiWii.RC:
            temp = struct.unpack('<'+'hhhhhhhhhhhh',data)
            print "temp", temp
            self.rcChannels['cmd'] = code
            self.rcChannels['roll']=temp[0]
            self.rcChannels['pitch']=temp[1]
            self.rcChannels['yaw']=temp[2]
            self.rcChannels['throttle']=temp[3]
            self.rcChannels['aux1'] = temp[4]
            self.rcChannels['aux2'] = temp[5]
            self.rcChannels['aux3'] = temp[6]
            self.rcChannels['aux4'] = temp[7]
            self.rcChannels['aux5'] = temp[8]
            self.rcChannels['aux6'] = temp[9]
            self.rcChannels['aux7'] = temp[10]
            self.rcChannels['aux8'] = temp[11]
            self.rcChannels['elapsed']=round(elapsed,3)
            self.rcChannels['timestamp']="%0.2f" % (time.time(),)
            return self.rcChannels
        elif code == MultiWii.RAW_IMU:
            temp = struct.unpack('<'+'hhhhhhhhh',data)
            self.rawIMU["cmd"] = code
            self.rawIMU['ax']=float(temp[0])
            self.rawIMU['ay']=float(temp[1])
            self.rawIMU['az']=float(temp[2])
            self.rawIMU['gx']=float(temp[3])
            self.rawIMU['gy']=float(temp[4])
            self.rawIMU['gz']=float(temp[5])
            self.rawIMU['elapsed']=round(elapsed,3)
            self.rawIMU['timestamp']="%0.2f" % (time.time(),)
            return self.rawIMU
        elif code == MultiWii.POS_EST:
            temp = struct.unpack('<'+'hhh',data)
            self.posest["cmd"] = code
            self.posest['x']=float(temp[0])
            self.posest['y']=float(temp[1])
            self.posest['z']=float(temp[2])
            self.posest['elapsed']=round(elapsed,3)
            self.posest['timestamp']="%0.2f" % (time.time(),)
            return self.posest
        # Position Estimate
        elif code == MultiWii.MOTOR:
            self.motor['cmd'] = code
            self.motor['m1']=float(temp[0])
            self.motor['m2']=float(temp[1])
            self.motor['m3']=float(temp[2])
            self.motor['m4']=float(temp[3])
            self.motor['elapsed']="%0.3f" % (elapsed,)
            self.motor['timestamp']="%0.2f" % (time.time(),)
            return self.motor
        else:
            return "No return error!: %d" % code

