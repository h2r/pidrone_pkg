#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas and Stefanie Tellex"
__copyright__ = "Copyright 2014 Altax.net, 2017"

__license__ = "GPL"
__version__ = "1.5"

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
        self.status = {}
        self.analog = {}
        self.boxids = []
        self.box = []
        self.elapsed = 0
        self.PRINT = 1

        self.ser = serial.Serial(serPort, 
                                 baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0,
                                 xonxoff=False,
                                 rtscts=False,
                                 dsrdtr=False,
                                 writeTimeout=2,
                             )

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
            self.ser.write("\n") # flush buffers all the way through.
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
            elif header != '':
                print "IGNORING HEADER: '%s'" % header
                result = struct.unpack('<B', header), 
                print "result", result
                raise
            else:
                pass
            time.sleep(0.01)
        datalengthraw = self.ser.read()
        try:
            datalength = struct.unpack('<b', datalengthraw)[0]
        except:
            print "data length raw", datalengthraw, len(datalengthraw)
            raise
        code = struct.unpack('<B', self.ser.read())[0]
        data = self.ser.read(datalength)
        checksum = self.ser.read()
        self.checkChecksum(data, checksum)  # noop now.
        elapsed = time.time() - start
        if code == MultiWii.ATTITUDE:
            temp = struct.unpack('<'+'hhh',data)
            self.attitude["cmd"] = code
            self.attitude['angx']=float(temp[0]/10.0)
            self.attitude['angy']=float(temp[1]/10.0)
            self.attitude['heading']=float(temp[2])
            self.attitude['elapsed']=round(elapsed,3)
            self.attitude['timestamp']=time.time()
            return self.attitude
        elif code == MultiWii.BOXIDS:
            temp = struct.unpack('<'+'b'*datalength,data)
            self.boxids = temp
            return self.boxids
        elif code == MultiWii.BOX:
            assert datalength % 2 == 0
            temp = struct.unpack('<'+'H'*(datalength/2), data)
            self.box = temp
            return self.box
        elif code == MultiWii.ANALOG:
            temp = struct.unpack('<'+'bHHH', data)
            self.analog["vbat"] = temp[0]
            self.analog["intPowerMEterSum"] = temp[1]
            self.analog["rssi"] = temp[2]
            self.analog["amperage"] = temp[3]
            self.analog['timestamp']=time.time()
            return self.analog
        elif code == MultiWii.BOXNAMES:
            print "datalength", datalength
            assert datalength % 2 == 0
            temp = struct.unpack('<'+'s' * datalength, data)
            temp = "".join(temp)[:-1].split(";")
            self.boxnames = temp
            return self.boxnames
        elif code == MultiWii.STATUS:
            temp = struct.unpack('<'+'HHHIb',data)
            self.status["cycleTime"] = temp[0]
            self.status["i2c_errors_count"] = temp[1]
            self.status["sensor"] = temp[2]
            self.status["flag"] = temp[3]
            self.status["global_conf.currentSet"] = temp[4]
            self.status['timestamp']=time.time()
            return self.status
        elif code == MultiWii.ACC_CALIBRATION:
            print "data", data
            print "len", len(data)

        elif code == MultiWii.IDENT:
            temp = struct.unpack('<'+'BBBI',data)
            self.ident["cmd"] = code
            self.ident["version"] = temp[0]
            self.ident["multitype"] = temp[1]
            self.ident["msp_version"] = temp[2]
            self.ident["capability"] = temp[3]
            self.ident['timestamp']=time.time()

            return self.ident
        elif code == MultiWii.RC:
            temp = struct.unpack('<'+'hhhhhhhhhhhh',data)
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
            self.rcChannels['timestamp']=time.time()
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
            self.rawIMU['timestamp']=time.time()
            return self.rawIMU
        elif code == MultiWii.POS_EST:
            temp = struct.unpack('<'+'hhh',data)
            self.posest["cmd"] = code
            self.posest['x']=float(temp[0])
            self.posest['y']=float(temp[1])
            self.posest['z']=float(temp[2])
            self.posest['elapsed']=round(elapsed,3)
            self.posest['timestamp']=time.time()
            return self.posest
        elif code == MultiWii.MOTOR:
            temp = struct.unpack('<'+'hhhhhhhh',data)
            self.motor['cmd'] = code
            self.motor['m1']=float(temp[0])
            self.motor['m2']=float(temp[1])
            self.motor['m3']=float(temp[2])
            self.motor['m4']=float(temp[3])
            self.motor['m5']=float(temp[4])
            self.motor['m6']=float(temp[5])
            self.motor['m7']=float(temp[6])
            self.motor['m8']=float(temp[7])
            self.motor['elapsed']="%0.3f" % (elapsed,)
            self.motor['timestamp']=time.time()
            return self.motor
        else:
            print "No return error!: %d" % code
            raise

    """ Implement me to check the checksum. """
    def checkChecksum(self, data, checksum):
        pass
    def close(self):
        self.ser.close()
