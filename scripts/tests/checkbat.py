from h2rMultiWii import MultiWii as MW

def voltage(data):
    return float(data['vbat'])/10.0

def main():
    board = MW('/dev/ttyUSB0')
    data = board.getData(MW.ANALOG)
    print 'BATTERY VOLTAGE IS\t{}v'.format(voltage(data))
    board.close()

if __name__ == '__main__':
    main()
