from h2rMultiWii import MultiWii as MW

def voltage(board):
    return float(data['vbat'])/10.0

if __name__ == '__main__':
    board = MW('/dev/ttyUSB0')
    data = board.getData(MW.ANALOG)
    print 'BATTERY VOLTAGE IS\t{}v'.format(voltage(board))
    board.close()
