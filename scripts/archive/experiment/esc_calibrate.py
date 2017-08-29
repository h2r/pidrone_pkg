from pyMultiwii import MultiWii
import time
import sys

"""
In config.h of the multiwii source code, there is a parameter called 
ESC_CALIB_CANNOT_FLY. Enabling that will pass the raw throttle values
through to the ESCs. This is useful for calibrating the ESCs all at once.

This script will set the PWM range on the ESCs and allow you to test motor 
speeds after calibrating.
"""

THROTTLE_MIN = 1000
THROTTLE_MAX = 2000

board = MultiWii("/dev/ttyACM0")

print 'Setting throttle to',THROTTLE_MAX
start = time.time()
board.sendCMD(8,MultiWii.SET_RAW_RC,[1500, 1500, 1500, THROTTLE_MAX])

raw_input('Press enter to advance.')
print time.time() - start, 'seconds elapsed.'
print 'Setting throttle to',THROTTLE_MIN
start = time.time()
board.sendCMD(8,MultiWii.SET_RAW_RC,[1500, 1500, 1500, THROTTLE_MIN])

raw_input('Press enter to advance.')
print time.time() - start, 'seconds elapsed.'


while True:
	val = 0
	try:
		raw = raw_input('Enter a throttle value: ')
		val = int(raw)
	except:
		if raw == 'q': 
			break
		else: 
			print raw, 'is not an integer.'
			continue	

	if val >= THROTTLE_MIN and val <= THROTTLE_MAX:
		board.sendCMD(8,MultiWii.SET_RAW_RC,[1500, 1500, 1500, val])
	else:
		print 'Value must be between', THROTTLE_MIN, 'and', THROTTLE_MAX
