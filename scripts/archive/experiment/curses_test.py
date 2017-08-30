import curses
import rospy
from pidrone_pkg.msg import RC

def main(screen):

	rospy.init_node('key_flight', anonymous=True)

	pub = rospy.Publisher('/pidrone/commands', RC, queue_size=1)
	msg = RC()
	msg.roll = 1500
	msg.pitch = 1500
	msg.yaw = 1500
	msg.throttle = 1000
	msg.aux1 = 1500
	msg.aux2 = 1500
	msg.aux3 = 1500
	msg.aux4 = 1500

	screen.nodelay(True)
	
	while not rospy.is_shutdown():
		key = ''
		try:
			key = screen.getkey()
		except curses.error:
			pass  # no keypress was ready
		
		msg.roll = 1500
		msg.pitch = 1500
		msg.yaw = 1500
		msg.aux1 = 1500
		msg.aux2 = 1500
		msg.aux3 = 1500
		msg.aux4 = 1500
		if key == 'w':
			msg.roll = 1600
		elif key == 's':
			msg.roll = 1400
		elif key == 'a':
			msg.pitch = 1400
		elif key == 'd':
			msg.pitch = 1600
		elif key == 'q':
			msg.yaw = 1400
		elif key == 'e':
			msg.yaw = 1600
		elif key == 'u':
			if msg.throttle <= 1900:
				msg.throttle += 100
		elif key == 'j':
			if msg.throttle >= 1100:
				msg.throttle -= 100
		elif key == 'h':
			msg.aux4 = 1600
		elif key == 'n':
			msg.aux4 = 1400

		screen.addstr(0, 0, 'Commands: {}'.format([msg.roll, msg.pitch, msg.yaw, msg.throttle, msg.aux4]))
		pub.publish(msg)
		time.sleep(.1)


if __name__ == '__main__':
	curses.wrapper(main)