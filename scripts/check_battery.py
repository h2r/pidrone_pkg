import rospy
import pidrone_pkg.msg._Battery

CELLS = 3
SAFE_VALUE = CELLS * 3
MAX_VALUE = CELLS * 4.2

def notify(data):
	global bat_value
	bat_value = data.vbat

def subscribe():
	global bat_value
	bat_value = 'The flight controller is not publishing the battery. Please check the USB connector.'
	rospy.init_node('battery', anonymous=True)
	rospy.Subscriber('/pidrone/battery', pidrone_pkg.msg.Battery,notify)
	rate = rospy.Rate(.1)
	while not rospy.is_shutdown():
		rate.sleep()
		print("Voltage: " + str(bat_value))
		if bat_value < SAFE_VALUE:
			print("Battery too low, below " + str(SAFE_VALUE) + ". Max safe voltage to charge to is " + str(MAX_VALUE) + ".")

if __name__ == '__main__':
	subscribe()
