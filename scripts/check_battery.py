import rospy
import pidrone_pkg.msg._Battery

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
		print(Voltage: + str(bat_value))

if __name__ == '__main__':
	subscribe()