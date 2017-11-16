import rospy
from h2rMultiWii import MultiWii
from steve_pkg.msg import PWM
from config import *

class Skyline:
	def __init__(self):
		self.board = MultiWii("/dev/ttyUSB0")
		self.pwm = PWM()
		self.accepting_control = False	# True when accepting roll,pitch,yaw,throttle commands
		self.busy = False				# True when performing atomic ops like arm/disarm
		rospy.Subscriber(SKYLINE_COMMAND_TOPIC, PWM, self.pwm_cmd_callback)
		rospy.Subscriber(JAVASCRIPT_COMMAND_TOPIC, Mode, self.mode_callback)
		# TODO: add publisher for skyline information

	def step(self):
		self.skyline_angle_data = self.board.getData(MultiWii.ATTITUDE)
 		self.skyline_voltage_data = self.board.getData(MultiWii.ANALOG)
 		if SKYLINE_ENABLED:
			self.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds)
	        self.board.receiveDataPacket()

	def cleanup(self):
		self.board.close()

	def idle(self):
		accepting_control = False
		self.set_pwm_raw([1500,1500,1500,1100])

	def arm(self):
		self.busy = True
		self.accepting_control = False
		self.set_pwm_raw([1500,1500,1500,1100])
		rospy.sleep(1)
		self.idle()
		self.busy = False

	def disarm(self):
		self.busy = True
		self.accepting_control = False	
		self.set_pwm_raw([1500,1500,1500,1100])
		rospy.sleep(1)
		self.busy = False

	def set_pwm_raw(self, desired_pwm):
		self.pwm.roll = desired_pwm[0]
		self.pwm.pitch = desired_pwm[1]
		self.pwm.yaw = desired_pwm[2]
		self.pwm.throttle = desired_pwm[3]

	def set_pwm(self, desired_pwm):
		if self.accepting_control: self.pwm = desired_pwm

	def pwm_cmd_callback(self, data):
		self.set_pwm(data)

	def mode_callback(self, data):
		if data.mode == 0:
			self.arm()
		if data.mode == 4:
			self.disarm()
		if data.mode == 5:
			if not self.busy: self.accepting_control = True


def main():
	rospy.init_node("skyline")
	s = Skyline()
	while not rospy.is_shutdown():
		s.step()

	s.cleanup()
	print 'EXIT'


if __name__ == '__main__':
	main()