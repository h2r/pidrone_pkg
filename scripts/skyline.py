import rospy
from h2rMultiWii import MultiWii

class Skyline:
	def __init__(self):
		self.board = MultiWii("/dev/ttyUSB0")

	def step(self):
		pass

	def cleanup(self):
		pass

def main():
	rospy.init_node("skyline")
	s = Skyline()
	while not rospy.is_shutdown():
		s.step()
	s.cleanup()
	print 'EXIT'


if __name__ == '__main__':
	main()