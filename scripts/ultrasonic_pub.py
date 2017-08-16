import RPi.GPIO as GPIO                    #Import GPIO library
import rospy
import time
from sensor_msgs.msg import Range

GPIO.setmode(GPIO.BCM)                     #Set GPIO pin numbering 

TRIG = 23                                  #Associate pin 23 to TRIG
ECHO = 24                                  #Associate pin 24 to ECHO

print "Distance measurement in progress"

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in

smoothed_distance = 0
alpha = 1.0

def get_range():
  global smoothed_distance
  global alpha
  
  pulse_timeout = rospy.get_time()
  pulse_start = None
  pulse_end = None

  missed_pulse = False
  GPIO.output(TRIG, False)                 #Set TRIG as LOW
  rospy.sleep(0.005)
  GPIO.output(TRIG, True)                  #Set TRIG as HIGH
  rospy.sleep(0.00001)                      #Delay of 0.00001 seconds
  GPIO.output(TRIG, False)                 #Set TRIG as LOW

  while GPIO.input(ECHO)==0:               #Check whether the ECHO is LOW
    pulse_start = rospy.get_time()              #Saves the last known time of LOW pulse
    if pulse_start - pulse_timeout > 0.05:
        return -1
    
  while GPIO.input(ECHO)==1:               #Check whether the ECHO is HIGH
    pulse_end = rospy.get_time()                #Saves the last known time of HIGH pulse 

  if pulse_start is None or pulse_end is None:
      return -2
  pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable

  distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance

  if distance > 2 and distance < 400:      #Check whether the distance is within range
      smoothed_distance = (1.0 - alpha) * smoothed_distance + alpha * distance
      print smoothed_distance, distance, smoothed_distance - distance
      return smoothed_distance
  else:
      return -3

if __name__ == "__main__":
    rospy.init_node("ultrasonic_pub")
    pub = rospy.Publisher('/pidrone/ultrasonic', Range, queue_size=1)
    rnge = Range()
    rnge.header.frame_id = "world"
    while not rospy.is_shutdown():
        rnge.header.stamp = rospy.get_rostime()
        rnge.range = get_range()
        print rnge.range
        pub.publish(rnge)
