#!/usr/bin/env python3

import rospy

import numpy as np

from sensor_msgs.msg import Range
from std_msgs.msg import Header

from dt_vl53l0x import \
    VL53L0X, \
    Vl53l0xAccuracyMode




class ToFNode(object):
    """
    This class implements the communication logic with a Time-of-Flight sensor on the i2c bus.
    It publishes both range measurements as well as display fragments to show on an LCD screen.

    NOTE: Out-of-range readings do not stop the stream of messages. Instead, a message with a
          range value well outside the domain [min_range, max_range] will be published.
          Such value is sensor-specific and at the time of this writing, this number is 8.0.
          As per the official ROS's documentation on the Range message
          (https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Range.html),
          "values < range_min or > range_max should be discarded".
          So, it is the consumer's responsibility to handle out-of-range readings.

    """

    def __init__(self):
        # get parameters


        self._i2c_address = 0x29
        self._sensor_name = "tof"
        self._frequency = int(max(1, 10))
        self._mode = rospy.get_param('~mode', 'BETTER')

        # create a VL53L0X sensor handler
        self._sensor = VL53L0X(i2c_address=self._i2c_address)
        self._sensor.open()
        # create publisher
        self._pub = rospy.Publisher(
            "~range",
            Range,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The distance to the closest object detected by the sensor"
        )

        # start ranging
        self._sensor.start_ranging()
        max_frequency = min(self._frequency, int(1.0 / self._accuracy.timing_budget))
        if self._frequency > max_frequency:
            self.logwarn("Frequency of " + self._frequency + " Hz not supported.")
            self._frequency = max_frequency
 
        # create timers
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / max_frequency), self._timer_cb)
        self._fragment_reminder = DTReminder(frequency=self._display_fragment_frequency)

    def _timer_cb(self, _):
        # detect range
        distance_mm = self._sensor.get_distance()
        # pack observation into a message
        msg = Range(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id="/tof"
            ),
            radiation_type=Range.INFRARED,
            field_of_view=self._accuracy.fov,
            min_range=self._accuracy.min_range,
            max_range=self._accuracy.max_range,
            range=distance_mm / 1000
        )
        # publish
        self._pub.publish(msg)

    def on_shutdown(self):
        # noinspection PyBroadException
        try:
            self._sensor.stop_ranging()
        except BaseException:
            pass


if __name__ == '__main__':
    node = ToFNode()
    rospy.spin()
