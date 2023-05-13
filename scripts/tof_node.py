#!/usr/bin/env python3

import rospy
import argparse
import numpy as np

from sensor_msgs.msg import Range
from std_msgs.msg import Header

from dt_vl53l0x import \
    VL53L0X, \
    Vl53l0xAccuracyMode


# addition 2023-03-11 by joshua-8 is a variance parameter and artificial noise to challenge the ukf filter more.
parser = argparse.ArgumentParser()
parser.add_argument("--variance", type=float, default=None,
                    help="how much random noise should be added to the distance readings? variance, meters")
args = parser.parse_args()
simulated_noise_sd = None
if args.variance:
    simulated_noise_sd = np.sqrt(args.variance)
    # numpy random uses standard deviation not variance but ukf seems to use variance
    print("simulating noise with standard deviation: "+str(simulated_noise_sd))


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

        self._i2c_address = 0x29
        self._sensor_name = "tof"
        self._mode = 'BETTER'

        # create a VL53L0X sensor handler
        self._sensor = VL53L0X()
        self._sensor.open()
        # create publisher
        self._pub = rospy.Publisher('/pidrone/range', Range, queue_size=1)

        # start ranging
        self._sensor.start_ranging()

        # create timers
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / 30), self._timer_cb)

    def _timer_cb(self, _):
        # detect range
        distance_mm = self._sensor.get_distance()

        # add noise with normal distribution and specified Standard Deviation
        # multiply by 1000 to convert from meters to millimeters
        if simulated_noise_sd is not None:
            distance_mm = distance_mm + 1000 * \
                np.random.normal(loc=0, scale=simulated_noise_sd)

        # pack observation into a message
        msg = Range(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id="/tof"
            ),
            radiation_type=Range.INFRARED,
            field_of_view=10,
            min_range=50 / 1000.0,
            max_range=1200 / 1000.0,
            range=distance_mm / 1000.0
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
    rospy.init_node("tof_node")
    node = ToFNode()
    rospy.spin()
