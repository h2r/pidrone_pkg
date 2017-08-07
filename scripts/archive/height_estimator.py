import rospy
import time
from sensor_msgs.msg import Range

last_reading = time.time()
curr_height = 10.0
curr_z_vel = 0
velocity_alpha = 0.1
height_alpha = 0.1
sensor_angle_alpha = 0.5
sensor_safe_zone = (3,30)
zpub = None
rng = Range()

def sensor_callback(data):
    global curr_height, curr_z_vel, last_reading
    global velocity_alpha, height_alpha, sensor_safe_zone, sensor_angle_alpha
    global zpub, rng

    measured_height = data.range # convert units as needed here
    measured_angle = 0 # we'll start populating this eventually

    dt = time.time() - last_reading # find elapsed time
    last_reading = time.time()

    predicted_height = curr_height + curr_z_vel * dt # predict the height based on previous velocity
    # we can add in control here as well when we want to

    angle_conf = 1.0/measured_angle if measured_angle > 0 else 1.0

    sensor_conf = 1.0
    if measured_height > sensor_safe_zone[1] + 1.0:
            sensor_conf = 1.0/(measured_height - sensor_safe_zone[1.0]) # make this nonlinear as needed?
    elif measured_height < sensor_safe_zone[0]:
            sensor_conf = 0.5 # yeah, i'm not sure what should go here....

    total_conf = sensor_angle_alpha * sensor_conf + (1.0 - sensor_angle_alpha) * angle_conf
    new_height = total_conf * measured_height + (1.0 - total_conf) * predicted_height
    rng.range = new_height
    rng.header.stamp = data.header.stamp

    # update velocity smoothly
    curr_z_vel = velocity_alpha * (new_height - curr_height)/dt + (1.0 - velocity_alpha) * curr_z_vel
    
    if zpub is not None: zpub.publish(rng)


if __name__ == '__main__':
    rospy.init_node('height_estimator')
    zpub = rospy.Publisher('/pidrone/z_est', Range, queue_size=1 )
    rospy.Subscriber('/pidrone/infrared', Range, sensor_callback)

    rospy.spin()
