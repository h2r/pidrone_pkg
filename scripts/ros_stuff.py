import tf
import tf2_ros
import rospy
import rospkg
import os

import geometry_msgs.msg
  

def setRosParams():
    with open('/proc/cpuinfo', 'r') as myfile:
        data=myfile.readlines()
        serial = data[-1].split()[-1][0:-1]
        print "serial", serial
        rospy.set_param('/manifest/robot_serial', serial)
    
    rospack = rospkg.RosPack()
    path = rospack.get_path('pidrone_pkg')
    with open('%s/urdf/pidrone.urdf' % path, 'r') as myfile:
        data = myfile.read()
        rospy.set_param('/robot_description', data)

def identity():
    transform = geometry_msgs.msg.TransformStamped()


    transform.header.stamp = rospy.Time.now()
    transform.transform.translation.x = 0
    transform.transform.translation.y = 0
    transform.transform.translation.z = 0
    transform.transform.rotation.x = 0
    transform.transform.rotation.y = 0
    transform.transform.rotation.z = 0
    transform.transform.rotation.w = 1
    return transform

def publishStaticTransforms():

    sbr = tf2_ros.StaticTransformBroadcaster()

    transforms = []

    
    transform = identity()
    transform.header.frame_id = "base"
    transform.child_frame_id = "base_link"
    transforms.append(transform)


    transform = identity()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "downward_cam_link"
    transforms.append(transform)


    transform = identity()
    transform.header.frame_id = "downward_cam_link"
    transform.child_frame_id = "downward_cam_optical_frame"
    transforms.append(transform)


    transform = identity()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "ir_link"
    transforms.append(transform)
    
    sbr.sendTransform(transforms)



    
def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    setRosParams()
    r = rospy.Rate(60)

    publishStaticTransforms()
    
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main()
