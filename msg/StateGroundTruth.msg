# Message to be used by the motion capture system and simulation scripts that
# output "ground truth" states. This differs from the State message, which is
# intended for use by state estimation nodes (such as nodes implementing UKFs)

geometry_msgs/PoseStamped pose_stamped
geometry_msgs/TwistStamped twist_stamped