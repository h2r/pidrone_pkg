Run camera_on_board.py on pi, and run camera_off_board.py on your laptop.
For ROS network issue, please check http://wiki.ros.org/ROS/Tutorials/MultipleMachines
For docker, you need to run the image with --net=host

# Example of environment variable  
setup.sh on pi
```bash
source ../../devel/setup.sh
export ROS_HOSTNAME="messi11"
export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"
export ROS_MASTER_URI=http://messi11:11311
```

setup.sh on laptop
```bash
source ../../devel/setup.sh
export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"
export ROS_MASTER_URI=http://messi11:11311
```
