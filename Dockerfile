# This image allows you to have a Ubuntu 16.04 + ROS noetic setup.
# You can mount the path to the 'robotdev' repository in your host
# machine to the same path in the container. We ask you to use the
# same username in the container as in your host machine. This
# simplifies the maintenance of the 'robotdev' repository.
# This setup relies on the nice ros:noetic image provided
# on Docker Hub.
# /author: Kaiyu Zheng
FROM ros:noetic

# Install software
RUN apt-get update
RUN apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y emacs \
RUN apt-get install -y sudo
RUN apt-get install -y python-pip
RUN apt-get install -y net-tools
RUN apt-get install -y iproute2
RUN apt-get install -y iputils-ping
RUN apt-get install -y openssh-client openssh-server
RUN apt-get install -y ros-noetic-desktop-full
RUN apt-get install -y ros-noetic-navigation
RUN apt-get install -y ros-noetic-ros-control ros-noetic-ros-controllers
RUN apt-get install -y ros-noetic-joy
RUN apt-get install -y ros-noetic-gmapping ros-noetic-navigation
RUN apt-get install -y ros-noetic-rviz-imu-plugin
RUN apt-get install -y ros-noetic-ar-track-alvar
RUN apt-get install -y ros-noetic-moveit
RUN apt-get install -y ros-noetic-moveit-commander
RUN apt-get install -y ros-noetic-moveit-visual-tools
RUN apt-get install -y ros-noetic-moveit-ros-visualization
RUN apt-get install -y ros-noetic-moveit-planners-ompl
RUN apt-get install -y ros-noetic-moveit-simple-controller-manager
RUN apt-get install -y ros-noetic-trac-ik-kinematics-plugin
RUN apt-get install -y gdb
RUN apt-get install -y mlocate
RUN apt-get install -y screen
RUN apt-get install -y emacs
RUN apt-get install -y git
RUN apt-get install -y netcat nmap wget iputils-ping openssh-client vim less
RUN apt-get install -y python-numpy
RUN apt-get install -y python-smbus
RUN apt-get install -y python-scipy
RUN apt-get install -y locate
RUN apt-get install -y ros-noetic-rosbridge-suite
RUN apt-get install -y ros-noetic-web-video-server
RUN apt-get install -y python-is-python3
# check out the version that has the buggy port of libmmal to 64 bit.
# this didn't actually work sadly, got a weird mmal error when trying
# to open picamera, but I think I still need it for raspicam_node.
RUN git clone https://github.com/raspberrypi/userland && cd userland && git checkout 4a57ea4107a4d48564242b21608ab259da5ced35 && ./buildme --aarch64

#RUN pip install picamera


ARG hostuser
ARG hostgroup
ARG hostuid
ARG hostgid
ARG hostname
ARG i2cgid
ARG dialoutgid
ARG videogid

RUN echo Host user is $hostuser:$hostuser
RUN groupadd --gid $hostgid $hostgroup
RUN groupmod --gid $i2cgid i2c; exit 0
RUN groupmod --gid $dialoutgid dialout
RUN groupmod --gid $videogid video
RUN adduser --disabled-password --gecos '' --gid $hostgid --uid $hostuid $hostuser
RUN adduser $hostuser sudo
RUN adduser $hostuser i2c
RUN adduser $hostuser dialout
RUN adduser $hostuser video
# Ensure sudo group users are not asked for a p3assword when using sudo command
# by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
/etc/sudoers

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

USER $hostuser
WORKDIR /home/$hostuser
ENV HOME=/home/$hostuser
RUN mkdir $HOME/repo
RUN mkdir -p $HOME/catkin_ws/src
RUN rosdep update


# RUN mkdir -p raspicam_node_ws/src && cd raspicam_node_ws/src/ && git clone -b noetic-devel https://github.com/UbiquityRobotics/raspicam_node && cd .. && source /opt/ros/noetic/setup.bash && catkin_make

# WORKDIR $HOME/catkin_ws/src
# RUN git clone -b noetic-devel https://github.com/UbiquityRobotics/raspicam_node.git

# print some info on start
RUN echo "echo -e 'Welcome! You are now in a docker container ().'" >> $HOME/.bashrc
RUN echo "echo -e \"Docker ID: $(basename $(cat /proc/1/cpuset))\"" >> $HOME/.bashrc
RUN touch /etc/ros/rosdep/sources.list.d/30-ubiquity.list
RUN echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" >> /etc/ros/rosdep/sources.list.d/30-ubiquity.list
RUN rosdep update
RUN cd $HOME/catkin_ws && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
# run echo "export LD_LIBRARY_PATH=/opt/vc/lib/:/home/$USER/raspicam_node_ws/devel/lib:\$LD_LIBRARY_PATH" >> $HOME/.bashrc

#run echo "export PYTHONPATH=\$PYTHONPATH:/home/$USER/raspicam_node_ws/devel/python2.7/dist-packages" >> $HOME/.bashrc

run echo "export PYTHONPATH=\$PYTHONPATH:/home/$USER/catkin_ws/src/pidrone_pkg/scripts" >> $HOME/.bashrc


RUN echo "export ROS_MASTER_URI=http://$hostname:11311" >> $HOME/.bashrc
RUN echo "cd $HOME/catkin_ws/src/pidrone_pkg && source setup.sh" >> $HOME/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$HOME/raspicam_node_ws/src" >> $HOME/.bashrc


CMD ["bash"]

