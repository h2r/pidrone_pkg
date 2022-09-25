# This image allows you to have a Ubuntu 16.04 + ROS Kinetic setup.
# You can mount the path to the 'robotdev' repository in your host
# machine to the same path in the container. We ask you to use the
# same username in the container as in your host machine. This
# simplifies the maintenance of the 'robotdev' repository.
# This setup relies on the nice ros:kinetic image provided
# on Docker Hub.
# /author: Kaiyu Zheng
FROM ros:kinetic

# Install software
RUN apt-get update
RUN apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y emacs
RUN apt-get install -y sudo
RUN apt-get install -y python-pip
RUN apt-get install -y net-tools
RUN apt-get install -y iproute2
RUN apt-get install -y iputils-ping
RUN apt-get install -y openssh-client openssh-server
RUN apt-get install -y ros-kinetic-desktop-full
RUN apt-get install -y ros-kinetic-navigation
RUN apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers
RUN apt-get install -y ros-kinetic-joy
RUN apt-get install -y ros-kinetic-gmapping ros-kinetic-navigation
RUN apt-get install -y ros-kinetic-rviz-imu-plugin
RUN apt-get install -y ros-kinetic-ar-track-alvar
RUN apt-get install -y ros-kinetic-moveit
RUN apt-get install -y ros-kinetic-moveit-commander
RUN apt-get install -y ros-kinetic-moveit-visual-tools
RUN apt-get install -y ros-kinetic-moveit-ros-visualization
RUN apt-get install -y ros-kinetic-moveit-planners-ompl
RUN apt-get install -y ros-kinetic-moveit-simple-controller-manager
RUN apt-get install -y ros-kinetic-trac-ik-kinematics-plugin
RUN apt-get install -y gdb
RUN apt-get install -y mlocate
RUN apt-get install -y screen
RUN apt-get install -y emacs
RUN apt-get install -y git
RUN apt-get install -y netcat nmap wget iputils-ping openssh-client vim less
RUN pip install picamera
RUN pip install filterpy

ARG hostuser
ARG hostgroup
ARG hostuid
ARG hostgid
ARG hostname

RUN echo Host user is $hostuser:$hostuser
RUN groupadd --gid $hostgid $hostgroup
RUN adduser --disabled-password --gecos '' --gid $hostgid --uid $hostuid $hostuser
RUN adduser $hostuser sudo
# Ensure sudo group users are not asked for a p3assword when using sudo command
# by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
/etc/sudoers

USER $hostuser
WORKDIR /home/$hostuser
ENV HOME=/home/$hostuser
RUN mkdir $HOME/repo
RUN mkdir -p $HOME/catkin_ws/


# print some info on start
RUN echo "echo -e 'Welcome! You are now in a docker container ().'" >> $HOME/.bashrc
RUN echo "echo -e \"Docker ID: $(basename $(cat /proc/1/cpuset))\"" >> $HOME/.bashrc
RUN echo "export ROS_MASTER_URI=http://$hostname:11311" >> $HOME/.bashrc
RUN echo "cd $HOME/catkin_ws/src/pidrone_pkg && source setup.sh" >> $HOME/.bashrc
CMD ["bash"]

