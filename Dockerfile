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
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils \
    emacs \
    sudo \
    python-pip \
    net-tools \
    iproute2 \
    iputils-ping \
    openssh-client openssh-server \
    ros-kinetic-desktop-full \
    ros-kinetic-navigation \
    ros-kinetic-ros-control ros-kinetic-ros-controllers \
    ros-kinetic-joy \
    ros-kinetic-gmapping ros-kinetic-navigation \
    ros-kinetic-rviz-imu-plugin \
    ros-kinetic-ar-track-alvar \
    ros-kinetic-moveit \
    ros-kinetic-moveit-commander \
    ros-kinetic-moveit-visual-tools \
    ros-kinetic-moveit-ros-visualization \
    ros-kinetic-moveit-planners-ompl \
    ros-kinetic-moveit-simple-controller-manager \
    ros-kinetic-trac-ik-kinematics-plugin \
    gdb \
    mlocate \
    screen \
    emacs \
    git \
    netcat nmap wget iputils-ping openssh-client vim less \
    python-numpy \
    python-smbus \
    python-scipy \
    locate \
    ros-kinetic-rosbridge-suite \
    ros-kinetic-web-video-server \
    nano

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
RUN groupadd --gid $hostgid $hostgroup && groupadd i2c
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

# print some info on start
RUN echo "echo -e 'Welcome! You are now in a docker container ().'" >> $HOME/.bashrc
RUN echo "echo -e \"Docker ID: $(basename $(cat /proc/1/cpuset))\"" >> $HOME/.bashrc

run echo "export LD_LIBRARY_PATH=/opt/vc/lib/:/home/$USER/raspicam_node_ws/devel/lib:\$LD_LIBRARY_PATH" >> $HOME/.bashrc

run echo "export PYTHONPATH=\$PYTHONPATH:/home/$USER/raspicam_node_ws/devel/python2.7/dist-packages" >> $HOME/.bashrc

run echo "export PYTHONPATH=\$PYTHONPATH:/home/$USER/catkin_ws/src/pidrone_pkg/scripts" >> $HOME/.bashrc


RUN echo "export ROS_MASTER_URI=http://$hostname:11311" >> $HOME/.bashrc
RUN echo "cd $HOME/catkin_ws/src/pidrone_pkg && source setup.sh" >> $HOME/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$HOME/raspicam_node_ws/src" >> $HOME/.bashrc
RUN sudo apt-get install libopencv-dev -y
RUN pip install --no-cache-dir pyserial

CMD ["bash"]



