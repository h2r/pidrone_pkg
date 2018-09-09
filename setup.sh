# This is an example from Stefanie's ethernet environment.  Copy to
# your ws root directory and change the variables appropriately.  This
# is supposed to be like ./baxter.sh.  It sets environment variables
# for your environment.
#
# The pi.screenrc file requires this to be in the ws root.  You may
# also set any other ROS environment variables here.


# on-board
#source ../../devel/setup.bash
#export ROS_HOSTNAME=messi11
#export ROS_MASTER_URI=http://kalessin:11311
#export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"

# off-board
source ../../devel/setup.bash
export ROS_IP=`hostname -I`
export ROS_HOSTNAME=baichuan-XPS
export ROS_MASTER_URI=http://baichuan-XPS:11311
export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"

# following two lines are for conda and docker, and remove the same line in .bashrc if it has
#export PATH=/home/baichuan/miniconda2/bin:$PATH
#export PYTHONPATH=/home/baichuan/miniconda2/envs/drone/lib/python2.7/site-packages:$PYTHONPATH
#source activate drone

#source ../../devel/setup.bash
#export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"
#export ROS_MASTER_URI=http://messi11:11311

