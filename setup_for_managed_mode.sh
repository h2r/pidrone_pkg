# This is an example from Stefanie's ethernet environment.  Copy to
# your ws root directory and change the variables appropriately.  This
# is supposed to be like ./baxter.sh.  It sets environment variables
# for your environment.
#
# The pi.screenrc file requires this to be in the ws root.  You may
# also set any other ROS environment variables here.

source ../../devel/setup.bash
[ -n "`hostname`" ] && my_hostname="`hostname`" || my_hostname='defaultdrone'
#export ROS_IP=192.168.42.1
export ROS_HOSTNAME="${my_hostname}"
export ROS_MASTER_URI="http://${my_hostname}:11311"
export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"
