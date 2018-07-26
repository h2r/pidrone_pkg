# PiDrone Pkg

For Stefanie Tellex's class

## How to Connect to the Drone

1. Plug in the battery or power supply to the drone
2. Connect to the wifi network corresponding to the name of your drone
3. `ssh -Y pi@192.168.42.1` or `ssh -Y pi@<nameofdrone>.local`

## How to Fly

- `roscd pidrone_pkg`
- `screen -c pi.screenrc`

### Programs Needed to Fly

In the screen session:

- Window 0: Used for roscore. Should launch automatically with `roscore`
- Window 1: Used for vim/editing. Will not launch automatically
- Window 2: Used for emacs/editing. Will not launch automatically
- Window 3: Used for multiwii/skyline32 control algorithm. `python state_controller.py`
- Window 4: Used for optical flow from the raspberry pi camera and localization. `python picam_localization_distance.py`
- Window 5: Used for ir sensor reading. `python infrared_pub.py`
- Window 6: Used joystick control. `python joy_node.py`

On your commputer running ros:

Download the [joystick interface library](http://wiki.ros.org/joy) and run `rosrun joy joy_node`

### Controls

- Button 6: Arm the drone
- Button 5: Disarm the drone
- Button 8: Fly

If you tap button 8, the drone will hover in the air. If you hold it down, you
can send velocity commands using the right joystick and altitude commands using
the left.

## Warnings

1. When connected to the power supply, if the drone draws too much power, the
   supply will not be able to keep up and the drone will shut off and crash
   into the ground fairly dramatically. It should only fall, since it is no
   longer recieving power, and this does not happen on battery power. Nobody
   has been hit by this!?

## Misc

Using [git flow](http://danielkummer.github.io/git-flow-cheatsheet/)
