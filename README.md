# PiDrone Pkg

For Stefanie Tellex's class

## How to Connect to the Drone

1. Plug in the battery or power supply to the drone.
2. Connect to the wifi network corresponding to the name of your drone.
3. `ssh duckiesky@192.168.42.1` or `ssh duckiesky@<nameofdrone>.local`.

## How to Fly

1. On drone, run the following:
   - `roscd pidrone_pkg`
   - `./start_pidrone_code.sh` (this launches a screen session)
2. On base station:
   - Open `pidrone_pkg/web/index.html` (a.k.a. web interface) in a web browser; either double-click .html file or drag-and-drop it into a web browser to get it to render as a web page (meaningless to open/edit the file itself!).
   - Type IP address or hostname of drone into `hostname` box and click `Connect`.

## Programs Needed to Fly

- In the screen session (launched by `./start_pidrone_code.sh`):

  - Window `0`: Used for roscore. Responsible for starting a ROS Master.
  - Window `1`: Used for flight controller node. Responsible for receiving desired behavior (e.g. arm drone, move drone forward, etc.) and sending corresponding lower-level signals to flight controller to achieve said behavior.
  - Window `2`: Used for PID controller node. Node manages multiple PID controllers, such as throttle PID controller.
  - Window `3`: Used for running a state estimation algorithm. List of supported algorithms includes: exponential moving average, 2D UKF, 7D UKF, and more.
  - Window `4`: Used for reading data from camera and doing either: 1) optical flow and dead reckoning or 2) localization. Can run optical flow and dead reckoning via `python vision_flow_and_phase.py`; can run localization onboard with `python vision_localization_onboard.py` or offboard with `python vision_localization_offboard.py`.
  - Window `5`: Used for reading data from the IR sensor and publishing it to a ROS topic.
  - Window `6`: Used to run a web server that lets a client receive information about the drone or send control input to the drone.
  - Window `7`: Used to run a web video server that lets a client receive camera image data.
  - Window `8`: Free window that can be used for anything.
  - Window `9`: Free window that can be used for anything.

- Switch screen windows using `` ` `` + `<window number>`. For example,  `` ` `` + `0` switches to window `0`.
- Exit the screen by using `` ` `` + `:` (i.e. `Shift` key + `;` key), then typing `quit` and pressing the `Enter` key. Altogether, you should see `:quit` at the bottom left of the terminal window before you press the `Enter` key.

## Controls

- See web interface for a list of drone input controls.

## How to Connect to Home Network

1. (Skip step if done before)
   On drone, run the following:
   - ```
      cd /etc/wpa_supplicant
      sudo ./generate_wpa_supplicant_conf.sh
      ```
      or alternatively
      ```
      roscd pidrone_pkg/
      cd networking
      sudo ./generate_wpa_supplicant_conf.sh
      sudo mv wpa_supplicant.conf /etc/wpa_supplicant
      ```
      When prompted, enter credentials for home network.
2. On drone, run the following:
   - ```
      roscd pidrone_pkg/
      python networking/connect_to_user_wifi.py
      ```
      or alternatively
      ```
      cd /etc/wpa_supplicant
      ./connect_to_user_wifi.sh
      ```
   - the ssh session to the drone will terminate shortly after. The wifi connection to the drone will disconnect.
3. On base station:
   - connect to home network (instead of drone network)
   - open a terminal or command prompt and run `ssh duckiesky@<hostname>`, where `<hostname>` is the hostname of the drone (found in file `/etc/hostname`). The default is `duckiesky-drone`.
   - A password prompt will appear. After entering the password, the ssh connection to the drone will be complete.

## Warnings

- Starting with release v2.2-kinetic, all releases work on ROS Kinetic (unless specified otherwise). Older releases do not - they only work up to ROS Indigo. Furthermore, releases v2.2-kinetic and higher do not have backward compatibility with older versions of ROS.
- When connected to the power supply, if the drone draws too much power, the
   supply will not be able to keep up and the drone will shut off and crash
   into the ground fairly dramatically. It should only fall, since it is no
   longer recieving power, and this does not happen on battery power. Nobody
   has been hit by this!?

## Misc

- Please use [git flow](http://danielkummer.github.io/git-flow-cheatsheet/) paradigm when contributing to this project.
