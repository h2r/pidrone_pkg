# Install Guide for Cleanflight and Control via MSP

## Compiling Firmware

Download [firmware](https://github.com/cleanflight/cleanflight) and compile with

`make TARGET=NAZE OPTIONS=USE_MSP_UART`

There is also a version located [cleanflight_2.1.0_NAZE.hex](here). 

## Flashing Fimware

1. Open the [cleanflight configurator](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb)
and go to the "Firmware Flasher" tab.  This tab is before you connect to the base station. 

![Firmware Flasher](pics/firmware_flasher.png)

2. Click "Load Firmware \[local\]" and load your custom firmware file from 
`cleanflight/obj/cleanflight_x.y.z_NAZE.hex`

![Load Firmware](pics/load_firmware.png)

3. Click the "Flash Firmware" button to flash the flight controller.

![Flash Firmware](pics/flash_firmware.png)

4. If this is a success, the bar at the bottom will say "Programming: 
SUCCESSFUL" and you are ready to move to the next step.

![Successful Flash](pics/success.png)

## Configuration Options

1. Plug in Skyline and click "Connect"

2. Go to "Ports" tab and disable SerialRX for UART2 and click "Save and Reboot"

3. Go to "Configuration" tab and flip the yaw by 180 degrees and click "Save and Reboot"

4. Also change the receiver to "MSP_RX" and click "Save and Reboot"

5. Go to the "Receiver" tab and change the input map to "AERT1234" and click "Save"

6. Go to the "Modes" tab

a. 

7. Plug the skyline back into the Pi and you should be set to fly!

## Other Options

### Throttle Angle Compensation

Go to "CLI" tab and type

`set thr_corr_value = XX` and `set thr_corr_angle = YY`

This will set it (linearly?) so that it adds `XX` to the throttle when at angle `YY`
