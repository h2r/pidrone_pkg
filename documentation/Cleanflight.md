# Install Guide for Cleanflight and Control via MSP

## Compiling Firmware

Download [firmware](https://github.com/cleanflight/cleanflight) and compile with

`make TARGET=NAZE OPTIONS=USE_MSP_UART`

## Flashing Fimware

1. Open the [cleanflight configurator](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb)
and go to the "Firmware Flasher" tab.

2. Click "Load Firmware \[local\]" and load your custom firmware file from 
`cleanflight/obj/cleanflight_x.y.z_NAZE.hex`

## Configuration Options

1. Plug in Skyline and click "Connect"

2. Go to "Ports" tab and disable SerialRX for UART2

3. Go to "Configuration" tab and flip the yaw by 180 degrees

4. Also change the receiver to "MSP_RX"

5. Go to the "Reveiver" tab and change the input map to "AERT1234"

6. Plug the skyline back into the Pi and you should be set to fly!
