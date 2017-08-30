## Sharp Infrared Range Finder

The sharp IR sensor shoots out an infrared beam and measures distance by the angle at which that infrared light is bounced back. 

It is an _analog_ sensor, which means that it outputs a range of voltages to indicate its measurement, as opposed to a _digital_ signal (1s and 0s). Unfortunately, the Raspberry Pi 3 does not have any way to read analog values, so we need a device to convert the analog voltage to a digital signal we can make sense of... 

Enter the Analog to Digital Converter (ADC)!

![ADC](pics/ir_setup/adc.jpg)

This nifty little device can read four analog signals (A0 - A4) and outputs a digital protocal (i2c) which the Pi can read.

### Prep the Pi Header

