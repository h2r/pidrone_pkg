## Sharp Infrared Range Finder

The sharp IR sensor shoots out an infrared beam and measures distance by the angle at which that infrared light is bounced back. 

It is an _analog_ sensor, which means that it outputs a range of voltages to indicate its measurement, as opposed to a _digital_ signal (1s and 0s). Unfortunately, the Raspberry Pi 3 does not have any way to read analog values, so we need a device to convert the analog voltage to a digital signal we can make sense of... 

Enter the Analog to Digital Converter (ADC)!

![ADC](pics/ir_setup/adc.jpg)

This nifty little device can read four analog signals (A0 - A4) and outputs a digital protocal (i2c) which the Pi can read.

### The Pi Header

Cut red, yellow, green, and black 24 AWG stranded wires to **4 inch** lengths. Strip about 0.5 cm on either side and tin both sides. 

Solder the wires to your 2x6 female header as shown in the image below. Be careful not to short adjacent pins.

![PI HEADER](pics/ir_setup/wires-on-header.jpg)

Take your infrared connector cable and cut it to **7 inch** length leaving the white stubby connector on. Strip 0.5cm.

![IR CABLE](pics/ir_setup/cut-ir-wire.jpg)

Twist the black and red wires from your connector and from your header together as shown.

![IR CABLE](pics/ir_setup/twist-header-ir.jpg)

Now solder your connectors to the ADC.

| Wire Color                          | ADC Pin |
|:------------------------------------|:-------:|
|    Red                              | VDD     |
|    Black                            | GND     |
|    Yellow (from Pi connector)       | SCL     |
|    Green                            | SDA     |
|    Yellow (from infrared connector) | A0      |

![SOLDERED ADC](pics/ir_setup/solder-to-adc.jpg)

Push the IR sensor connector wires towards the mounting holes, and the Pi connector wires in the opposite direction (as shown). Apply hot glue over the wires to reinforce your solder joints.

![SOLDERED ADC](pics/ir_setup/hot-glue-adc.jpg)

Use short M1.5 bolts to screw the ADC to the Pi mount on the flat side of the mount. 

![SOLDERED ADC](pics/ir_setup/adc-mount.jpg)