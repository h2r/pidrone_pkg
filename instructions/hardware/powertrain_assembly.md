## Hardware Assembly


### Check your components
  Before you start assembling your drone, it's important to make sure you have the right components. For the hardware assembly, you will need:
  
* 4 ESCs (Electronic Speed Controllers). These convert DC power from the battery to [three-phase AC](https://en.wikipedia.org/wiki/Three-phase_electric_power), which drives the motor at the desired speed.

![ESC PACKS](pics/esc-packaging.jpg)

* 4 Motors. These brushless motors are driven by three phase AC (hence the three wires). On the bottom of the boxes, you will notice a part number of the form SE1806-***_XY_***, where ***_X_*** is either 11 or 13, and ***_Y_*** is either CW or CCW. The ***_X_*** refers to the number of coils inside the motor. Make sure all of your motors have the same ***_X_*** so they spin at the same speed. The ***_Y_*** refers to clockwise or counter clockwise. You need two of each.

![MOTOR BOXES](pics/motor-boxes.jpg)

* 4 Props. Your bag should contain 2 CW and 2 CCW propellers.
TODO: Picture of Props

* 1 frame kit. This is the skeleton of your quadcopter.

![FRAME BOX](pics/frame-box.jpg)

* 1 Power Distribution Board (PDB). This lets you distribute power from the battery to the motors and the Raspberry Pi.

![PDB](pics/pdb.jpg)

* 1 Battery lead. This will let you connect a battery to the PDB.

![BATTERY CONNECTOR](pics/tinned-batt-connector.jpg)

* 1 Skyline box
TODO: Image of skyline box

![Box for flight controller](pics/components/skyline.jpg)

* 1 Raspberry Pi

![Raspberry Pi](pics/components/pi.jpg)

* 1 Battery Elimination Circuit (BEC). This is a 5V [switching regulator](https://www.dimensionengineering.com/info/switching-regulators) used to power the Pi.

![Battery Elimination Circuit](pics/components/bec.jpg)

* 1 IR sensor

![IR Sensor](pics/components/ir.jpg)

* 1 camera

![Camera](pics/components/camera.jpg)

* 4 standoffs, with 4 M3 bolts

![SHORT SCREWS IN STANDOFF](pics/m3-in-standoffs.jpg)

* 3mm heatshrink (cut into 12, 1cm pieces)

![HEATSHRINK](pics/shrinkwrap-cut.jpg)

* 4 zipties

### Bolt on the Motors
Two of the motors are counter-clockwise threaded and the other two are clockwise threaded. Screw the included nuts that came each motor onto the motor shaft so you don't forget which is which. 

TODO: Add image in next line

The motors with the red nuts will spin counter-clockwise and the black clockwise, so it is critical that they are attached to the correct arms. Image REF shows which side of the frame is the front. The front right motor will spin CCW, and the front left motor will spin CW. Diagonal motors spin in the same direction. The next image shows how the propellers will spin:
![MOTOR DIRECTIONS](pics/motor-directions.jpg)
In order to ensure the nuts tend to tighten during flight instead of loosen, you want each nut to thread in the opposite direction as its motor spins. For example, this means the front right motor's nut should tighten in the CW direction. When mounting the motors, make sure to route the wire through the hole in the frame to the underside of the frame _before_ bolting on the motor. 

![FRAME WITH MOTORS](pics/motor-orientation.jpg)

Use two of the long M3 bolts included with the motor and screw them in tightly.

![SINGLE MOTOR BOLT IN](pics/single-motor-installation.jpg)

Screw the short M3 bolts into the brass 6mm M3 standoffs.

Use a hex key to screw the standoffs through the PDB into grooves in the frame in the shown orientation. Apply a downward pressure as you are turning to ensure the standoffs bite into the plastic. Be careful not to overtighten the standoffs and strip out the groove.

![MOUNT PDB](pics/screw-standoffs.jpg)

### Solder ESCs to Motors

Place your 3mm diameter heatshrink tubing cut to 1cm sections onto each of the motor leads.

![HEATSHRINK ON WIRES](pics/motor-wires-shrinkwrap.jpg)

If you have not done so already, you will need to cut off the connectors on the ends of the ESC wires, and strip the plastic casing. You need to do this for the two input wires and the three output wires, as shown in the picture:

![CUT AND STRIP ESCs](pics/esc-cut-strip.jpg)

Tin the ESC wires thoroughly if you haven't already. Tin the motor wires as well. (Note: They come pre-tinned, but with a different, higher temperature solder and not quite enough of it)

![TIN WIRES](pics/tinned-wires.jpg)

Solder the ESC wires to the motor wires. It does not matter which ESC wire is connected to which motor wire. However, routing will be easier if you make sure they are not too twisted.

![SOLDER TOGETHER](pics/motors-wires-no-shrinkwarp-2.jpg)

After soldering each motor to its corresponding ESC, move the heatshrink over the solder joint and use a heat gun to shrink it.

![HEATSHRINK OVER JOINTS](pics/motor-esc-shrinkwrapped.jpg)


## Solder the PDB

Just like the human body has a circulatory system to carry oxygen-rich blood to wherever it is needed, the drone has a power distribution board (PDB) to take the all-important battery power and send it to every component. In this step, you will mount 4 ESCs (electronic speed controllers), a BEC (battery eliminator circuit), a battery lead with XT60 connector, and a battery monitor lead to your PDB (power distribution board). 

### Strip Wires

Cut off all the bullet connectors on the thick wires of the ESC as close to the connector as possible. Leave the PWM signal connector alone.
Strip 0.5 cm from each of the wires you just cut. 

![ESC PRE POST SNIP](pics/esc-cut-strip.jpg)

Strip 0.5cm off your battery lead with XT60 connector.

![XT60 PIGTAIL STRIPPED](pics/xt60.jpg)

Inside the plastic box labeled, SKYLINE 32, find the battery monitor lead, cut off the two larger connectors, and strip 1cm.

![SKYLINE BATTERY MONITOR PRE POST STRIP](pics/monitor-cut-strip.jpg)

### Tin PDB and Stripped wires.

Tin all 20 ESC leads.
Twist the battery monitor leads around the BEC leads and tin.

![BEC AND BATT MONITOR](pics/bec-and-monitor-lead.jpg)

Thoroughly tin the battery lead with XT60 connector. It is important that solder flows all the way through the exposed wire. 

![TINNED BATT LEAD](pics/pigtail-soldering.JPG)

Tin the pads on the PDB as shown. Leave the 5V OUT and 12V OUT pads alone. 

![TINNED PDB](pics/pdb-tinned.jpg)

### Connect components to the PDB

Solder the tinned components to the PDB as shown. Make sure Red goes to **+** and black/brown goes to *-*.

![PDB OCTOPUS FAR](pics/pdb-complete-far.jpg)

![PDB OCTOPUS CLOSE](pics/pdb-complete-close.jpg)

## Routing
Now everything has been soldered to the PDB, and you can start worrying about wire management. Route ESC wires nicely with zip ties. There is no single correct way to do this. You just need to ensure that the wires will not be cut by the spinning props. It is best to secure the ESCs and wires underneath the frame. The next several images provide some examples for how you might want to route your wires and ESCs.

![ROUTE ZIPTIE](pics/ziptie-location.jpg)

![WIRES IN ZIPTIE](pics/esc-routing.jpg)

![ZIPPED TIE](pics/ziptied-esc.jpg)

![ALL FOUR ZIPTIES](pics/ziptied-all-motors.jpg)


## Camera Attachment
Now you will attach the camera to your frame. Place double-sided sticky tape around the camera, as shown in the picture.

![TAPED CAMERA](pics/camera_install/tape-cam.jpg)

Place the camera through the hole in the back of the frame, as in the picture. Make sure it is straight. 

![CAMERA PLACEMENT](pics/camera_install/cam-on-frame.jpg)

## IR Attachment
Zip tie the IR sensor onto the bottom of the frame's front platform, as in the picture.
![IR Attachment Location](pics/zip-ir-1.jpg)

