#!/bin/sh

echo gpio | sudo tee /sys/class/leds/led1/trigger

echo 1 | sudo tee /sys/class/leds/led1/brightness
sleep 1
echo 0 | sudo tee /sys/class/leds/led1/brightness
sleep 1
echo 1 | sudo tee /sys/class/leds/led1/brightness
sleep 1
echo 0 | sudo tee /sys/class/leds/led1/brightness
sleep 1
echo 1 | sudo tee /sys/class/leds/led1/brightness
sleep 1
echo 0 | sudo tee /sys/class/leds/led1/brightness
sleep 1


echo input | sudo tee /sys/class/leds/led1/trigger
