#!/bin/sh

# Configures the network to be in managed mode.  You must have set up
#a a logical interface named 'client' in /etc/interfaces with this
#line:
# iface client inet dhcp

# run as root!
systemctl stop hostapd
systemctl stop isc-dhcp-server.service
ifdown wlan0
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
ifup wlan0=client

