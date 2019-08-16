#!/bin/bash

# Purpose:    Stops the device from running as a DHCP 
#             server, then puts it on the network 
#             specified in WPA_SUPP_CONF_FILE.
# NOTE:       Re-tooled from connect_to_user_wifi.py
#             in h2r/pidrone_pkg github repo.
# Author:     Nishant Kumar
# Contact:    nishant_kumar1@brown.edu

WPA_SUPP_CONF_FILE="/etc/wpa_supplicant/wpa_supplicant.conf"
WIFI="user_wifi"

if [[ ! -r "${WPA_SUPP_CONF_FILE}" ]]; then
	echo "Missing required conf file ${WPA_SUPP_CONF_FILE}. Please create it and try again."
	exit 1
fi

# Stopping Services
sudo systemctl stop hostapd
sudo systemctl stop isc-dhcp-server.service
sudo ifdown wlan0

# Starting wpa_supplicant
sudo wpa_supplicant -B -i wlan0 -c "${WPA_SUPP_CONF_FILE}"
sudo ifup wlan0="${WIFI}"
