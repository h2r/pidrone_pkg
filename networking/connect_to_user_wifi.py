import os

if __name__ == "__main__":
# Stopping Services
    os.system("sudo systemctl stop hostapd")

    os.system("sudo systemctl stop isc-dhcp-server.service")
    os.system("sudo ifdown wlan0")
# Starting wpa_supplicant
    os.system("sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf")
    os.system("sudo ifup wlan0=user_wifi")
