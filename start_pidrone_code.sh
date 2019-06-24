static_ip=192.168.42.1
ip_addr="`ip addr show wlan0 | grep 'inet ' | awk '{print $2}' | cut -f1 -d '/'`"

echo "IP address is: ${ip_addr}"
if [[ "${ip_addr}" == "${static_ip}" ]]; then
    echo "Switching to master mode setup"
    cp setup_for_master_mode.sh setup.sh
else
    echo "Switching to managed mode setup"
    cp setup_for_managed_mode.sh setup.sh
fi

sleep 2

screen -c pi.screenrc
