#!/bin/bash

# Purpose:    Generates a new wpa_supplicant.conf file by 
#             inserting user-provided network credentials
#             (with hashed password for security) into a 
#             template wpa_supplicant conf file.
# Author:     Nishant Kumar
# Contact:    nishant_kumar1@brown.edu

WPA_CONF_TEMPLATE="wpa_supplicant.conf.template"
OUTPUT_FILE="wpa_supplicant.conf"
BACKUP_FILE="${OUTPUT_FILE}"".bak"
TEMP_FILE_FOR_GENERATED_NETWORK="/tmp/generated_network.XXXX"

function print_wpa_passphrase_error() {
	if [[ -n "${temp_generated}" ]]; then
		echo "${temp_generated}" | sed '/#.*$/d'
	fi
}

function cleanup() {
	echo "Cleaning up..."
	shred -n 50 -z -u "${TEMP_FILE_FOR_GENERATED_NETWORK}" > /dev/null 2>&1
	echo Done
}

trap "ec=\$?; cleanup; exit \$ec" EXIT INT
trap "ec=\$?; print_wpa_passphrase_error; exit \$ec" ERR

read -ep "Enter network's ssid: " ssid
read -esp "Enter network's password: " pass1
echo
read -esp "Verify network's password: " pass2
echo

if [[ "$pass1" != "$pass2" ]];
then
	echo "Password verification failed. Please try again."
	exit 1
fi

if [[ -f "${OUTPUT_FILE}" ]];
then
	cp "${OUTPUT_FILE}" "${BACKUP_FILE}"
	echo Backed up current "${OUTPUT_FILE}" to "${BACKUP_FILE}"
fi

temp_generated="$(echo "${pass1}" | wpa_passphrase "${ssid}")"
echo "${temp_generated}" | sed '/#.*$/d' > "${TEMP_FILE_FOR_GENERATED_NETWORK}"
echo "Generated new network configuration"

# If reached this far, ERR trap did not trigger due to wpa_passphrase error.
# So if ERR trap triggers after this point, it should not print temp_generated,
# since wpa_passphrase is not to blame (i.e. temp_generated does not contain
# an error message). Therefore, setting temp_generated to blank so that 
# print_wpa_passphrase_error func does not print it. 
temp_generated=


sed -e "/<network>/r ${TEMP_FILE_FOR_GENERATED_NETWORK}" -e '//d' < "${WPA_CONF_TEMPLATE}" > "${OUTPUT_FILE}"
echo "Inserted new network configuration into template ${WPA_CONF_TEMPLATE}, then overwrote ${OUTPUT_FILE}"
