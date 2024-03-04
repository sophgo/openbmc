#!/bin/bash
jtag_dev=/dev/jtag0
imagePath=$1
image=$imagePath/image-bmc
output=""
max_attempts=2
attempt_count=0
echo "CPLD update $1"

while [[ ! $output =~ Success! && $attempt_count -lt $max_attempts ]]; do
    output=$(svf -n ${jtag_dev} -p  ${image} 2>&1)
    attempt_count=$((attempt_count+1))
    sleep 1
done

if [[ $output =~ Success! ]]; then
  echo "CPLD update successed!"
  systemctl restart sophgo-cpld-monitor.service
else
  echo "CPLD update failed!"
  exit 1
fi
