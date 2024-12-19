#!/bin/bash
jtag_dev=/dev/jtag0
output=""
max_attempts=2
attempt_count=0


function install_jtag_driver()
{
  modprobe jtag-aspeed-internal
}

function uninstall_jtag_driver()
{
  modprobe -r jtag-aspeed-internal
}

function reset_cpld()
{
  gpioset 0 31=1
  sleep 1
  gpioset 0 31=0
  usleep 30000
  gpioset 0 31=1
}

echo "CPLD update $1"

if [ $(ls /dev | grep jtag | wc -l) == 0 ] ;then
  modprobe jtag-aspeed-internal
    if [ $? -ne 0 ] ;then
      echo "Jtag driver install failed!"
      exit 1
    fi
fi


while [[ $attempt_count -lt $max_attempts ]]; do
    output=$(svf -n ${jtag_dev} -p  /tmp/images/$1/image-bmc 2>&1)
    if [[ $output =~ Success! ]]; then
      break;
    fi
    attempt_count=$((attempt_count+1))
    sleep 1
done

if [[ $output =~ Success! ]]; then
  echo "CPLD update successed!"
  # reset cpld
  reset_cpld
  systemctl restart sophgo-cpld-monitor.service
else
  echo "CPLD update failed!"
  exit 1
fi
