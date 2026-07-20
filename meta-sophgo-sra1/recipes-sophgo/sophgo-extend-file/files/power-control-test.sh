#!/bin/bash
address="172.26.161.71"
WPO="warm power off"
FPO="force power off"
cycle_count=1

#$1:host ip address
#$2:cycle_count

function powerOnTest()
{
    local count=1
    echo "power on"
	busctl set-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host RequestedHostTransition s xyz.openbmc_project.State.Host.Transition.On
 	sleep 30
    while ! ping -c 1 $1 &> /dev/null; do
	    echo "Ping $1, Attempt $count: Unsuccessful"
	    ((count++))
	    if [ $count -gt 60 ]
	    then
	    	echo "$2-power on-!!!!!!!!!!!!!!!!!!!"
	    	exit 1
	    fi
	    sleep 10
	done
}

function forcePowerOffTest()
{
    local count=1
    echo "force poweroff"
	busctl set-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/chassis0 xyz.openbmc_project.State.Chassis RequestedPowerTransition s xyz.openbmc_project.State.Chassis.Transition.Off
    sleep 20
    while :
	do
 		output=$(busctl get-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host CurrentHostState)
		if [[ $output == *"xyz.openbmc_project.State.Host.HostState.Off"* ]]; then
    		echo "CurrentHostState is Off"
            break
		else
		 	((count++))
	    	if [ $count -gt 60 ]
	    	then
	    		echo "force poweroff-!!!!!!!!!!!!!!!!!!!"
	    		exit 1
	    	fi
   			sleep 10
		fi
	done
}

function warmPowerOffTest()
{
    local count=1
	echo "warm poweroff"
	busctl set-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host RequestedHostTransition s xyz.openbmc_project.State.Host.Transition.Off
	sleep 20
    while :
	do
 		output=$(busctl get-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host CurrentHostState)
		if [[ $output == *"xyz.openbmc_project.State.Host.HostState.Off"* ]]; then
    		echo "CurrentHostState is Off"
            break
		else
		 	((count++))
	    	if [ $count -gt 60 ]
	    	then
	    		echo "warm poweroff-!!!!!!!!!!!!!!!!!!!"
	    		exit 1
	    	fi
   			sleep 10
		fi
	done
}

function warmRebootTest()
{
    local count=1
	echo "warm reboot"
	busctl set-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host RequestedHostTransition s xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot


	sleep 40
	while ! ping -c 1 $1 &> /dev/null; do
	    echo "Ping $1, Attempt $count: Unsuccessful"
	    ((count++))
	    if [ $count -gt 60 ]
	    then
	    	echo "warm reboot-!!!!!!!!!!!!!!!!!!!"
	    	exit 1
	    fi
	    sleep 10
	done
}

function forceRebootTest()
{
    local count=1
    echo "force reboot"
	busctl set-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host RequestedHostTransition s xyz.openbmc_project.State.Host.Transition.Reboot
    sleep 40
	while ! ping -c 1 $1 &> /dev/null; do
	    echo "Ping $1, Attempt $count: Unsuccessful"
	    ((count++))
	    if [ $count -gt 60 ]
	    then
	    	echo "force reboot-!!!!!!!!!!!!!!!!!!!"
	    	exit 1
	    fi
	    sleep 10
	done

}

function is_powerOn()
{
    local l_output=$(busctl get-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host CurrentHostState)
    if [[ $l_output == *"xyz.openbmc_project.State.Host.HostState.Running"* ]]; then
        echo "1"
    else
        echo "0"
    fi
}


if [ $(is_powerOn) = 1 ] ;then
    while [ $cycle_count -le $2 ]; do
        echo "cycle times: $cycle_count"
        forceRebootTest $1
        warmRebootTest $1
        warmPowerOffTest
        powerOnTest $1 $WPO
        forcePowerOffTest
        powerOnTest $1 $FPO
        ((cycle_count++))
    done
else
    while [ $cycle_count -le $2 ]; do
        echo "cycle times: $cycle_count"
        powerOnTest $1 "first"
        forceRebootTest $1
        warmRebootTest $1
        warmPowerOffTest
        powerOnTest $1 $WPO
        forcePowerOffTest
        powerOnTest $1 $FPO
        ((cycle_count++))
    done
fi