#!/bin/bash

cycle_count=1


function setPowerOn()
{
    busctl set-property                                     \
        	xyz.openbmc_project.State.Host              \
    		/xyz/openbmc_project/state/host0            \
		xyz.openbmc_project.State.Host              \
		RequestedHostTransition                     \
		s                                           \
		xyz.openbmc_project.State.Host.Transition.On
}

function setPowerCycle()
{
    busctl set-property                                    \
                xyz.openbmc_project.State.Host             \
    		/xyz/openbmc_project/state/chassis0        \
		xyz.openbmc_project.State.Chassis          \
		RequestedPowerTransition                   \
		s                                          \
		xyz.openbmc_project.State.Chassis.Transition.PowerCycle
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




if [ $(is_powerOn) = 0 ] ;then
    setPowerOn
fi

while [ $(is_powerOn) = 0 ]; do
	sleep 3
done

while [ $cycle_count -le $1 ]; do
	echo "power cycle test times: $cycle_count"
	setPowerCycle
	while [ $(is_powerOn) = 0 ]; do
		echo "power off"
		sleep 5
	done
	echo "power on"
	sleep 2
	((cycle_count++))
done