#!/bin/bash


# fan-control: not support float
rate_mux=15500
pwm_mux=255
CPU0_tmp=40.0
CPU1_tmp=40.0
max_t=40.0
CPU0_tmp_original=4000
CPU1_tmp_original=4000
max_t_original=4000
PWM_percent=40
pwm=102
cpu0_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a080.i2c-bus/i2c-0/0-004c/hwmon/"
cpu1_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a100.i2c-bus/i2c-1/1-004c/hwmon/"
# start from 1
fan_file="/sys/devices/platform/ahb/ahb:apb/1e610000.pwm_tach/1e610000.pwm_tach:tach/hwmon"
fan2_file=${fan_file}/*/"fan9_input"
fan4_file=${fan_file}/*/"fan11_input"
fan6_file=${fan_file}/*/"fan13_input"
fan9_file=${fan_file}/*/"fan15_input"
fan2_value=0
fan4_value=0
fan6_value=0
fan9_value=0
fan_fault_count=0
fan_value_min=3000
poweroff_tmp_for_fanfault=50
power_state_on="xyz.openbmc_project.State.Host.HostState.Running"
power_state_off="xyz.openbmc_project.State.Host.HostState.Off"
dbus_name="xyz.openbmc_project.State.Host"
dbus_path="/xyz/openbmc_project/state/host0"
dbus_inf="xyz.openbmc_project.State.Host"
dbus_get_property="CurrentHostState"
dbus_get_method="get-property"

dbus_set_property="RequestedHostTransition"
dbus_set_method="set-property"
# force poweroff
set_force_power_off="xyz.openbmc_project.State.Chassis.Transition.Off"
set_graceful_power_off="xyz.openbmc_project.State.Host.Transition.Off"
property_type="s"
power_state=$power_state_on
function write_pwm(){
    for file in `ls $1`
    do
        if [ -d $1"/"$file ]
        then
            cd $1"/"$file
            if [ -e $2 ]
            then
            echo "$3" > $2
            fi
        fi
    done
}


while true;
do
    if [ -d "$cpu0_i2c_dir" ]; then
        CPU0_tmp_original=$(cat ${cpu0_i2c_dir}/*/temp2_input)
    else
        echo "can not get cpu0 tmp"
        CPU0_tmp_original=40000
    fi
    echo $CPU0_tmp_original
    if [ -d "$cpu1_i2c_dir" ]; then
        CPU1_tmp_original=$(cat ${cpu1_i2c_dir}/*/temp2_input)
    else
        CPU1_tmp_original=40000
    fi
    echo $CPU1_tmp_original
    # CPU0_tmp=$(echo "scale=2; $CPU0_tmp_original/1000"|bc)
    CPU0_tmp=$(($CPU0_tmp_original/1000))
    echo $CPU0_tmp
    # CPU1_tmp=$(echo "scale=2; $CPU1_tmp_original/1000"|bc)
    CPU1_tmp=$(($CPU1_tmp_original/1000))
    echo $CPU1_tmp


    # if [ `echo "$CPU0_tmp < $CPU1_tmp"|bc` -eq 1 ] ; then
    if [  "$CPU0_tmp" -lt "$CPU1_tmp" ] ; then
        max_t=$CPU1_tmp
    else
        max_t=$CPU0_tmp
    fi
    echo $max_t

	if [ $max_t -ge 75 ];then
		PWM_proportion=100
	elif [ $max_t -ge 70 ] && [ $max_t -lt 75 ] ;then
		PWM_proportion=85
	elif [ $max_t -ge 60 ] && [ $max_t -lt 70 ] ;then
		PWM_proportion=65
	elif [ $max_t -ge 50 ] && [ $max_t -lt 60 ] ;then
		PWM_proportion=50
	elif [ $max_t -ge 40 ] && [ $max_t -lt 50 ] ;then
		PWM_proportion=40
	elif [ $max_t -lt 40 ] ;then
		PWM_proportion=40
	fi
    echo $PWM_proportion
	pwm=$(($PWM_proportion*$pwm_mux/100))
	echo $pwm
	write_pwm /sys/class/hwmon pwm1 $pwm


    # monitor fan state when system is poweron
    power_state=$(busctl $dbus_get_method $dbus_name $dbus_path $dbus_inf $dbus_get_property | sed 's/\"//g')
    power_state=${power_state#*" "}
    echo $power_state

    if [ $power_state = $power_state_on ];then

        fan2_value=$(cat ${fan2_file})
        echo "fan2:$fan2_value"
        if [ $fan2_value -lt $fan_value_min ];then
            let fan_fault_count++
        fi
        fan4_value=$(cat ${fan4_file})
        echo "fan4:$fan4_value"
        if [ $fan4_value -lt $fan_value_min ];then
            let fan_fault_count++
        fi
        fan6_value=$(cat ${fan6_file})
        echo "fan6:$fan6_value"
        if [ $fan6_value -lt $fan_value_min ];then
            let fan_fault_count++
        fi
        fan9_value=$(cat ${fan9_file})
        echo "fan9:$fan9_value"
        if [ $fan9_value -lt $fan_value_min ];then
            let fan_fault_count++
        fi
        echo "fan_fault_count:$fan_fault_count"
        # judgment
        if [ $max_t -ge $poweroff_tmp_for_fanfault ] && [ $fan_fault_count -ge 2 ];then
            echo " Fan fault, power down "
            # power off, 2 ways
            # busctl set-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host RequestedHostTransition s xyz.openbmc_project.State.Chassis.Transition.Off
            busctl $dbus_set_method $dbus_name $dbus_path $dbus_inf $dbus_set_property $property_type $set_force_power_off
            # gpioset 0 71=0
            # sleep 15
            # gpioset 0 71=1
        fi
        fan_fault_count=0
    else
        echo "system is poweroff"
    fi
	sleep 5
done

