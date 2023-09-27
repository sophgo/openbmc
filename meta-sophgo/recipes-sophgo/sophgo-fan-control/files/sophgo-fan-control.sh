#!/bin/bash


# fan-control: not support float
rate_mux=15500
pwm_mux=255
CPU0_tmp=70.0
CPU1_tmp=70.0
CPU0_tmp_original=70000
CPU1_tmp_original=70000
PWM_percent=45
pwm=102
cpu0_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a080.i2c-bus/i2c-0/0-004c/hwmon/"
cpu1_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a100.i2c-bus/i2c-1/1-004c/hwmon/"
switch0_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a180.i2c-bus/i2c-2/2-004c/hwmon/"
switch1_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a200.i2c-bus/i2c-3/3-004c/hwmon/"
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
dbus_set_method="set-property"
dbus_get_method="get-property"
power_state_on="xyz.openbmc_project.State.Host.HostState.Running"
power_state_off="xyz.openbmc_project.State.Host.HostState.Off"
dbus_name="xyz.openbmc_project.State.Host"


dbus_chassis_path="/xyz/openbmc_project/state/chassis0"
dbus_chassis_inf="xyz.openbmc_project.State.Chassis"
dbus_set_chassis_property="RequestedPowerTransition"


dbus_host_path="/xyz/openbmc_project/state/host0"
dbus_host_inf="xyz.openbmc_project.State.Host"
dbus_set_host_property="RequestedHostTransition"
dbus_get_host_property="CurrentHostState"
# force poweroff
set_force_power_off="xyz.openbmc_project.State.Chassis.Transition.Off"
property_type="s"
power_state=$power_state_on

power_off_flag=0
power_on_flag=0


PWM_SET_PER=45
PWM_PRE_PER=0
PWM_OUT_PER=0
PWM_CHANGE_STE=1

ERROR_STATE_NUM=0

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
    # power state
    power_state=$(busctl $dbus_get_method $dbus_name $dbus_host_path $dbus_host_inf $dbus_get_host_property | sed 's/\"//g')
    power_state=${power_state#*" "}

    # CPU
    if [ -d "$cpu0_i2c_dir" ]; then
        CPU0_tmp_original=$(cat ${cpu0_i2c_dir}/*/temp2_input)
    else
        echo "fan-control: error 0"
        CPU0_tmp_original=70000
        let ERROR_STATE_NUM++
    fi

    if [ -d "$cpu1_i2c_dir" ]; then
        CPU1_tmp_original=$(cat ${cpu1_i2c_dir}/*/temp2_input)
    else
       echo "fan-control: error 1"
        CPU1_tmp_original=70000
        let ERROR_STATE_NUM++
    fi
    CPU0_tmp=$(($CPU0_tmp_original/1000))
    CPU1_tmp=$(($CPU1_tmp_original/1000))

    if [  "$CPU0_tmp" -lt "$CPU1_tmp" ]; then
        max_t=$CPU1_tmp
    else
        max_t=$CPU0_tmp
    fi

	if [ $max_t -ge 65 ]; then
		PWM_CPU_PER=100
	elif [ $max_t -ge 56 ] && [ $max_t -lt 64 ]; then
		PWM_CPU_PER=75
	elif [ $max_t -ge 48 ] && [ $max_t -le 55 ]; then
		PWM_CPU_PER=55
	elif [ $max_t -lt 48 ]; then
		PWM_CPU_PER=45
	fi

    # echo "fan-control: speed cpu $PWM_CPU_PER"

    # SWITCH
    if [ -d "$switch0_i2c_dir" ]; then
        S0_tmp_original=$(cat ${switch0_i2c_dir}/*/temp2_input)
    else
        echo "fan-control: error 2"
        S0_tmp_original=85000
        let ERROR_STATE_NUM++
    fi

    if [ -d "$switch1_i2c_dir" ]; then
        S1_tmp_original=$(cat ${switch1_i2c_dir}/*/temp2_input)
    else
        echo "fan-control: error 3"
        S1_tmp_original=85000
        let ERROR_STATE_NUM++
    fi
    S0_tmp=$(($S0_tmp_original/1000))
    S1_tmp=$(($S1_tmp_original/1000))


    if [  "$S0_tmp" -lt "$S1_tmp" ]; then
        max_t=$S1_tmp
    else
        max_t=$S0_tmp
    fi

	if [ $max_t -ge 82 ];then
		PWM_S_PER=100
	elif [ $max_t -ge 77 ] && [ $max_t -lt 81 ]; then
		PWM_S_PER=75
	elif [ $max_t -ge 72 ] && [ $max_t -le 76 ]; then
		PWM_S_PER=55
	elif [ $max_t -lt 72 ] ;then
		PWM_S_PER=45
	fi

    # echo "fan-control: speed switch $PWM_S_PER"

    if [  "$PWM_CPU_PER" -lt "$PWM_S_PER" ]; then
        PWM_SET_PER=$PWM_S_PER
    else
        PWM_SET_PER=$PWM_CPU_PER
    fi


    # echo "fan-control: speed set $PWM_SET_PER"

    if [ "$power_state"x = "$power_state_on"x ] ;then
        if [ $(cat ${fan2_file}) -le 0 ] || [ $(cat ${fan4_file}) -le 0 ] || [ $(cat ${fan6_file}) -le 0 ] || [ $(cat ${fan9_file}) -le 0 ]
        then
            echo "fan-control: error 4"
            let ERROR_STATE_NUM++
        fi
    fi



    if [ $ERROR_STATE_NUM -gt 0 ]; then
        PWM_OUT_PER=100
    else
        if [ $PWM_PRE_PER -le 0 ]; then
            PWM_PRE_PER=$PWM_SET_PER;
            PWM_OUT_PER=$PWM_SET_PER;
        else
            if [ $PWM_PRE_PER -lt $PWM_SET_PER ]; then
                PWM_OUT_PER=$(($PWM_PRE_PER+$PWM_CHANGE_STE))
            elif [ $PWM_PRE_PER -gt $PWM_SET_PER ]; then
                PWM_OUT_PER=$(($PWM_PRE_PER-$PWM_CHANGE_STE))
            elif [ $PWM_PRE_PER -eq $PWM_SET_PER ]; then
                PWM_OUT_PER=$PWM_PRE_PER
            fi
        fi
    fi

    # echo "fan-control: speed $PWM_OUT_PER"
	pwm=$(($PWM_OUT_PER*$pwm_mux/100))
	write_pwm /sys/class/hwmon pwm1 $pwm

    PWM_PRE_PER=$PWM_OUT_PER
    ERROR_STATE_NUM=0

    # monitor fan state when system is poweron
    if [ "$power_state"x = "$power_state_on"x ]; then

        if [ $power_on_flag -eq 0 ]; then
            echo "fan-control: system is powerOn"
            let power_on_flag++
        fi

        fan2_value=$(cat ${fan2_file})
        if [ $fan2_value -lt $fan_value_min ]; then
            let fan_fault_count++
            echo "fan2:$fan2_value"
        fi
        fan4_value=$(cat ${fan4_file})
        if [ $fan4_value -lt $fan_value_min ]; then
            let fan_fault_count++
            echo "fan4:$fan4_value"
        fi
        fan6_value=$(cat ${fan6_file})
        if [ $fan6_value -lt $fan_value_min ]; then
            let fan_fault_count++
            echo "fan6:$fan6_value"
        fi
        fan9_value=$(cat ${fan9_file})
        if [ $fan9_value -lt $fan_value_min ]; then
            let fan_fault_count++
            echo "fan9:$fan9_value"
        fi
        # judgment
        if [ $max_t -ge $poweroff_tmp_for_fanfault ] && [ $fan_fault_count -ge 2 ]; then
            echo " fan-control:Fan fault, power down "
            echo "fan-control:fan_fault_count $fan_fault_count"
            busctl $dbus_set_method $dbus_name $dbus_chassis_path $dbus_chassis_inf $dbus_set_chassis_property $property_type $set_force_power_off
        fi
        fan_fault_count=0
        power_off_flag=0
    else
        if [ $power_off_flag -eq 0 ]; then
            echo "fan-control: system is poweroff"
            let power_off_flag++
            power_on_flag=0
        fi
    fi

	sleep 3

done


