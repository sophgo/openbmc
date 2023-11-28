#!/bin/bash

# Global string variables
cpu0_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a080.i2c-bus/i2c-0/0-004c/hwmon/"
cpu1_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a100.i2c-bus/i2c-1/1-004c/hwmon/"
switch0_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a180.i2c-bus/i2c-2/2-004c/hwmon/"
switch1_i2c_dir="/sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a200.i2c-bus/i2c-3/3-004c/hwmon/"
fan_file="/sys/devices/platform/ahb/ahb:apb/1e610000.pwm_tach/1e610000.pwm_tach:tach/hwmon"
fan2_file=${fan_file}/*/"fan9_input"
fan4_file=${fan_file}/*/"fan11_input"
fan6_file=${fan_file}/*/"fan13_input"
fan9_file=${fan_file}/*/"fan15_input"

riser_i2c_file="/usr/sbin/sophgo-riser-i2c-ctr.sh"
riser_i2c_file_exit=0
ai_card_exit=0

# Global variables for counting
power_off_flag=0
power_on_flag=0
PWM_PRE_PER=0
PWM_OUT_PER=0

# Global variables for reference
pwm_mux=255
PWM_CHANGE_STE=1
fan_value_min=3000
print_flag=0


# Function Definition
function get_cpu0_temp(){
    local l_value=0
    if [ -d "$cpu0_i2c_dir" ]; then
        l_value=$(cat ${cpu0_i2c_dir}/*/temp2_input)
    else
        l_valuel_value=70000
    fi
    local l_CPU0_temp=$(($l_value/1000))
    echo "$l_CPU0_temp"
}

function get_cpu1_temp(){
    local l_value=0
    if [ -d "$cpu1_i2c_dir" ]; then
        l_value=$(cat ${cpu1_i2c_dir}/*/temp2_input)
    else
        l_value=70000
    fi
    local l_CPU1_temp=$(($l_value/1000))
    echo "$l_CPU1_temp"
}

function get_switch0_temp(){
    local l_value=0
    if [ -d "$switch0_i2c_dir" ]; then
        l_value=$(cat ${switch0_i2c_dir}/*/temp2_input)
    else
        l_value=85000
    fi
    local l_switch0_temp=$(($l_value/1000))
    echo "$l_switch0_temp"
}

function get_switch1_temp(){
    local l_value=0
    if [ -d "$switch1_i2c_dir" ]; then
        l_value=$(cat ${switch1_i2c_dir}/*/temp2_input)
    else
        l_value=85000
    fi
    local l_switch1_tmp=$(($l_value/1000))
    echo "$l_switch1_tmp"
}


function get_nvme1_temp(){
    local l_nvme0_exit_state=$(i2cdetect -y 6 | grep 6a | wc -l)
    local l_value=0
    if [ $l_nvme0_exit_state = 1 ] ;then
        l_value=$(busctl get-property xyz.openbmc_project.NVMeSensor \
                         /xyz/openbmc_project/sensors/temperature/NVMe_1_Temp \
                         xyz.openbmc_project.Sensor.Value Value)
        if [ $? = 0 ] ;then
            l_value=${l_value#*" "}
        fi
    fi
    echo "$l_value"
}

function get_nvme2_temp(){
    local l_nvme1_exit_state=$(i2cdetect -y 11 | grep 6a | wc -l)
    local l_value=0
    if [ $l_nvme1_exit_state = 1 ] ;then
        l_value=$(busctl get-property xyz.openbmc_project.NVMeSensor \
                         /xyz/openbmc_project/sensors/temperature/NVMe_2_Temp \
                         xyz.openbmc_project.Sensor.Value Value)
        if [ $? = 0 ] ;then
            l_value=${l_value#*" "}
        fi
    fi
    echo "$l_value"
}

function get_max_value(){
    local l_max_value=0
    if [ $# -ge 1 ] ;then
        for arg in "$@"
        do
            if [ $arg -gt $l_max_value ] ;then
                l_max_value=$arg
            fi
        done
    fi
    echo "$l_max_value"
}

#  No heat sink installed
function calculate_aicard_pwm_per(){
    local l_PWM_PER=45
    if [ $1 -ge 66 ]; then
		l_PWM_PER=100
	elif [ $1 -ge 56 ] && [ $1 -le 65 ]; then
		l_PWM_PER=75
	elif [ $1 -ge 46 ] && [ $1 -le 55 ]; then
		l_PWM_PER=55
	elif [ $1 -lt 46 ]; then
		l_PWM_PER=45
	fi
    echo "$l_PWM_PER"
}
#  No heat sink installed
function calculate_nvme_pwm_per(){
    local l_PWM_PER=45
    if [ $1 -ge 71 ]; then
		l_PWM_PER=100
	elif [ $1 -ge 61 ] && [ $1 -le 70 ]; then
		l_PWM_PER=75
	elif [ $1 -ge 56 ] && [ $1 -le 60 ]; then
		l_PWM_PER=55
	elif [ $1 -lt 56 ]; then
		l_PWM_PER=45
	fi
    echo "$l_PWM_PER"
}

function calculate_cpu_pwm_per(){
    local l_PWM_PER=45
    if [ $1 -ge 65 ]; then
		l_PWM_PER=100
	elif [ $1 -ge 56 ] && [ $1 -le 64 ]; then
		l_PWM_PER=75
	elif [ $1 -ge 48 ] && [ $1 -le 55 ]; then
		l_PWM_PER=55
	elif [ $1 -lt 48 ]; then
		l_PWM_PER=45
	fi
    echo "$l_PWM_PER"
}

function calculate_switch_PWM_per(){
    local l_PWM_PER=45
    if [ $1 -ge 82 ]; then
		l_PWM_PER=100
	elif [ $1 -ge 77 ] && [ $1 -le 81 ]; then
		l_PWM_PER=75
	elif [ $1 -ge 72 ] && [ $1 -le 76 ]; then
		l_PWM_PER=55
	elif [ $1 -lt 72 ]; then
		l_PWM_PER=45
	fi
    echo "$l_PWM_PER"
}


function get_fan_state(){
    local l_error_num=0
    if [ $(cat ${fan2_file}) -le $fan_value_min ]
    then
        let l_error_num++
    fi
    if [ $(cat ${fan4_file}) -le $fan_value_min ]
    then
        let l_error_num++
    fi
    if [ $(cat ${fan6_file}) -le $fan_value_min ]
    then
        let l_error_num++
    fi
    if [ $(cat ${fan9_file}) -le $fan_value_min ]
    then
        let l_error_num++
    fi
    echo "$l_error_num"
}

is_power_on() {
    local l_power_state=$(busctl get-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 \
            xyz.openbmc_project.State.Host CurrentHostState | sed 's/\"//g')
    l_power_state=${l_power_state#*" "}
    if [ "$l_power_state"x = "xyz.openbmc_project.State.Host.HostState.Running"x ] ;then
        echo "1"
    else
        echo "0"
    fi
}

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



function force_poweroff(){
    busctl set-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/chassis0 \
                 xyz.openbmc_project.State.Chassis RequestedPowerTransition s \
                 xyz.openbmc_project.State.Chassis.Transition.Off
}

source $riser_i2c_file

if [ $? -eq 0 ] ;then
    riser_i2c_file_exit=1
else
    riser_i2c_file_exit=0
fi

while true;
do
    # system is poweron
    if [ $(is_power_on) = 1 ] ;then

        if [ $power_on_flag -eq 0 ]; then
            echo "fan-control: system is powerOn"
            let power_on_flag++
        fi

        #CPU
        CPU0_tmp=$(get_cpu0_temp)
        CPU1_tmp=$(get_cpu1_temp)
        CPU_tmp=$(get_max_value $CPU0_tmp $CPU1_tmp)
        PWM_C_PER=$(calculate_cpu_pwm_per $CPU_tmp)


        # SWITCH
        switch0_tmp=$(get_switch0_temp)
        switch1_tmp=$(get_switch1_temp)
        switch_tmp=$(get_max_value $switch0_tmp $switch1_tmp)
        PWM_S_PER=$(calculate_switch_PWM_per $switch_tmp)

        # NVMe
        NVMe1_tmp=$(get_nvme1_temp)
        NVMe2_tmp=$(get_nvme2_temp)
        NVMe_tmp=$(get_max_value $NVMe1_tmp $NVMe2_tmp)
        PWM_N_PER=$(calculate_nvme_pwm_per $NVMe_tmp)

        # AI Card
        if [ $riser_i2c_file_exit ] ;then
            if [ $(is_1684_card_exist_on_all_risercard) ] ;then
                AI_card_temp=$(riser_get_max_temp_of_aicard)
                PWM_AI_PER=$(calculate_aicard_pwm_per $AI_card_temp)
            fi
        else
            AI_card_temp=0
            PWM_AI_PER=0
        fi

        # PWM_SET_PER
        PWM_SET_PER=$(get_max_value $PWM_C_PER $PWM_S_PER $PWM_N_PER $PWM_AI_PER)


        # pwm decision
        if [ $(get_fan_state) -ge 1 ] ;then
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


        # monitor fan state and cpu temp
        if [ $CPU_tmp -ge 50 ] && [ $(get_fan_state) -ge 2 ]; then
            echo " fan-control:Fan fault, power down "
            force_poweroff
        fi
        power_off_flag=0
    # when system is poweroff
    else
        if [ $power_off_flag -eq 0 ]; then
            echo "fan-control: system is poweroff"
            let power_off_flag++
            power_on_flag=0
        fi

        PWM_OUT_PER=45
    fi


    pwm_value=$(($PWM_OUT_PER*$pwm_mux/100))
	write_pwm /sys/class/hwmon pwm1 $pwm_value


    if [ $1 ] || [ $print_flag -eq 1 ] ;then
        echo "fan-control: CPU0_tmp          $CPU0_tmp"
        echo "fan-control: CPU1_tmp          $CPU1_tmp"
        echo "fan-control: CPU_tmp           $CPU_tmp"
        echo "fan-control: PWM_C_PER         $PWM_C_PER"
        echo "fan-control: switch0_tmp       $switch0_tmp"
        echo "fan-control: switch1_tmp       $switch1_tmp"
        echo "fan-control: switch_tmp        $switch_tmp"
        echo "fan-control: PWM_S_PER         $PWM_S_PER"
        echo "fan-control: NVMe1_tmp         $NVMe1_tmp"
        echo "fan-control: NVMe2_tmp         $NVMe2_tmp"
        echo "fan-control: NVMe_tmp          $NVMe_tmp"
        echo "fan-control: PWM_N_PER         $PWM_N_PER"
        echo "fan-control: Aicard_tmp        $AI_card_temp"
        echo "fan-control: PWM_AI_PER        $PWM_AI_PER"
        echo "fan-control: PWM_SET_PER       $PWM_SET_PER"
        echo "fan-control: PWM_PRE_PER       $PWM_PRE_PER"
        echo "fan-control: PWM_OUT_PER       $PWM_OUT_PER"
        echo "fan-control: pwm_value         $pwm_value"
    fi

    # Global variable update
    PWM_PRE_PER=$PWM_OUT_PER

	sleep 3

done


