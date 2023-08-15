fan_dbus_name="xyz.openbmc_project.FanSensor"
fan_bus_path[0]="/xyz/openbmc_project/sensors/fan_tach/Fan2"
fan_bus_path[1]="/xyz/openbmc_project/sensors/fan_tach/Fan4"
fan_bus_path[2]="/xyz/openbmc_project/sensors/fan_tach/Fan6"
fan_bus_path[3]="/xyz/openbmc_project/sensors/fan_tach/Fan9"


psu_dbus_name="xyz.openbmc_project.PSUSensor"

psu1_hwmon_path="/sys/bus/i2c/devices/4-0058/hwmon"
psu1_bus_path[0]="/xyz/openbmc_project/sensors/current/PSU1_Output_Current"
psu1_bus_path[1]="/xyz/openbmc_project/sensors/power/PSU1_Input_Power"
psu1_bus_path[2]="/xyz/openbmc_project/sensors/temperature/PSU1_Temperature"
# psu1_bus_path[0]="/xyz/openbmc_project/sensors/power/PSU1_Output_Power"
# psu1_bus_path[0]="/xyz/openbmc_project/sensors/fan_pwm/Pwm_PSU1_Fan_1"
# psu1_bus_path[0]="/xyz/openbmc_project/sensors/fan_tach/PSU1_Fan_Speed_1"
# psu1_bus_path[0]="/xyz/openbmc_project/sensors/current/PSU1_Input_Current"
# psu1_bus_path[0]="/xyz/openbmc_project/sensors/voltage/PSU1_Input_Voltage"
# psu1_bus_path[0]="/xyz/openbmc_project/sensors/voltage/PSU1_Output_Voltage"

psu2_hwmon_path="/sys/bus/i2c/devices/4-0059/hwmon"
psu2_bus_path[0]="/xyz/openbmc_project/sensors/current/PSU2_Output_Current"
psu2_bus_path[1]="/xyz/openbmc_project/sensors/power/PSU2_Input_Power"
psu2_bus_path[2]="/xyz/openbmc_project/sensors/temperature/PSU2_Temperature"
# psu2_bus_path[0]="/xyz/openbmc_project/sensors/power/PSU2_Output_Power"
# psu2_bus_path[0]="/xyz/openbmc_project/sensors/fan_pwm/Pwm_PSU2_Fan_1"
# psu2_bus_path[0]="/xyz/openbmc_project/sensors/fan_tach/PSU2_Fan_Speed_1"
# psu2_bus_path[0]="/xyz/openbmc_project/sensors/current/PSU2_Input_Current"
# psu2_bus_path[0]="/xyz/openbmc_project/sensors/voltage/PSU2_Input_Voltage"
# psu2_bus_path[0]="/xyz/openbmc_project/sensors/voltage/PSU2_Output_Voltage"




tmp_dbus_name="xyz.openbmc_project.HwmonTempSensor"
tmp_bus_path[0]="/xyz/openbmc_project/sensors/temperature/LOCAL_TEMP1"
tmp_bus_path[1]="/xyz/openbmc_project/sensors/temperature/LOCAL_TEMP2"
tmp_bus_path[2]="/xyz/openbmc_project/sensors/temperature/LOCAL_TEMP3"
tmp_bus_path[3]="/xyz/openbmc_project/sensors/temperature/LOCAL_TEMP4"
tmp_bus_path[4]="/xyz/openbmc_project/sensors/temperature/LOCAL_TEMP5"
tmp_bus_path[5]="/xyz/openbmc_project/sensors/temperature/TEMP_CPU0_INNER"
tmp_bus_path[6]="/xyz/openbmc_project/sensors/temperature/TEMP_CPU0_OUTER"
tmp_bus_path[7]="/xyz/openbmc_project/sensors/temperature/TEMP_CPU1_INNER"
tmp_bus_path[8]="/xyz/openbmc_project/sensors/temperature/TEMP_CPU1_OUTER"



sensors_warning_inf="xyz.openbmc_project.Sensor.Threshold.Warning"
sensors_critical_inf="xyz.openbmc_project.Sensor.Threshold.Critical"
sensors_WarningAlarmLow="WarningAlarmLow"
sensors_WarningAlarmHigh="WarningAlarmHigh"
sensors_CriticalAlarmLow="CriticalAlarmLow"
sensors_CriticalAlarmHigh="CriticalAlarmHigh"

result_critical=0
result_warning=0

frontpanel_warning_led_trigger=/sys/class/leds/frontpanel_warning_led/trigger
frontpanel_warning_led_brightness=/sys/class/leds/frontpanel_warning_led/brightness


power_state_on="xyz.openbmc_project.State.Host.HostState.Running"

sleep 30

while true;
do
    power_state=$(busctl get-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 \
            xyz.openbmc_project.State.Host CurrentHostState | sed 's/\"//g')
    power_state=${power_state#*" "}
    # echo $power_state
    if [ "$power_state"x = "$power_state_on"x ] ;then

        for var in ${fan_bus_path[@]};
        do
            result_WarningAlarmLow=$(busctl get-property $fan_dbus_name $var $sensors_warning_inf $sensors_WarningAlarmLow)
            if [ $? = 0 ] ;then
                result_WarningAlarmLow=${result_WarningAlarmLow#*" "}
                if [ "$result_WarningAlarmLow"x = "true"x ] ;then
                    let result_warning++
                fi
            fi
            result_WarningAlarmHigh=$(busctl get-property $fan_dbus_name $var $sensors_warning_inf $sensors_WarningAlarmHigh)
            if [ $? = 0 ] ;then
                result_WarningAlarmHigh=${result_WarningAlarmHigh#*" "}
                if [ "$result_WarningAlarmHigh"x = "true"x ] ;then
                    let result_warning++
                fi
            fi
            result_CriticalAlarmLow=$(busctl get-property $fan_dbus_name $var $sensors_critical_inf $sensors_CriticalAlarmLow)
            if [ $? = 0 ] ;then
                result_CriticalAlarmLow=${result_CriticalAlarmLow#*" "}
                if [ "$result_CriticalAlarmLow"x = "true"x ] ;then
                    let result_critical++
                fi
            fi
            result_CriticalAlarmHigh=$(busctl get-property $fan_dbus_name $var $sensors_critical_inf $sensors_CriticalAlarmHigh)
            if [ $? = 0 ] ;then
                result_CriticalAlarmHigh=${result_CriticalAlarmHigh#*" "}
                if [ "$result_CriticalAlarmHigh"x = "true"x ] ;then
                    let result_critical++
                fi

            fi
        done
    fi
    # 4-0058
    if [ -d "$psu1_hwmon_path" ] ;then
        for var in ${psu1_bus_path[@]};
        do
            result_WarningAlarmHigh=$(busctl get-property $psu_dbus_name $var $sensors_warning_inf $sensors_WarningAlarmHigh)
            if [ $? = 0 ] ;then
                result_WarningAlarmHigh=${result_WarningAlarmHigh#*" "}
                if [ "$result_WarningAlarmHigh"x = "true"x ] ;then
                    let result_warning++
                fi
            fi
            result_CriticalAlarmHigh=$(busctl get-property $psu_dbus_name $var $sensors_critical_inf $sensors_CriticalAlarmHigh)
            if [ $? = 0 ] ;then
                result_CriticalAlarmHigh=${result_CriticalAlarmHigh#*" "}
                if [ "$result_CriticalAlarmHigh"x = "true"x ] ;then
                    let result_critical++
                fi
            fi
        done
    fi
    # 4-0059
    if [ -d "$psu2_hwmon_path" ] ;then
        for var in ${psu2_bus_path[@]};
        do
            result_WarningAlarmHigh=$(busctl get-property $psu_dbus_name $var $sensors_warning_inf $sensors_WarningAlarmHigh)
            if [ $? = 0 ] ;then
                result_WarningAlarmHigh=${result_WarningAlarmHigh#*" "}
                if [ "$result_WarningAlarmHigh"x = "true"x ] ;then
                    let result_warning++
                fi
            fi
            result_CriticalAlarmHigh=$(busctl get-property $psu_dbus_name $var $sensors_critical_inf $sensors_CriticalAlarmHigh)
            if [ $? = 0 ] ;then
                result_CriticalAlarmHigh=${result_CriticalAlarmHigh#*" "}
                if [ "$result_CriticalAlarmHigh"x = "true"x ] ;then
                    let result_critical++
                fi
            fi
        done
    fi

    for var in ${tmp_bus_path[@]};
    do
        result_WarningAlarmLow=$(busctl get-property $tmp_dbus_name $var $sensors_warning_inf $sensors_WarningAlarmLow)
        if [ $? = 0 ] ;then
            result_WarningAlarmLow=${result_WarningAlarmLow#*" "}
            if [ "$result_WarningAlarmLow"x = "true"x ] ;then
                let result_warning++
            fi
        fi
        result_WarningAlarmHigh=$(busctl get-property $tmp_dbus_name $var $sensors_warning_inf $sensors_WarningAlarmHigh)
        if [ $? = 0 ] ;then
            result_WarningAlarmHigh=${result_WarningAlarmHigh#*" "}
            if [ "$result_WarningAlarmHigh"x = "true"x ] ;then
                let result_warning++
            fi
        fi
        result_CriticalAlarmLow=$(busctl get-property $tmp_dbus_name $var $sensors_critical_inf $sensors_CriticalAlarmLow)
        if [ $? = 0 ] ;then
            result_CriticalAlarmLow=${result_CriticalAlarmLow#*" "}
            if [ "$result_CriticalAlarmLow"x = "true"x ] ;then
                let result_critical++
            fi
        fi
        result_CriticalAlarmHigh=$(busctl get-property $tmp_dbus_name $var $sensors_critical_inf $sensors_CriticalAlarmHigh)
        if [ $? = 0 ] ;then
            result_CriticalAlarmHigh=${result_CriticalAlarmHigh#*" "}
            if [ "$result_CriticalAlarmHigh"x = "true"x ] ;then
                let result_critical++
            fi
        fi

    done

    if [ $result_critical -ge 1 ] || [ $result_warning -ge 1 ];then
        echo "cirtical : $result_critical , warning: $result_warning"
        echo "timer" > $frontpanel_warning_led_trigger
    else
        echo "none" > $frontpanel_warning_led_trigger
        echo 0 > $frontpanel_warning_led_brightness
    fi

    sleep 5
    result_critical=0
done


