#!/bin/bash

riser_i2c_file="/usr/sbin/sophgo-riser-i2c-function.sh"
riser_i2c_file_exit=0
ai_card_exit=0
no_ai_card_value=-1
option_end_flag="--"

dbus_name="xyz.openbmc_project.FanControl"
dbus_path="/xyz/openbmc_project/sensors/temperature/AiCard_Temp"
dbus_intf="xyz.openbmc_project.Sensor.Value"
dbus_property_tmp="Value"
property_type_tmp="d"
dbus_property_isexist="sg1684xExistState"
property_type_isexist="i"


source $riser_i2c_file
if [ $? -eq 0 ] ;then
    while true;
    do
        result=$(is_1684_card_exist_on_all_risercard)
        # echo "result=${result}"
        if [ $result -eq 1 ] ;then
            AI_card_temp=$(riser_get_max_temp_of_aicard)
            # echo ${AI_card_temp}
            busctl set-property ${dbus_name} ${dbus_path} ${dbus_intf} ${dbus_property_tmp} ${property_type_tmp} ${AI_card_temp}
            busctl set-property ${dbus_name} ${dbus_path} ${dbus_intf} ${dbus_property_isexist} ${property_type_isexist} 1
        else
            busctl set-property ${dbus_name} ${dbus_path} ${dbus_intf} ${dbus_property_tmp} ${property_type_tmp} ${option_end_flag} ${no_ai_card_value}
            busctl set-property ${dbus_name} ${dbus_path} ${dbus_intf} ${dbus_property_isexist} ${property_type_isexist} 0
        fi
        # echo "riser scan"
        sleep 3
    done
else
    echo "source /usr/sbin/sophgo-riser-i2c-function.sh failed."
    exit 1
fi

