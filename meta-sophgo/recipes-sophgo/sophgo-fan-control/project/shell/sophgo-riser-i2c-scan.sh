#!/bin/bash

riser_i2c_file="/usr/sbin/sophgo-riser-i2c-function.sh"
riser_i2c_file_exit=0
ai_card_exit=0

dbus_name="xyz.openbmc_project.FanControl"
dbus_path="/xyz/openbmc_project/sensors/temperature/AiCard_Temp"
dbus_intf="xyz.openbmc_project.Sensor.Value"
dbus_property="Value"
property_type="d"


source $riser_i2c_file
if [ $? -eq 0 ] ;then
    while true;
    do
        result=$(is_1684_card_exist_on_all_risercard)
        # echo "result=${result}"
        if [ $result -eq 1 ] ;then
            AI_card_temp=$(riser_get_max_temp_of_aicard)
            # echo ${AI_card_temp}
            busctl set-property ${dbus_name} ${dbus_path} ${dbus_intf} ${dbus_property} ${property_type} ${AI_card_temp}
        fi
        # echo "riser scan"
        sleep 3
    done
else
    echo "source /usr/sbin/sophgo-riser-i2c-function.sh failed."
    exit 1
fi

