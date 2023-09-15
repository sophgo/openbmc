#!/bin/bash

powerDbusName="xyz.openbmc_project.PSUSensor"
powerDbusPath="/xyz/openbmc_project/sensors/power/PSU1_Output_Power"

fanSpeedDbusName="xyz.openbmc_project.FanSensor"
fanSpeedDbusPath="/xyz/openbmc_project/sensors/fan_tach/Fan2"

tempDbusName="xyz.openbmc_project.HwmonTempSensor"
temp0DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_CPU0_INNER"
temp1DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_CPU1_INNER"
tempS0DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_SWITCH0_INNER"
tempS1DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_SWITCH1_INNER"

dbusIface="xyz.openbmc_project.Sensor.Value"
dbusProperty="Value"


while true;
do
    power=$(busctl get-property $powerDbusName $powerDbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    power=${power#*" "}
    # power=$(echo "$power" | sed "s/\\..*$//")

    temp0=$(busctl get-property $tempDbusName $temp0DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    temp0=${temp0#*" "}
    # temp0=$(echo "$temp0" | sed "s/\\..*$//")

    temp1=$(busctl get-property $tempDbusName $temp1DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    temp1=${temp1#*" "}
    # temp1=$(echo "$temp1" | sed "s/\\..*$//")

    tempS0=$(busctl get-property $tempDbusName $tempS0DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    tempS0=${tempS0#*" "}
    # tempS0=$(echo "$tempS0" | sed "s/\\..*$//")

    tempS1=$(busctl get-property $tempDbusName $tempS1DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    tempS1=${tempS1#*" "}
    # tempS1=$(echo "$tempS1" | sed "s/\\..*$//")

    fan=$(busctl get-property $fanSpeedDbusName $fanSpeedDbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    fan=${fan#*" "}

    echo "$power $temp0 $temp1 $tempS0 $tempS1 $fan" >> /tmp/temp-test-record.txt

    sleep 5
done