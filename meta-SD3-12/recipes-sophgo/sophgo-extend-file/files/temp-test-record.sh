#!/bin/bash

powerDbusName="xyz.openbmc_project.PSUSensor"
power1DbusPath="/xyz/openbmc_project/sensors/power/PSU1_Output_Power"
power2DbusPath="/xyz/openbmc_project/sensors/power/PSU2_Output_Power"

fanSpeedDbusName="xyz.openbmc_project.FanSensor"
fan2SpeedDbusPath="/xyz/openbmc_project/sensors/fan_tach/Fan2"
fan4SpeedDbusPath="/xyz/openbmc_project/sensors/fan_tach/Fan4"
fan6SpeedDbusPath="/xyz/openbmc_project/sensors/fan_tach/Fan6"
fan9SpeedDbusPath="/xyz/openbmc_project/sensors/fan_tach/Fan9"



tempDbusName="xyz.openbmc_project.HwmonTempSensor"
temp0DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_CPU0_INNER"
temp1DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_CPU1_INNER"
tempS0DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_SWITCH0_INNER"
tempS1DbusPath="/xyz/openbmc_project/sensors/temperature/TEMP_SWITCH1_INNER"

tempNvmeDbusName="xyz.openbmc_project.NVMeSensor"
tempNvme0DbusPath="/xyz/openbmc_project/sensors/temperature/NVMe_1_Temp"
# tempNvme1DbusPath="/xyz/openbmc_project/sensors/temperature/nvme1"

dbusIface="xyz.openbmc_project.Sensor.Value"
dbusProperty="Value"

pwmControlDbusName="xyz.openbmc_project.FanSensor"
pwmDbusPath="/xyz/openbmc_project/control/fanpwm/Pwm_3"
pwmTargetDbusIface="xyz.openbmc_project.Control.FanPwm"
pwmProperty="Target"


while true;
do
    #  power
    power1=$(busctl get-property $powerDbusName $power1DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    power1=${power1#*" "}

    power2=$(busctl get-property $powerDbusName $power2DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    power2=${power2#*" "}

    #  temp cpu
    temp0=$(busctl get-property $tempDbusName $temp0DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    temp0=${temp0#*" "}
    temp1=$(busctl get-property $tempDbusName $temp1DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    temp1=${temp1#*" "}

    # temp switch
    tempS0=$(busctl get-property $tempDbusName $tempS0DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    tempS0=${tempS0#*" "}
    tempS1=$(busctl get-property $tempDbusName $tempS1DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    tempS1=${tempS1#*" "}

    # temp nvme
    tempN0=$(busctl get-property $tempNvmeDbusName $tempNvme0DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    tempN0=${tempN0#*" "}
    # tempN1=$(busctl get-property $tempNvmeDbusName $tempNvme1DbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    # tempN1=${tempN1#*" "}

    # fan tach
    fan2=$(busctl get-property $fanSpeedDbusName $fan2SpeedDbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    fan2=${fan2#*" "}
    fan4=$(busctl get-property $fanSpeedDbusName $fan4SpeedDbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    fan4=${fan4#*" "}
    fan6=$(busctl get-property $fanSpeedDbusName $fan6SpeedDbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    fan6=${fan6#*" "}
    fan9=$(busctl get-property $fanSpeedDbusName $fan9SpeedDbusPath $dbusIface $dbusProperty | sed 's/\"//g')
    fan9=${fan9#*" "}

    # fan pwm
    pwm=$(busctl get-property $pwmControlDbusName $pwmDbusPath $pwmTargetDbusIface $pwmProperty | sed 's/\"//g')
    pwm=${pwm#*" "}

    echo "$power1 $power2 $temp0 $temp1 $tempS0 $tempS1 $tempN0 $fan2 $fan4 $fan6 $fan9 $pwm" >> /tmp/temp-test-record.txt

    sleep 5
done