#!/bin/bash

dbus_name="xyz.openbmc_project.State.CPLD"
hardVersionPath="/xyz/openbmc_project/state/cpld/hardVersion"
hardVersionIface="xyz.openbmc_project.State.Cpld.Version"

softVersionPath="/xyz/openbmc_project/state/cpld/softVersion"
softVersionIface="xyz.openbmc_project.State.Cpld.Version"
versionProperty="VERSION"

cpuAPowerStatePath="/xyz/openbmc_project/state/cpld/cpuAPowerState"
cpuAPowerStateIface="xyz.openbmc_project.State.Cpld.CpuAPowerState"
cpuAPWStateArry[0]="PG_VPP_A1"
cpuAPWStateArry[1]="PG_VPP_A0"
cpuAPWStateArry[2]="PG_VDDQ_A1"
cpuAPWStateArry[3]="PG_VDDQ_A0"
cpuAPWStateArry[4]="PG_VTT_A1"
cpuAPWStateArry[5]="PG_VTT_A0"
cpuAPWStateArry[6]="PG_PCIE_PHY_A"
cpuAPWStateArry[7]="PG_VDDC_A"

cpuBPowerStatePath="/xyz/openbmc_project/state/cpld/cpuBPowerState"
cpuBPowerStateIface="xyz.openbmc_project.State.Cpld.CpuBPowerState"
cpuBPWStateArry[0]="PG_VPP_B1"
cpuBPWStateArry[1]="PG_VPP_B0"
cpuBPWStateArry[2]="PG_VDDQ_B1"
cpuBPWStateArry[3]="PG_VDDQ_B0"
cpuBPWStateArry[4]="PG_VTT_B1"
cpuBPWStateArry[5]="PG_VTT_B0"
cpuBPWStateArry[6]="PG_PCIE_PHY_B"
cpuBPWStateArry[7]="PG_VDDC_B"



otherPowerStatePath="/xyz/openbmc_project/state/cpld/otherPowerState"
otherPowerStateIface="xyz.openbmc_project.State.Cpld.OtherPowerState"
otherPWStateArry[0]="PG_VDD_3V3"
otherPWStateArry[1]="PG_VDD_1V8"
otherPWStateArry[2]="PG_PCIE_H_1V8_B"
otherPWStateArry[3]="PG_DDR_PHY_B"
otherPWStateArry[4]="PG_PCIE_H_1V8_A"
otherPWStateArry[5]="PG_DDR_PHY_A"
otherPWStateArry[6]="cpua_pwrok"
otherPWStateArry[7]="cpub_pwrok"

psuStatePath="/xyz/openbmc_project/state/cpld/psuState"
psuStateIface="xyz.openbmc_project.State.Cpld.PsuState"
psuStateArry[0]="PSU1_PRSNT_N"
psuStateArry[1]="PSU0_PRSNT_N"
psuStateArry[2]="PSU1_ALERT_N"
psuStateArry[3]="PSU0_ALERT_N"
psuStateArry[4]="PSU1_PWROK"
psuStateArry[5]="PSU0_PWROK"

efuseStatePath="/xyz/openbmc_project/state/cpld/efuseState"
efuseStateIface="xyz.openbmc_project.State.Cpld.EfuseState"
efuseStateArry[0]="NCP0_GOK"
efuseStateArry[1]="NCP0_D_OC"
efuseStateArry[2]="NCP1_GOK"
efuseStateArry[3]="NCP1_D_OC"


value=$(busctl get-property $dbus_name $softVersionPath $softVersionIface $versionProperty | sed 's/\"//g')
value=${value#*" "}
echo -e "\nCPLD Version: $value"

echo -e "\nCPUA Power State:"
for var in ${cpuAPWStateArry[@]};
do
    value=$(busctl get-property $dbus_name $cpuAPowerStatePath $cpuAPowerStateIface $var | sed 's/\"//g')
    value=${value#*" "}
    echo -e "$var\t$value"
done

echo -e "\nCPUB Power State:"
for var in ${cpuBPWStateArry[@]};
do
    value=$(busctl get-property $dbus_name $cpuBPowerStatePath $cpuBPowerStateIface $var | sed 's/\"//g')
    value=${value#*" "}
    echo -e "$var\t$value"
done

echo -e "\nOther Power State:"
for var in ${otherPWStateArry[@]};
do
    value=$(busctl get-property $dbus_name $otherPowerStatePath $otherPowerStateIface $var | sed 's/\"//g')
    value=${value#*" "}
    echo -e "$var\t$value"
done

echo -e "\nPSU State:"
for var in ${psuStateArry[@]};
do
    value=$(busctl get-property $dbus_name $psuStatePath $psuStateIface $var | sed 's/\"//g')
    value=${value#*" "}
    echo -e "$var\t$value"
done

echo -e "\nEFUSE State:"
for var in ${efuseStateArry[@]};
do
    value=$(busctl get-property $dbus_name $efuseStatePath $efuseStateIface $var | sed 's/\"//g')
    value=${value#*" "}
    echo -e "$var\t$value"
done