#!/bin/bash

function show_func_desc()
{
	echo "resetCpld			-reset cpld"
	echo "probeJtag			-modprobe jtag driver"
	echo "biosFlashSwitchToBmc      -switch bios flash spi interface to bmc"
	echo "biosFlashSwitchToHost     -switch bios flash spi interface to host"
	echo "biosFlashMtdCreat         -create bios mtd"
	echo "disable_wdt               -disable wdt function"
	echo "enable_wdt                -enable wdt function"
	echo "is_wdtEnabled             -returns the enable status of wdt"
	echo "is_powerOn                -returns power status"
	echo "setPowerOn                -power on host"
	echo "setPowerCycle             -power cycle"
	echo "setPowerWarmReboot        -power warm reboot"
	echo "setPowerForceReboot       -power force reboot"
	echo "setPowerForceOff          -power force off"
	echo "setPowerWarmOff           -power warm off"
	echo "showSystemdServices       -Display a list of all systemd services"
	echo "restartGpioService        -restart gpio control service"
	echo "restartFanControlService  -restart fan control service"
	echo "restartEntityService      -restart entity-manager service"
	echo "getBootSource             -return boot source value"
	echo "setBootPxeSource          -set pxe boot"
	echo "setBootHddSource          -set hdd boot"
	echo "setBootDefaultSource      -set default boot"
}

# cpld
function resetCpld()
{
	gpioset 0 31=1
	sleep 1
	gpioset 0 31=0
	usleep 30000
	gpioset 0 31=1
}
function probeJtag()
{
	modprobe jtag-aspeed-internal
}
# bios flash
function biosFlashSwitchToBmc()
{
	gpioset 0 56=0
	gpioset 0 57=0
	gpioset 0 58=0
	gpioset 0 59=0

}
function biosFlashSwitchToHost()
{
	gpioset 0 56=1
	gpioset 0 57=1
	gpioset 0 58=1
	gpioset 0 59=1
}
function biosFlashMtdCreat()
{
	biosFlashSwitchToBmc
	echo "Unbind the ASpeed spi1 driver"
	echo 1e630000.spi > /sys/bus/platform/drivers/ASPEED_FMC_SPI/unbind
	sleep 2
	echo "--- Bind the ASpeed spi1 driver"
	echo 1e630000.spi > /sys/bus/platform/drivers/ASPEED_FMC_SPI/bind
	sleep 2
}
# WDT
function disable_wdt()
{
	busctl set-property \
	xyz.openbmc_project.Gpio \
	/xyz/openbmc_project/gpio/wdtEnable \
	xyz.openbmc_project.Gpio.wdtEnable \
	wdtEnableState \
	b \
	false
}
function enable_wdt()
{
	busctl set-property \
	xyz.openbmc_project.Gpio \
	/xyz/openbmc_project/gpio/wdtEnable \
	xyz.openbmc_project.Gpio.wdtEnable \
	wdtEnableState \
	b \
	true
}

function is_wdtEnabled()
{
	busctl get-property \
	xyz.openbmc_project.Gpio \
	/xyz/openbmc_project/gpio/wdtEnable \
	xyz.openbmc_project.Gpio.wdtEnable \
	wdtEnableState
}

# power
function is_powerOn()
{
	local l_output=$(busctl get-property xyz.openbmc_project.State.Host /xyz/openbmc_project/state/host0 xyz.openbmc_project.State.Host CurrentHostState)
	if [[ $l_output == *"xyz.openbmc_project.State.Host.HostState.Running"* ]]; then
		echo "1"
	else
		echo "0"
	fi
}

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

function setPowerWarmReboot()
{
	busctl set-property \
		xyz.openbmc_project.State.Host \
		/xyz/openbmc_project/state/host0 \
		xyz.openbmc_project.State.Host \
		RequestedHostTransition \
		s \
		xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot
}

function setPowerForceReboot()
{
	busctl set-property \
	xyz.openbmc_project.State.Host \
	/xyz/openbmc_project/state/host0 \
	xyz.openbmc_project.State.Host \
	RequestedHostTransition \
	s \
	xyz.openbmc_project.State.Host.Transition.Reboot
}

function setPowerForceOff()
{
	busctl set-property \
	xyz.openbmc_project.State.Host \
	/xyz/openbmc_project/state/chassis0 \
	xyz.openbmc_project.State.Chassis \
	RequestedPowerTransition \
	s \
	xyz.openbmc_project.State.Chassis.Transition.Off
}

function setPowerWarmOff()
{
	busctl set-property \
	xyz.openbmc_project.State.Host \
	/xyz/openbmc_project/state/host0 \
	xyz.openbmc_project.State.Host \
	RequestedHostTransition \
	s \
	xyz.openbmc_project.State.Host.Transition.Off
}

# systemd service
function showSystemdServices()
{
	systemctl list-units --type=service
}

# systemd restart
function restartGpioService()
{
	systemctl restart \
		xyz.openbmc_project.Chassis.Control.Gpio.service
}
function restartFanControlService()
{
	systemctl restart \
		sophgo-fan-control.service
}
function restartEntityService()
{
	rm /var/configuration/system.json
	systemctl restart \
		xyz.openbmc_project.EntityManager.service
}
# boot source
function getBootSource()
{
	busctl get-property \
	xyz.openbmc_project.Settings \
	/xyz/openbmc_project/control/host0/boot \
	xyz.openbmc_project.Control.Boot.Source \
	BootSource
}

function setBootPxeSource()
{
	busctl set-property \
	xyz.openbmc_project.Settings \
	/xyz/openbmc_project/control/host0/boot \
	xyz.openbmc_project.Control.Boot.Source \
	BootSource \
	s \
	xyz.openbmc_project.Control.Boot.Source.Sources.Network
}

function setBootHddSource()
{
	busctl get-property \
	xyz.openbmc_project.Settings \
	/xyz/openbmc_project/control/host0/boot \
	xyz.openbmc_project.Control.Boot.Source \
	BootSource \
	s \
	xyz.openbmc_project.Control.Boot.Source.Sources.Disk
}

function setBootDefaultSource()
{
	busctl set-property \
	xyz.openbmc_project.Settings \
	/xyz/openbmc_project/control/host0/boot \
	xyz.openbmc_project.Control.Boot.Source \
	BootSource \
	s \
	xyz.openbmc_project.Control.Boot.Source.Sources.Default
}

# dbus show
function showEntityDbus()
{
	local dbus_name="xyz.openbmc_project.EntityManager"
	echo "${dbus_name}:"
	busctl tree ${dbus_name}
}
function showFanDbus()
{
	local dbus_name="xyz.openbmc_project.FanSensor"
	echo "${dbus_name}:"
	busctl tree ${dbus_name}
}
function showHwmonTempDbus()
{
	local dbus_name="xyz.openbmc_project.HwmonTempSensor"
	echo "${dbus_name}:"
	busctl tree ${dbus_name}
}
function showFruDbus()
{
	local dbus_name="xyz.openbmc_project.FruDevice"
	echo "${dbus_name}:"
	busctl tree ${dbus_name}
}
function showMapperDbus()
{
	local dbus_name="xyz.openbmc_project.ObjectMapper"
	echo "${dbus_name}:"
	busctl tree ${dbus_name}
}
function showPsuDbus()
{
	local dbus_name="xyz.openbmc_project.PSUSensor"
	echo "${dbus_name}:"
	busctl tree ${dbus_name}
}
function showNvmeDbus()
{
	local dbus_name="xyz.openbmc_project.NVMeSensor"
	echo "${dbus_name}:"
	busctl tree ${dbus_name}
}


function showSophgoFru()
{
	cat /sys/devices/platform/ahb/ahb:apb/ahb:apb:bus@1e78a000/1e78a780.i2c-bus/i2c-14/14-0050/eeprom
}

function is_fw_version_getting()
{
	busctl get-property \
	xyz.openbmc_project.Gpio \
	/xyz/openbmc_project/gpio/fwVersion \
	xyz.openbmc_project.Gpio.fwVersion \
	versionGetingState
}

function is_version_get_enabled()
{
	busctl get-property \
	xyz.openbmc_project.Gpio \
	/xyz/openbmc_project/gpio/fwVersion \
	xyz.openbmc_project.Gpio.fwVersion \
	versionEnableState
}

function disable_version_get()
{
	busctl set-property \
	xyz.openbmc_project.Gpio \
	/xyz/openbmc_project/gpio/fwVersion \
	xyz.openbmc_project.Gpio.fwVersion \
	versionEnableState \
	b \
	false
}
function enable_version_get()
{
	busctl set-property \
	xyz.openbmc_project.Gpio \
	/xyz/openbmc_project/gpio/fwVersion \
	xyz.openbmc_project.Gpio.fwVersion \
	versionEnableState \
	b \
	true
}