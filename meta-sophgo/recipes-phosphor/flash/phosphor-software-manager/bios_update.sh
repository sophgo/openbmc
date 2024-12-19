#!/bin/sh


imagePath=$1
image=$imagePath/image-bmc
image_purpose=$imagePath/image-host
image0=$imagePath/image-host0
image1=$imagePath/image-host1

dbus_name="xyz.openbmc_project.State.Host"
dbus_path="/xyz/openbmc_project/state/host0"
dbus_inf="xyz.openbmc_project.State.Host"

dbus_set_property="RequestedHostTransition"
dbus_set_method="set-property"
property_type="s"

set_graceful_reboot="xyz.openbmc_project.State.Host.Transition.ForceWarmReboot"

function echoerr() {
	echo 1>&2 "ERROR: $@"
}

function disable_wdt() {
	busctl set-property xyz.openbmc_project.Gpio /xyz/openbmc_project/gpio/wdtEnable xyz.openbmc_project.Gpio.wdtEnable wdtEnableState b false
}
function enable_wdt() {
	busctl set-property xyz.openbmc_project.Gpio /xyz/openbmc_project/gpio/wdtEnable xyz.openbmc_project.Gpio.wdtEnable wdtEnableState b true
}

function flashSwitchToBmc() {
    gpioset 0 56=0
    gpioset 0 57=0
    gpioset 0 58=0
    gpioset 0 59=0
}


function flashSwitchToHost() {
    gpioset 0 56=1
    gpioset 0 57=1
    gpioset 0 58=1
    gpioset 0 59=1
}


function  abnormal_exit() {
	# rm -rf $imagePath
	# open WDT
	enable_wdt
	flashSwitchToHost
	exit 1
}

function creat_mtd () {
	# Check the image-host partition available
	HOST_MTD_0=$(< /proc/mtd grep "image-host0" | sed -n 's/^\(.*\):.*/\1/p')
	HOST_MTD_1=$(< /proc/mtd grep "image-host1" | sed -n 's/^\(.*\):.*/\1/p')
	if [ -z "$HOST_MTD_0" ] || [ -z "$HOST_MTD_1" ];
	then
		# Check the ASpeed SMC driver bound before
		HOST_SPI=/sys/bus/platform/drivers/ASPEED_FMC_SPI/1e630000.spi
		if [ -d "$HOST_SPI" ]; then
			echo "Unbind the ASpeed spi1 driver"
			echo 1e630000.spi > /sys/bus/platform/drivers/ASPEED_FMC_SPI/unbind
			sleep 2
		fi

		# If the image-host partition is not available, then bind again driver
		echo "--- Bind the ASpeed spi1 driver"
		echo 1e630000.spi > /sys/bus/platform/drivers/ASPEED_FMC_SPI/bind
		sleep 2

		HOST_MTD_0=$(< /proc/mtd grep "image-host0" | sed -n 's/^\(.*\):.*/\1/p')
		HOST_MTD_1=$(< /proc/mtd grep "image-host1" | sed -n 's/^\(.*\):.*/\1/p')
		if [ -z "$HOST_MTD_0" ] || [ -z "$HOST_MTD_1" ];
		then
			echo "Fail to probe Host SPI-NOR device"
			abnormal_exit
		fi
	fi
}


function findmtd() {
	m=$(grep -xl "$1" /sys/class/mtd/*/name)
	m=${m%/name}
	m=${m##*/}
	echo $m
}

function toobig() {
	if test $(stat -L -c "%s" "$1") -gt $(cat /sys/class/mtd/"$2"/size)
	then
		return 0
	fi
	return 1
}



# close WDT
disable_wdt

cp $image $image0
cp $image $image1
rm -f $image
imglist=$(echo $image_purpose*)
if test "$imglist" = "$image*" -a ! -e "$imglist"
then
	# shell didn't expand the wildcard, so no files exist
	echo "No images found to update."
	imglist=
fi

flashSwitchToBmc

sleep 1

creat_mtd


for f in $imglist
do
	m=$(findmtd ${f##*/})
	if test -z "$m"
	then
		echoerr "Unable to find mtd partition for ${f##*/}."
		continue
	fi
	if test -n "$checksize" && toobig "$f" "$m"
	then
		echoerr "Image ${f##*/} too big for $m."
		continue
	fi

	if test ! -s $f
	then
		echo "Skipping empty update of ${f#$image}."
		rm $f
		continue
	fi
	m=$(findmtd ${f##*/})
	echo "Updating ${f##*/}..."

	flashcp -v $f /dev/$m

	if [ $? -eq 0 ]; then
		echo "Successfully updated ${f##*/}."
	else
		echo "Failed updated ${f##*/}, exit."
		abnormal_exit
	fi

done



rm -rf $imagePath
flashSwitchToHost
#open WDT
enable_wdt
sleep 1
#set dbus property to reboot host
busctl $dbus_set_method $dbus_name $dbus_path $dbus_inf $dbus_set_property $property_type $set_graceful_reboot
