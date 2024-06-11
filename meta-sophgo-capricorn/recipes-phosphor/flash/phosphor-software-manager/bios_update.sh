#!/bin/sh


imagePath=$1
image=$imagePath/image-bmc
image_purpose=$imagePath/image-host

dbus_name="xyz.openbmc_project.State.Host"
dbus_path="/xyz/openbmc_project/state/host0"
dbus_inf="xyz.openbmc_project.State.Host"

dbus_set_property="RequestedHostTransition"
dbus_set_method="set-property"
property_type="s"

set_graceful_reboot="xyz.openbmc_project.State.Host.Transition.ForceWarmReboot"

echoerr() {
	echo 1>&2 "ERROR: $@"
}

function  abnormal_exit() {
	# rm -rf $imagePath
	flashSwitchToHost
	exit 1
}

creat_mtd () {
	# Check the image-host partition available
	HOST_MTD=$(< /proc/mtd grep "image-host" | sed -n 's/^\(.*\):.*/\1/p')
	if [ -z "$HOST_MTD" ];
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

		HOST_MTD=$(< /proc/mtd grep "image-host" | sed -n 's/^\(.*\):.*/\1/p')
		if [ -z "$HOST_MTD" ];
		then
			echo "Fail to probe Host SPI-NOR device"
			abnormal_exit
		fi
	fi
}


findmtd() {
	m=$(grep -xl "$1" /sys/class/mtd/*/name)
	m=${m%/name}
	m=${m##*/}
	echo $m
}

toobig() {
	if test $(stat -L -c "%s" "$1") -gt $(cat /sys/class/mtd/"$2"/size)
	then
		return 0
	fi
	return 1
}

flashSwitchToBmc() {
    # gpioset 0 43=0
	busctl set-property xyz.openbmc_project.Gpio /xyz/openbmc_project/gpio/biosflash xyz.openbmc_project.Gpio.BiosFlash BiosFlashTransition s "bmc"
}


flashSwitchToHost() {
    # gpioset 0 43=1
	busctl set-property xyz.openbmc_project.Gpio /xyz/openbmc_project/gpio/biosflash xyz.openbmc_project.Gpio.BiosFlash BiosFlashTransition s "host"
}

cp $image $image_purpose
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

done


rm -rf $imagePath
flashSwitchToHost
sleep 1
# set dbus property to reboot host
busctl $dbus_set_method $dbus_name $dbus_path $dbus_inf $dbus_set_property $property_type $set_graceful_reboot
