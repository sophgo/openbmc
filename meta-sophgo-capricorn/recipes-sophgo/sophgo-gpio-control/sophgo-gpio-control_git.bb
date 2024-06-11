SUMMARY = "Chassis Gpio Control service for Intel based platforms"
DESCRIPTION = "Chassis Gpio Control service for Intel based platforms"

# SRC_URI = "git://github.com/openbmc/x86-power-control.git;protocol=https;branch=master"
# SRCREV = "99e8f9dfe6ed99e201f5920c512587fe3af3cdb9"



FILESPATH := "${THISDIR}/project/:"


SRC_URI = " \
           file://config/gpio-config.json \
           file://service_files/xyz.openbmc_project.Chassis.Control.Gpio.service \
           file://src/gpio_control.cpp \
           file://src/gpio_control.hpp \
           file://MAINTAINERS \
           file://meson_options.txt \
           file://meson.build \
           file://OWNERS \
           file://README.md \
           "


S = "${WORKDIR}"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

inherit meson systemd pkgconfig
inherit obmc-phosphor-dbus-service

SYSTEMD_SERVICE:${PN} = "xyz.openbmc_project.Chassis.Control.Gpio.service"

DEPENDS += " \
    boost \
    i2c-tools \
    libgpiod \
    nlohmann-json \
    sdbusplus \
    phosphor-logging \
  "
FILES:${PN}  += "${systemd_system_unitdir}/xyz.openbmc_project.Chassis.Control.Gpio.service"