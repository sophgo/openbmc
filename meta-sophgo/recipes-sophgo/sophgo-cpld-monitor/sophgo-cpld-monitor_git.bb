SUMMARY = "Monitor CPLD info"
DESCRIPTION = "Monitor CPLD info"

FILESPATH := "${THISDIR}/project/:"

SRC_URI = " \
           file://config/sophgo-cpld-monitor.json \
           file://service_files/sophgo-cpld-monitor.service \
           file://src/cpld_monitor.cpp \
           file://src/cpld_monitor.hpp \
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


SYSTEMD_AUTO_ENABLE = "enable"
SYSTEMD_SERVICE_${PN} = "sophgo-cpld-monitor.service"
FILES:${PN}  += "${systemd_system_unitdir}/sophgo-cpld-monitor.service"


DEPENDS += " \
    boost \
    i2c-tools \
    libgpiod \
    nlohmann-json \
    sdbusplus \
    phosphor-logging \
  "