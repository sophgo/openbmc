# https://www.coder.work/article/6433710

SUMMARY = "Led control"
DESCRIPTION = ""

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""
FILESPATH := "${THISDIR}/project/:"

SRC_URI = " \
    file://src/led_control.cpp \
    file://src/led_control.hpp \
    file://service_files/sophgo-led-control.service \
    file://config/led-config.json \
    file://meson_options.txt \
    file://meson.build \
"

S = "${WORKDIR}"

RDEPENDS:${PN} += "bash"

inherit meson systemd pkgconfig
inherit obmc-phosphor-dbus-service



SYSTEMD_AUTO_ENABLE = "enable"
SYSTEMD_SERVICE_${PN} += "sophgo-led-control.service"
FILES:${PN}  += "${systemd_system_unitdir}/sophgo-led-control.service"



DEPENDS += " \
    boost \
    i2c-tools \
    libgpiod \
    nlohmann-json \
    sdbusplus \
    phosphor-logging \
  "

