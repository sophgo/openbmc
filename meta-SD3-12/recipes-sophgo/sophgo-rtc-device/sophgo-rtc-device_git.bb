SUMMARY = " Operating RTC device"
DESCRIPTION = " Operating RTC device"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

FILESPATH := "${THISDIR}/files:"



SRC_URI += " \
    file://src/sophgo-rtc-device.cpp \
    file://src/sophgo-rtc-device.hpp \
    file://meson.build \
    file://meson_options.txt \
    file://sophgo-rtc-device.service \
    file://config/rtc-device-config.json \
"



S = "${WORKDIR}"



RDEPENDS:${PN} += "bash"

inherit meson systemd pkgconfig
inherit obmc-phosphor-dbus-service


DEPENDS += " \
    boost \
    i2c-tools \
    libgpiod \
    nlohmann-json \
    sdbusplus \
    phosphor-logging \
  "

TARGET_CC_ARCH += "${LDFLAGS}"
# do_install() {
#    systemctl --root=${D} enable sophgo-rtc-device.service
# }