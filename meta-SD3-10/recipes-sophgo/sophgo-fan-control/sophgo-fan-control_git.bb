# https://www.coder.work/article/6433710

SUMMARY = "Fan control"
DESCRIPTION = ""

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""
FILESPATH := "${THISDIR}/project/:"

SRC_URI = " \
    file://src/fan_control.cpp \
    file://src/fan_control.hpp \
    file://service_files/sophgo-fan-control.service \
    file://service_files/sophgo-riser-i2c-ctr.service \
    file://config/fan-config.json \
    file://meson_options.txt \
    file://meson.build \
    file://shell/sophgo-riser-i2c-function.sh \
    file://shell/sophgo-riser-i2c-scan.sh \
"

S = "${WORKDIR}"

RDEPENDS:${PN} += "bash"

inherit meson systemd pkgconfig
inherit obmc-phosphor-dbus-service



SYSTEMD_AUTO_ENABLE = "enable"
SYSTEMD_SERVICE_${PN} += "sophgo-fan-control.service sophgo-riser-i2c-ctr.service"
FILES:${PN}  += "${systemd_system_unitdir}/sophgo-fan-control.service ${systemd_system_unitdir}/sophgo-riser-i2c-ctr.service"



DEPENDS += " \
    boost \
    i2c-tools \
    libgpiod \
    nlohmann-json \
    sdbusplus \
    phosphor-logging \
  "


do_install:append() {

    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/shell/sophgo-riser-i2c-function.sh ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/shell/sophgo-riser-i2c-scan.sh ${D}/${sbindir}

    systemctl --root=${D} enable sophgo-fan-control.service
    systemctl --root=${D} enable sophgo-riser-i2c-ctr.service
}