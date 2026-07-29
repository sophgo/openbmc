SUMMARY = "get rtc time"
DESCRIPTION = "get rtc time"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

FILESPATH := "${THISDIR}/files:"



SRC_URI += " \
    file://Makefile \
    file://sophgo-rtc-device.c \
    file://sophgo-rtc-device.service \
"



S = "${WORKDIR}"

do_compile () {
    pwd
    make
}

RDEPENDS:${PN} += "bash"

inherit  systemd bash-completion obmc-phosphor-systemd

SYSTEMD_AUTO_ENABLE = "enable"
SYSTEMD_SERVICE_${PN} = "sophgo-rtc-device.service"
FILES:${PN}  += "${systemd_system_unitdir}/sophgo-rtc-device.service"


TARGET_CC_ARCH += "${LDFLAGS}"
do_install() {
    install -d ${D}${bindir}
    install -m 0755 sophgo-rtc-dev ${D}${bindir}

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/sophgo-rtc-device.service ${D}${systemd_system_unitdir}
}