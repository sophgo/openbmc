# https://www.coder.work/article/6433710

SUMMARY = "led control"
DESCRIPTION = "Script to do any first boot init, started as a systemd service which removes itself once finished"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""


S = "${WORKDIR}"

RDEPENDS:${PN} += "bash"

SRC_URI += " \
    file://sophgo-led-control.sh \
    file://sophgo-led-control.service \
"

inherit allarch systemd bash-completion obmc-phosphor-systemd



SYSTEMD_AUTO_ENABLE = "enable"
SYSTEMD_SERVICE_${PN} = "sophgo-led-control.service"
FILES:${PN}  += "${systemd_system_unitdir}/sophgo-led-control.service"

do_install () {

    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/sophgo-led-control.sh ${D}/${sbindir}

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/sophgo-led-control.service ${D}${systemd_system_unitdir}
}