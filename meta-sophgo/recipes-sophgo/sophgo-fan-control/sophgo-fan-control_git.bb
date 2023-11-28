# https://www.coder.work/article/6433710

SUMMARY = "Initial boot script"
DESCRIPTION = "Script to do any first boot init, started as a systemd service which removes itself once finished"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""


S = "${WORKDIR}"

RDEPENDS:${PN} += "bash"

SRC_URI += " \
    file://sophgo-fan-control.sh \
    file://sophgo-fan-control.service \
    file://sophgo-riser-i2c-ctr.sh \
"

inherit allarch systemd bash-completion obmc-phosphor-systemd



SYSTEMD_AUTO_ENABLE = "enable"
SYSTEMD_SERVICE_${PN} = "sophgo-fan-control.service"
FILES:${PN}  += "${systemd_system_unitdir}/sophgo-fan-control.service"

do_install () {

    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/sophgo-fan-control.sh ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/sophgo-riser-i2c-ctr.sh ${D}/${sbindir}

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/sophgo-fan-control.service ${D}${systemd_system_unitdir}
}