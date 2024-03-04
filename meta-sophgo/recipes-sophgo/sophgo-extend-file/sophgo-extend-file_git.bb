FILESPATH := "${THISDIR}/files/:"
LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

SRC_URI += " \
           file://localtime \
           file://set-fan-rate.sh \
           file://get-cpld-info.sh \
           file://temp-test-record.sh \
           file://set-fan-rate@.service \
           "


S = "${WORKDIR}"
RDEPENDS:${PN} += "bash"


SYSTEMD_SERVICE:${PN} = " \
    set-fan-rate@.service \
"
FILES:${PN}  += "${systemd_system_unitdir}/set-fan-rate@.service"

do_install () {
    install -d ${D}/${sysconfdir}
    install -m 0644 ${WORKDIR}/localtime ${D}${sysconfdir}/
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/get-cpld-info.sh ${D}/${sbindir}
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/set-fan-rate.sh ${D}/${sbindir}
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/temp-test-record.sh ${D}/${sbindir}

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/set-fan-rate@.service ${D}${systemd_system_unitdir}
}




