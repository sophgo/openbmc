FILESPATH := "${THISDIR}/files/:"
LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

SRC_URI += " \
           file://localtime \
           file://set-fan-rate.sh \
           file://get-cpld-info.sh \
           file://temp-test-record.sh \
           file://power-control-test.sh \
           "


S = "${WORKDIR}"
RDEPENDS:${PN} += "bash"


do_install () {
    install -d ${D}/${sysconfdir}
    install -m 0644 ${WORKDIR}/localtime ${D}${sysconfdir}/
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/get-cpld-info.sh ${D}/${sbindir}
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/set-fan-rate.sh ${D}/${sbindir}
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/temp-test-record.sh ${D}/${sbindir}
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/power-control-test.sh ${D}/${sbindir}
}




