FILESEXTRAPATHS:prepend:cypress-s25hx := "${THISDIR}/files:"

RDEPENDS:${PN}:append:cypress-s25hx = " mtd-utils"
SRC_URI:append:cypress-s25hx = "file://obmc-init.sh"

do_install:append:cypress-s25hx() {
    install -m 0755 ${WORKDIR}/obmc-init.sh ${D}/init
    install -m 0755 ${WORKDIR}/obmc-shutdown.sh ${D}/shutdown
}



FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI:append = " \
                    file://obmc-shutdown.sh \
                "
do_install:append() {
    install -m 0755 ${WORKDIR}/obmc-shutdown.sh ${D}/shutdown
}

