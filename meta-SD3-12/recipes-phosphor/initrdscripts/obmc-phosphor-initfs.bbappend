FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append = " file://obmc-shutdown.sh \
                 "
do_install:append() {
    install -m 0755 ${WORKDIR}/obmc-shutdown.sh ${D}/shutdown
}
