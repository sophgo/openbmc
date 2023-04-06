FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

FILES:${PN}:append = " ${sysconfdir}/nbd-proxy/state "
SRC_URI:append = " file://state_hook "

do_install:append() {
    install -d ${D}${sysconfdir}/nbd-proxy/
    install -m 0755 ${WORKDIR}/state_hook ${D}${sysconfdir}/nbd-proxy/state
}
