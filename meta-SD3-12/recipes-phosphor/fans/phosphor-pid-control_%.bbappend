FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

# inherit obmc-phosphor-systemd

# do_install:append() {
#     install -d ${D}/${prefix_native}/share/swampd/
#     install -m 0644 ${WORKDIR}/config.json ${D}/${prefix_native}/share/swampd/
# }