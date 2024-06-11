# FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

# SRC_URI:append = " \
#     file://00-bmc-eth0.network \
# "

# do_install:append() {
#     install -d ${D}${sysconfdir}/systemd/network
#     install -m 0644 ${WORKDIR}/00-bmc-eth0.network ${D}${sysconfdir}/systemd/network
# }

# FILES:${PN}:append = " \
#     ${sysconfdir}/systemd/network/00-bmc-eth0.network \
# "
