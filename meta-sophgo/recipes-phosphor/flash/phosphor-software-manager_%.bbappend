FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:ast-abr = " \
    file://0001-mirror-boot-for-static.patch \
    file://0002-obmc-flash-bmc-for-abr.patch \
"

SYSTEMD_SERVICE:${PN}-updater:append:ast-abr = " \
    obmc-flash-bmc-mirroruboot.service \
"
