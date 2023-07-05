FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:ast-abr = " \
    file://0001-mirror-boot-for-static.patch \
    file://0002-obmc-flash-bmc-for-abr.patch \
"

# SRC_URI:append:ast-abr = " \
#     file://0001-mirror-boot-for-static.patch \
# "


SRC_URI:append = " file://0003-sophgo-obmc-flash-host-bios-service.patch "

SRC_URI += " \
    file://bios_update.sh \
"


SYSTEMD_SERVICE:${PN}-updater:append:ast-abr = " \
    obmc-flash-bmc-mirroruboot.service \
"
PACKAGECONFIG[flash_bios] = "-Dhost-bios-upgrade=enabled"

FILES:${PN} += "/lib/systemd/system/obmc-flash-host-bios@.service"

FILES:${PN} += "/usr/sbin/bios_update.sh"

do_install:append() {
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/bios_update.sh ${D}/${sbindir}
}