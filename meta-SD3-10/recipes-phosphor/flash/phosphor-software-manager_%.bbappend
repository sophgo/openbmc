FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:ast-abr = " \
    file://0001-mirror-boot-for-static.patch \
    file://0002-obmc-flash-bmc-for-abr.patch \
"

# SRC_URI:append:ast-abr = " \
#     file://0001-mirror-boot-for-static.patch \
# "


SRC_URI:append = " file://0003-sophgo-obmc-flash-host-bios-service.patch \
                   file://0004-sophgo-activation-cpp.patch \
                   file://0005-sophgo-activation-hpp.patch \
                   file://0006-sophgo-item_updater-cpp.patch \
                   file://0007-sophgo-item_updater-hpp.patch \
                   file://0008-sophgo-activation-cpp.patch \
                 "

SRC_URI += " \
    file://bios_update.sh \
    file://cpld_update.sh \
    file://sophgo-cpld-update@.service \
"


SYSTEMD_SERVICE:${PN}-updater:append:ast-abr = " \
    obmc-flash-bmc-mirroruboot.service \
"

SYSTEMD_SERVICE:${PN} = " \
    sophgo-cpld-update@.service \
"

FILES:${PN}  += "${systemd_system_unitdir}/sophgo-cpld-update@.service"

PACKAGECONFIG[flash_bios] = "-Dhost-bios-upgrade=enabled"

FILES:${PN} += "/lib/systemd/system/obmc-flash-host-bios@.service"

FILES:${PN} += "/usr/sbin/bios_update.sh"
FILES:${PN} += "/usr/sbin/cpld_update.sh"

do_install:append() {
    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/bios_update.sh ${D}/${sbindir}

    install -d ${D}/${sbindir}
    install -m 0755 ${WORKDIR}/cpld_update.sh ${D}/${sbindir}

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/sophgo-cpld-update@.service ${D}${systemd_system_unitdir}
}