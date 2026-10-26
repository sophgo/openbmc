FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += "file://0028-MCTP-Daemon-D-Bus-interface-definition.patch \
            file://0032-update-meson-build-for-MCTP-interfaces.patch \
            file://0033-sophgo-software-version-interface.patch \
            file://0035-Device-errors-yaml.patch \
            file://0036-sophgo-updating-errors.patch \
            "
