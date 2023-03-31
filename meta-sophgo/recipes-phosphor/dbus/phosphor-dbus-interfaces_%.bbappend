FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += "file://0028-MCTP-Daemon-D-Bus-interface-definition.patch \
            file://0032-update-meson-build-for-MCTP-interfaces.patch \
            "
