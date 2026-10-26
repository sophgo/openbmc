FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

EXTRA_OEMESON:append= " \
    -Dredfish-dbus-log=enabled \
    -Dhttp-body-limit=128 \
    "

SRC_URI:append = " \
    file://0001-Add-sophgo-custom-features.patch \
    "

EXTRA_OEMESON:append = " \
    -Drest=enabled \
    "
