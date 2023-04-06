FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

EXTRA_OEMESON:append= " \
    -Dredfish-dbus-log=enabled \
    -Dhttp-body-limit=128 \
    "

SRC_URI:append = " \
    file://0001-virtual-media-websocket-buffer-overflw.patch \
    file://0002-Support-websocket-control-frame-callback.patch \
    file://0003-Modify-Content-Security-Policy-CSP-to-adapt-WebAssem.patch \
    file://0004-sophgo-get-cpld-info-toUI.patch \
    "
