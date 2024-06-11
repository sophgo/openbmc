FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

EXTRA_OEMESON:append= " \
    -Dredfish-dbus-log=enabled \
    -Dhttp-body-limit=128 \
    "

SRC_URI:append = " \
    file://0001-virtual-media-websocket-buffer-overflw.patch \
    file://0002-Support-websocket-control-frame-callback.patch \
    file://0003-Modify-Content-Security-Policy-CSP-to-adapt-WebAssem.patch \
    file://0004-add-solswitch-cpldversion-system.patch \
    file://0005-add-solroute-redfidh.patch \
    file://0006-sophgo-mask-user-asdbg.patch \
    file://0007-sophgo-identifyLed.patch \
    file://0008-sophgo-systemshpp-add-fanspeed.patch \
    file://0009-sophgo-ethernet.patch \
    "
EXTRA_OEMESON:append = " \
    -Drest=enabled \
    "