FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

EXTRA_OEMESON:append= " \
    -Dredfish-dbus-log=enabled \
    -Dhttp-body-limit=128 \
    "

SRC_URI:append = " \
    file://0001-virtual-media-websocket-buffer-overflw.patch \
    file://0002-Support-websocket-control-frame-callback.patch \
    file://0003-Modify-Content-Security-Policy-CSP-to-adapt-WebAssem.patch \
    file://0004-add-cpld-version-and-sol-port-switching.patch \
    file://0005-mask-user-asdbg.patch \
    file://0006-fixed-IndicatorLed-management-bug.patch \
    file://0007-add-fan-speed-management-function.patch \
    file://0008-fixed-ethernet-management-bug.patch \
    file://0009-add-SOL-log-export-function.patch \
    file://0010-add-1684-board-temp-display-function.patch \
    "
EXTRA_OEMESON:append = " \
    -Drest=enabled \
    "