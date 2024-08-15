FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

EXTRA_OEMESON:append= " \
    -Dredfish-dbus-log=enabled \
    -Dhttp-body-limit=128 \
    "

SRC_URI:append = " \
    file://0001-virtual-media-websocket-buffer-overflw.patch \
    file://0002-Support-websocket-control-frame-callback.patch \
    file://0003-Modify-Content-Security-Policy-CSP-to-adapt-WebAssem.patch \
    file://0004-sophgo-add-cpld-info.patch \
    file://0005-sophgo-add-SOL-channel-switching.patch \
    file://0006-sophgo-add-cpld-firmware-management.patch \
    file://0007-sophgo-mask-user-asdbg.patch \
    file://0008-sophgo-fixed-IndicatorLed-management-bug.patch \
    file://0009-sophgo-add-fan-speed-management-function.patch \
    file://0010-sophgo-fixed-ethernet-management-bug.patch \
    file://0011-sophgo-add-SOL-log-export-function.patch \
    file://0012-sophgo-add-1684-board-temp-display-function.patch \
    "


EXTRA_OEMESON:append = " \
    -Drest=enabled \
    "