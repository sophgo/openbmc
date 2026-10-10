FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://0001-Potentially-connect-udc-device-incorrectly.patch "
SRC_URI:append = " file://0002-Add-control-for-aspeed-format.patch "
SRC_URI:append = " file://0003-Avoid-frame-drop.patch "
SRC_URI:append = " file://0004-Add-support-of-partial-jpeg.patch"

