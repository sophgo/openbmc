FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://0002-Add-control-for-aspeed-format.patch "
SRC_URI:append = " file://0003-Avoid-frame-drop.patch "

