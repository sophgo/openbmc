FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://0001-remove-ncsi-netlink.patch "

PACKAGECONFIG:remove = "default-ipv6-accept-ra"

