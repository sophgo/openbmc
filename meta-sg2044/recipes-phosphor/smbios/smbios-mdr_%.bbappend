FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"
PACKAGECONFIG:append = " smbios-ipmi-blob"
PACKAGECONFIG:remove = " cpuinfo"

SRC_URI += "file://0001-fix-smbios-table-parsing-bug.patch"

