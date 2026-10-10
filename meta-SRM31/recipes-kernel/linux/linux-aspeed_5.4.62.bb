KBRANCH = "aspeed-master-v5.4"
LINUX_VERSION ?= "5.4.62"

# Tag for v00.04.11
SRCREV = "5fa0cdddb4b9707ea93796ad15ac0b0290807ee1"

require linux-aspeed.inc

LIC_FILES_CHKSUM = "file://COPYING;md5=bbea815ee2795b2f4230826c0c6b8814"

DEPENDS += "lzop-native"
DEPENDS += "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-secure-config-native', '', d)}"

SRC_URI:append = " file://ipmi_ssif.cfg "
SRC_URI:append = " file://mtd_test.cfg "
