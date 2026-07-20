SUMMARY = "ether PHY test"
DESCRIPTION = "ether PHY test"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

FILESPATH := "${THISDIR}/files:"



SRC_URI += " \
    file://Makefile \
    file://phytool.c \
    file://phytool.h \
    file://print_mv6.c \
    file://print_phy.c \
    file://mv6tool.8 \
    file://phytool.8 \
"



S = "${WORKDIR}"

RDEPENDS:${PN} += "bash"

inherit  bash-completion



TARGET_CC_ARCH += "${LDFLAGS}"

do_compile () {
    pwd
    make
}


do_install() {
    install -d ${D}${bindir}
    install -m 0755 phytool ${D}${bindir}
}