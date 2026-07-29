SUMMARY = "ether PHY test"
DESCRIPTION = "ether PHY test"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

FILESPATH := "${THISDIR}/files:"



SRC_URI += " \
    file://Makefile \
    file://ephy.c \
"



S = "${WORKDIR}"

do_compile () {
    pwd
    make
}

RDEPENDS:${PN} += "bash"

inherit  bash-completion



TARGET_CC_ARCH += "${LDFLAGS}"
do_install() {
    install -d ${D}${bindir}
    install -m 0755 phytool ${D}${bindir}
}