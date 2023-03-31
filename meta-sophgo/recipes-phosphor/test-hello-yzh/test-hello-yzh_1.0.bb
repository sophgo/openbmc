SUMMARY = "Chassis test "
DESCRIPTION = "this is a test app"

# FILESEXTRAPATHS_append := "${THISDIR}/files:"
LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

SRC_URI = "file://Makefile \
           file://test-hello-yzh.c"




S = "${WORKDIR}"

do_compile () {
    pwd
    make
}
# INSANE_SKIP_${PN} = "ldflags"
# INSANE_SKIP_${PN}-dev = "ldflags"
TARGET_CC_ARCH += "${LDFLAGS}"
do_install() {
    install -d ${D}${bindir}
    install -m 0755 testhelloyzh ${D}${bindir}

    # install -d ${D}${sysconfdir}/init.d
    # install -m 0755 ${WORKDIR}/source-code/test.sh ${D}${sysconfdir}/init.d/test.sh
}