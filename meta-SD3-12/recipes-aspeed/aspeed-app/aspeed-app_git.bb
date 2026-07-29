LICENSE = "GPL-2.0-or-later"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/GPL-2.0-or-later;md5=fed54355545ffd980b814dab4a3b312c"

inherit pkgconfig meson

SRC_URI = " git://github.com/AspeedTech-BMC/aspeed_app.git;protocol=https;branch=${BRANCH} "
PV = "1.0+git${SRCPV}"

# Tag for v00.01.08
SRCREV = "7245d73f53097e9b9ffc75d1fda88d18072974d9"
BRANCH = "master"

S = "${WORKDIR}/git"

DEPENDS += "openssl"
RDEPENDS:${PN} += "openssl"

FILES:${PN}:append = " /usr/share/* "
