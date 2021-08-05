# Recipe created by recipetool
# This is the basis of a recipe and may need further editing in order to be fully functional.
# (Feel free to remove these comments when editing.)

# Unable to find any files that looked like license statements. Check the accompanying
# documentation and source headers and set LICENSE and LIC_FILES_CHKSUM accordingly.
#
# NOTE: LICENSE is being set to "CLOSED" to allow you to at least start building - if
# this is not accurate with respect to the licensing of the software being built (it
# will not be in most cases) you must specify the correct value before using this
# recipe for anything other than initial testing/development!
LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

S = "${WORKDIR}/git"

SRC_URI = " git://github.com/AspeedTech-BMC/aspeed_app.git;protocol=https;branch=${BRANCH} \
            file://meson.build \
            file://video_ioctl.h \
"
PV = "1.0+git${SRCPV}"

# Tag for v00.01.02
SRCREV = "ca004c08d1e35dbe46641507b11df4304b4b3d29"
BRANCH = "master"
inherit meson

do_configure_prepend() {
  cp ${WORKDIR}/meson.build ${S}
  cp ${WORKDIR}/video_ioctl.h ${S}/ast_rvas/rvas_lib
}

FILES_${PN}_append = " /usr/share/* "