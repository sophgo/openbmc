
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

DEPENDS:append = " sg2042-yaml-config"

EXTRA_OEMESON:append= " \
    -Dsensor-yaml-gen=${STAGING_DIR_HOST}${datadir}/sg2042-yaml-config/ipmi-sensors.yaml \
    -Dfru-yaml-gen=${STAGING_DIR_HOST}${datadir}/sg2042-yaml-config/ipmi-fru-read.yaml \
    "

SRC_URI:append = " file://0001-chassis-add-0x3052-and-0x3053-cmd.patch"