
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

DEPENDS:append = " sg2042-yaml-config"

EXTRA_OEMESON:append= " \
    -Dsensor-yaml-gen=${STAGING_DIR_HOST}${datadir}/sg2042-yaml-config/ipmi-sensors.yaml \
    -Dfru-yaml-gen=${STAGING_DIR_HOST}${datadir}/sg2042-yaml-config/ipmi-fru-read.yaml \
    "
