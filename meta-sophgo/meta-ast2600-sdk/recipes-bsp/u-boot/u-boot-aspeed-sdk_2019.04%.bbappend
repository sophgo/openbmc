FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append:ast-mmc = " \
    file://u-boot-env-ast2600.txt \
    "
