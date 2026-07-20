FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

PACKAGECONFIG = " \
    adcsensor \
    intelcpusensor \
    exitairtempsensor \
    fansensor \
    hwmontempsensor \
    intrusionsensor \
    ipmbsensor \
    mcutempsensor \
    psusensor \
    nvmesensor \
    external \
    "

# PACKAGECONFIG += " \
#     nvmesensor \
#     "

SRC_URI:append = " file://0000-sophgo-add-tmp451.patch "
SRC_URI:append = " file://0001-sophgo-add-ct7451.patch "