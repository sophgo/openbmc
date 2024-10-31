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

SRC_URI:append = " file://0001-HwmonTemp-Add-tmp451-and-ct7451.patch "
SRC_URI:append = " file://0002-PSU-Delete-pwm.patch "