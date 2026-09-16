FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

PACKAGECONFIG = " \
    adcsensor \
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

SRC_URI:append = " file://0001-sophgo-sensors.patch "

