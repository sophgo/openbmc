# FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"
# SRC_URI:append = " file://ethanolx-baseboard.json \
#                    file://ethanolx-chassis.json"

# do_install:append() {
#      rm -f ${D}/usr/share/entity-manager/configurations/*.json
#      install -d ${D}/usr/share/entity-manager/configurations
#      install -m 0444 ${WORKDIR}/ethanolx-baseboard.json ${D}/usr/share/entity-manager/configurations
#      install -m 0444 ${WORKDIR}/ethanolx-chassis.json ${D}/usr/share/entity-manager/configurations
# }
FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"
SRC_URI:append = " file://0000-sophgo-fru.patch "
SRC_URI:append = " file://0001-sophgo-fru-utils-hpp-readFRUContents.patch "
SRC_URI:append = " file://0002-sophgo-fru-device-readFRUContents.patch "
SRC_URI:append = " file://0003-sophgo-fru-utils-cpp-readFRUContents.patch "


SRC_URI += " \
            file://GC550PMP_psu.json \
            file://Sophgo_board.json \
            file://GC1300PMP_psu.json \
            file://blacklist.json \
            file://nvme_ExAscend.json \
            "

do_install:append() {

    install -d ${D}/${prefix_native}/share/entity-manager/configurations/
    install -m 0644 ${WORKDIR}/Sophgo_board.json ${D}/${prefix_native}/share/entity-manager/configurations

    install -d ${D}/${prefix_native}/share/entity-manager/configurations/
    install -m 0644 ${WORKDIR}/GC550PMP_psu.json ${D}/${prefix_native}/share/entity-manager/configurations

    install -d ${D}/${prefix_native}/share/entity-manager/configurations/
    install -m 0644 ${WORKDIR}/GC1300PMP_psu.json ${D}/${prefix_native}/share/entity-manager/configurations

    install -d ${D}/${prefix_native}/share/entity-manager/configurations/
    install -m 0644 ${WORKDIR}/nvme_ExAscend.json ${D}/${prefix_native}/share/entity-manager/configurations

    install -m 0444 ${WORKDIR}/blacklist.json -D -t ${D}/usr/share/entity-manager
}

FILES:${PN}  += "${prefix_native}/share/entity-manager/configurations/Sophgo_board.json"
FILES:${PN}  += "${prefix_native}/share/entity-manager/configurations/GC550PMP_psu.json"
FILES:${PN}  += "${prefix_native}/share/entity-manager/configurations/GC1300PMP_psu.json"
FILES:${PN}  += "${prefix_native}/share/entity-manager/configurations/nvme_ExAscend.json"
FILES:${PN}  += "${prefix_native}/share/entity-manager/blacklist.jso"