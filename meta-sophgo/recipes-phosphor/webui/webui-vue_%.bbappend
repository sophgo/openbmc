
FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"



SRC_URI:append = " file://0001-Update-to-vue-5.0.8.patch "
SRC_URI:append = " file://0002-Use-aspeed-s-novnc-fork.patch "
SRC_URI:append = " file://0003-update-icon.patch "
SRC_URI:append = " file://0004-Add-cpld-info.patch "
SRC_URI:append = " file://0005-update-SOL-UI.patch "
SRC_URI:append = " file://0006-Add-cpld-firmware-management-UI.patch "
SRC_URI:append = " file://0007-enable-subscribe.patch "
SRC_URI:append = " file://0008-update-inventory-UI.patch "
SRC_URI:append = " file://0009-add-fan-speed-control-UI.patch "
SRC_URI:append = " file://0010-Cancel-asynchronous-reporting-of-fan-sata-and-hdd-status.patch "
SRC_URI:append = " file://0011-update-network-UI.patch "
SRC_URI:append = " file://0012-Add-SOL-log-export-function.patch "
SRC_URI:append = " file://0013-Add-1684-board-temperature-display-function.patch "




SRC_URI += "file://sg2042.svg;subdir=git/src/assets/images \
            file://sophgo.svg;subdir=git/src/assets/images \
            file://sopho.svg;subdir=git/src/assets/images \
            file://FirmwareCardsCpld.vue;subdir=git/src/views/Operations/Firmware \
            file://FanSpeedStore.js;subdir=git/src/store/modules/Settings/ \
            file://FanSpeed/FanSpeed.vue \
            file://FanSpeed/index.js \
            "


CURRENT_FILE_DIR := "${@os.path.dirname(d.getVar('FILE', True))}"


do_add_files() {
    echo "---${WORKDIR}---${S}---${THISDIR}---${FILE}---${THISDIR}/${PN}---${CURRENT_FILE_DIR}---"

    install -d ${S}/src/views/Settings/FanSpeed
    # cp ${WORKDIR}/FanSpeed/FanSpeed.vue ${S}/src/views/Settings/FanSpeed/
    # cp ${WORKDIR}/FanSpeed/index.js ${S}/src/views/Settings/FanSpeed/
    cp ${CURRENT_FILE_DIR}/webui-vue/FanSpeed/FanSpeed.vue ${S}/src/views/Settings/FanSpeed/
    cp ${CURRENT_FILE_DIR}/webui-vue/FanSpeed/index.js ${S}/src/views/Settings/FanSpeed/
}


addtask do_add_files after do_unpack before do_patch


