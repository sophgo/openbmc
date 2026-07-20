FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://0001-Update-to-vue-5.0.8.patch "
SRC_URI:append = " file://0002-Use-aspeed-s-novnc-fork.patch "
SRC_URI:append = " file://0003-update-icon.patch "
SRC_URI:append = " file://0004-add-cpld-version-and-sol-port-switching.patch "
SRC_URI:append = " file://0005-enable-subscribe.patch "
SRC_URI:append = " file://0006-update-inventory-UI.patch "
SRC_URI:append = " file://0007-add-fan-speed-control-UI.patch "
SRC_URI:append = " file://0008-update-network-UI.patch "
SRC_URI:append = " file://0009-add-SOL-log-export-function.patch "
SRC_URI:append = " file://0010-add-1684-board-temperature-display-function.patch "


# close the following content when using the devtool command
SRC_URI += "file://sg2042.svg;subdir=git/src/assets/images \
            file://sophgo.svg;subdir=git/src/assets/images \
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