FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://0001-Update-to-vue-5.0.8.patch "
SRC_URI:append = " file://0002-Use-aspeed-s-novnc-fork.patch "
SRC_URI:append = " file://0003-sophgo-eslintrcjs.patch "
SRC_URI:append = " file://0004-sophgo-AppHeadervue.patch "
SRC_URI:append = " file://0005-sophgo-LoginLayoutvue.patch "
SRC_URI:append = " file://0006-sophgo-AppHeaderspecjssnap.patch "

SRC_URI:append = " file://0007-add-sol-switch-en-US.patch "
SRC_URI:append = " file://0008-add-cpld-version-GloableStore.patch "
SRC_URI:append = " file://0009-add-sol-port-Operations-ControlStore.patch "
SRC_URI:append = " file://0010-add-uidled-WebSocketPlugin.patch "
SRC_URI:append = " file://0011-add-solport-SerialOverLanConsole.patch "

SRC_URI:append = " file://0012-sophgo-cpldversion-Firmware.patch "
SRC_URI:append = " file://0013-sophgo-enable-subscribe.patch "
SRC_URI:append = " file://0014-sophgo-inventory.patch "

SRC_URI:append = " file://0015-sophgo-AppNavigationMixin-add-fanspeed.patch "
SRC_URI:append = " file://0016-sophgo-ibm-add-fanspeed.patch "
SRC_URI:append = " file://0017-sophgo-intel-add-fanspeed.patch "
SRC_URI:append = " file://0018-sophgo-ibm-add-fanspeed.patch "
SRC_URI:append = " file://0019-sophgo-intel-add-fanspeed.patch "
SRC_URI:append = " file://0020-sophgo-enus-add-fanspeed.patch "
SRC_URI:append = " file://0021-sophgo-ruru-add-fanspeed.patch "
SRC_URI:append = " file://0022-sophgo-routes-add-fanspeed.patch "
SRC_URI:append = " file://0023-sophgo-index-add-fanspeed.patch "
SRC_URI:append = " file://0024-sophgo-NetworkStore.patch "
SRC_URI:append = " file://0025-sophgo-NetworkGlobalSettings.patch "
SRC_URI:append = " file://0026-sophgo-SerialOverLanConsole-add-log-load.patch "



# close the following content when using the devtool command
SRC_URI += "file://sg2042.svg;subdir=git/src/assets/images \
            file://sophgo.svg;subdir=git/src/assets/images \
            file://FirmwareCardsCpld.vue;subdir=git/src/views/Operations/Firmware \
            file://FanSpeedStore.js;subdir=git/src/store/modules/Settings/ \
            file://FanSpeed/FanSpeed.vue \
            file://FanSpeed/index.js \
            "
do_create_FanSpeed_dir() {
    # echo "---${WORKDIR}---${S}---${THISDIR}---"
    install -d ${S}/src/views/Settings/FanSpeed
    cp ${WORKDIR}/FanSpeed/FanSpeed.vue ${S}/src/views/Settings/FanSpeed/
    cp ${WORKDIR}/FanSpeed/index.js ${S}/src/views/Settings/FanSpeed/
}
addtask do_create_FanSpeed_dir after do_unpack before do_patch