FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"



SRC_URI:append = " file://0001-Update-to-vue-5.0.8.patch "
SRC_URI:append = " file://0002-Use-aspeed-s-novnc-fork.patch "
SRC_URI:append = " file://0003-sophgo-eslintrcjs.patch "
SRC_URI:append = " file://0004-sophgo-AppHeadervue.patch "
SRC_URI:append = " file://0005-sophgo-LoginLayoutvue.patch "
SRC_URI:append = " file://0006-sophgo-GlobalStoreJs.patch "
SRC_URI:append = " file://0007-sophgo-ServerPowerOperationsvue.patch "
SRC_URI:append = " file://0008-sophgo-AppHeaderspecjssnap.patch "
SRC_URI:append = " file://0009-sophgo-en-Usjs.patch "
SRC_URI:append = " file://0010-sophgo-en-US.patch "
SRC_URI:append = " file://0011-sophgo-GlobalStore.patch "
SRC_URI:append = " file://0012-sophgo-ControlStore.patch "
SRC_URI:append = " file://0013-sophgo-SerialOverLanConsole.patch "
SRC_URI:append = " file://0014-sophgo-ServerPowerOperations.patch "
SRC_URI:append = " file://0015-sophgo-cpldversion-en-usjs.patch "
SRC_URI:append = " file://0016-sophgo-cpldversion-GlobalStore.patch "
SRC_URI:append = " file://0017-sophgo-cpldversion-Firmware.patch "
SRC_URI:append = " file://0018-sophgo-enable-subscribe.patch "
SRC_URI:append = " file://0019-sophgo-WebSocketPlugin.patch "
SRC_URI:append = " file://0020-sophgo-inventory.patch "
SRC_URI:append = " file://0021-sophgo-navigation-add-fanspeed.patch "
SRC_URI:append = " file://0022-sophgo-naviga-ibm-add-fanspeed.patch "
SRC_URI:append = " file://0023-sophgo-router-ibm-add-fanspeed.patch "
SRC_URI:append = " file://0024-sophgo-naviga-intel-add-fanspeed.patch "
SRC_URI:append = " file://0025-sophgo-router-intel-add-fanspeed.patch "
SRC_URI:append = " file://0026-sophgo-enUS.patch "
SRC_URI:append = " file://0027-sophgo-ruRU.patch "
SRC_URI:append = " file://0028-sophgo-route-add-fanspeed.patch "
SRC_URI:append = " file://0029-sophgo-index-add-fanspeed.patch "
SRC_URI:append = " file://0030-sophgo-enUs.patch "
SRC_URI:append = " file://0031-sophgo-WebSocketPlugin.patch "




SRC_URI += "file://sg2042.svg;subdir=git/src/assets/images \
            file://sophgo.svg;subdir=git/src/assets/images \
            file://sopho.svg;subdir=git/src/assets/images \
            file://FirmwareCardsCpld.vue;subdir=git/src/views/Operations/Firmware \
            file://FanSpeedStore.js;subdir=git/src/store/modules/Settings/ \
            file://FanSpeed/FanSpeed.vue \
            file://FanSpeed/index.js \
            "
do_create_FanSpeed_dir() {
    echo "---${WORKDIR}---${S}---${THISDIR}---"
    install -d ${S}/src/views/Settings/FanSpeed
    cp ${WORKDIR}/FanSpeed/FanSpeed.vue ${S}/src/views/Settings/FanSpeed/
    cp ${WORKDIR}/FanSpeed/index.js ${S}/src/views/Settings/FanSpeed/
}
addtask do_create_FanSpeed_dir after do_unpack before do_patch





