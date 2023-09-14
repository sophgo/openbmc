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




SRC_URI += "file://sg2042.svg;subdir=git/src/assets/images \
            file://sophgo.svg;subdir=git/src/assets/images \
            file://sopho.svg;subdir=git/src/assets/images \
            file://FirmwareCardsCpld.vue;subdir=git/src/views/Operations/Firmware \
            "

# SRC_URI += "file://sg2042.svg;subdir=../../../../../workspace/sources/webui-vue/src/assets/images/ \
#             file://sophgo.svg;subdir=../../../../../workspace/sources/webui-vue/src/assets/images/ \
#             file://sopho.svg;subdir=../../../../../workspace/sources/webui-vue/src/assets/images/ \
#             "