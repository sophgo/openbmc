FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"

SRC_URI:append = " file://0001-Update-to-vue-5.0.8.patch "
SRC_URI:append = " file://0002-Use-aspeed-s-novnc-fork.patch "
SRC_URI:append = " file://0003-sophgo-eslintrcjs.patch "
SRC_URI:append = " file://0004-sophgo-AppHeadervue.patch "
SRC_URI:append = " file://0005-sophgo-LoginLayoutvue.patch "
SRC_URI:append = " file://0006-sophgo-GlobalStoreJs.patch "
SRC_URI:append = " file://0007-sophgo-ServerPowerOperationsvue.patch "
SRC_URI:append = " file://0008-sophgo-AppHeaderspecjssnap.patch "
SRC_URI:append = " file://0009-sophgo-en-Usjs.patch "



SRC_URI += "file://sg2042.svg;subdir=git/src/assets/images/ \
            file://sophgo.svg;subdir=git/src/assets/images/ \
            file://sopho.svg;subdir=git/src/assets/images/ \
            "

# SRC_URI += "file://sg2042.svg;subdir=../../../../../workspace/sources/webui-vue/src/assets/images/ \
#             file://sophgo.svg;subdir=../../../../../workspace/sources/webui-vue/src/assets/images/ \
#             file://sopho.svg;subdir=../../../../../workspace/sources/webui-vue/src/assets/images/ \
#             "