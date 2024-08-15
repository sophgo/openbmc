
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

PACKAGE_ARCH = "ast2600-sophgo"



SRC_URI:append = " file://0001-fix-ssif-polling-bug.patch "
SRC_URI:append = " file://0002-add-ct7451-in-lm90-module.patch "
SRC_URI:append = " file://0003-modify-bmc-pcie-device-class-code.patch "
SRC_URI:append = " file://0004-gigadevice-add-gd25lb512me.patch "


SRC_URI:append = " file://jtag_aspeed.cfg "


SRC_URI += "file://aspeed-g6-sophgo.dtsi;subdir=git/arch/arm/boot/dts \
            file://aspeed-ast2600-sophgo.dts;subdir=git/arch/arm/boot/dts \
            file://sophgo_defconfig;subdir=git/arch/arm/configs \
            "




