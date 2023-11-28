
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

PACKAGE_ARCH = "ast2600-sophgo"

SRC_URI:append = " file://0000-sophgo-ssif_bmc.patch "
SRC_URI:append = " file://0001-sophgo-gigadevice.patch "
# SRC_URI:append = " file://0002-sophgo-bmcdev-ipmiinterface.patch "
SRC_URI:append = " file://0003-sophgo-lm90-add-ct7451.patch "

# KERNEL_DEVICETREE += "aspeed-ast2600-sophgo.dtb"
# S = "${WORKDIR}"
# SRC_URI += "file://aspeed-g6-sophgo.dtsi;subdir=../../../../../workspace/sources/linux-aspeed/arch/arm/boot/dts \
#             file://aspeed-ast2600-sophgo.dts;subdir=../../../../../workspace/sources/linux-aspeed/arch/arm/boot/dts \
#             file://sophgo_defconfig;subdir=../../../../../workspace/sources/linux-aspeed/arch/arm/configs \
#             "

SRC_URI += "file://aspeed-g6-sophgo.dtsi;subdir=git/arch/arm/boot/dts \
            file://aspeed-ast2600-sophgo.dts;subdir=git/arch/arm/boot/dts \
            file://sophgo_defconfig;subdir=git/arch/arm/configs \
            "

