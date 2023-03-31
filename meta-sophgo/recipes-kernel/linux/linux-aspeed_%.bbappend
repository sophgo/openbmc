
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

PACKAGE_ARCH = "ast2600-sophgo"

# KERNEL_DEVICETREE += "aspeed-ast2600-sophgo.dtb"

# SRC_URI += "file://aspeed-g6-sophgo.dtsi;subdir=../../../../../workspace/sources/linux-aspeed/arch/arm/boot/dts \
#             file://aspeed-ast2600-sophgo.dts;subdir=../../../../../workspace/sources/linux-aspeed/arch/arm/boot/dts \
#             "

SRC_URI += "file://aspeed-g6-sophgo.dtsi;subdir=git/arch/arm/boot/dts \
            file://aspeed-ast2600-sophgo.dts;subdir=git/arch/arm/boot/dts \
            "

