# sophgo append



SRC_URI += "file://ast2600-sophgo.dts;subdir=git/arch/arm/dts \
            file://ast2600-sophgo.dtsi;subdir=git/arch/arm/dts \
            file://ast2600-uboot-sophgo.dtsi;subdir=git/arch/arm/dts \
            file://Makefile;subdir=git/arch/arm/dts \
          "



# SRC_URI += " \
#             file://ast2600-sophgo.dtsi;subdir=../../../../../workspace/sources/u-boot-aspeed-sdk/arch/arm/dts \
#             file://ast2600-uboot-sophgo.dtsi;subdir=../../../../../workspace/sources/u-boot-aspeed-sdk/arch/arm/dts \
#             file://ast2600-sophgo.dts;subdir=../../../../../workspace/sources/u-boot-aspeed-sdk/arch/arm/dts \
#             file://Makefile;subdir=../../../../../workspace/sources/u-boot-aspeed-sdk/arch/arm/dts \
#             "


SRC_URI += " file://0001-u-boot-remove-PCIE1RC-from-ast2600_groups.patch \
             file://0002-evb_ast2600-add-board_late_init.patch \
             file://0003-deconfig.patch \
            "

FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

PACKAGE_ARCH = "ast2600-sophgo"

UBOOT_DEVICETREE = "ast2600-sophgo"