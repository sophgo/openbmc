require u-boot-common-aspeed-sdk_${PV}.inc

UBOOT_MAKE_TARGET ?= "DEVICE_TREE=${UBOOT_DEVICETREE}"

require u-boot-aspeed.inc

PROVIDES += "u-boot"
DEPENDS += "bc-native dtc-native"
DEPENDS += "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-secure-config-native', '', d)}"

UBOOT_ENV_SIZE_ast-mmc = "0x10000"
UBOOT_ENV_ast-mmc = "u-boot-env"
UBOOT_ENV_SUFFIX_ast-mmc = "bin"
UBOOT_ENV_FILE_ast-mmc = "u-boot-env-ast2600.txt"

do_compile_append() {
    if [ -n "${UBOOT_ENV}" ]
    then
        # Generate redundant environment image
        ${B}/tools/mkenvimage -r -s ${UBOOT_ENV_SIZE} -o ${WORKDIR}/${UBOOT_ENV_BINARY} ${WORKDIR}/${UBOOT_ENV_FILE}
    fi
}