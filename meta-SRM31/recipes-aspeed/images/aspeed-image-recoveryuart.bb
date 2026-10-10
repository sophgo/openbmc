DESCRIPTION = "Generate recovery image via UART for ASPEED BMC SoCs"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${ASPEEDSDKBASE}/LICENSE;md5=a3740bd0a194cd6dcafdc482a200a56f"
PACKAGE_ARCH = "${MACHINE_ARCH}"

PR = "r0"

DEPENDS = " \
    aspeed-image-tools-native \
    virtual/bootloader \
    virtual/kernel \
    "

do_patch[noexec] = "1"
do_configure[noexec] = "1"
do_compile[noexec] = "1"
do_install[noexec] = "1"

inherit deploy

# Image composition
UBOOT_SPL_IMAGE ?= "u-boot-spl.bin"
SUBOOT_SPL_IMAGE ?= "s_${UBOOT_SPL_IMAGE}"
RECOVERY_INPUT_IMAGE ?= "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', '${SUBOOT_SPL_IMAGE}', '${UBOOT_SPL_IMAGE}', d)}"
RECOVERY_OUTPUT_IMAGE ?=  "recovery_${RECOVERY_INPUT_IMAGE}"

OUTPUT_IMAGE_DIR ?= "${S}/output"
SOURCE_IMAGE_DIR ?= "${S}/source"

do_deploy () {
    if [ -z ${SPL_BINARY} ]; then
        echo "To support ASPEED recovery image via uart, u-boot should support spl."
        exit 1
    fi

    if [ -d ${SOURCE_IMAGE_DIR} ]; then
        rm -rf ${SOURCE_IMAGE_DIR}
    fi

    if [ -d ${OUTPUT_IMAGE_DIR} ]; then
        rm -rf ${OUTPUT_IMAGE_DIR}
    fi

    install -d ${SOURCE_IMAGE_DIR}
    install -d ${OUTPUT_IMAGE_DIR}

    # Install recovery input image into source directory
    # Generate recovery image via UART
    install -m 0644 ${DEPLOY_DIR_IMAGE}/${RECOVERY_INPUT_IMAGE} ${SOURCE_IMAGE_DIR}
    sh ${STAGING_BINDIR_NATIVE}/gen_uart_booting_image.sh ${SOURCE_IMAGE_DIR}/${RECOVERY_INPUT_IMAGE} ${OUTPUT_IMAGE_DIR}/${RECOVERY_OUTPUT_IMAGE}

    # Deploy recovery image via UART
    install -d ${DEPLOYDIR}
    install -m 644 ${OUTPUT_IMAGE_DIR}/${RECOVERY_OUTPUT_IMAGE} ${DEPLOYDIR}/.
}

do_deploy[depends] += " \
    virtual/kernel:do_deploy \
    virtual/bootloader:do_deploy \
    ${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)} \
    "

addtask deploy before do_build after do_compile
