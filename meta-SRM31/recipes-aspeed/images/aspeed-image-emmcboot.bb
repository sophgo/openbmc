DESCRIPTION = "Generate image boot from eMMC for ASPEED BMC SoCs"
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
UBOOT_SUFFIX ?= "bin"
ASPEED_IMAGE_UBOOT_SPL_IMAGE ?= "u-boot-spl"
ASPEED_IMAGE_UBOOT_FIT_IMAGE ?= "u-boot-fitImage"
ASPEED_EMMC_IMAGE_UBOOT_SPL_IMAGE ?= "emmc_${ASPEED_IMAGE_UBOOT_SPL_IMAGE}"
ASPEED_EMMC_IMAGE_UBOOT_IMAGE ?= "u-boot"
ASPEED_EMMC_IMAGE_UBOOT_SPL_SIZE_KB ?= "64"
ASPEED_EMMC_IMAGE_MERGE_UBOOT_SIZE_KB ?= "1024"
ASPEED_EMMC_IMAGE_MERGE_UBOOT_IMAGE ?= "emmc_image-u-boot"
ASPEED_SECURE_BOOT ?= "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'yes', 'no', d)}"

OUTPUT_IMAGE_DIR ?= "${S}/output"
SOURCE_IMAGE_DIR ?= "${S}/source"

do_deploy () {
    if [ -z ${SPL_BINARY} ]; then
        echo "To support ASPEED boot from emmc, u-boot should support spl."
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

    if [ ${ASPEED_SECURE_BOOT} == "no" ]; then
        install -m 0644 ${DEPLOY_DIR_IMAGE}/${ASPEED_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} ${SOURCE_IMAGE_DIR}
        gen_emmc_boot_image.py ${SOURCE_IMAGE_DIR}/${ASPEED_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} ${SOURCE_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX}
        install -m 0644 ${SOURCE_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} ${OUTPUT_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX}
    else
        install -m 0644 ${DEPLOY_DIR_IMAGE}/${ASPEED_EMMC_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} ${SOURCE_IMAGE_DIR}
    fi

    install -m 0644 ${DEPLOY_DIR_IMAGE}/${ASPEED_EMMC_IMAGE_UBOOT_IMAGE}.${UBOOT_SUFFIX} ${SOURCE_IMAGE_DIR}
    if [ ${UBOOT_FITIMAGE_ENABLE} == "1" ]; then
        install -m 0644 ${DEPLOY_DIR_IMAGE}/${ASPEED_IMAGE_UBOOT_FIT_IMAGE} ${SOURCE_IMAGE_DIR}
    fi

    # Generate emmc_image-u-boot for boot from eMMC
    dd bs=1k count=${ASPEED_EMMC_IMAGE_MERGE_UBOOT_SIZE_KB} \
        if=/dev/zero of=${OUTPUT_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_MERGE_UBOOT_IMAGE}
    dd bs=1k count=${ASPEED_EMMC_IMAGE_UBOOT_SPL_SIZE_KB} conv=notrunc \
        if=${SOURCE_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} of=${OUTPUT_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_MERGE_UBOOT_IMAGE}

    if [ ${UBOOT_FITIMAGE_ENABLE} == "1" ]; then

        dd bs=1k seek=${ASPEED_EMMC_IMAGE_UBOOT_SPL_SIZE_KB} conv=notrunc \
            if=${SOURCE_IMAGE_DIR}/${ASPEED_IMAGE_UBOOT_FIT_IMAGE}  of=${OUTPUT_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_MERGE_UBOOT_IMAGE}
    else
        dd bs=1k seek=${ASPEED_EMMC_IMAGE_UBOOT_SPL_SIZE_KB} conv=notrunc \
            if=${SOURCE_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_UBOOT_IMAGE}.${UBOOT_SUFFIX}  of=${OUTPUT_IMAGE_DIR}/${ASPEED_EMMC_IMAGE_MERGE_UBOOT_IMAGE}
    fi

    # Deploy image for boot from emmc
    install -d ${DEPLOYDIR}
    install -m 644 ${OUTPUT_IMAGE_DIR}/* ${DEPLOYDIR}/.
}

do_deploy[depends] += " \
    virtual/kernel:do_deploy \
    virtual/bootloader:do_deploy \
    ${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)} \
    "

addtask deploy before do_build after do_compile
