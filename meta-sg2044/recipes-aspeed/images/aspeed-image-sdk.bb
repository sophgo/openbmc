SUMMARY = "Two partition MTD image with u-boot and kernel"
HOMEPAGE = "https://github.com/openbmc/meta-aspeed"
LICENSE = "MIT"
PACKAGE_ARCH = "${MACHINE_ARCH}"

inherit ${@bb.utils.contains('MACHINE_FEATURES', 'ast-mmc', 'image', 'deploy', d)}

UBOOT_SUFFIX ?= "bin"
ASPEED_IMAGE_UBOOT_SPL_IMAGE ?= "u-boot-spl"
ASPEED_IMAGE_UBOOT_IMAGE ?= "u-boot"
ASPEED_IMAGE_UBOOT_OFFSET_KB ?= "0"
ASPEED_IMAGE_UBOOT_SPL_SIZE_KB ?= "64"
ASPEED_IMAGE_KERNEL_OFFSET_KB ?= "1024"
ASPEED_IMAGE_SIZE_KB ?= "24576"
ASPEED_IMAGE_KERNEL_IMAGE ?= "fitImage-${INITRAMFS_IMAGE}-${MACHINE}-${MACHINE}"
ASPEED_IMAGE_NAME ?= "all.bin"
ASPEED_BOOT_EMMC ?= "${@bb.utils.contains('MACHINE_FEATURES', 'ast-mmc', 'yes', 'no', d)}"

IMAGE_FSTYPES:ast-mmc += "wic.xz mmc-ext4-tar"
IMAGE_FEATURES:ast-mmc += "read-only-rootfs-delayed-postinsts"

do_compile() {
    if [ ${ASPEED_BOOT_EMMC} == "yes" ] ; then
        echo "MMC mode should not run this task to generate aspeed-sdk.bin"
        exit 1
    fi

    uboot_offset=${ASPEED_IMAGE_UBOOT_OFFSET_KB}

    dd if=/dev/zero bs=1k count=${ASPEED_IMAGE_SIZE_KB} | \
        tr '\000' '\377' > ${B}/aspeed-sdk.bin

    if [ ! -z ${SPL_BINARY} ] ; then
        dd bs=1k conv=notrunc seek=${ASPEED_IMAGE_UBOOT_OFFSET_KB} \
            if=${DEPLOY_DIR_IMAGE}/${ASPEED_IMAGE_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} \
            of=${B}/aspeed-sdk.bin
        uboot_offset=${ASPEED_IMAGE_UBOOT_SPL_SIZE_KB}
    fi

    dd bs=1k conv=notrunc seek=${uboot_offset} \
        if=${DEPLOY_DIR_IMAGE}/${ASPEED_IMAGE_UBOOT_IMAGE}.${UBOOT_SUFFIX} \
        of=${B}/aspeed-sdk.bin \

    dd bs=1k conv=notrunc seek=${ASPEED_IMAGE_KERNEL_OFFSET_KB} \
        if=${DEPLOY_DIR_IMAGE}/${ASPEED_IMAGE_KERNEL_IMAGE} \
        of=${B}/aspeed-sdk.bin
}

do_compile:ast-mmc() {
    :
}

do_deploy() {
    if [ ${ASPEED_BOOT_EMMC} == "yes" ] ; then
        echo "MMC mode should not run this task to generate ${ASPEED_IMAGE_NAME}"
        exit 1
    fi

    sdk_image_size="$(wc -c ${B}/aspeed-sdk.bin | awk '{print $1}')"
    let sdk_image_size/=1024

    if [ ${sdk_image_size} -gt ${ASPEED_IMAGE_SIZE_KB} ] ; then
        echo "Actual SDK image size (${sdk_image_size}kb) is larger than allowed size ${ASPEED_IMAGE_SIZE_KB}kb"
        exit 1
    fi

    install -m644 -D ${B}/aspeed-sdk.bin ${DEPLOYDIR}/${ASPEED_IMAGE_NAME}
}

do_deploy:ast-mmc() {
    :
}

do_compile[depends] = " \
    virtual/kernel:do_deploy \
    u-boot:do_deploy \
    ${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)} \
    "
do_fetch[noexec] = "1"
do_unpack[noexec] = "1"
do_patch[noexec] = "1"
do_configure[noexec] = "1"
do_install[noexec] = "1"
deltask do_populate_sysroot
do_package[noexec] = "1"
deltask do_package_qa
do_packagedata[noexec] = "1"
deltask do_package_write_ipk
deltask do_package_write_deb
deltask do_package_write_rpm
addtask deploy before do_build after do_compile
