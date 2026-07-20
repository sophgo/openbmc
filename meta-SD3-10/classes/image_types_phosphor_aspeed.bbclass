FLASH_UBOOT_SPL_IMAGE ?= "u-boot-spl"
FLASH_UBOOT_IMAGE ?= "u-boot"

do_generate_image_uboot_file() {
    image_dst="$1"
    uboot_offset=${FLASH_UBOOT_OFFSET}

    if [ ! -z ${SPL_BINARY} ]; then
        dd bs=1k conv=notrunc seek=${FLASH_UBOOT_OFFSET} \
            if=${DEPLOY_DIR_IMAGE}/${FLASH_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} \
            of=${image_dst}
        uboot_offset=${FLASH_UBOOT_SPL_SIZE}
    fi

    dd bs=1k conv=notrunc seek=${uboot_offset} \
        if=${DEPLOY_DIR_IMAGE}/${FLASH_UBOOT_IMAGE}.${UBOOT_SUFFIX} \
        of=${image_dst}
}

# Include the full u-boot-spl and u-boot in the final static image
python do_generate_static:append() {
    uboot_offset = int(d.getVar('FLASH_UBOOT_OFFSET', True))
    spl_binary = d.getVar('SPL_BINARY', True)

    if spl_binary:
        _append_image(os.path.join(d.getVar('DEPLOY_DIR_IMAGE', True),
                                   '%s.%s' % (
                                    d.getVar('FLASH_UBOOT_SPL_IMAGE', True),
                                    d.getVar('UBOOT_SUFFIX', True))),
                      int(d.getVar('FLASH_UBOOT_OFFSET', True)),
                      int(d.getVar('FLASH_UBOOT_SPL_SIZE', True)))
        uboot_offset += int(d.getVar('FLASH_UBOOT_SPL_SIZE', True))

    _append_image(os.path.join(d.getVar('DEPLOY_DIR_IMAGE', True),
                               '%s.%s' % (
                                    d.getVar('FLASH_UBOOT_IMAGE', True),
                                    d.getVar('UBOOT_SUFFIX', True))),
                  uboot_offset,
                  int(d.getVar('FLASH_KERNEL_OFFSET', True)))
}

do_make_ubi:append() {
    # Concatenate the uboot and ubi partitions
    uboot_offset=${FLASH_UBOOT_OFFSET}

    if [ ! -z ${SPL_BINARY} ]; then
        dd bs=1k conv=notrunc seek=${FLASH_UBOOT_OFFSET} \
            if=${DEPLOY_DIR_IMAGE}/${FLASH_UBOOT_SPL_IMAGE}.${UBOOT_SUFFIX} \
            of=${IMGDEPLOYDIR}/${IMAGE_NAME}.ubi.mtd
        uboot_offset=${FLASH_UBOOT_SPL_SIZE}
    fi

    dd bs=1k conv=notrunc seek=${uboot_offset} \
        if=${DEPLOY_DIR_IMAGE}/${FLASH_UBOOT_IMAGE}.${UBOOT_SUFFIX} \
        of=${IMGDEPLOYDIR}/${IMAGE_NAME}.ubi.mtd
}

do_make_ubi[depends] += "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)}"
do_generate_ubi_tar[depends] += "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)}"
do_generate_static_tar[depends] += "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)}"
do_generate_ext4_tar[depends] += "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)}"
do_generate_static[depends] += "${@bb.utils.contains('MACHINE_FEATURES', 'ast-secure', 'aspeed-image-secureboot:do_deploy', '', d)}"
