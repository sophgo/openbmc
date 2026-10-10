IMAGE_FSTYPES = "${INITRAMFS_FSTYPES}"

require recipes-core/images/core-image-minimal.bb

IMAGE_INSTALL:append = " \
        packagegroup-oss-apps \
        packagegroup-oss-libs \
        packagegroup-oss-intel-pmci \
        packagegroup-aspeed-apps \
        packagegroup-aspeed-crypto \
        packagegroup-aspeed-ssif \
        ${@bb.utils.contains('MACHINE_FEATURES', 'ast-ssp', 'packagegroup-aspeed-ssp', '', d)} \
        packagegroup-aspeed-mtdtest \
        packagegroup-aspeed-ktools \
        packagegroup-aspeed-usbtools \
        "

EXTRA_IMAGE_FEATURES:append = " \
        nfs-client \
        "
