FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"
IMAGE_INSTALL:append = " \
        webui-vue \
        libmctp \
        entity-manager \
        dbus-sensors \
        test-hello-yzh \
        "

IMAGE_INSTALL:append = " \
        packagegroup-oss-apps \
        packagegroup-oss-libs \
        packagegroup-oss-intel-pmci \
        packagegroup-aspeed-obmc-apps \
        packagegroup-aspeed-apps \
        packagegroup-aspeed-crypto \
        packagegroup-aspeed-ssif \
        packagegroup-aspeed-obmc-inband \
        ${@bb.utils.contains('MACHINE_FEATURES', 'ast-ssp', 'packagegroup-aspeed-ssp', '', d)} \
        packagegroup-aspeed-mtdtest \
        packagegroup-aspeed-usbtools \
        ${@bb.utils.contains('DISTRO_FEATURES', 'tpm', \
            bb.utils.contains('MACHINE_FEATURES', 'tpm2', 'packagegroup-security-tpm2', '', d), \
            '', d)} \
        "

# Only install in AST26xx series rofs as the free space of AST2500 rofs is not enough.
IMAGE_INSTALL:append:aspeed-g6 = " \
        packagegroup-aspeed-ktools \
       "

EXTRA_IMAGE_FEATURES:append = " \
        nfs-client \
        ${@bb.utils.contains('DISTRO_FEATURES', 'obmc-ubi-fs', 'read-only-rootfs-delayed-postinsts', '', d)} \
        ${@bb.utils.contains('DISTRO_FEATURES', 'phosphor-mmc', 'read-only-rootfs-delayed-postinsts', '', d)} \
        "

# OBMC_IMAGE_EXTRA_INSTALL_append_${MACHINE} = "test-hello"

OVERLAY_MKFS_OPTS:cypress-s25hx:static-rwfs-jffs2 = " -c 16 -e 262144 --pad=${RWFS_SIZE} "

do_generate_rwfs_static:static-rwfs-jffs2() {
    rwdir=$(pwd)
    rwdir=${rwdir}/jffs2
    image=rwfs.jffs2

    rm -rf $rwdir $image > /dev/null 2>&1
    mkdir -p ${rwdir}/cow
    rwdir=${rwdir}/cow

    ${JFFS2_RWFS_CMD}  ${OVERLAY_MKFS_OPTS} --squash-uids
}

inherit image_types_phosphor_aspeed
