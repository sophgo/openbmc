SUMMARY = "Packagegroup for Open Source"

PR = "r1"

PACKAGE_ARCH="${TUNE_PKGARCH}"

inherit packagegroup

PROVIDES = "${PACKAGES}"

PACKAGES = " \
    ${PN}-apps \
    ${PN}-libs \
    ${PN}-intel-pmci \
    "

SUMMARY:${PN}-apps = "Open Source Applications"
RDEPENDS:${PN}-apps = " \
    mdio-tool \
    gperf \
    iperf3 \
    pciutils \
    ethtool \
    mmc-utils \
    memtester \
    i3c-tools \
    i2c-tools \
    xdma-test \
    libpeci \
    dhrystone \
    nbd-client \
    iozone3 \
    ncsi-netlink \
    hdparm \
    stressapptest \
    e2fsprogs-mke2fs \
    nvme-cli \
    ${@d.getVar('PREFERRED_PROVIDER_u-boot-fw-utils', True) or 'u-boot-fw-utils'} \
    aer-inject \
    "

SUMMARY:${PN}-intel-pmci = "Open Source Intel PMCI Applications"
RDEPENDS:${PN}-intel-pmci = " \
    libmctp-intel-test \
    mctpd \
    "

RDEPENDS:${PN}-apps:append:aspeed-g6 = " \
    fio \
    freeipmi \
    freeipmi-ipmi-raw \
    "

RDEPENDS:${PN}-apps:append:cypress-s25hx = " \
    mtd-utils \
    "

SUMMARY:${PN}-libs = "Open Source Library"
RDEPENDS:${PN}-libs = " \
    libgpiod \
    libgpiod-tools \
    "
