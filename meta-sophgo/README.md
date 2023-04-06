# Create build environment
## Prerequisite
### Ubuntu 18.04
```
sudo apt-get install gawk wget git-core diffstat unzip texinfo gcc-multilib \
     build-essential chrpath socat libsdl1.2-dev xterm
```

### Fedora
```
sudo yum install gawk make wget tar bzip2 gzip python unzip perl patch \
     diffutils diffstat git cpp gcc gcc-c++ glibc-devel texinfo chrpath \
     ccache perl-Data-Dumper perl-Text-ParseWords perl-Thread-Queue socat \
     findutils which SDL-devel xterm
```

Reference:
- [OpenBMC/README.md](https://github.com/openbmc/openbmc#1-prerequisite)
- [Yocto Project Quick Build](https://docs.yoctoproject.org/brief-yoctoprojectqs/index.html)

## Target the machine
```
source setup <machine> [build_dir]
Target machine must be specified. Use one of:
ast2500-default
ast2500-default-54
ast2600-dcscm
ast2600-default
ast2600-default-510
ast2600-default-54
ast2600-ecc
ast2600-emmc
ast2600-emmc-secure-rsa2048-sha256
ast2600-emmc-secure-rsa4096-sha512
ast2600-ncsi
ast2600-pfr
ast2600-usbadp
ast2600-secure-rsa2048-sha256
ast2600-secure-rsa2048-sha256-ncot
ast2600-secure-rsa2048-sha256-o1
ast2600-secure-rsa2048-sha256-o2-pub
ast2600-secure-rsa3072-sha384
ast2600-secure-rsa3072-sha384-o1
ast2600-secure-rsa3072-sha384-o2-pub
ast2600-secure-rsa4096-sha512
ast2600-secure-rsa4096-sha512-o1
ast2600-secure-rsa4096-sha512-o2-pub
ast2600-a2
ast2600-a2-54
ast2600-ecc
ast2600-a2-emmc
ast2600-a2-emmc-secure-rsa2048-sha256
ast2600-a2-emmc-secure-rsa4096-sha512
ast2600-a2-secure-rsa2048-sha256
ast2600-a2-secure-rsa2048-sha256-ncot
ast2600-a2-secure-rsa2048-sha256-o1
ast2600-a2-secure-rsa2048-sha256-o2-pub
ast2600-a2-secure-rsa3072-sha384
ast2600-a2-secure-rsa3072-sha384-o1
ast2600-a2-secure-rsa3072-sha384-o2-pub
ast2600-a2-secure-rsa4096-sha512
ast2600-a2-secure-rsa4096-sha512-o1
ast2600-a2-secure-rsa4096-sha512-o2-pub
ast2600-a1
ast2600-a1-54
ast2600-ecc
ast2600-a1-secure-rsa2048-sha256
ast2600-a1-secure-rsa2048-sha256-ncot
ast2600-a1-secure-rsa2048-sha256-o1
ast2600-a1-secure-rsa2048-sha256-o2-pub
ast2600-a1-secure-rsa3072-sha384
ast2600-a1-secure-rsa3072-sha384-o1
ast2600-a1-secure-rsa3072-sha384-o2-pub
ast2600-a1-secure-rsa4096-sha512
ast2600-a1-secure-rsa4096-sha512-o1
ast2600-a1-secure-rsa4096-sha512-o2-pub
```

```
source setup ast2600-sophgo [build_dir]
```


## Build OpenBMC firmware

```
bitbake obmc-phosphor-image
```

# Output image
After you successfully built the image, the image file can be found in: `[build_dir]/tmp/work/deploy/images/${MACHINE}/`.

## OpenBMC firmware

### Boot from SPI image
- `image-bmc`: whole flash image
- `image-u-boot`: u-boot-spl.bin + u-boot.bin
- `image-kernel`: Linux Kernel FIT image
- `image-rofs`: read-only root file system

### Boot from SPI with secure boot image
- `image-bmc`: whole flash image
- `image-u-boot`: s_u-boot-spl.bin(RoT) + u-boot.bin (CoT1)
- `image-kernel`: Linux Kernel FIT Image the same as fitImage-${INITRAMFS_IMAGE}-${MACHINE}-${MACHINE} (CoT2)
- `image-rofs`: read-only root file system
- `s_u-boot-spl`: u-boot-spl.bin processed with socsec tool signing for RoT image
- `u-boot`: u-boot.bin processed with verified boot signing for CoT1 image
- `fitImage-${INITRAMFS_IMAGE}-${MACHINE}-${MACHINE}`: fitImage-${INITRAMFS_IMAGE}-${MACHINE}-${MACHINE} processed with verified boot signing for CoT2 image
- `otp_image`: OTP image

### Boot from eMMC image
- `emmc_image-u-boot`: u-boot-spl.bin + u-boot.bin processed with gen\_emmc\_image.py for boot partition
- `obmc-phosphor-image-${MACHINE}.wic.xz`: compressed emmc flash image for user data partition

### Boot from eMMC with secure boot image
- `s_emmc_image-u-boot`: s_u-boot-spl.bin(RoT) + u-boot.bin(CoT1) for boot partition
- `obmc-phosphor-image-${MACHINE}.wic.xz`: compressed emmc flash image for user data partition
- `s_u-boot_spl`: u-boot-spl.bin processed with socsec tool signing for RoT image
- `u-boot`: u-boot.bin processed with verified boot signing for CoT1 image
- `fitImage-${INITRAMFS_IMAGE}-${MACHINE}-${MACHINE}`: fitImage-${INITRAMFS_IMAGE}-${MACHINE}-${MACHINE} processed with verified boot signing for CoT2 image
- `otp_image`: OTP image

### Recovery Image via UART
- `recovery_u-boot-spl` : u-boot-spl.bin processed with gen_uart_booting_image.sh for recovery image via UART
- `recovery_s_u-boot-spl` : s_u-boot-spl.bin processed with gen_uart_booting_image.sh for recovery image via UART with secure boot

# Free Open Source Software (FOSS)
The Yocto/OpenBMC build system supports to provide the following things to meet the FOSS requirement.
- Source code must be provided.
- License text for the software must be provided.
- Compilation scripts and modifications to the source code must be provided.

The Yocto Project generates a license manifest during image creation that is located in ${DEPLOY_DIR}/licenses/image_name-datestamp to assist with any audits.
During the creation of your image, the source and patch from all recipes that deploy packages to the image is placed within subdirectories of DEPLOY_DIR/sources on the LICENSE for each recipe.
Please refer to [Working With Licenses](https://docs.yoctoproject.org/dev-manual/common-tasks.html#working-with-licenses) for detail.

To create it, please add the following settings in `local.conf`.
By default, it only creates for `GPL, LGPL and AGPL` LICENSE. User can add `COPYLEFT_LICENSE_INCLUDE = "*"` to create for all LICENSE.
Please refer to `archiver.bbclass` for detail.

```
INHERIT += "archiver"
ARCHIVER_MODE[src] = "original"
ARCHIVER_MODE[recipe] = "1"
COPYLEFT_LICENSE_INCLUDE = "*"
```

