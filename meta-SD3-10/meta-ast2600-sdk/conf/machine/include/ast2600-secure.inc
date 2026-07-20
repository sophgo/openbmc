# Secure Boot Settings
require conf/machine/include/ast-secure.inc

ASPEED_SECURE_BOOT_TARGET = "ast2600"

# OpenBMC Settings
FLASH_UBOOT_SPL_IMAGE = "s_u-boot-spl"

# ASPEED Initramfs Settings
ASPEED_IMAGE_UBOOT_SPL_IMAGE = "s_u-boot-spl"

# ASPEED EMMC Settings
ASPEED_EMMC_IMAGE_UBOOT_SPL_IMAGE = "s_u-boot-spl"
ASPEED_EMMC_IMAGE_MERGE_UBOOT_IMAGE = "s_emmc_image-u-boot"

