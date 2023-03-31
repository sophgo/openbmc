SPL_SIGN_ENABLE = "1"
SPL_SIGN_KEYDIR ?= "${STAGING_DATADIR_NATIVE}/aspeed-secure-config/ast2600/security/key"
SPL_SIGN_KEYNAME ?= "test_bl2_4096"

UBOOT_SIGN_ENABLE = "1"
UBOOT_SIGN_KEYDIR ?= "${STAGING_DATADIR_NATIVE}/aspeed-secure-config/ast2600/security/key"
UBOOT_SIGN_KEYNAME ?= "test_bl3_4096"

# Kernel / U-Boot fitImage Hash Algo
FIT_HASH_ALG ?= "sha512"
UBOOT_FIT_HASH_ALG ?= "sha512"

# Kernel / U-Boot fitImage Signature Algo
FIT_SIGN_ALG ?= "rsa4096"
UBOOT_FIT_SIGN_ALG ?= "rsa4096"

# Size of private keys in number of bits
FIT_SIGN_NUMBITS ?= "4096"
UBOOT_FIT_SIGN_NUMBITS ?= "4096"
