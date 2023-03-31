SPL_SIGN_ENABLE = "1"
SPL_SIGN_KEYDIR ?= "${STAGING_DATADIR_NATIVE}/aspeed-secure-config/ast2600/security/key"
SPL_SIGN_KEYNAME ?= "test_bl2_3072"

UBOOT_SIGN_ENABLE = "1"
UBOOT_SIGN_KEYDIR ?= "${STAGING_DATADIR_NATIVE}/aspeed-secure-config/ast2600/security/key"
UBOOT_SIGN_KEYNAME ?= "test_bl3_3072"

# Kernel / U-Boot fitImage Hash Algo
FIT_HASH_ALG ?= "sha384"
UBOOT_FIT_HASH_ALG ?= "sha384"

# Kernel / U-Boot fitImage Signature Algo
FIT_SIGN_ALG ?= "rsa3072"
UBOOT_FIT_SIGN_ALG ?= "rsa3072"

# Size of private keys in number of bits
FIT_SIGN_NUMBITS ?= "3072"
UBOOT_FIT_SIGN_NUMBITS ?= "3072"
