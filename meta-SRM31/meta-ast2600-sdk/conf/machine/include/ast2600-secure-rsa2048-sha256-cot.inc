SPL_SIGN_ENABLE = "1"
SPL_SIGN_KEYDIR ?= "${STAGING_DATADIR_NATIVE}/aspeed-secure-config/ast2600/security/key"
SPL_SIGN_KEYNAME ?= "test_bl2_2048"

UBOOT_SIGN_ENABLE = "1"
UBOOT_SIGN_KEYDIR ?= "${STAGING_DATADIR_NATIVE}/aspeed-secure-config/ast2600/security/key"
UBOOT_SIGN_KEYNAME ?= "test_bl3_2048"

# Kernel / U-Boot fitImage Hash Algo
FIT_HASH_ALG ?= "sha256"
UBOOT_FIT_HASH_ALG ?= "sha256"

# Kernel / U-Boot fitImage Signature Algo
FIT_SIGN_ALG ?= "rsa2048"
UBOOT_FIT_SIGN_ALG ?= "rsa2048"

# Size of private keys in number of bits
FIT_SIGN_NUMBITS ?= "2048"
UBOOT_FIT_SIGN_NUMBITS ?= "2048"

