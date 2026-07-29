FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
FILESEXTRAPATHS:prepend:df-ast-img-sdk := "${THISDIR}/files/df-ast-img:"

SRC_URI:append = " file://fw_env_ast2600_nor.config "
SRC_URI:append:ast-mmc = " file://fw_env_ast2600_mmc.config "

ENV_CONFIG_FILE = "fw_env_ast2600_nor.config"
ENV_CONFIG_FILE:ast-mmc = "fw_env_ast2600_mmc.config"
