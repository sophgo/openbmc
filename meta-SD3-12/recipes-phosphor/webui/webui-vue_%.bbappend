FILESEXTRAPATHS:append := ":${THISDIR}/${PN}"
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://0001-Add-sophgo-custom-features-to-webui-vue.patch "
SRC_URI:append = " file://0002-Adapt-to-upstream-webui-vue-webpack4-vue-cli-4.5.19.patch "
