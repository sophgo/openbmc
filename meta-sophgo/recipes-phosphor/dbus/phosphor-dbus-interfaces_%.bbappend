FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += "file://0028-MCTP-Daemon-D-Bus-interface-definition.patch \
            file://0032-update-meson-build-for-MCTP-interfaces.patch \
            file://0033-sophgo-software-version-interface.patch \
            file://0034-sophgo-software-meson.patch \
            file://0035-Device-errors-yaml.patch \
            file://Updating/meson.build \
            file://Updating.errors.yaml;subdir=git/yaml/xyz/openbmc_project/Software/  \
            "




do_create_Updating_dir() {
    echo "---${WORKDIR}---${S}---${THISDIR}---${FILE}---${THISDIR}/${PN}---${CURRENT_FILE_DIR}---"
    install -d ${S}/gen/xyz/openbmc_project/Software/Updating
    cp ${WORKDIR}/Updating/meson.build ${S}/gen/xyz/openbmc_project/Software/Updating/
}
addtask do_create_Updating_dir after do_unpack before do_patch