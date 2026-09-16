# Common Pitfalls — OpenBMC Upstream Patch Sync

Catalog of every failure mode encountered in practice, with the exact error
message and the fix. Organized by the bitbake task where the failure appears.

## Table of contents

- [do_patch failures](#do_patch-failures)
- [do_compile failures](#do_compile-failures)
- [do_configure failures](#do_configure-failures)
- [do_package / do_package_qa failures](#do_package--do_package_qa-failures)
- [do_rootfs failures](#do_rootfs-failures)
- [devtool-specific traps](#devtool-specific-traps)
- [Build environment issues](#build-environment-issues)

---

## do_patch failures

### QA: Missing Upstream-Status

**Error:** `QA Issue: Missing Upstream-Status in patch`

Newer Yocto (scarthgap+) requires every patch to declare whether it's been sent
upstream, is a backport, or is downstream-only. Add a header to the patch file:

```
From: Name <email>
Date: ...
Subject: [PATCH] description

Upstream-Status: Inappropriate [downstream specific workaround]
Signed-off-by: Name <email>
---
```

Valid values: `Pending`, `Backport [commit-hash]`, `Submitted [link]`,
`Inappropriate [reason]`, `Denied [reason]`.

### QA: Fuzz detected

**Error:** `QA Issue: Fuzz detected` followed by `Hunk #N succeeded at M with fuzz F (offset O lines)`

The patch applies but with fuzz. Newer Yocto treats this as an error. Fix by
refreshing the patch context — the line numbers in the hunk headers need to
match the current upstream source. Use `devtool modify` to apply with fuzz,
then `git format-patch` to export a clean patch.

### Patch does not apply

**Error:** `Hunk #N FAILED at M` / `rejects in file <file>`

The upstream code changed too much. Use the devtool workflow (Phase 3 in
SKILL.md): remove the patch from bbappend, `devtool modify`, make the changes
in the workspace source, build, export, restore bbappend.

---

## do_compile failures

### -Werror=maybe-uninitialized

**Error:** `error: '<var>' may be used uninitialized [-Werror=maybe-uninitialized]`

Newer GCC (13.x) has improved static analysis and catches cases older GCC
missed. The variable needs an initializer. Do not suppress the warning — fix
the code by initializing the variable at declaration.

### -Werror=format-security

**Error:** `error: format not a string literal and no format arguments [-Werror=format-security]`

Usually triggered by `printf(variable)` instead of `printf("%s", variable)`.
Fix by adding a format string.

### Deprecated library functions

**Error:** `error: '<function>' is deprecated: Since OpenSSL 3.0 [-Werror=deprecated-declarations]`

The recipe or the code it compiles uses functions deprecated in the new library
version. Two options: (a) update the code to use the modern replacement, (b) if
a downstream recipe pins an old library version, remove the pin and update the
consumer code. Option (b) is almost always the right answer — the pin was
usually a shortcut that blocks all other components from using the new library.

### Logging API changes

**Error:** Various errors involving `BMCWEB_LOG_DEBUG <<`, `lg2::info <<`, etc.

Upstream switched from stream-style logging to function-style. Change:
- `BMCWEB_LOG_DEBUG << "msg " << var;` → `BMCWEB_LOG_DEBUG("msg {}", var);`
- `res.body() = json.dump();` → `res.write(json.dump());`
- `res.addHeader("Content-Type", "...")` → `res.addHeader(boost::beast::http::field::content_type, "...")`

---

## do_configure failures

### gcc-plugins incompatibility

**Error:** Kernel build fails with `gsi_iterator_update` or similar gcc-plugin errors

The kernel's gcc-plugins framework is incompatible with the newer GCC version.
Disable in the defconfig:
```
# CONFIG_GCC_PLUGINS is not set
```

### CMake version too new/old

**Error:** `CMake Error: Compatibility with CMake < X.Y will be removed`

The recipe's CMakeLists.txt uses a deprecated minimum version. Add to the
recipe: `OECMAKE_EXTRA = "-DCMAKE_POLICY_VERSION_MINIMUM=X.Y"` or patch the
CMakeLists.txt.

---

## do_package / do_package_qa failures

### usrmerge violation

**Error:** `QA Issue: <pkg> package is not obeying usrmerge distro feature. /lib should be relocated to /usr. [usrmerge]`

The recipe installs files to `/lib/...` but usrmerge requires `/usr/lib/...`.

For recipes with `do_install`:
```bitbake
do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware
    # NOT ${D}/lib/firmware or ${D}${baselib}/firmware
}
```

For upstream build systems that hardcode `/lib` (e.g., CMakeLists with
`DESTINATION /lib/systemd/system/`):
```bitbake
do_install:append() {
    if [ -d ${D}/lib/systemd/system ]; then
        install -d ${D}${systemd_system_unitdir}
        mv ${D}/lib/systemd/system/*.service ${D}${systemd_system_unitdir}/
        rm -rf ${D}/lib/systemd
        rmdir ${D}/lib 2>/dev/null || true
    fi
}
```

Key variables: `${nonarch_base_libdir}` = `/usr/lib` (firmware, udev rules),
`${systemd_system_unitdir}` = `/usr/lib/systemd/system` (systemd units).

### installed-vs-shipped

**Error:** `QA Issue: Files/directories were installed but not shipped in any package: /usr/lib/systemd/system/foo.service`

A file was installed but not listed in `FILES`. Usually caused by an install
path change (usrmerge) making the old `FILES` entry stale. Update:
```bitbake
# Old (broken):
FILES:${PN} += "/lib/systemd/system/foo.service"
# New:
FILES:${PN} += "${systemd_system_unitdir}/foo.service"
```

---

## do_rootfs failures

### Package debris / file conflict

**Error:** `To remove package debris, try 'opkg remove <pkg>'.`

Two packages install the same file. opkg only detects this at do_rootfs time.
To find the conflict: look at the ipk contents of the failing package
(`ar p <pkg>.ipk data.tar.zst | zstd -d | tar tf`) and check which files already
exist in the rootfs from another package (`grep -l <filename> rootfs/var/lib/opkg/info/*.list`).

Fix: remove the redundant recipe or drop the conflicting item from a
packagegroup's RDEPENDS.

### LICENSE too vague for SPDX

**Error:** `do_create_spdx: Cannot find any text for license BSD`

The SPDX generator needs an exact license variant. Check the actual LICENSE
file in the source to determine which BSD variant it is:
```bitbake
LICENSE = "BSD-3-Clause"  # not "BSD"
```

---

## devtool-specific traps

### workspace/appends leftover

**Symptom:** After `devtool finish`, `bitbake -c patch <component>` says
`task do_patch not found` or similar.

Cause: `workspace/appends/<component>.bbappend` with `inherit externalsrc` is
still active. Fix:
```bash
devtool reset <component>
rm -rf workspace/appends/<component>*
```

### devtool update-recipe pollutes upstream

**Symptom:** The upstream recipe (`.bb` file in `meta-phosphor/`) now has your
patches in `SRC_URI`.

Cause: used `devtool update-recipe` or `devtool finish` with the wrong layer
argument. Fix: `git checkout -- meta-phosphor/recipes-*/<component>/` to restore
the upstream recipe, then redo with manual `git format-patch` + copy to your
layer.

### Custom tasks conflict with devtool modify

**Symptom:** `devtool modify <component>` fails during do_patch or do_unpack.

Cause: the bbappend has a custom task (e.g., `do_add_files`, `do_create_*_dir`)
that copies files into the source tree, conflicting with devtool's externalsrc.
Fix: temporarily comment out the custom task and its `addtask` line from the
bbappend, run devtool modify, then after finishing, evaluate whether the custom
task is still needed (often the files it copied can be committed into the
patch itself).

---

## Build environment issues

### TEMPLATECONF path broken after rebase

**Symptom:** `Error: TEMPLATECONF value points to nonexistent directory`

Cause: `conf/templateconf.cfg` in the build directory references a path that
moved during rebase (e.g., `meta-poky/conf/templates/default` → `poky/meta-poky/conf/templates/default`).

Fix: either update the file, or pass explicitly:
```bash
export TEMPLATECONF=/path/to/meta-xxx/conf/templates/default
source oe-init-build-env <build-dir>
```

### bblayers.conf layer paths broken

**Symptom:** `WARNING: No bb files in default matched BBFILE_PATTERN_*`

Cause: layer paths in `conf/bblayers.conf` reference directories that moved
during rebase (e.g., `meta-poky` → `poky/meta-poky`).

Fix: update `bblayers.conf` paths. Also check `LAYERSERIES_COMPAT_*` in each
layer's `conf/layer.conf` — after a rebase the release name may have changed
(e.g., `langdale` → `scarthgap`).
