---
name: openbmc-upstream-patch-sync
description: Fix all downstream patches after rebasing an OpenBMC layer onto a newer upstream. Use when the user says things like "同步上游后 patch 打不上了", "rebase 完 openbmc 帮我修 patch", "bitbake 镜像构建挂了", "do_patch 失败", "升级 openbmc 上游后编译错误", or any time a Yocto/OpenBMC layer's patches no longer apply or compile after an upstream rebase. Covers patch triage, devtool-based patch rewriting, full-image build fallback, and the common QA/usrmerge/compiler pitfalls that only surface during a full image build.
---

# OpenBMC Upstream Patch Sync

After a downstream OpenBMC layer is rebased onto a newer upstream, the layer's
patches often stop applying or compiling. This skill walks through fixing all of
them, from initial triage through full-image build validation.

The process has five phases: **survey, per-patch triage, devtool rewrite,
full-image build fallback, wrap-up**. Work through them in order. Only stop and
ask the user when you hit an error you cannot resolve or a decision with real
risk (dropping a patch, changing a hardware protocol, deleting a recipe).

## Core principle

**The goal is working functionality on the new upstream, not preserving patches
for their own sake.** A patch is a means, not an end. When upstream already
implements the same thing, drop our patch. When upstream changed the code but
not the intent, rewrite our patch. When the feature is no longer needed, drop it
after confirming with the user.

Never pin SRCREV or freeze a version to make an old patch keep working — that
defeats the purpose of tracking upstream.

## Phase 1: Survey

Before touching anything, map the damage:

1. Find all patch files referenced by bbappends in the downstream layer:
   ```bash
   find meta-*/recipes-* -name "*.bbappend" -exec grep -l "\.patch" {} \;
   ```
2. Group them by component. For each component, run a quick patch check:
   ```bash
   cd <build-dir> && bash -c 'source ../oe-init-build-env . >/dev/null 2>&1; bitbake -c patch <component> 2>&1 | tail -5'
   ```
3. Produce a table: component, number of patches, applies/fails, likely cause
   (context drift / upstream rewrote / upstream removed / QA issue).
4. Present the table to the user with a suggested processing order (most patches
   first, since those components usually carry the most custom functionality).

Components with 0-1 patches are easy to overlook in this phase but will surface
in Phase 4 — note them anyway.

## Phase 2: Per-patch triage decision tree

For each patch that fails to apply, decide:

- **Upstream already covers it** → drop the patch. Verify by finding the
  equivalent implementation in upstream source and cite the upstream commit hash
  in your reasoning. This is the best outcome — one less thing to maintain.
- **Upstream rewrote the code but the intent still applies** → rewrite the patch
  using the devtool workflow (Phase 3). Read the upstream code carefully — the
  function may have been restructured, but the logic you need to change is
  usually still there.
- **Upstream barely touched it, just line numbers shifted** → refresh the patch
  context (update hunk offsets). This is the simplest case.
- **The feature is no longer needed** → confirm with the user, then drop. Common
  examples: hardware-specific workarounds for silicon revisions no longer
  shipped, tools that upstream now ships itself, recipes for platforms the
  product no longer uses.

When a patch modifies a **hardware protocol** (bus framing, register layouts,
wire formats), treat it as high-risk: read the entire upstream function, not
just the hunks the old patch touched. Upstream may have added new code paths
that fall into the same trap the patch was fixing (e.g., a new timeout handler
that writes the same malformed packet the patch was correcting).

## Phase 3: devtool workflow for rewriting patches

Use devtool to rewrite patches on actual upstream source. Do not hand-edit
patch files unless it's a pure context-line refresh — devtool gives you a real
build to verify against.

### Setup

1. Temporarily remove the failing patch lines from the bbappend (keep the file,
   just comment out or delete the `SRC_URI:append` lines for broken patches).
2. If the bbappend has custom tasks (e.g., `do_add_files`, `do_create_*`) that
   copy files into the source tree, comment those out too — they often conflict
   with devtool's externalsrc mechanism.
3. Run:
   ```bash
   cd <build-dir> && bash -c 'source ../oe-init-build-env . >/dev/null 2>&1; devtool modify <component>'
   ```
   Source lands in `<build-dir>/workspace/sources/<component>/`.

### Edit and verify

4. Make the needed changes directly in the workspace source tree.
5. Build to verify:
   ```bash
   devtool build <component>
   ```
   Fix compile errors iteratively. Common issues after an upstream bump:
   - Logging API changes (e.g., stream-style `LOG << "msg"` → function-style
     `LOG("msg")`)
   - Header reorganization (missing includes)
   - Stricter compiler warnings (see Phase 4 notes on `-Werror`)
   - API signature changes in libraries the component links against

### Export

6. Commit in the workspace git repo, then export:
   ```bash
   cd workspace/sources/<component>
   git add -A && git commit -m "<description>"
   git format-patch -1 -o /tmp/out/
   ```
7. Copy the exported patch back to the layer's patch directory, update the
   bbappend to reference it (restore the line you removed in step 1, pointing
   at the new file). If you commented out custom tasks in step 2, evaluate
   whether they're still needed — often the files they copied can simply be
   committed into the patch itself, eliminating the custom task entirely.
8. Clean up:
   ```bash
   devtool reset <component>
   rm -rf workspace/appends/<component>*
   ```
   **Do not use `devtool update-recipe`** — it writes the patch into the
   upstream public recipe (.bb file), polluting upstream. And **do not use
   `devtool finish`** without manually checking `workspace/appends/` afterwards
   — leftover bbappends there with `inherit externalsrc` will break subsequent
   bitbake runs for that component with confusing "task do_patch not found"
   errors.
9. Final check:
   ```bash
   bitbake -c patch <component>
   bitbake -c compile <component>
   ```

## Phase 4: Full-image build fallback

After all known components are fixed, run the full image build. This is the
safety net that catches what Phase 1 missed:

```bash
bitbake obmc-phosphor-image
```

It will fail. That's expected. Each failure exposes one previously-hidden issue.
Fix them one at a time and re-run. The failure categories you'll see:

### Patch application failures (do_patch)
- **QA: Missing Upstream-Status** — newer Yocto requires every patch to declare
  its upstream status. Add a header like:
  ```
  Upstream-Status: Inappropriate [downstream specific workaround]
  ```
  or `Upstream-Status: Backport [upstream-commit-hash]` as appropriate.
- **QA: Fuzz detected** — the patch applies with fuzz 1-2. Newer Yocto treats
  fuzz as an error. Refresh the patch context.

### Compile failures (do_compile)
- **Stricter compiler** — a newer GCC (e.g., 13.x) turns previously-tolerated
  patterns into `-Werror` failures. `-Werror=maybe-uninitialized` is the most
  common. Fix the code, don't suppress the warning.
- **Library API changes** — e.g., openssl 1.1 → 3.x deprecates functions. If a
  downstream recipe pins an old library version to avoid this, remove the pin
  and fix the consumer code instead.

### Packaging failures (do_package / do_package_qa)
- **usrmerge violation** — scarthgap+ requires `/lib` → `/usr/lib`. Any recipe
  or CMakeLists that installs to `/lib/...` must be changed. Use
  `${nonarch_base_libdir}` for firmware/udev rules, `${systemd_system_unitdir}`
  for systemd units. If the upstream build system hardcodes `/lib`, add a
  `do_install:append` that moves the files to the correct location.
- **installed-vs-shipped** — a file was installed but not listed in `FILES`.
  Usually caused by the install path changing (e.g., usrmerge) so the old
  `FILES` entry no longer matches. Update `FILES` to the new path.

### Rootfs failures (do_rootfs)
- **File conflicts** — two packages install the same file. opkg reports
  "package debris" at do_rootfs time, not do_install. Find which two recipes
  install the file, decide which should own it, and remove it from the other
  (usually by dropping it from a packagegroup or deleting a redundant recipe).
- **License/SPDX** — vague `LICENSE = "BSD"` is rejected by the SPDX generator.
  Use the exact variant: `BSD-3-Clause`, `BSD-2-Clause`, etc. Check the actual
  LICENSE file in the source to determine which.

## Phase 5: Wrap-up

1. Commit all layer changes on the rebase branch with a comprehensive commit
   message covering what was fixed, what was dropped, and why.
2. Report to the user: branch structure, per-component status table, what was
   dropped and why, what still needs hardware validation.
3. Remind the user about any stashed work on other branches that predates the
   rebase.

## Interaction policy

Work autonomously through the phases. Only stop and ask the user when:

- You hit the same error twice in a row with different attempted fixes
- You're about to drop a patch or recipe whose purpose isn't obviously obsolete
- You're about to change a hardware protocol implementation
- The build environment itself is broken (missing tools, network failures)

For routine decisions (context refresh, Upstream-Status headers, usrmerge path
fixes, LICENSE clarification, devtool workflow steps), just do them and report
afterwards.

## Communication style

Report progress in the user's language. When you finish a component, briefly
say what was done and why. When the full image build finally passes, give a
summary table of everything that was fixed. Cite file paths as
`path/to/file:line` so the user can navigate.

## Reference files

- `references/common-pitfalls.md` — detailed catalog of every failure mode seen
  in practice, with the exact error message and the fix. Consult when you hit
  an error not covered above.
