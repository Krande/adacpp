# CHANGELOG



## v0.30.0 (2026-09-24)

### Feature

* feat(wasm): the IFC member scan reaches the browser as JSONL (#67)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`9ed92fe`](https://github.com/Krande/adacpp/commit/9ed92fef94320e83aafbae3d110228900f6bee72))


## v0.29.0 (2026-09-24)

### Feature

* feat(cad): IfcMemberScan reports each member&#39;s material (#66)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`b7909b0`](https://github.com/Krande/adacpp/commit/b7909b004959315aef460d0281902511dee01fec))


## v0.28.0 (2026-09-23)

### Feature

* feat(cad): read_step_shapes applies an import-time transform (#65)

Co-authored-by: Claude Opus 5.5 (1M context) &lt;noreply@anthropic.com&gt; ([`5232dea`](https://github.com/Krande/adacpp/commit/5232deae4c6754bc456f4c168ff28fc5db2b00fe))


## v0.27.0 (2026-09-23)

### Feature

* feat(ifc): the member scan carries the swept area&#39;s outline and plane (#64) ([`e990f1e`](https://github.com/Krande/adacpp/commit/e990f1e39c4eb1917f9dea9b2efd2237808bb0ca))


## v0.26.0 (2026-09-23)

### Feature

* feat(ifc): read what a product IS, without tessellating it (#63) ([`fd769fe`](https://github.com/Krande/adacpp/commit/fd769fe4f0141a7c3773d5c5efb0281569b3f000))


## v0.25.5 (2026-09-23)

### Fix

* fix(cad): face_id keys on TShape AND Location, matching IsSame (#62)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`a77d412`](https://github.com/Krande/adacpp/commit/a77d412d6203eb27ae0588b89906a79948a39aeb))


## v0.25.4 (2026-09-23)

### Fix

* fix(cad): edges() reports each edge once, and keeps located copies apart (#61)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`85277d5`](https://github.com/Krande/adacpp/commit/85277d50c3f571a6edc58235b25780b5840f3a86))


## v0.25.3 (2026-09-22)

### Fix

* fix(build): export only PyInit from the native extension (#60)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`3ff38b0`](https://github.com/Krande/adacpp/commit/3ff38b042499b0a0a214e0aff48a2c0b55378177))


## v0.25.2 (2026-09-21)

### Fix

* fix(ci): derive the release asset list instead of restating it (#59)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`3709f18`](https://github.com/Krande/adacpp/commit/3709f182a1ef00145718e44826ac6f108a8c2b5e))


## v0.25.1 (2026-09-21)

### Fix

* fix(occt): keep the global mesher thread pool usable across fork() (#58)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`68ec39f`](https://github.com/Krande/adacpp/commit/68ec39f651b00334b59b67ab9980b6710ca1ac2f))


## v0.25.0 (2026-09-21)

### Feature

* feat(cad): replay a stored section table, and sweep without analysis data (#57)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`b7c5887`](https://github.com/Krande/adacpp/commit/b7c588755920a307c33f007b2197d401efad6608))


## v0.24.0 (2026-09-21)

### Feature

* feat(ifc): stream_ifc_to_glb(include_guids=...) for subset streaming (#56)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`56a8701`](https://github.com/Krande/adacpp/commit/56a8701dcca68c87744716c143e801d9cf1672cc))


## v0.23.0 (2026-09-21)

### Chore

* chore(ci): put both deputy pins on v0.8.1, and correct the stale one (#54)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`6edcbf0`](https://github.com/Krande/adacpp/commit/6edcbf00f02afd6d95c381690ebb67b3e5e260dc))

### Feature

* feat: prismatic extrusion core, curved-face imprint, grid-fitted B-spline faces (#55)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`e27d83c`](https://github.com/Krande/adacpp/commit/e27d83c6139a7b4306f53b1514d09502aa29ec47))


## v0.22.0 (2026-09-17)

### Chore

* chore: bump nanobind to 3.0.1, cmake to 4.4.3, tinygltf to 2.9.1 (#50)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`c09b1a3`](https://github.com/Krande/adacpp/commit/c09b1a3bd277343b2e352918e3a439f6e7f0fd7b))

* chore: bump deputy pin to v0.5.6 (#51)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`cbe2ff1`](https://github.com/Krande/adacpp/commit/cbe2ff11264c840d7498a85510991135038f2133))

### Feature

* feat: Oblique tapered solids, and a torus builder (#53)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`97419cb`](https://github.com/Krande/adacpp/commit/97419cb9b82c39f8dc5ffcb4f1b18bec9ff0dac6))

* feat: wasm OCCT to V8_0_0, lint tooling bumps, setup-pixi 0.10.2 (#52)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`9a70961`](https://github.com/Krande/adacpp/commit/9a709618f5f0771a33bd2cb4a5c028b37e558a11))


## v0.21.0 (2026-09-04)

### Chore

* chore: migrate CI from action-toolbox to deputy (#49)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`43580ee`](https://github.com/Krande/adacpp/commit/43580ee3783423f11778fc83dc220957c3f7e54f))

### Feature

* feat: build against OCCT 8.0.0 (#48)

Co-authored-by: Claude Opus 5 (1M context) &lt;noreply@anthropic.com&gt; ([`fdfeb02`](https://github.com/Krande/adacpp/commit/fdfeb0268d9171afc832c7ba14caee186b73cce8))


## v0.20.0 (2026-08-07)

### Feature

* feat: native is_planar_face verb for loft face parity (#46)

Co-authored-by: Claude Opus 4.8 &lt;noreply@anthropic.com&gt; ([`2b824f5`](https://github.com/Krande/adacpp/commit/2b824f528c2f5970a2266eecb44b121398394e71))


## v0.19.0 (2026-07-26)

### Feature

* feat: weld thick curved-shell cap-wall seams via boundary-first CDT (#45)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`3d321fc`](https://github.com/Krande/adacpp/commit/3d321fc311a45f4685d36bb2e2ad90e410934f6d))


## v0.18.0 (2026-07-20)

### Feature

* feat: AP242 mapped instancing and closed-B-spline tessellation fixes (#44)

Co-authored-by: Claude Fable 5 &lt;noreply@anthropic.com&gt; ([`1192c30`](https://github.com/Krande/adacpp/commit/1192c307dbcdb57af7ddddd610cf8c000bf78911))


## v0.17.1 (2026-07-18)

### Fix

* fix: keep faces whose inner (hole) loop is empty in the B-rep emitters (#43)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`b7135e4`](https://github.com/Krande/adacpp/commit/b7135e423aa2b5c92a454381a144b02f87f814a8))


## v0.17.0 (2026-07-17)

### Feature

* feat: attach OCC-free wasm modules to releases + adacpp CLI with STEP/IFC conversion paths (#42)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`0805d9d`](https://github.com/Krande/adacpp/commit/0805d9d01b6f7a694003a4cce11a51f08fc6f450))


## v0.16.1 (2026-07-17)

### Fix

* fix: per-face STEP colours without --face-regions, plus CLI positional args + pipeline validation (#41)

Co-authored-by: Claude Opus 4.8 &lt;noreply@anthropic.com&gt; ([`aada2a1`](https://github.com/Krande/adacpp/commit/aada2a1066c62674f51c3ce37aadba0a067d35c2))


## v0.16.0 (2026-07-17)

### Feature

* feat: native OCC-free STEP/IFC to GLB with tessellation tracks and colour/curved-surface fidelity (#40)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`3e5f9fc`](https://github.com/Krande/adacpp/commit/3e5f9fc4fd5a087f7a4a8d8f6763854b68d98225))


## v0.15.0 (2026-07-15)

### Feature

* feat: Add imprint_planar_faces; fix the win-64 build and the cad API generator&#39;s encoding (#38) ([`ac3f0fa`](https://github.com/Krande/adacpp/commit/ac3f0fa703ecab58570ec5700d74be9bbea63d14))


## v0.14.0 (2026-07-15)

### Feature

* feat: selectable tessellation tracks — boundary pinning by default + a watertight CDT track (#39)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`3095175`](https://github.com/Krande/adacpp/commit/309517567fd6ea9354ccceddc378e6cd92ac430e))


## v0.13.2 (2026-07-14)

### Fix

* fix: guard file-based glb_diff binding out of the wasm build (#37)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`d46bdd9`](https://github.com/Krande/adacpp/commit/d46bdd925f350a17693a3f3f3f03ab114fad566b))


## v0.13.1 (2026-07-13)

### Fix

* fix: put json.hpp on the pyodide + glbdiff wasm include path (#36)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`6dcd182`](https://github.com/Krande/adacpp/commit/6dcd1824ae9dc5e3697e76add3f4f855dd6f7950))


## v0.13.0 (2026-07-13)

### Feature

* feat: pure-C++ IFC reader/writer, zero-copy NGEOM streams, tessellation perf (#35)

Co-authored-by: Claude Fable 5 &lt;noreply@anthropic.com&gt; ([`d0ba3a6`](https://github.com/Krande/adacpp/commit/d0ba3a60773063525c6349a059346097396d9180))


## v0.12.0 (2026-07-03)

### Feature

* feat: native IFC4x3 alignment swept-area solid (no OCC) (#33)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: Claude Opus 4.8 &lt;noreply@anthropic.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`037d026`](https://github.com/Krande/adacpp/commit/037d026323d02fdb4cf6a79cf21cda2c8ed64011))


## v0.11.0 (2026-06-30)

### Feature

* feat: native dep-free STEP/IFC geometry pipeline (OCC-free reader, writers, full ng:: coverage) (#32)

Co-authored-by: Claude Opus 4.8 &lt;noreply@anthropic.com&gt; ([`1e7ee04`](https://github.com/Krande/adacpp/commit/1e7ee041d02214d29036ed14a04745e952ebe078))


## v0.10.1 (2026-06-25)

### Fix

* fix: OCC AdvancedFace build for closed-revolution + near-planar faces (#31)

Co-authored-by: Claude Opus 4.8 &lt;noreply@anthropic.com&gt; ([`fc3e084`](https://github.com/Krande/adacpp/commit/fc3e0847d100f8837a9cba2d8070a4d367931486))


## v0.10.0 (2026-06-24)

### Feature

* feat: OCC-free NGEOM geometry layer + libtess2 tessellator + ifcopenshell taxonomy kernel (#30)

Co-authored-by: Claude Opus 4.8 &lt;noreply@anthropic.com&gt; ([`d6b89cd`](https://github.com/Krande/adacpp/commit/d6b89cddeccd5005eab0f41eec40dbe2aa0ec1df))


## v0.9.0 (2026-06-15)

### Feature

* feat: wasm conversion engine — STEP write, OCCT isolation, IFC serialize (pyodide 0.29.4) (#29)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`d7948e7`](https://github.com/Krande/adacpp/commit/d7948e754c52224c32069c152c86d1c3d36e1dba))


## v0.8.0 (2026-06-14)

### Feature

* feat: add build_swept_disk_solid for IfcSweptDiskSolid parity (#27)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`cb36bb3`](https://github.com/Krande/adacpp/commit/cb36bb3a1a9ed2e16193910b24000f3e253b1fd4))


## v0.7.0 (2026-06-08)

### Feature

* feat: cylindrical/conical/toroidal AdvancedFace builders (#26)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`90b4247`](https://github.com/Krande/adacpp/commit/90b4247e5bceac14594acdcc93c982cbebeb8a17))


## v0.6.0 (2026-06-07)

### Feature

* feat: sew_faces + ShapeFix p-curve tessellation retry (#25)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`642d5f2`](https://github.com/Krande/adacpp/commit/642d5f2f2d7d7223cd75495d908a5f0d1b6dc03f))


## v0.5.0 (2026-06-05)

### Feature

* feat: zero-copy NumPy mesh buffers + batch tessellation (#24)

Co-authored-by: Claude Opus 4.8 &lt;noreply@anthropic.com&gt; ([`9027eb0`](https://github.com/Krande/adacpp/commit/9027eb0cc6b223a609d6942e5d9b8024e60baefe))


## v0.4.2 (2026-06-03)

### Chore

* chore: publish pyodide wasm base image to ghcr on tag (#22)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`49bce65`](https://github.com/Krande/adacpp/commit/49bce65aeb642e53977fed6ba32f711ad4077165))

### Fix

* fix: bump setup-pixi to v0.9.6 across CI workflows (#23)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`7262ad4`](https://github.com/Krande/adacpp/commit/7262ad4c0fb394bb0985512a9d58ec6d0b43427f))


## v0.4.1 (2026-06-03)

### Fix

* fix: cad backend parity (#21)

Co-authored-by: Claude Opus 4.8 (1M context) &lt;noreply@anthropic.com&gt; ([`b2c63dc`](https://github.com/Krande/adacpp/commit/b2c63dc2e411fdfb449d1fc100d98e4eaca68645))


## v0.4.0 (2026-06-02)

### Feature

* feat: cad handler (#20) ([`e32f97d`](https://github.com/Krande/adacpp/commit/e32f97dd2434ff39775a3024bf84e270d4c4fc81))


## v0.3.0 (2026-04-30)

### Chore

* chore: bump the dependencies group with 2 updates (#18)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`9d60d04`](https://github.com/Krande/adacpp/commit/9d60d0420184b3e7e224abb8676f2d66e61dae86))

### Feature

* feat: Remove scikit-build-core in favor of using only cmake (#17)

Co-authored-by: Claude Opus 4.7 (1M context) &lt;noreply@anthropic.com&gt; ([`6bb17d8`](https://github.com/Krande/adacpp/commit/6bb17d839e411e836bfa2ec734290225db1ea082))


## v0.2.1 (2025-11-28)

### Chore

* chore: bump prefix-dev/setup-pixi from 0.9.1 to 0.9.2 in the dependencies group (#15)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`cb9fac3`](https://github.com/Krande/adacpp/commit/cb9fac36f22ec5be3fb82338af6916a3a9258ca4))

* chore: bump prefix-dev/setup-pixi from 0.9.0 to 0.9.1 in the dependencies group (#14)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`d42c93a`](https://github.com/Krande/adacpp/commit/d42c93acd9810c84ed8ebc08da146b9029dd02e7))

* chore: bump the dependencies group with 2 updates (#13)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`07cd9bf`](https://github.com/Krande/adacpp/commit/07cd9bf3fba0a8d89e3c56f1b4b80c71aac109d9))

* chore: bump prefix-dev/setup-pixi from 0.8.10 to 0.8.14 in the dependencies group (#12)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`993c351`](https://github.com/Krande/adacpp/commit/993c3514642fa37414e1e3c4c555df0e002b8d44))

* chore: bump prefix-dev/setup-pixi from 0.8.8 to 0.8.10 in the dependencies group (#11)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`cda452a`](https://github.com/Krande/adacpp/commit/cda452ad48180f835a2058564e98c17482f6ef7f))

* chore: bump prefix-dev/setup-pixi from 0.8.3 to 0.8.8 in the dependencies group (#10)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`a1aa818`](https://github.com/Krande/adacpp/commit/a1aa8180b1961fcd0cbd07bb6d1fe4d889ab62bd))

* chore: bump prefix-dev/setup-pixi from 0.8.2 to 0.8.3 in the dependencies group (#9)

Signed-off-by: dependabot[bot] &lt;support@github.com&gt;
Co-authored-by: dependabot[bot] &lt;49699333+dependabot[bot]@users.noreply.github.com&gt; ([`02c5305`](https://github.com/Krande/adacpp/commit/02c5305d477baba3e8a84a3b150283474f74f9a6))

### Fix

* fix: improve GMSH library detection and enhance STEP file reader para… (#16) ([`7c55a3a`](https://github.com/Krande/adacpp/commit/7c55a3ac5f92ffca984e4bd3b1595937c94930df))


## v0.2.0 (2025-02-14)

### Feature

* feat: Add test wasm output and major refactor (#8) ([`276e3c9`](https://github.com/Krande/adacpp/commit/276e3c9902948b8912aea2ce6f3f95cdecffeba4))


## v0.1.1 (2024-09-04)

### Fix

* fix: ifco v08 support (#6) ([`539d503`](https://github.com/Krande/adacpp/commit/539d5037af0bdbee8ab3fcc407bb9a5e8369d14d))

### Unknown

* remove hardcoded osx sysroot and deployment target ([`03aef29`](https://github.com/Krande/adacpp/commit/03aef299b4a61103a92440b1b10c9ee6625f8544))

* update conda build variant name ([`17a2a88`](https://github.com/Krande/adacpp/commit/17a2a88f19fc872ade4e557c2ab9636ec17a161e))


## v0.1.0 (2024-01-24)

### Chore

* chore: Update pr-review.yaml (#3) ([`704fcf8`](https://github.com/Krande/adacpp/commit/704fcf8a97bf61342ead3047c5a7f703d111600b))

### Feature

* feat: New CI tooling, local dev setup and stp to glb parser (#4) ([`a2df512`](https://github.com/Krande/adacpp/commit/a2df5129f8cce0b711627a5535dbf93e720a4df5))

### Unknown

* update ci tooling ([`e89631a`](https://github.com/Krande/adacpp/commit/e89631aa199fff4ededcc62dde8ed46ea0376ae0))

* Fix/ci conda (#1)

* keep work dir to ensure that conda build succeeds

* minor change in ci workflow name

* start to work on fixing unix build

* fix failing linux compilations

* attempt to fix macos issues

* try with a newer osx ci runner

* try forcing the minimum osx version in the CMakeLists.txt file

* set the ci runner version to 10.14 in hopes of better availability

* try listing all available osx sdk&#39;s on runner

* set osx sysroot in conda_build_config.yaml

* add a extra ci file

* try downloading the sdk explicitly

* try downloading the sdk explicitly

* make another attempt to fix osx by manually downloading osx sdk

* set osx target before project definition

* try to use pyproject.toml for all osx version tagging

* try setting the minimum in the CMakeLists.txt

* try setting in the pyproject.toml file

* add extra checks in the CMakeLists.txt

* dp not override the cxx flags in the pyproject.toml

* try setting the args in the conda_build_config.yaml

* override sysroot and deployment target inside CMakeLists.txt

* bump minimum osx sdk to 10.15

* fix missing vector3 reference ([`e6f6112`](https://github.com/Krande/adacpp/commit/e6f6112d8422e6dc0ebd0a76f8ff9de40d742c73))

* initial commit ([`5764ab0`](https://github.com/Krande/adacpp/commit/5764ab06cf28f579d2b390e13647e801b2ca11c1))
