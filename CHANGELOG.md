# CHANGELOG



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
