# CHANGELOG



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
