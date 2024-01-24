# CHANGELOG



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
