# Ada-CPP

A drop-in replacement library for adapy created to improve performance using c++.
The compilation and binding methods are based on the [nanobind-minimal repo](https://github.com/Krande/nanobind-minimal)

## Performance metrics
Todo

## Installation

First install the pre-requisites for both occt and nanobind + build requirements from conda-forge.

```bash
mamba env update -f environment.build.yml --prune
```

Activate the environment and install the package in editable mode.

```bash
pip install --no-build-isolation .
```

### Conda Build install

Installing as conda package

```bash
mamba mambabuild . -c conda-forge --python 3.11 --override-channels
mamba install --use-local ada-cpp
```


