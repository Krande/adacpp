# Ada-CPP

A drop-in replacement library for adapy created to improve performance using c++.
The compilation and binding methods are based on the [nanobind-minimal repo](https://github.com/Krande/nanobind-minimal)


## Installation

First install the pre-requisites for both occt and nanobind + build requirements from conda-forge.

```bash
mamba env update -f environment.build.yml --prune
```

Activate the environment and install the package in editable mode.

```bash
pip install --no-build-isolation -e .
```

### Conda Build install

Installing as conda package

```bash
mamba mambabuild . -c conda-forge --python 3.11 --override-channels
mamba install --use-local ada-cpp
```

### Local IDE development

You can use the presets in the CMakePresets.json file. 
But first you must create a `.env.json` file (which will be ignored by git) where you point to 
the conda env `environment.build.yml`. The .env.json file should look like this,
where you fill in the path to your conda env as the "PREFIX" value.

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "env-vars",
      "hidden": true,
      "environment": {
        "PREFIX": "C:/miniforge3/envs/ada-cpp"
      }
    }
  ]
}
```


## Performance metrics
Todo
