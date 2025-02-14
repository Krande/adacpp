# Ada-CPP

A drop-in replacement library for adapy created to improve performance using c++.
The compilation and binding methods are based on the [nanobind-minimal repo](https://github.com/Krande/nanobind-minimal)


## Installation

Adacpp uses [pixi.sh](https://pixi.sh/latest/) to handle dependencies and building the project.

### Conda Build install

Build and test conda package

```bash
pixi run build
```

### Local IDE development

The presets in the CMakePresets.json file are set to use the pixi `build` environment. So for local development, you can use the following commands:

```bash
pixi install -e build
``` 

## Performance metrics
Todo
