# gr-shader-language-server
Language server implementation for GLSL

## Status
Currently, this LSP implementation can be interfaced with using either TCP or stdio.

### Current Features

- Diagnostics
- Completion
- Hover
- Jump to def

### Planned Features

- Workspace symbols
- Find references

## Compile

To build this project, you just need Git, a valid C++20 compiler, CMake, and Ninja.

This project uses vcpkg to acquire its dependencies, and so if you have it installed
globally, then we'll use that. If it's not installed globally, vcpkg will be cloned
automatically via git. If for some reason the vcpkg/downloads and vcpkg/buildtrees
are massive folders, you can safely delete them after this project successfully configures.

Decide on a toolchain and config, then run the below commands to build/install

`toolchain` recommended options:
 - `cl-x86_64-windows-msvc`  (Windows)
 - `gcc-x86_64-linux-gnu`    (Linux)
 - `clang-aarch64-macos-gnu` (M1/M2 MacOS)

`config` options:
 - `debug`
 - `relwithdebinfo`
 - `release`

```
cmake --preset=<toolchain>
cmake --build --preset=<toolchain>-<config>
```

## Install

```
cmake --install ./.out/<preset>/
```

## Usage

You can run `glsl-language-server` to use a HTTP server to handle IO. Alternatively, run
`glsl-language-server --stdin` to handle IO on stdin.
