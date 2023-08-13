# glsl-language-server
Language server implementation for GLSL

## Status
Currently this LSP implementation can be interfaced with using either HTTP or stdio.

### Current Features

- Diagnostics
- Completion
- Hover
- Jump to def

### Planned Features

- Workspace symbols
- Find references

## Compile

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

You can run `glslls` to use a HTTP server to handle IO. Alternatively, run
`glslls --stdin` to handle IO on stdin.

## Editor Examples
The following are examples of how to run `glslls` from various editors that support LSP.

### Emacs

[lsp-mode](https://github.com/emacs-lsp/lsp-mode/) has this language server
integrated into the core. This assumes you have [glsl-mode](https://github.com/jimhourihan/glsl-mode)
installed. See the lsp-mode's [GLSL](https://emacs-lsp.github.io/lsp-mode/page/lsp-glsl/)
for more details.

### Neovim

[lspconfig](https://github.com/neovim/nvim-lspconfig) offers a ready-to-go configuration:

```lua
require'lspconfig'.glslls.setup{}
```
