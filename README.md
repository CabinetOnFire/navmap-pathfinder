# navmap-pathfinder

Rust DLL for `/tg/station` navmap pathfinding.

The search algorithm intentionally uses an incremental A* implementation. Each
`start` or `resume` call works for approximately five milliseconds and then returns so we don't take too long on a run and yield to other procs.

## Requirements

- Rust 1.89 or newer
- 32-bit Rust targets for BYOND:
  - `i686-pc-windows-msvc`
  - `i686-unknown-linux-gnu`
- 64-bit targets for OpenDream and diagnostics:
  - `x86_64-pc-windows-msvc`
  - `x86_64-unknown-linux-gnu`
- BYOND 516.1674 or newer when running the DLL

For Linux cross-builds, install the appropriate 32-bit development packages. Windows MSVC
builds require the Visual Studio C++ build tools.

On Windows, the Linux tasks use Docker Desktop with its WSL2 backend and `cross`:

```powershell
cross --version
docker info
```

If WSL was just enabled, restart Windows first, then start Docker Desktop. `docker info`
must show a running server before the Linux VS Code tasks or `cross build` commands can run.

## Build

32-bit Windows:

```powershell
cargo build --release --target i686-pc-windows-msvc
# target/i686-pc-windows-msvc/release/navmap_pathfinder.dll
# target/i686-pc-windows-msvc/release/navmap_pathfinder.pdb
```

32-bit Linux:

```sh
PKG_CONFIG_ALLOW_CROSS=1 cargo build --release --target i686-unknown-linux-gnu
# target/i686-unknown-linux-gnu/release/libnavmap_pathfinder.so
```

64-bit Windows:

```powershell
cargo build --release --target x86_64-pc-windows-msvc --features allow_non_32bit
# target/x86_64-pc-windows-msvc/release/navmap_pathfinder.dll
# target/x86_64-pc-windows-msvc/release/navmap_pathfinder.pdb
```

64-bit Linux:

```sh
PKG_CONFIG_ALLOW_CROSS=1 cargo build --release --target x86_64-unknown-linux-gnu --features allow_non_32bit
# target/x86_64-unknown-linux-gnu/release/libnavmap_pathfinder.so
```

The release workflow publishes the 64-bit files as `navmap_pathfinder64.dll`,
`navmap_pathfinder64.pdb`, and `libnavmap_pathfinder64.so`

Every build also creates `target/navmap_pathfinder.dm`. Include that file in the DM project
alongside the matching library.

## VS Code

Press F5 and choose `Build navmap-pathfinder (win32)` for the default BYOND build. The
`Build navmap-pathfinder (win64)`, `Build navmap-pathfinder (linux32)`, and
`Build navmap-pathfinder (linux64)` configurations are available from the launch
configuration selector. Linux tasks use [cross](https://github.com/cross-rs/cross), which
must be installed and requires Docker.

The `scripts/prepare_binaries.sh` script builds all four GNU targets and collects them in
`target/publish`.

GitHub Actions runs the Linux checks and release build automatically on pushes to `master`
and pull requests. It can also be started manually from the workflow page with **Run
workflow**.
