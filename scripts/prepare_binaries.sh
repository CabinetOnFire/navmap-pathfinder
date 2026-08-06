#!/bin/bash
set -euo pipefail

echo '==== Make sure cross is installed ===='
cargo install cross

echo '==== Linux 32-bit build ===='
rustup target add i686-unknown-linux-gnu
PKG_CONFIG_ALLOW_CROSS=1 cargo build --release --target i686-unknown-linux-gnu
cp target/navmap_pathfinder.dm target/navmap_pathfinder.linux.dm

echo '==== Linux 64-bit build ===='
rustup target add x86_64-unknown-linux-gnu
PKG_CONFIG_ALLOW_CROSS=1 cargo build --release --target x86_64-unknown-linux-gnu --features allow_non_32bit

echo '==== Windows 32-bit GNU build ===='
cross build --release --target i686-pc-windows-gnu
cmp target/navmap_pathfinder.dm target/navmap_pathfinder.linux.dm

echo '==== Windows 64-bit GNU build ===='
cross build --release --target x86_64-pc-windows-gnu --features allow_non_32bit
cmp target/navmap_pathfinder.dm target/navmap_pathfinder.linux.dm
rm target/navmap_pathfinder.linux.dm

echo '==== Organize files ===='
DEST=target/publish/
rm -rf "$DEST"
mkdir -p "$DEST"
cp \
    target/navmap_pathfinder.dm \
    target/i686-unknown-linux-gnu/release/libnavmap_pathfinder.so \
    target/i686-pc-windows-gnu/release/navmap_pathfinder.dll \
    "$DEST"
cp target/x86_64-unknown-linux-gnu/release/libnavmap_pathfinder.so \
    "$DEST/libnavmap_pathfinder64.so"
cp target/x86_64-pc-windows-gnu/release/navmap_pathfinder.dll \
    "$DEST/navmap_pathfinder64.dll"
ls -lh --color=auto "$DEST"
