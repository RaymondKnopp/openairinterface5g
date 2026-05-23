<!-- SPDX-License-Identifier: CC-BY-4.0 -->

# Building OAI for Qualcomm DragonWing IQ-9/X (ARM)

This document covers cross-compiling OAI for the ARM cores of the Qualcomm
DragonWing IQ-9/X platform (SA9000P SoC, Kryo 780 / Cortex-A78).
The Hexagon DSP component is not covered here.

The toolchain file is `cmake_targets/cross-arm-dragonwing.cmake`, which uses the
GCC cross-compiler bundled with the Hexagon SDK rather than the Ubuntu package.
The build procedure otherwise follows the same two-step pattern as the generic
ARM64 cross-compile described in `cross-compile.md`.

> **Note on the compiler:** The Hexagon SDK includes two compilers.
> `hexagon-clang` (LLVM 19) targets the **Hexagon DSP only** and cannot produce
> ARM binaries.  The ARM cross-compiler is `aarch64-none-linux-gnu-gcc`
> (Arm GNU Toolchain 11.3.1) located under
> `tools/gcc_tools_64/bin/` in the SDK tree.

[[_TOC_]]

---

## 1 Prerequisites on the build host

### 1.1 Hexagon SDK

The SDK must be installed, defaulting to `/opt/Hexagon_SDK/6.4.0.2`.
Run the SDK's environment script once per shell session (or add to your profile):

```shell
source /opt/Hexagon_SDK/6.4.0.2/setup_sdk_env.source
```

If the SDK is installed elsewhere, pass `-DHEXAGON_SDK_ROOT=<path>` to every
`cmake` invocation in this guide.

### 1.2 OAI native build dependencies

The host machine needs OAI's standard build tools.  If not installed yet:

```shell
cmake_targets/build_oai -I
```

### 1.3 ARM64 target libraries

OAI links against several libraries that must be available for the `aarch64`
architecture at link time.  The standard way is Ubuntu multiarch.

> **Caveat:** The `dpkg --add-architecture` approach requires that your
> apt sources serve `aarch64` packages.  Ubuntu 22.04 / 24.04 on a standard
> x86 host works out of the box via `ports.ubuntu.com`.  If you are on a
> non-standard host, inside a corporate mirror, or inside a container without
> network access, you may need to adapt the sources list below — or provide
> the libraries from the DragonWing device filesystem instead (see
> [Section 4](#4-alternative-sysroot-from-device)).

#### 1.3.1 Enable the arm64 architecture in apt

**Ubuntu 24.04 (Noble)** — the sources are in `ubuntu.sources` format:

```shell
sudo dpkg --add-architecture arm64

# Restrict the existing ubuntu.sources to amd64 only
sudo sed -i '/^Components:/a Architectures: amd64' \
    /etc/apt/sources.list.d/ubuntu.sources

# Add a separate arm64 source pointing to ports.ubuntu.com
sudo tee /etc/apt/sources.list.d/arm-cross-compile-sources.list <<'EOF'
deb [arch=arm64] http://ports.ubuntu.com/ noble main restricted
deb [arch=arm64] http://ports.ubuntu.com/ noble-updates main restricted
deb [arch=arm64] http://ports.ubuntu.com/ noble universe
deb [arch=arm64] http://ports.ubuntu.com/ noble-updates universe
deb [arch=arm64] http://ports.ubuntu.com/ noble multiverse
deb [arch=arm64] http://ports.ubuntu.com/ noble-updates multiverse
deb [arch=arm64] http://ports.ubuntu.com/ noble-backports main restricted universe multiverse
EOF
```

**Ubuntu 22.04 (Jammy)** — sources are still in the old `sources.list` format:

```shell
sudo dpkg --add-architecture arm64

sudo cp /etc/apt/sources.list "/etc/apt/sources.list.$(date).backup"
sudo sed -i -E "s/(deb)\ (http:.+)/\1\ [arch=amd64]\ \2/" \
    /etc/apt/sources.list

sudo tee /etc/apt/sources.list.d/arm-cross-compile-sources.list <<'EOF'
deb [arch=arm64] http://ports.ubuntu.com/ jammy main restricted
deb [arch=arm64] http://ports.ubuntu.com/ jammy-updates main restricted
deb [arch=arm64] http://ports.ubuntu.com/ jammy universe
deb [arch=arm64] http://ports.ubuntu.com/ jammy-updates universe
deb [arch=arm64] http://ports.ubuntu.com/ jammy multiverse
deb [arch=arm64] http://ports.ubuntu.com/ jammy-updates multiverse
deb [arch=arm64] http://ports.ubuntu.com/ jammy-backports main restricted universe multiverse
EOF
```

#### 1.3.2 Install the arm64 packages

```shell
sudo apt-get update
sudo apt-get install --yes \
    libc6-dev-i386 \
    libreadline-dev:arm64 \
    libgnutls28-dev:arm64 \
    libconfig-dev:arm64 \
    libsctp-dev:arm64 \
    libssl-dev:arm64 \
    libtool:arm64 \
    zlib1g-dev:arm64 \
    libyaml-cpp-dev:arm64
```

> **Note:** `libc6-dev-i386` is for the host (code-generation tools), not the
> target.  All other packages with `:arm64` suffix are for the cross-linked
> ARM binaries.

> The list above is believed to be complete but may grow as new OAI features
> are enabled.  If cmake reports a missing package, install `<package>:arm64`
> and re-run cmake.

---

## 2 Build

### 2.1 Step 1 — native host tools

These are x86 binaries that cmake runs during the cross-compile step to
generate LDPC processing code and the T-tracer event IDs.

```shell
cd <oai-root>
rm -rf ran_build
mkdir -p ran_build/build ran_build/build-dragonwing

cd ran_build/build
cmake ../../..
make -j$(nproc) ldpc_generators generate_T
```

### 2.2 Step 2 — cross-compile for DragonWing

```shell
cd ../build-dragonwing

cmake ../../.. -GNinja \
    -DCMAKE_TOOLCHAIN_FILE=../../../cmake_targets/cross-arm-dragonwing.cmake \
    -DNATIVE_DIR=../build

# Example targets — add or remove as needed
ninja nr-softmodem nr-cuup nr-uesoftmodem \
      params_libconfig coding rfsimulator
```

If the Hexagon SDK is not at the default path, add:

```
-DHEXAGON_SDK_ROOT=/your/path/to/Hexagon_SDK/6.4.0.2
```

The `QUALCOMM_DRAGONWING=1` flag is set automatically by the toolchain file and
causes CMake to use `-mcpu=cortex-a78` instead of the generic `-march=armv8.2-a`.

---

## 3 Deploy via adb

```shell
# Verify the device is reachable
adb devices

# Create a destination directory on the device
adb shell mkdir -p /data/oai

# Push the binaries
adb push ran_build/build-dragonwing/nr-softmodem   /data/oai/
adb push ran_build/build-dragonwing/nr-cuup         /data/oai/
adb push ran_build/build-dragonwing/nr-uesoftmodem  /data/oai/

# Push shared libraries that OAI loads at runtime
adb push ran_build/build-dragonwing/libparams_libconfig.so /data/oai/
adb push ran_build/build-dragonwing/libcoding.so           /data/oai/
adb push ran_build/build-dragonwing/librfsimulator.so      /data/oai/

# On the device, set LD_LIBRARY_PATH before running
adb shell "export LD_LIBRARY_PATH=/data/oai && /data/oai/nr-softmodem --help"
```

If the standard system libraries (libgnutls, libssl, libconfig, …) are not
present on the device, they must be pushed alongside the OAI binaries:

```shell
# Example: find and push the arm64 shared libs from the build host
for lib in libgnutls libssl libcrypto libconfig libsctp; do
    find /usr/lib/aarch64-linux-gnu -name "${lib}.so*" -exec \
        adb push {} /data/oai/ \;
done
```

---

## 4 Alternative: sysroot from device

If the Ubuntu multiarch packages are not available on the build host, you can
extract the device's root filesystem and use it as a sysroot instead.

```shell
# Pull relevant library directories from the device
adb pull /usr/lib       dragonwing-sysroot/usr/lib
adb pull /usr/include   dragonwing-sysroot/usr/include
adb pull /lib           dragonwing-sysroot/lib

# Then configure cmake with an explicit sysroot
cmake ../../.. -GNinja \
    -DCMAKE_TOOLCHAIN_FILE=../../../cmake_targets/cross-arm-dragonwing.cmake \
    -DNATIVE_DIR=../build \
    -DCMAKE_SYSROOT=$(pwd)/dragonwing-sysroot \
    -DCMAKE_FIND_ROOT_PATH=$(pwd)/dragonwing-sysroot
```

You will also need to regenerate the pkg-config search path:

```shell
export PKG_CONFIG_LIBDIR=$(pwd)/dragonwing-sysroot/usr/lib/pkgconfig:$(pwd)/dragonwing-sysroot/usr/lib/aarch64-linux-gnu/pkgconfig
```

---

## 5 Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `Could NOT find GnuTLS` | arm64 packages not installed or wrong `PKG_CONFIG_LIBDIR` | Check Section 1.3; verify `pkg-config --list-all` finds arm64 packages |
| `cannot find -lgnutls` at link | Linker not searching `/usr/lib/aarch64-linux-gnu` | Verify that toolchain file `CMAKE_EXE_LINKER_FLAGS_INIT` is being applied |
| `Unsupported architecture` from apt | Host apt sources not configured for arm64 | Redo Section 1.3.1 |
| Binary crashes with `SIGILL` on device | Wrong `-mcpu` / `-march` flag | The toolchain defaults to `cortex-a78`; if running on a Silver (A55) core, add `-DCMAKE_C_FLAGS=-mcpu=cortex-a55` |
| `adb: error: failed to copy` | `/data/oai` directory doesn't exist or no permission | `adb shell mkdir -p /data/oai` or use a writable path like `/data/local/tmp/oai` |
