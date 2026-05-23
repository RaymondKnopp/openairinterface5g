# SPDX-License-Identifier: LicenseRef-CSSL-1.0
# CMake toolchain for Qualcomm DragonWing IQ-9/X (ARM component, no Hexagon DSP).
# Uses the GCC aarch64 cross-compiler bundled with the Hexagon SDK 6.4.x.
#
# Note: the SDK's clang (hexagon-clang) targets only the Hexagon DSP; the ARM
# cross-compiler is the Arm GNU Toolchain GCC (aarch64-none-linux-gnu-gcc).
#
# Prerequisites – same as the generic cross-arm.cmake, install arm64 target libs:
#   sudo dpkg --add-architecture arm64
#   sudo apt-get install -y libgnutls28-dev:arm64 libconfig-dev:arm64 \
#       libsctp-dev:arm64 libssl-dev:arm64 zlib1g-dev:arm64 libreadline-dev:arm64
#
# Two-step build (see doc/cross-compile.md for details):
#
#   Step 1 – native host tools (ldpc generators, T-tracer id generator):
#     mkdir -p ran_build/build ran_build/build-dragonwing
#     cd ran_build/build
#     cmake ../../..
#     make -j$(nproc) ldpc_generators generate_T
#
#   Step 2 – cross-compiled ARM binaries:
#     cd ../build-dragonwing
#     cmake ../../.. -GNinja \
#         -DCMAKE_TOOLCHAIN_FILE=../../../cmake_targets/cross-arm-dragonwing.cmake \
#         -DNATIVE_DIR=../build
#     ninja nr-softmodem nr-cuup nr-uesoftmodem params_libconfig coding rfsimulator
#
#   Deploy via adb:
#     adb push ran_build/build-dragonwing/nr-softmodem /data/oai/
#
# Optional overrides:
#   -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2   (if SDK is in a different location)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(NOT DEFINED HEXAGON_SDK_ROOT)
    set(HEXAGON_SDK_ROOT "/opt/Hexagon_SDK/6.4.0.2")
endif()
set(_DW_TC "${HEXAGON_SDK_ROOT}/tools/gcc_tools_64/bin/aarch64-none-linux-gnu")

set(CMAKE_C_COMPILER   "${_DW_TC}-gcc")
set(CMAKE_CXX_COMPILER "${_DW_TC}-g++")
set(CMAKE_AR           "${_DW_TC}-ar")

# The SDK GCC's built-in sysroot has an incomplete glibc (missing L_tmpnam and
# similar symbols in stdio.h).  Override with the Ubuntu system headers by
# prepending them via -I, which takes priority over the compiler's sysroot
# search paths.  /usr/include is the host glibc (architecture-neutral API);
# /usr/include/aarch64-linux-gnu has the arm64-specific bits/ and sys/ headers.
set(CMAKE_C_FLAGS_INIT   "-I/usr/include/aarch64-linux-gnu -I/usr/include")
set(CMAKE_CXX_FLAGS_INIT "-I/usr/include/aarch64-linux-gnu -I/usr/include")

# Linker: search the arm64 multiarch directory so arm64 .so files are found
# before any host x86 libraries.
set(CMAKE_EXE_LINKER_FLAGS_INIT    "-L/usr/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-L/usr/lib/aarch64-linux-gnu")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "-L/usr/lib/aarch64-linux-gnu")

# Direct pkg-config at the arm64 multiarch .pc files
set(ENV{PKG_CONFIG_LIBDIR} "/usr/lib/aarch64-linux-gnu/pkgconfig")

# Tell cmake's find_library() where Ubuntu multiarch arm64 libs live
set(CMAKE_LIBRARY_PATH "/usr/lib/aarch64-linux-gnu")

set(CROSS_COMPILE 1)
set(QUALCOMM_DRAGONWING 1)

# Paths to host-compiled code-generation tools (built in Step 1 above)
set(bnProc_gen_128_DIR    ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
set(bnProc_gen_avx2_DIR   ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
set(bnProc_gen_avx512_DIR ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
set(cnProc_gen_128_DIR    ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
set(cnProc_gen_avx2_DIR   ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
set(cnProc_gen_avx512_DIR ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
set(genids_DIR            ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
set(_check_vcd_DIR        ${CMAKE_CURRENT_BINARY_DIR}/${NATIVE_DIR})
