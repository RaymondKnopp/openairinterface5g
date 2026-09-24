# SPDX-License-Identifier: LicenseRef-CSSL-1.0
#
# CMakeLists.txt — Hexagon DSP LDPC decoder offload.
#
# Builds two distinct targets depending on build type:
#
#   OS_TYPE=HLOS (ARM cross-compile):
#     → libldpc_hexagon.so   ARM stub loaded by OAI via dlopen in place of libldpc.so
#
#   Default (Hexagon cross-compile):
#     → libldpc_hexagon_skel.so  DSP skeleton pushed to /vendor/lib/rfsa/adsp/
#
# ── ARM (HLOS) build — standalone ───────────────────────────────────────────
#
#   cmake -B build-arm \
#       -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2 \
#       -DOAI_SRC_ROOT=/path/to/openairinterface5g \
#       -DCMAKE_TOOLCHAIN_FILE=<oai>/cmake_targets/cross-arm-dragonwing.cmake \
#       -DOS_TYPE=HLOS \
#       <oai>/openair1/PHY/CODING/nrLDPC_decoder/nrLDPC_decoder_hexagon
#   cmake --build build-arm
#
# ── ARM (HLOS) build — integrated OAI top-level ──────────────────────────────
#
#   cmake -B build-dragonwing \
#       -DCMAKE_TOOLCHAIN_FILE=<oai>/cmake_targets/cross-arm-dragonwing.cmake \
#       -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2 \
#       -DHEXAGON_LDPC=ON \
#       <oai>
#   cmake --build build-dragonwing --target ldpc_hexagon
#
# ── DSP (Hexagon) build — always standalone ───────────────────────────────────
#
#   cmake -B build-dsp \
#       -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2 \
#       -DOAI_SRC_ROOT=/path/to/openairinterface5g \
#       -DCMAKE_TOOLCHAIN_FILE=/opt/Hexagon_SDK/6.4.0.2/build/cmake/hexagon_toolchain.cmake \
#       -DDSP_VERSION=v73 \
#       <oai>/openair1/PHY/CODING/nrLDPC_decoder/nrLDPC_decoder_hexagon
#   cmake --build build-dsp
#
# ── Deploy ──────────────────────────────────────────────────────────────────
#
#   adb push build-dragonwing/libldpc_hexagon.so       /data/oai/lib/
#   adb push build-dsp/libldpc_hexagon_skel.so         /vendor/lib/rfsa/adsp/

cmake_minimum_required(VERSION 3.22)
project(ldpc_hexagon C)

# ---------------------------------------------------------------------------
# Detect integrated (OAI add_subdirectory) vs standalone build.
# hexagon_fun.cmake is designed for standalone use only: it sets global
# include_directories, uses the plain target_link_libraries signature, and
# validates internal variables that are unset in a parent CMake project.
# In integrated mode we replicate the needed behaviour with standard CMake.
# ---------------------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(LDPC_HEX_STANDALONE TRUE)
endif()

# ---------------------------------------------------------------------------
# Hexagon SDK root
# ---------------------------------------------------------------------------
if(NOT HEXAGON_SDK_ROOT)
    set(HEXAGON_SDK_ROOT "/opt/Hexagon_SDK/6.4.0.2")
    message(STATUS "HEXAGON_SDK_ROOT not set; defaulting to ${HEXAGON_SDK_ROOT}")
endif()

# ---------------------------------------------------------------------------
# OAI source root — required for LDPC decoder headers
# ---------------------------------------------------------------------------
if(NOT IS_ABSOLUTE "${OAI_SRC_ROOT}")
    # Relative paths (e.g. -DOAI_SRC_ROOT=.) resolve against CMAKE_CURRENT_SOURCE_DIR
    # (the hexagon directory), not the cmake invocation directory — so they always
    # give the wrong result.  This file lives at a known depth; auto-detect instead.
    get_filename_component(OAI_SRC_ROOT
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
    message(STATUS "OAI_SRC_ROOT auto-detected as ${OAI_SRC_ROOT}")
endif()

# ---------------------------------------------------------------------------
# Include paths shared by ARM stub and DSP skel
# ---------------------------------------------------------------------------
set(ldpc_dec_dir ${OAI_SRC_ROOT}/openair1/PHY/CODING/nrLDPC_decoder)

set(common_incs
    ${CMAKE_CURRENT_SOURCE_DIR}/inc        # ldpc_hexagon.idl lives here
    ${CMAKE_CURRENT_BINARY_DIR}            # qaic writes generated .h here
    ${HEXAGON_SDK_ROOT}/incs
    ${HEXAGON_SDK_ROOT}/incs/stddef
    ${HEXAGON_SDK_ROOT}/ipc/fastrpc/rpcmem/inc
    ${ldpc_dec_dir}                        # nrLDPCdecoder_defs.h, nrLDPC_types.h, …
    ${OAI_SRC_ROOT}                        # for openair1/PHY/CODING/coding_defs.h
)

# ===========================================================================
# Standalone path — uses hexagon_fun.cmake (works for standalone cmake -B)
# ===========================================================================
if(LDPC_HEX_STANDALONE)

    # DSP_VERSION default must be known before PREBUILT_LIB_DIR derivation below.
    if(NOT DSP_VERSION)
        set(DSP_VERSION "v73")
        message(STATUS "DSP_VERSION not set; defaulting to ${DSP_VERSION}. "
                       "Check IQ-9075 ISA version with: "
                       "adb shell cat /sys/devices/soc0/soc_id")
    endif()

    # hexagon_fun.cmake requires PREBUILT_LIB_DIR = "hexagon_toolv<N>_<DSP_VERSION>".
    # cmake_configure.bash normally passes this, but a direct cmake -B invocation
    # does not.  Derive it from the Hexagon compiler path when it is missing.
    if(NOT PREBUILT_LIB_DIR)
        # Compiler is at <HEXAGON_TOOLS_ROOT>/bin/hexagon-clang.
        # HEXAGON_TOOLS_ROOT may be a cmake var (passed via -D), an env var, or
        # inherited from the toolchain file; fall back to CMAKE_C_COMPILER path.
        if(NOT HEXAGON_TOOLS_ROOT AND DEFINED ENV{HEXAGON_TOOLS_ROOT})
            set(HEXAGON_TOOLS_ROOT "$ENV{HEXAGON_TOOLS_ROOT}")
        endif()
        set(_htroot "${HEXAGON_TOOLS_ROOT}")
        if(NOT _htroot AND CMAKE_C_COMPILER)
            get_filename_component(_htroot "${CMAKE_C_COMPILER}" DIRECTORY)
            get_filename_component(_htroot "${_htroot}" DIRECTORY)
        endif()
        if(_htroot)
            # _htroot is e.g. ".../HEXAGON_Tools/19.0.04" — get the version leaf
            get_filename_component(_ver_str "${_htroot}" NAME)
            string(REGEX MATCH "^([0-9]+)" _ "${_ver_str}")
        endif()
        if(CMAKE_MATCH_1)
            set(PREBUILT_LIB_DIR "hexagon_toolv${CMAKE_MATCH_1}_${DSP_VERSION}")
        else()
            set(PREBUILT_LIB_DIR "hexagon_toolv19_${DSP_VERSION}")
        endif()
        message(STATUS "PREBUILT_LIB_DIR auto-derived: ${PREBUILT_LIB_DIR}")
    endif()

    include(${HEXAGON_SDK_ROOT}/build/cmake/hexagon_fun.cmake)

    # ── ARM (HLOS) stub ────────────────────────────────────────────────────
    if(${OS_TYPE} MATCHES "HLOS")

        add_library(ldpc_hexagon SHARED
            ${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon_stub.c
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ldpc_hexagon_arm.c
            ${OAI_SRC_ROOT}/openair1/PHY/CODING/nrLDPC_encoder/ldpc_encoder_optim8segmulti.c
        )

        build_idl(inc/ldpc_hexagon.idl ldpc_hexagon)
        include_directories(${common_incs})

        choose_dsprpc(cdsp dsprpc)
        link_custom_library(ldpc_hexagon ${dsprpc})
        link_custom_library(ldpc_hexagon rpcmem)

        if(TARGET ldpc_segment)
            target_link_libraries(ldpc_hexagon PRIVATE ldpc_segment)
        endif()

        set_common_compile_and_link_options(ldpc_hexagon)
        install(TARGETS ldpc_hexagon LIBRARY DESTINATION lib)

    # ── DSP (Hexagon) skel ─────────────────────────────────────────────────
    else()

        add_library(ldpc_hexagon_skel SHARED
            ${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon_skel.c
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ldpc_hexagon_imp.c
        )

        build_idl(inc/ldpc_hexagon.idl ldpc_hexagon_skel)
        include_directories(${common_incs})

        target_compile_options(ldpc_hexagon_skel PRIVATE -O3 -mhvx -mhvx-length=128B)

        # Remove -lc from the shared library link rule.
        # The hexagon_toolchain.cmake PIC_SHARED_LD_FLAGS template appends -lc,
        # which pulls in the SDK libc.so.  libc.so's malloc uses fabs/sqrt
        # internally, causing those symbols to appear as undefined in the skel.
        # IQ-9075 unsigned-PD does not export these math symbols → AEE_EUNABLETOLOAD.
        # All libc symbols we actually need (malloc, calloc, free, memcpy, memset,
        # vsnprintf) are provided by the DSP QuRT runtime at dlopen time anyway.
        string(REPLACE " -lc" "" CMAKE_C_CREATE_SHARED_LIBRARY
               "${CMAKE_C_CREATE_SHARED_LIBRARY}")

        # VTCM allocation via HAP_compute_res_acquire (CRM API).
        # OFF by default: including HAP_compute_res.h adds weak undefined
        # compute_resource_* symbols to the skel's dynsym.  The IQ-9075 RTLD
        # rejects these at dlopen time in unsigned-PD (AEE_EUNABLETOLOAD).
        # Enable only when running signed-PD or on a device whose DSP runtime
        # exports compute_resource_acquire.
        option(LDPC_HEX_VTCM
            "Allocate LDPC working set in VTCM via CRM API (requires signed-PD; breaks unsigned-PD on IQ-9075)"
            OFF)
        if(LDPC_HEX_VTCM)
            target_compile_definitions(ldpc_hexagon_skel PRIVATE LDPC_HEX_VTCM)
            message(STATUS "ldpc_hexagon_skel: VTCM enabled (signed-PD / CRM API)")
        else()
            message(STATUS "ldpc_hexagon_skel: VTCM disabled (default; unsigned-PD safe)")
        endif()

        copy_binaries(ldpc_hexagon_skel)

    endif()

# ===========================================================================
# Integrated path — standard CMake only (avoids hexagon_fun.cmake conflicts).
# Only builds the ARM stub; the DSP skel is always built standalone.
# ===========================================================================
else()

    # ── T_IDs.h — generate at configure time for cross-compile ────────────
    # The encoder pulls in log.h → T.h → T_IDs.h (a generated file).
    # The normal build-time rule requires native tools (genids, _check_vcd)
    # that are unavailable in a cross-compile without NATIVE_DIR.
    # cmake itself runs on the x86 host, so we can compile genids.c with the
    # host gcc via execute_process and generate T_IDs.h right now.
    set(_t_ids_h "${CMAKE_BINARY_DIR}/common/utils/T/T_IDs.h")
    if(NOT EXISTS "${_t_ids_h}")
        set(_t_dir "${OAI_SRC_ROOT}/common/utils/T")
        set(_genids_host "${CMAKE_BINARY_DIR}/ldpc_hexagon_genids_host")
        message(STATUS "ldpc_hexagon: generating T_IDs.h with host gcc ...")
        execute_process(
            COMMAND gcc -o "${_genids_host}" "${_t_dir}/genids.c"
            RESULT_VARIABLE _gcc_ret
            ERROR_VARIABLE  _gcc_err)
        if(NOT _gcc_ret EQUAL 0)
            message(FATAL_ERROR "ldpc_hexagon: failed to compile genids.c: ${_gcc_err}")
        endif()
        file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/common/utils/T")
        execute_process(
            COMMAND "${_genids_host}" "${_t_dir}/T_messages.txt" "${_t_ids_h}"
            RESULT_VARIABLE _gen_ret
            ERROR_VARIABLE  _gen_err)
        if(NOT _gen_ret EQUAL 0)
            message(FATAL_ERROR "ldpc_hexagon: failed to run genids: ${_gen_err}")
        endif()
        message(STATUS "ldpc_hexagon: T_IDs.h generated at ${_t_ids_h}")
    endif()

    # ── qaic IDL compiler ──────────────────────────────────────────────────
    set(QAIC_EXE "${HEXAGON_SDK_ROOT}/ipc/fastrpc/qaic/bin/qaic")
    if(NOT EXISTS "${QAIC_EXE}")
        message(FATAL_ERROR "qaic not found at ${QAIC_EXE}. "
                            "Check HEXAGON_SDK_ROOT=${HEXAGON_SDK_ROOT}")
    endif()

    set(IDL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/inc/ldpc_hexagon.idl")

    add_custom_command(
        OUTPUT  "${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon.h"
                "${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon_stub.c"
                "${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon_skel.c"
        COMMAND "${QAIC_EXE}" -mdll
                -o "${CMAKE_CURRENT_BINARY_DIR}"
                "-I${HEXAGON_SDK_ROOT}/incs"
                "-I${HEXAGON_SDK_ROOT}/incs/stddef"
                "${IDL_SRC}"
        DEPENDS "${IDL_SRC}"
        COMMENT "qaic: generating FastRPC stub/skel from ldpc_hexagon.idl"
        VERBATIM)

    add_custom_target(ldpc_hexagon_idl
        DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon_stub.c"
                "${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon.h")

    # ── ARM stub library ───────────────────────────────────────────────────
    add_library(ldpc_hexagon SHARED
        "${CMAKE_CURRENT_BINARY_DIR}/ldpc_hexagon_stub.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/ldpc_hexagon_arm.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/nrLDPC_coding_hexagon.c"
        "${OAI_SRC_ROOT}/openair1/PHY/CODING/nrLDPC_encoder/ldpc_encoder_optim8segmulti.c"
        # The batch decoder replaces ldpc_segment's nrLDPC_coding_decoder, so we
        # cannot link the whole object library (duplicate symbol). Take the two
        # pieces we still need: the rate-matching helpers our prep calls, and the
        # segment encoder, which is unchanged -- only the decode is batched.
        "${OAI_SRC_ROOT}/openair1/PHY/CODING/nrLDPC_coding/nrLDPC_coding_segment/nr_rate_matching.c"
        "${OAI_SRC_ROOT}/openair1/PHY/CODING/nrLDPC_coding/nrLDPC_coding_segment/nrLDPC_coding_segment_encoder.c"
    )
    add_dependencies(ldpc_hexagon ldpc_hexagon_idl)

    target_include_directories(ldpc_hexagon PRIVATE ${common_incs})

    # ── Hexagon SDK prebuilt libraries ─────────────────────────────────────
    # cdsprpc: FastRPC transport for the cDSP domain (UbuntuARM_aarch64 variant
    # for DragonWing HLOS; android_aarch64 as fallback).
    find_library(CDSPRPC_LIB
        NAMES cdsprpc
        PATHS "${HEXAGON_SDK_ROOT}/ipc/fastrpc/remote/ship/UbuntuARM_aarch64"
              "${HEXAGON_SDK_ROOT}/ipc/fastrpc/remote/ship/android_aarch64"
        NO_DEFAULT_PATH
        REQUIRED)
    message(STATUS "ldpc_hexagon: cdsprpc → ${CDSPRPC_LIB}")

    # rpcmem: All rpcmem symbols (init/alloc/free/deinit) come from the device's
    # libcdsprpc.so.  The SDK's rpcmem.a contains only deprecated no-op stubs for
    # rpcmem_init/rpcmem_deinit; if linked, they shadow libcdsprpc.so's real init
    # and leave the DMA heap uninitialised so rpcmem_alloc() fails (error 104 on open).
    # Do NOT link rpcmem.a here.

    target_link_libraries(ldpc_hexagon PRIVATE "${CDSPRPC_LIB}")

    # This library exports the TB-level nrLDPC_coding_* entry points the softmodem
    # and the phy simulators load, as well as the four legacy LDPC* symbols
    # ldpctest uses. The coding entry points now come from
    # src/nrLDPC_coding_hexagon.c (one DSP dispatch per transport block) instead
    # of ldpc_segment (one per code block), which is why ldpc_segment is compiled
    # in piecewise above rather than linked whole -- linking it would collide on
    # nrLDPC_coding_decoder.
    #
    # Do not simply drop the coding entry points: without them the production
    # loader fails with "undefined symbol: nrLDPC_coding_init".
    target_include_directories(ldpc_hexagon PRIVATE
        "${OAI_SRC_ROOT}/openair1/PHY/CODING/nrLDPC_coding/nrLDPC_coding_segment")
    target_link_libraries(ldpc_hexagon PRIVATE log_headers)

    set_target_properties(ldpc_hexagon PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

endif()
