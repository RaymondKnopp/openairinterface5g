# SPDX-License-Identifier: LicenseRef-CSSL-1.0

add_library(coding MODULE
            3gpplte_sse.c
            3gpplte.c
            3gpplte_turbo_decoder_sse_8bit.c
            3gpplte_turbo_decoder_sse_16bit.c
            3gpplte_turbo_decoder_avx2_16bit.c
            3gpplte_turbo_decoder.c
)
set_target_properties(coding PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(coding PRIVATE log_headers)

add_library(ldpc_orig MODULE
            nrLDPC_decoder/nrLDPC_decoder.c
            nrLDPC_encoder/ldpc_encoder.c
)
set_target_properties(ldpc_orig PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(ldpc_orig PRIVATE ldpc_segment)

add_library(ldpc MODULE
            nrLDPC_decoder/nrLDPC_decoder.c
            nrLDPC_encoder/ldpc_encoder_optim8segmulti.c
)
set_target_properties(ldpc PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(ldpc PRIVATE ldpc_segment)

add_dependencies(ldpctest ldpc ldpc_orig)
add_dependencies(nr-softmodem ldpc ldpc_orig)
add_dependencies(nr-uesoftmodem ldpc ldpc_orig)
add_dependencies(nr_ulsim ldpc ldpc_orig)
add_dependencies(nr_ulschsim ldpc ldpc_orig)
add_dependencies(nr_dlsim ldpc ldpc_orig)
add_dependencies(nr_dlschsim ldpc ldpc_orig)
if (ENABLE_TESTS)
  add_dependencies(tests ldpc ldpc_orig)
endif()

add_library(crc_byte OBJECT crc_byte.c)

add_subdirectory(nrLDPC_coding)
add_subdirectory(nrPolar_tools)
add_subdirectory(nrSmallBlock)

# Build the Hexagon LDPC offload ARM stub (libldpc_hexagon.so).
# Requires the Hexagon SDK.  The DSP skel (libldpc_hexagon_skel.so) is
# always built separately with the hexagon-clang toolchain — see
# nrLDPC_decoder/nrLDPC_decoder_hexagon/README.md.
#
# Enable with:  cmake ... -DHEXAGON_LDPC=ON -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2
option(HEXAGON_LDPC "Build Hexagon LDPC DSP offload ARM stub (libldpc_hexagon.so)" OFF)

if(HEXAGON_LDPC)
    if(NOT HEXAGON_SDK_ROOT)
        message(FATAL_ERROR "HEXAGON_LDPC=ON requires -DHEXAGON_SDK_ROOT=<path>")
    endif()
    # Tell the subdirectory's CMakeLists to build the ARM stub, not the DSP skel.
    set(OS_TYPE "HLOS")
    add_subdirectory(nrLDPC_decoder/nrLDPC_decoder_hexagon)
    # ldpctest should also be able to test the hexagon stub
    add_dependencies(ldpctest ldpc_hexagon)
endif()
