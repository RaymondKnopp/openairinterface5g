# SPDX-License-Identifier: LicenseRef-CSSL-1.0

# we need at least gcc-11 to build xran, so let's enforce it here
# (because xran might not check it, and we have control here)
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11.0
   OR CMAKE_C_COMPILER_VERSION VERSION_LESS 11.0)
 message(FATAL_ERROR "you need to have at least gcc/g++-11 to use 7.2 FHI through xran")
endif()

# use env var PKG_CONFIG_PATH to override paths to libdpdk.pc
pkg_check_modules(dpdk REQUIRED libdpdk)
pkg_check_modules(numa REQUIRED numa)

add_library(oran_fhlib_5g MODULE
            oran_isolate.c
            oaioran.c
            oran-config.c
            oran-init.c
)

# 11.1.6, not develop's 11.1.8: the in-tree NXP o-du-phy fork is a K-release 11.1.6
# carrying the DPAA2/fsl-mc patches, and this branch keeps its own radio/fhi_72
# sources, which need no 11.1.8 library API -- every xran_* symbol they call that is
# absent from 11.1.6's api/ (xran_fh_tx_send_slot, _ant, xran_send_cp_slot,
# xran_compress_iq_section) is defined OAI-side in oaioran.c.  Moving to 11.1.8 means
# porting NXP's DPAA2 patches onto the newer K release first.
set(K_VERSION 11.1.6)

add_boolean_option(xran_DOWNLOAD ON "Download and build xran library if not found in xran_LOCATION" OFF)

find_package(xran)
if(xran_FOUND)
  if(xran_VERSION VERSION_GREATER_EQUAL 5 AND xran_VERSION VERSION_LESS 6)
    message(FATAL_ERROR "xran E release not supported starting from tag 2026.w08.\nPlease switch to K version ${K_VERSION}")
  elseif(xran_VERSION VERSION_GREATER_EQUAL 6 AND xran_VERSION VERSION_LESS 7)
    message(FATAL_ERROR "xran F release not supported starting from tag 2026.w29.\nPlease switch to K version ${K_VERSION}")
  elseif(xran_VERSION VERSION_EQUAL K_VERSION)
    if(dpdk_VERSION VERSION_LESS 20.11)
      message(FATAL_ERROR "We require min DPDK version 20.11, but found ${dpdk_VERSION}")
    endif()
    target_compile_definitions(oran_fhlib_5g PRIVATE K_RELEASE)
    # NXP's o-du-phy fork inserts the 802.1Q tag in software from per-plane fields
    # (the DPAA2 PMD has no VLAN offload); upstream xran leaves tagging to the NIC/VF.
    file(STRINGS "${xran_INCLUDE_DIR}/xran_fh_o_du.h" _xran_vlan_fields REGEX "cp_vlan_tag")
    if(_xran_vlan_fields)
      target_compile_definitions(oran_fhlib_5g PRIVATE XRAN_HAS_CP_UP_VLAN_TAG)
      message(STATUS "xran has per-plane VLAN tags: software VLAN insertion from vlan_tag")
    endif()
  else()
    message(FATAL_ERROR "Found xran version ${xran_VERSION} but needed ${K_VERSION} (K release)")
  endif()
elseif(NOT xran_FOUND AND xran_DOWNLOAD)
  if(dpdk_VERSION VERSION_LESS 20.11)
    message(FATAL_ERROR "We require min DPDK version 20.11, but found ${dpdk_VERSION}")
  endif()
  include(ExternalProject)
  set(xran_DIR "${CMAKE_BINARY_DIR}/.deps/src/extern_xran/fhi_lib/lib")
  ExternalProject_Add(extern_xran
      PREFIX ${CMAKE_BINARY_DIR}/.deps
      GIT_REPOSITORY https://github.com/openairinterface/o-du-phy.git
      GIT_TAG ${K_VERSION}
      CONFIGURE_COMMAND ""  # nothing to configure
      BUILD_COMMAND cd fhi_lib/lib && WIRELESS_SDK_TOOLCHAIN=gcc RTE_SDK=${dpdk_PREFIX} XRAN_DIR=.. make XRAN_LIB_SO=1
      BUILD_IN_SOURCE ON
      INSTALL_COMMAND ""
      BUILD_BYPRODUCTS ${xran_DIR}/build/libxran.so
  )
  add_library(xran::xran UNKNOWN IMPORTED)
  add_dependencies(xran::xran extern_xran)
  # This is a workaround for the fact that included directories of an imported
  # target should exist in the filesystem already at the configuration time.
  # ref: https://gitlab.kitware.com/cmake/cmake/-/issues/15052
  file(MAKE_DIRECTORY "${xran_DIR}/api/")
  file(MAKE_DIRECTORY "${xran_DIR}/build/")
  set_target_properties(xran::xran PROPERTIES
    IMPORTED_LOCATION "${xran_DIR}/build/libxran.so"
    INTERFACE_INCLUDE_DIRECTORIES "${xran_DIR}/api/"
  )
  message(STATUS "xran not found. Will clone and build automatically K release ${K_VERSION}")
  target_compile_definitions(oran_fhlib_5g PRIVATE K_RELEASE)
else()
  message(FATAL_ERROR "could not find xran, and auto-download disabled. Update
  xran_LOCATION, or set -Dxran_DOWNLOAD=ON")
endif()

# Ignore xran-specific warning: we don't care/can't change the following warning, so suppress
# alignment 1 of 'struct XYZ' is less than 2
add_compile_options(-Wno-packed-not-aligned)

set_target_properties(oran_fhlib_5g PROPERTIES COMPILE_FLAGS "-fvisibility=hidden -march=native")
target_link_libraries(oran_fhlib_5g PRIVATE xran::xran ${dpdk_LINK_LIBRARIES} pthread dl rt m numa)
target_link_libraries(oran_fhlib_5g PRIVATE log_headers)
target_include_directories(oran_fhlib_5g PRIVATE ${dpdk_INCLUDE_DIRS})
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  find_package(armral REQUIRED)
  target_sources(oran_fhlib_5g PRIVATE armral_bfp_compression.c)
  target_link_libraries(oran_fhlib_5g PRIVATE armral)
endif()

message(STATUS "Building FHI72 CUS library")

set_target_properties(oran_fhlib_5g PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
add_custom_command(TARGET oran_fhlib_5g POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink liboran_fhlib_5g.so liboai_transpro.so
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_boolean_option(OAI_FHI72_MPLANE OFF "Activate OAI's FHI 7.2 M-plane support" OFF)
if(OAI_FHI72_MPLANE)
  add_subdirectory(mplane)
endif()
