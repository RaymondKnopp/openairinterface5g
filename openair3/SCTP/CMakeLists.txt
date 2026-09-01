# SPDX-License-Identifier: LicenseRef-CSSL-1.0

find_package(sctp REQUIRED)

# Optional userspace SCTP stack for F1-C and N2, used when the kernel has no
# CONFIG_IP_SCTP. This switch only decides whether the backend is COMPILED IN --
# which backend is USED is decided at runtime in oai_sctp.c, by trying to open a
# kernel SCTP socket. It has to be that way round: OAI is cross-compiled, so a
# configure-time probe would test the build host's kernel rather than the
# target's, and one image may be booted against kernels that differ in SCTP
# support. See the header comment in openair3/SCTP/oai_sctp.h.
option(ENABLE_USRSCTP "Build the usrsctp fallback backend for F1-C/N2" OFF)

add_library(SCTP_CLIENT STATIC
  sctp_common.c
  sctp_eNB_task.c
  sctp_eNB_itti_messaging.c
  oai_sctp.c
)
target_include_directories(SCTP_CLIENT PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SCTP_CLIENT PRIVATE sctp::sctp)

if(ENABLE_USRSCTP)
  # sctp_werror defaults ON upstream and turns any new-compiler warning into a
  # build failure in third-party code; sctp_build_programs pulls in example
  # binaries we do not want.
  CPMAddPackage(
    NAME usrsctp
    GITHUB_REPOSITORY sctplab/usrsctp
    GIT_TAG 0.9.5.0
    OPTIONS
      "sctp_build_programs OFF"
      "sctp_build_shared_lib OFF"
      "sctp_werror OFF"
  )
  # The backend MUST live in its own target. usrsctp does
  # target_include_directories(usrsctp PUBLIC .) and ships its own
  # usrsctplib/netinet/sctp.h, so anything that links it directly gets FreeBSD's
  # netinet/sctp.h instead of the system one -- which breaks sctp_eNB_task.c,
  # a file that does not use usrsctp at all (MSG_NOTIFICATION goes undeclared).
  # Linking usrsctp PRIVATE into this small library confines that include path
  # to the one source file that wants it, while still carrying the archive
  # through to the final link.
  add_library(oai_sctp_usrsctp STATIC oai_sctp_usrsctp.c)
  target_compile_definitions(oai_sctp_usrsctp PRIVATE ENABLE_USRSCTP)
  target_link_libraries(oai_sctp_usrsctp PRIVATE usrsctp)
  target_include_directories(oai_sctp_usrsctp PRIVATE ${OPENAIR_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(SCTP_CLIENT PRIVATE ENABLE_USRSCTP)
  target_link_libraries(SCTP_CLIENT PRIVATE oai_sctp_usrsctp)
endif()

target_link_libraries(SCTP_CLIENT PRIVATE asn1_nr_rrc_hdrs asn1_lte_rrc_hdrs)
