# SPDX-License-Identifier: LicenseRef-CSSL-1.0
#
# libldpc_hexagon_skel.so — the DSP half of the LDPC offload.
#
#   make / make clean
#
# WHY THIS EXISTS. The skel used to be built by the SDK's CMake path into
# build-dsp/. That tree no longer configures ("CMAKE_ASM_COMPILER ... is not a
# full path to an existing compiler tool"), so the skel could not be rebuilt at
# all -- which blocks any change to the DSP side, including carrying Kprime over
# the wire. Rather than fight the SDK toolchain files, this mirrors
# nr_phy_common/hexagon_inner_rx/dsp/Makefile, whose flag set is known to produce
# a loadable skel on this device.
#
# The flags are NOT decorative; see that Makefile for the full account. Briefly:
# -G0 with the G0/pic target libraries (mixing small-data models breaks
# GP-relative addressing), -Bsymbolic, --wrap on the allocators so allocation
# goes to the FastRPC-aware heap, -lc so libc.so is a real NEEDED entry, and a
# soname. A skel built without them loads but fails in ways that point nowhere
# near the build.
#
# Flags recovered from the old build-dsp tree
# (CMakeFiles/ldpc_hexagon_skel.dir/flags.make) so this stays equivalent to what
# shipped: -mv73 -G0 -O2 -mhvx -mhvx-length=128B, no LDPC_HEX_VTCM (VTCM is
# opt-in and needs a testsig; see README).

SDK      ?= /opt/Hexagon_SDK/6.4.0.2
HEXTOOLS ?= $(SDK)/tools/HEXAGON_Tools/19.0.04/Tools
CC       := $(HEXTOOLS)/bin/hexagon-clang
QAIC     := $(SDK)/ipc/fastrpc/qaic/bin/qaic

TOP      := ..
OAI_ROOT ?= $(abspath $(TOP)/../../../../..)
GEN      := gen
TARGET   := libldpc_hexagon_skel.so

HEXLIB := $(HEXTOOLS)/target/hexagon/lib

# LDPC_HEX_GATHER: acquire a little VTCM and use vgather for the deinterleave,
# which is a stride-Qm read that no power-of-two shuffle can do for Qm = 6.
# Requires signed PD (we have it via testsig-<serial>.so); the code falls back to
# the scalar deinterleave if the VTCM acquire fails, so an unsigned-PD device
# still works. Note this pulls in HAP_compute_res.h and therefore weak
# compute_resource_* symbols -- see the LDPC_HEX_VTCM comment in the source.
CFLAGS := -mv73 -G0 -O2 -Wall -fpic -fno-zero-initialized-in-bss -fdata-sections \
          -mhvx -mhvx-length=128B -DLDPC_HEX_GATHER -DLDPC_HEX_VTCM \
          $(EXTRA_CFLAGS)   # A/B hook, e.g. EXTRA_CFLAGS=-DLDPC_HEX_MSG_DCFETCH=1

INCS := -I$(SDK)/rtos/qurt/computev73/include \
        -I$(SDK)/rtos/qurt/computev73/include/qurt \
        -I$(SDK)/rtos/qurt/computev73/include/posix \
        -I$(SDK)/ipc/fastrpc/rtld/ship/hexagon_toolv19_v73 \
        -I$(SDK)/ipc/fastrpc/rpcmem/inc \
        -isystem $(SDK)/incs \
        -isystem $(SDK)/incs/stddef \
        -isystem $(SDK)/ipc/fastrpc/incs \
        -I$(TOP)/inc -I$(GEN) \
        -I$(OAI_ROOT)/openair1/PHY/CODING/nrLDPC_decoder \
        -I$(OAI_ROOT)

LDFLAGS := -mv73 -G0 -fpic -shared \
           -Wl,-Bsymbolic \
           -Wl,-L$(HEXLIB)/v73/G0/pic -Wl,-L$(HEXLIB)/ \
           -Wl,--no-threads \
           -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free \
           -Wl,--wrap=realloc -Wl,--wrap=memalign \
           -Wl,-soname,$(TARGET)

OBJS := $(GEN)/ldpc_hexagon_skel.o $(GEN)/ldpc_hexagon_imp.o

all: $(TARGET)

$(GEN)/ldpc_hexagon_skel.c: $(TOP)/inc/ldpc_hexagon.idl
	mkdir -p $(GEN)
	$(QAIC) -mdll -o $(GEN) -I $(SDK)/incs -I $(SDK)/incs/stddef $<

$(GEN)/ldpc_hexagon_skel.o: $(GEN)/ldpc_hexagon_skel.c
	$(CC) $(CFLAGS) $(INCS) -c -o $@ $<

$(GEN)/ldpc_hexagon_imp.o: $(TOP)/src/ldpc_hexagon_imp.c $(GEN)/ldpc_hexagon_skel.c
	mkdir -p $(GEN)
	$(CC) $(CFLAGS) $(INCS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ -Wl,--start-group $(OBJS) -Wl,--end-group -lc

clean:
	rm -rf $(GEN) $(TARGET)

.PHONY: all clean
