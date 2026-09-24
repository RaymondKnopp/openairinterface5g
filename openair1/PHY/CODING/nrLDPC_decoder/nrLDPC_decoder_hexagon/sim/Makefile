HEXAGON_TOOLS := /opt/Hexagon_SDK/6.4.0.2/tools/HEXAGON_Tools/19.0.04/Tools
CC  := $(HEXAGON_TOOLS)/bin/hexagon-clang
SIM := $(HEXAGON_TOOLS)/bin/hexagon-sim

OAI_SRC_ROOT ?= $(abspath ../../../../../..)
LDPC_DEC_DIR := $(OAI_SRC_ROOT)/openair1/PHY/CODING/nrLDPC_decoder

CFLAGS := -O3 -mv73 -mhvx -mhvx-length=128B $(EXTRA_CFLAGS)
# ../inc for ldpc_hexagon_queue.h, which sizes g_workers[]
INCS   := -I$(LDPC_DEC_DIR) -I$(OAI_SRC_ROOT) -I../inc

all: ldpc_sim_test

ldpc_sim_test: ldpc_sim_main.c ../src/ldpc_hexagon_imp.c
	$(CC) $(CFLAGS) $(INCS) -o $@ ldpc_sim_main.c -lhexagon

STATSFILE ?= sim_stats.txt

sim: ldpc_sim_test
	$(SIM) -mv73 --statsfile $(STATSFILE) ./ldpc_sim_test

clean:
	rm -f ldpc_sim_test sim_stats.txt
