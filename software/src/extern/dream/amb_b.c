/* SPDX-License-Identifier: GPL-3.0-only */
/* (Loom) the engine a second time: track B's DREAM, its own state and its own names (dream_b_names.h); the
 * same source as A's (amb.c) */
#include "dream_b_names.h"
#if defined(__IMXRT1062__)
#define DREAM_RAM2 __attribute__((section(".externalram"), aligned(4)))   /* (its state in PSRAM: dream_port.h) */
#endif
#include "amb.c"
