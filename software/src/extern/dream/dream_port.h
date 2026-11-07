/* SPDX-License-Identifier: GPL-3.0-only */
/* Loom (Teensy 4.1) port of NACHTFAHRT's engine (amb.c): where things go in memory. RAM1 is
 * nearly full: the code runs from flash (DREAM_FLASH, at -O2), the constant tables stay in flash
 * (DREAM_CONST), the engine's state goes to RAM2 (DREAM_RAM2) and its delay lines to PSRAM
 * (EN_POOL). Neither RAM2 nor PSRAM is cleared at start: dream_zero() (generated at the end of
 * amb.c) clears every variable moved there, before en_init(). On the host all empty. */
#ifndef DREAM_PORT_H
#define DREAM_PORT_H
#if defined(__IMXRT1062__)
#define DREAM_FLASH __attribute__((section(".flashmem"), optimize("Os")))
#define DREAM_CONST __attribute__((section(".progmem")))
#ifndef DREAM_RAM2  /* (amb_b.c: track B's engine keeps its state in PSRAM: RAM2 is the heap) */
#define DREAM_RAM2 __attribute__((section(".dmabuffers"), aligned(4)))
#endif
#define DREAM_PSRAM __attribute__((section(".externalram"), aligned(4)))
#else
#define DREAM_FLASH
#define DREAM_CONST
#define DREAM_RAM2
#define DREAM_PSRAM
#endif
void dream_zero(void);
#endif
