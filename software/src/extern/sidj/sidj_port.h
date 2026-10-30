/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Loom (Teensy 4.1) port of SiDJ's C64 and SID engine: where things go in memory.
 * RAM1 (ITCM / DTCM) is nearly full, so the code runs from flash (SIDJ_FLASH), the constant
 * tables stay in flash (SIDJ_CONST) and the large scratch tables go to RAM2 (SIDJ_RAM2: not
 * zeroed at start, so only for tables filled before use). Elsewhere (the host) all empty. */
#ifndef SIDJ_PORT_H
#define SIDJ_PORT_H
#if defined(__IMXRT1062__)
#define SIDJ_FLASH __attribute__((section(".flashmem"), optimize("O2")))
#define SIDJ_CONST __attribute__((section(".progmem")))
#define SIDJ_RAM2 __attribute__((section(".dmabuffers"), used, aligned(32)))
#else
#define SIDJ_FLASH
#define SIDJ_CONST
#define SIDJ_RAM2
#endif
#endif
