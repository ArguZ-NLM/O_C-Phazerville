#pragma once
// PSRAM blocks for the audio engines, without the audio dropping out.
//
// The core's allocator clears a block when it is handed out and again when
// it is given back (megabytes for FIBRE, DRAFT, SPOOL...). Done in one go,
// that fills the CPU's data cache with written-to PSRAM lines; every audio
// update then has to write them out to the chip before it can load its own
// data, runs late, and blocks of sound are lost (~50 ms when FIBRE came in).
//
// Here a block is cleared when it is handed out and when it comes back as
// before, but in small pieces, each written out to the chip at once.
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "smalloc.h"
extern "C" uint8_t external_psram_size;

// zero n bytes of PSRAM, 4 KB at a time, none of it left in the cache
inline void PsramZero(void* p, size_t n) {
  char* c = static_cast<char*>(p);
  while (n) {
    const size_t k = n < 4096 ? n : 4096;
    memset(c, 0, k);
    arm_dcache_flush_delete(c, k);
    c += k;
    n -= k;
  }
}

inline bool IsPsram(const void* p) { return (uint32_t(p) >> 28) == 7; }

// a zeroed block, in PSRAM if there is room (else in RAM2, like extmem_calloc)
inline void* PsramAlloc(size_t n) {
#if defined(ARDUINO_TEENSY41)
  if (external_psram_size) {
    smalloc_pool& pool = extmem_smalloc_pool;
    const int z = pool.do_zero;
    pool.do_zero = 0;
    void* p = sm_malloc_pool(&pool, n);
    pool.do_zero = z;
    if (p) {
      // (cleared all the same: an engine still writing while its block
      // went back would leave something in "free" memory)
      PsramZero(p, n);
      return p;
    }
  }
#endif
  return calloc(n, 1);
}

inline void PsramFree(void* p) {
  if (!p) return;
#if defined(ARDUINO_TEENSY41)
  if (IsPsram(p)) {
    // the block's sizes sit in its header, just before it (smalloc: rsz,
    // usz, tag); after the user's usz bytes come a check tag (one header
    // long, cleared by sm_free itself) and the slack, up to rsz + a header
    const size_t rsz = static_cast<const size_t*>(p)[-3];
    const size_t usz = static_cast<const size_t*>(p)[-2];
    constexpr size_t HDR = 3 * sizeof(size_t);
    PsramZero(p, usz);
    smalloc_pool& pool = extmem_smalloc_pool;
    const int z = pool.do_zero;
    pool.do_zero = 0;
    sm_free_pool(&pool, p);  // (checks the tag first)
    pool.do_zero = z;
    if (rsz > usz) PsramZero(static_cast<char*>(p) + usz + HDR, rsz - usz);
    return;
  }
#endif
  free(p);
}
