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

// the memory budget: what the module has (0, 8 or 16 MB of PSRAM, read at
// start) and what Loom's blocks hold of it, counted as they are taken and
// given back, so a device can ask what is left without walking the pool
inline size_t psram_used = 0;
extern "C" uint32_t _extram_start, _extram_end;  // (the linker's: Loom's fixed EXTMEM variables)
// what the pool can hand out: the chips less the fixed EXTMEM variables
inline size_t PsramTotal() {
#if defined(ARDUINO_TEENSY41)
  if (!external_psram_size) return 0;
  const size_t fixed = size_t(reinterpret_cast<char*>(&_extram_end) - reinterpret_cast<char*>(&_extram_start));
  return (size_t(external_psram_size) << 20) - fixed;
#else
  return 0;
#endif
}
inline size_t PsramRoom() { const size_t t = PsramTotal(); return t > psram_used ? t - psram_used : 0; }

#ifdef PSRAM_TRACE
// (PsramTrace.cpp: USB 'A' lists who holds the PSRAM; these blocks don't
// go through extmem_malloc, so they are told about here)
void PsramTraceAdd(void* p, size_t n, uint32_t at);
void PsramTraceDrop(void* p);
// who took the block: PsramAlloc's caller (it is never inlined, so this
// is the device's own code)
#define PSRAM_HERE() uint32_t(__builtin_return_address(0))
#endif

// a zeroed block, in PSRAM if there is room (else in RAM2, like extmem_calloc)
__attribute__((noinline)) inline void* PsramAlloc(size_t n) {
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
      psram_used += n;
#ifdef PSRAM_TRACE
      PsramTraceAdd(p, n, PSRAM_HERE());
#endif
      return p;
    }
  }
#endif
  return calloc(n, 1);
}

inline void PsramFree(void* p);

// working memory an engine touches every block: RAM2 if there is room
// (quicker), PSRAM when RAM2's heap is short. Zeroed. FastFree gives
// either kind back
inline void* FastAlloc(size_t n) {
  void* p = calloc(n, 1);
  return p ? p : PsramAlloc(n);
}
inline void FastFree(void* p);

inline void PsramFree(void* p) {
  if (!p) return;
#if defined(ARDUINO_TEENSY41)
  if (IsPsram(p)) {
#ifdef PSRAM_TRACE
    PsramTraceDrop(p);
#endif
    // the block's sizes sit in its header, just before it (smalloc: rsz,
    // usz, tag); after the user's usz bytes come a check tag (one header
    // long, cleared by sm_free itself) and the slack, up to rsz + a header
    const size_t rsz = static_cast<const size_t*>(p)[-3];
    const size_t usz = static_cast<const size_t*>(p)[-2];
    psram_used = psram_used > usz ? psram_used - usz : 0;
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

inline void FastFree(void* p) {
  if (!p) return;
#if defined(ARDUINO_TEENSY41)
  if (IsPsram(p)) { PsramFree(p); return; }
#endif
  free(p);
}
