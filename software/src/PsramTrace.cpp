// PSRAM trace (Loom builds with PSRAM_TRACE): every extmem_ block taken
// and who took it (the caller's address), so USB 'A' can list what holds
// the PSRAM. The linker sends extmem_malloc / calloc / realloc / free here
// (-Wl,--wrap=...).
#ifdef PSRAM_TRACE
#include <Arduino.h>
#include <smalloc.h>

extern "C" {
void* __real_extmem_malloc(size_t);
void* __real_extmem_calloc(size_t, size_t);
void* __real_extmem_realloc(void*, size_t);
void __real_extmem_free(void*);
}

namespace {
struct Entry { void* p; uint32_t size; uint32_t caller; };
constexpr int N = 128;
Entry table[N];
void Add(void* p, size_t size, uint32_t caller) {
  if (!p) return;
  for (auto& e : table) if (!e.p) { e = {p, uint32_t(size), caller}; return; }
}
void Drop(void* p) {
  if (!p) return;
  for (auto& e : table) if (e.p == p) { e.p = nullptr; return; }
}
}

extern "C" {
void* __wrap_extmem_malloc(size_t size) {
  void* p = __real_extmem_malloc(size);
  Add(p, size, uint32_t(__builtin_return_address(0)));
  return p;
}
void* __wrap_extmem_calloc(size_t n, size_t size) {
  void* p = __real_extmem_calloc(n, size);
  Add(p, n * size, uint32_t(__builtin_return_address(0)));
  return p;
}
void* __wrap_extmem_realloc(void* old, size_t size) {
  void* p = __real_extmem_realloc(old, size);
  if (p) { Drop(old); Add(p, size, uint32_t(__builtin_return_address(0))); }
  return p;
}
void __wrap_extmem_free(void* p) {
  Drop(p);
  __real_extmem_free(p);
}
}

// USB 'A': the blocks in use, biggest first
void PsramTraceReport() {
  uint32_t total = 0;
  bool shown[N] = {};
  Serial.println("--- PSRAM blocks (KB, caller) ---");
  for (;;) {
    int best = -1;
    for (int i = 0; i < N; i++)
      if (table[i].p && !shown[i] && (best < 0 || table[i].size > table[best].size)) best = i;
    if (best < 0) break;
    shown[best] = true;
    total += table[best].size;
    Serial.printf("%6lu KB  0x%08lx\n", (unsigned long)(table[best].size / 1024), (unsigned long)table[best].caller);
    delay(2);
  }
  Serial.printf("--- %lu KB in all ---\n", (unsigned long)(total / 1024));
  {  // the pool's own count (slow: it walks the PSRAM)
    size_t pool = 0, user = 0, free_b = 0;
    int nblk = 0;
    sm_malloc_stats_pool(&extmem_smalloc_pool, &pool, &user, &free_b, &nblk);
    Serial.printf("--- pool: %u KB used in %d blocks, %u KB free ---\n", unsigned(user / 1024), nblk, unsigned(free_b / 1024));
  }
  // the biggest piece one buffer could still take (slow: only on request)
  size_t lo = 0, hi = 8u << 20;
  while (hi - lo > 4096) {
    const size_t mid = (lo + hi) / 2;
    void* t = __real_extmem_malloc(mid);
    if (t) { __real_extmem_free(t); lo = mid; } else hi = mid;
  }
  Serial.printf("--- biggest free piece %lu KB ---\n", (unsigned long)(lo / 1024));
}
#else
void PsramTraceReport() {}
#endif
