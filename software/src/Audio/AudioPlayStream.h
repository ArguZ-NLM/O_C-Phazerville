#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>

// Plays a long file that is streamed from the SD card, for the Sculpt
// PLAYER. The audio interrupt only ever reads PSRAM:
//  - the ring holds the stretch ahead of the play head, refilled by the
//    main loop (Service) from the card;
//  - the cache holds the first second of the loop, so the jump from its end
//    back to its start never waits for the card.
// The main loop owns the file; this class only has the buffers and the
// play head. Forward playback (SPEED 0..2x) for now.
class AudioPlayStream : public AudioStream {
public:
  AudioPlayStream() : AudioStream(0, nullptr) {}

  // --- main loop ---
  bool Acquire(float ring_secs, float cache_secs) {
    Release();
    ring_cap = size_t(ring_secs * AUDIO_SAMPLE_RATE_EXACT);
    cache_cap = size_t(cache_secs * AUDIO_SAMPLE_RATE_EXACT);
    for (int c = 0; c < 2; c++) {
      ring[c] = static_cast<int16_t*>(extmem_malloc(ring_cap * 2));
      cache[c] = static_cast<int16_t*>(extmem_malloc(cache_cap * 2));
    }
    if (!ready()) { Release(); return false; }
    return true;
  }
  void Release() {
    playing = false;
    for (int c = 0; c < 2; c++) {
      if (ring[c]) extmem_free(ring[c]);
      if (cache[c]) extmem_free(cache[c]);
      ring[c] = cache[c] = nullptr;
    }
    ring_cap = cache_cap = 0;
  }
  bool ready() const { return ring[0] && ring[1] && cache[0] && cache[1]; }

  // a new file of n frames: nothing buffered yet
  void Open(size_t n) {
    __disable_irq();
    playing = false; frames = n; pos = 0.0; ws = 0; we = n;
    rs = 0; rn = 0; cs = 0; cn = 0;
    __enable_irq();
  }
  void SetWindow(size_t a, size_t b) { ws = a; we = std::max(b, a + AUDIO_BLOCK_SAMPLES); }
  void SetSpeed(float s) { speed = std::max(0.0f, std::min(2.0f, s)); }
  void SetLevel(float l) { level = l; }
  void SetSeam(float s) { seam = std::max(16.0f, s); }
  void Play() { pos = double(ws); playing = true; }
  void Stop() { playing = false; }
  bool Playing() const { return playing; }
  float HeadPos() const { return frames ? float(pos / frames) : 0.0f; }
  bool Underran() { const bool u = underrun; underrun = false; return u; }

  // what the main loop should read next: the cache (for the loop's start),
  // else the ring ahead of the head. Returns false when both are full.
  // `into` / `at` / `count`: where to put it, which frames, how many.
  bool Next(int16_t*& l, int16_t*& r, size_t& at, size_t& count, size_t chunk) {
    if (!frames) return false;
    // the loop's start into the cache
    if (cs != ws) { __disable_irq(); cs = ws; cn = 0; __enable_irq(); }
    const size_t cache_want = std::min(cache_cap, we - ws);
    if (cn < cache_want) {
      at = cs + cn; count = std::min(chunk, cache_want - cn);
      l = cache[0] + cn; r = cache[1] + cn;
      pending_cache = true;
      return true;
    }
    pending_cache = false;
    // the ring: frames after the head (after the cache when the head is in it)
    const size_t head = size_t(pos);
    const size_t from = (head >= cs && head < cs + cn) ? cs + cn : head;
    if (from < rs || from > rs + rn) {  // the head jumped (loop, START): start over
      __disable_irq(); rs = from; rn = 0; __enable_irq();
    } else if (from > rs + 1024) {      // let go of what's played
      __disable_irq(); const size_t d = from - 1024 - rs; rs += d; rn -= d; __enable_irq();
    }
    const size_t next = rs + rn;
    if (next >= we || rn + chunk > ring_cap - 1024) return false;
    at = next;
    count = std::min(chunk, we - next);
    // the ring wraps: hand out the part up to its end
    const size_t slot = next % ring_cap;
    count = std::min(count, ring_cap - slot);
    l = ring[0] + slot; r = ring[1] + slot;
    return true;
  }
  // after the main loop has filled what Next() asked for
  void Filled(size_t count) {
    __disable_irq();
    if (pending_cache) cn += count; else rn += count;
    __enable_irq();
  }

  void update() override;

private:
  int16_t* ring[2] = {};
  int16_t* cache[2] = {};
  size_t ring_cap = 0, cache_cap = 0;
  volatile size_t frames = 0, ws = 0, we = 0;
  volatile size_t rs = 0, rn = 0;  // ring: frames rs .. rs + rn
  volatile size_t cs = 0, cn = 0;  // cache: frames cs .. cs + cn
  volatile double pos = 0.0;
  volatile bool playing = false, underrun = false;
  volatile float speed = 1.0f, level = 1.0f, seam = 256.0f;
  bool pending_cache = false;

  // one frame, or false when it isn't buffered (yet)
  bool Frame(size_t i, int16_t& l, int16_t& r) const {
    if (i >= cs && i < cs + cn) { l = cache[0][i - cs]; r = cache[1][i - cs]; return true; }
    if (i >= rs && i < rs + rn) { const size_t k = i % ring_cap; l = ring[0][k]; r = ring[1][k]; return true; }
    return false;
  }
};

FASTRUN void AudioPlayStream::update() {
  if (!playing || !ready()) return;
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    return;
  }
  const float sp = speed, g = level / 32768.0f;
  const size_t a = ws, b = we;
  const float wl = float(b - a), fade = std::min(float(seam), wl * 0.25f);
  double p = pos;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const size_t i = size_t(p);
    int16_t l0 = 0, r0 = 0, l1, r1;
    float yl = 0.0f, yr = 0.0f;
    if (Frame(i, l0, r0)) {
      if (!Frame(i + 1 < b ? i + 1 : a, l1, r1)) { l1 = l0; r1 = r0; }
      const float fr = float(p - double(i));
      const float o = float(p - double(a));
      const float e = std::min(1.0f, std::min(o, wl - o) / fade);  // the seam
      yl = (l0 + fr * (l1 - l0)) * g * e;
      yr = (r0 + fr * (r1 - r0)) * g * e;
    } else {
      underrun = true;
    }
    out[0]->data[n] = __SSAT(int32_t(yl * 32767.0f), 16);
    out[1]->data[n] = __SSAT(int32_t(yr * 32767.0f), 16);
    p += sp;
    if (p >= double(b)) p = double(a) + (p - double(b));
    if (p < double(a)) p = double(a);
  }
  pos = p;
  transmit(out[0], 0);
  transmit(out[1], 1);
  release(out[0]);
  release(out[1]);
}
