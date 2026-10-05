#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>

// Plays a long file that is streamed from the SD card, for the Loom
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
  void SetCrossfade(bool on) { xfade_loop = on; }
  // the crossfade, centred on the edges: h either side of END into h either
  // side of START (the cache then starts h before START); 0 when there is
  // too little room, then XfLen: the last f frames into the loop's first f
  size_t Half(size_t a, size_t b) const {
    if (!(xfade_loop && loop_)) return 0;
    const size_t h = std::min<size_t>({size_t(seam / 2), (b - a) / 2, a, frames > b ? frames - b : 0,
                                       cache_cap > 128 ? cache_cap / 2 - 64 : 0});
    return h >= 16 ? h : 0;
  }
  size_t XfLen(size_t a, size_t b) const {
    if (!(xfade_loop && loop_) || Half(a, b)) return 0;
    return std::min<size_t>({size_t(seam), (b - a) / 2, cache_cap > 64 ? cache_cap - 64 : 0});
  }
  void Play() { pos = double(ws); playing = true; }
  void Retrigger() { if (playing) retrig = true; else Play(); }  // dips ~5 ms, no click
  void SetLoop(bool l) { loop_ = l; }
  void Stop() { playing = false; }  // fades out over ~5 ms
  bool Playing() const { return playing; }
  float HeadPos() const { return frames ? float(pos / frames) : 0.0f; }
  bool Underran() { const bool u = underrun; underrun = false; return u; }

  // what the main loop should read next: the cache (for the loop's start),
  // else the ring ahead of the head. Returns false when both are full.
  // `into` / `at` / `count`: where to put it, which frames, how many.
  bool Next(int16_t*& l, int16_t*& r, size_t& at, size_t& count, size_t chunk) {
    if (!frames) return false;
    // the loop's start into the cache
    const size_t c0 = ws - Half(ws, we);  // centred crossfade: from h before START
    if (cs != c0) { __disable_irq(); cs = c0; cn = 0; __enable_irq(); }
    const size_t cache_want = std::min(cache_cap, we - c0);
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
    const size_t stop = std::min<size_t>(size_t(frames), size_t(we + Half(ws, we)));  // past END for the crossfade
    if (next >= stop || rn + chunk > ring_cap - 1024) return false;
    at = next;
    count = std::min(chunk, stop - next);
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
  volatile bool playing = false, underrun = false, retrig = false, loop_ = true;
  float run_g = 0.0f;  // start / stop glide
  volatile float speed = 1.0f, level = 1.0f, seam = 256.0f;
  volatile bool xfade_loop = false;
  bool pending_cache = false;

  // one frame, or false when it isn't buffered (yet)
  bool Frame(size_t i, int16_t& l, int16_t& r) const {
    if (i >= cs && i < cs + cn) { l = cache[0][i - cs]; r = cache[1][i - cs]; return true; }
    if (i >= rs && i < rs + rn) { const size_t k = i % ring_cap; l = ring[0][k]; r = ring[1][k]; return true; }
    return false;
  }
};

FASTRUN void AudioPlayStream::update() {
  if ((!playing && run_g <= 0.0f) || !ready()) return;
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    return;
  }
  const float sp = speed, g = level / 32768.0f;
  const size_t a = ws, b = we;
  const float wl = float(b - a), fade = std::min(float(seam), wl * 0.25f);
  // crossfade seam (looping): the last xl frames before END fade into the
  // loop's first xl frames (from the cache); then on from just after them
  const bool xon = xfade_loop && loop_;
  const size_t xh = Half(a, b);           // centred
  const size_t xl_ = xh ? 0 : XfLen(a, b); // else: the loop's own start
  const double xs = double(b) - double(xh ? xh : xl_);
  double p = pos;
  constexpr float run_step = 1.0f / (0.005f * AUDIO_SAMPLE_RATE_EXACT);
  const bool on = playing && !retrig;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    run_g = on ? std::min(1.0f, run_g + run_step) : std::max(0.0f, run_g - run_step);
    const size_t i = size_t(p);
    int16_t l0 = 0, r0 = 0, l1, r1;
    float yl = 0.0f, yr = 0.0f;
    if (Frame(i, l0, r0)) {
      if (!Frame(i + 1 < b || xh ? i + 1 : a, l1, r1)) { l1 = l0; r1 = r0; }
      const float fr = float(p - double(i));
      const float o = float(p - double(a));
      const float e = xon ? 1.0f : std::min(1.0f, std::min(o, wl - o) / fade);  // the seam
      yl = (l0 + fr * (l1 - l0)) * g * e;
      yr = (r0 + fr * (r1 - r0)) * g * e;
      if (xon && (xh || xl_ > 1) && p >= xs) {
        const double into = p - xs;
        const size_t k = a - xh + size_t(into);
        int16_t il0, ir0, il1, ir1;
        if (Frame(k, il0, ir0)) {
          if (!Frame(k + 1, il1, ir1)) { il1 = il0; ir1 = ir0; }
          const float fk = float(into - std::floor(into));
          const float t = float(into / double(xh ? 2 * xh : xl_));
          const float go = cosf(t * 1.5707963f), gi = sinf(t * 1.5707963f);
          yl = yl * go + (il0 + fk * (il1 - il0)) * g * gi;
          yr = yr * go + (ir0 + fk * (ir1 - ir0)) * g * gi;
        }
      }
    } else {
      underrun = true;
    }
    out[0]->data[n] = __SSAT(int32_t(yl * run_g * 32767.0f), 16);
    out[1]->data[n] = __SSAT(int32_t(yr * run_g * 32767.0f), 16);
    p += sp;
    if (xh && p >= double(b + xh)) {
      p = double(a + xh) + (p - double(b + xh));  // centred crossfade: on from after it
    } else if (!xh && p >= double(b)) {
      p = double(a) + double(xl_) + (p - double(b));  // crossfade: on from after the faded-in part
      if (!loop_) { playing = false; run_g = 0.0f; p = double(a); }  // once through
    }
    if (p < double(a)) p = double(a);
    if (retrig && run_g <= 0.0f) { retrig = false; p = double(a); }  // faded out: jump
  }
  pos = p;
  transmit(out[0], 0);
  transmit(out[1], 1);
  release(out[0]);
  release(out[1]);
}
