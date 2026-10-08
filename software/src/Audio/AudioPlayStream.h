#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

// Plays a long file that is streamed from the SD card, for the Loom
// PLAYER. The audio interrupt only ever reads PSRAM:
//  - the ring holds the stretch ahead of the play head, refilled by the
//    main loop (Service) from the card;
//  - two caches hold the first second of the loop (one for the loop playing,
//    one for a new START waiting for the seam), so the jump from its end
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
      for (int k = 0; k < 2; k++) cache[k][c] = static_cast<int16_t*>(extmem_malloc(cache_cap * 2));
    }
    if (!ready()) { Release(); return false; }
    return true;
  }
  void Release() {
    playing = false;
    for (int c = 0; c < 2; c++) {
      if (ring[c]) extmem_free(ring[c]);
      for (int k = 0; k < 2; k++) { if (cache[k][c]) extmem_free(cache[k][c]); cache[k][c] = nullptr; }
      ring[c] = nullptr;
    }
    ring_cap = cache_cap = 0;
  }
  bool ready() const { return ring[0] && ring[1] && cache[0][0] && cache[0][1] && cache[1][0] && cache[1][1]; }

  // a new file of n frames: nothing buffered yet
  void Open(size_t n) {
    __disable_irq();
    playing = false; frames = n; pos = 0.0; ws = 0; we = n; nws = 0; nwe = n; h_l = 0; lt_a = 0; lt_b = n;
    rs = 0; rn = 0; cs[0] = cs[1] = 0; cn[0] = cn[1] = 0;
    __enable_irq();
  }
  // a new loop window: taken over in the audio update, under a ~3 ms dip
  void SetWindow(size_t a, size_t b) {
    __disable_irq();
    nws = a; nwe = std::max(b, a + AUDIO_BLOCK_SAMPLES);
    if (!playing || !frames) { ws = nws; we = nwe; lt_a = nws; lt_b = nwe; }
    __enable_irq();
  }
  void SetSpeed(float s) { speed = std::max(0.0f, std::min(2.0f, s)); }
  void SetLevel(float l) { level = l; }
  void SetSeam(float s) { seam = std::max(16.0f, s); }
  void SetCrossfade(bool on) { xfade_loop = on; }
  // the crossfade, centred on the edges: h either side of END into h either
  // side of START (the cache then starts h before START); 0 when there is
  // too little room, then XfLen: the last f frames into the loop's first f
  // ta / tb: the window the loop goes on with (a new one waits for the seam)
  size_t Half(size_t a, size_t b, size_t ta = SIZE_MAX, size_t tb = 0) const {
    if (!(xfade_loop && loop_)) return 0;
    if (ta == SIZE_MAX) { ta = a; tb = b; }
    const size_t h = std::min<size_t>({size_t(seam / 2), (b - a) / 2, (tb - ta) / 2, ta,
                                       frames > b ? frames - b : 0,
                                       cache_cap > 128 ? cache_cap / 2 - 64 : 0});
    return h >= 16 ? h : 0;
  }
  size_t XfLen(size_t a, size_t b, size_t ta = SIZE_MAX, size_t tb = 0) const {
    if (ta == SIZE_MAX) { ta = a; tb = b; }
    if (!(xfade_loop && loop_) || Half(a, b, ta, tb)) return 0;
    return std::min<size_t>({size_t(seam), (b - a) / 2, (tb - ta) / 2, cache_cap > 64 ? cache_cap - 64 : 0});
  }
  void Play() { pos = double(ws); playing = true; }
  // Play() n frames from now, counted in the audio (start on a beat); -1:
  // none waiting
  void PlayIn(int32_t frames) { play_in = std::max<int32_t>(0, frames); }
  void CancelPlay() { play_in = -1; }
  bool PlayWaiting() const { return play_in >= 0; }
  void Retrigger() { if (playing) retrig = true; else Play(); }  // dips ~5 ms, no click
  void SetLoop(bool l) { loop_ = l; }
  void Stop() { playing = false; }  // fades out over ~5 ms
  bool Playing() const { return playing; }
  bool Silent() const { return !playing && run_g <= 0.0f; }  // faded right out
  float HeadPos() const { return frames ? float(pos / frames) : 0.0f; }
  bool Underran() { const bool u = underrun; underrun = false; return u; }
  void Debug(char* buf, size_t n) const {
    snprintf(buf, n, "STRM play %d win %u-%u set %u-%u into %u-%u h %u pos %u cache %u+%u %u+%u ring %u+%u",
             int(playing), unsigned(ws), unsigned(we), unsigned(nws), unsigned(nwe), unsigned(lt_a), unsigned(lt_b),
             unsigned(h_l), unsigned(pos), unsigned(cs[0]), unsigned(cn[0]), unsigned(cs[1]), unsigned(cn[1]),
             unsigned(rs), unsigned(rn));
  }

  // what the main loop should read next: the cache (for the loop's start),
  // else the ring ahead of the head. Returns false when both are full.
  // `into` / `at` / `count`: where to put it, which frames, how many.
  bool Next(int16_t*& l, int16_t*& r, size_t& at, size_t& count, size_t chunk) {
    if (!frames) return false;
    // the loop's start into the cache
    // the start crossed into (h before it when centred): in the cache that
    // already has it, else in the one the head isn't playing from
    const size_t c0 = lt_a - h_l;
    const size_t head = size_t(pos);
    int k = cs[0] == c0 ? 0 : cs[1] == c0 ? 1 : -1;
    if (k < 0) {
      k = (head >= cs[0] && head < cs[0] + cn[0]) ? 1 : 0;
      __disable_irq(); cs[k] = c0; cn[k] = 0; __enable_irq();
    }
    const size_t cache_want = std::min(cache_cap, size_t(lt_b) - c0);
    if (cn[k] < cache_want) {
      at = cs[k] + cn[k]; count = std::min(chunk, cache_want - cn[k]);
      l = cache[k][0] + cn[k]; r = cache[k][1] + cn[k];
      pending_cache = k + 1;
      return true;
    }
    pending_cache = 0;
    // the ring: frames after the head (after a cache when the head is in it)
    size_t from = head;
    for (int j = 0; j < 2; j++) if (head >= cs[j] && head < cs[j] + cn[j]) from = cs[j] + cn[j];
    if (from < rs || from > rs + rn) {  // the head jumped (loop, START): start over
      __disable_irq(); rs = from; rn = 0; __enable_irq();
    } else if (from > rs + 1024) {      // let go of what's played
      __disable_irq(); const size_t d = from - 1024 - rs; rs += d; rn -= d; __enable_irq();
    }
    const size_t next = rs + rn;
    const size_t stop = std::min<size_t>(size_t(frames), size_t(we + h_l));  // past END for the crossfade
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
    if (pending_cache) cn[pending_cache - 1] += count; else rn += count;
    __enable_irq();
  }

  void update() override;

private:
  int16_t* ring[2] = {};
  int16_t* cache[2][2] = {};  // [which][channel]
  size_t ring_cap = 0, cache_cap = 0;
  volatile size_t frames = 0, ws = 0, we = 0;
  volatile size_t rs = 0, rn = 0;  // ring: frames rs .. rs + rn
  volatile size_t cs[2] = {}, cn[2] = {};  // caches: frames cs .. cs + cn
  volatile double pos = 0.0;
  volatile bool playing = false, underrun = false, retrig = false, loop_ = true;
  volatile int32_t play_in = -1;
  float run_g = 0.0f;  // start / stop glide
  volatile float speed = 1.0f, level = 1.0f, seam = 256.0f;
  volatile bool xfade_loop = false;
  volatile size_t nws = 0, nwe = 0;  // the window asked for (SetWindow)
  float win_g = 1.0f;                // dip while the window changes
  float ur_g = 1.0f, last_l = 0.0f, last_r = 0.0f;  // underrun glide
  bool wrapped = false;  // just came round: the next window / crossfade may change
  volatile size_t h_l = 0;           // the crossfade half in use (changes away from the seam)
  volatile size_t lt_a = 0, lt_b = 0; // ... and the window it crosses into
  int pending_cache = 0;  // 1 / 2: Next() handed out cache 0 / 1

  // one frame, or false when it isn't buffered (yet)
  bool Frame(size_t i, int16_t& l, int16_t& r) const {
    for (int k = 0; k < 2; k++)
      if (i >= cs[k] && i < cs[k] + cn[k]) { l = cache[k][0][i - cs[k]]; r = cache[k][1][i - cs[k]]; return true; }
    if (i >= rs && i < rs + rn) { const size_t k = i % ring_cap; l = ring[0][k]; r = ring[1][k]; return true; }
    return false;
  }
};

FASTRUN void AudioPlayStream::update() {
  if (play_in >= 0 && ready()) {  // a start waiting for its moment (a beat)
    const int32_t left = play_in - AUDIO_BLOCK_SAMPLES;
    if (left < 0) { play_in = -1; pos = double(ws); playing = true; }
    else play_in = left;
  }
  if ((!playing && run_g <= 0.0f) || !ready()) return;
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    return;
  }
  const float sp = speed, g = level / 32768.0f;
  // a new window: when the head is well inside it, simply taken (playback
  // goes on unchanged); when the head would have to jump, a ~3 ms dip out,
  // take it, back in
  constexpr float win_step = 1.0f / (0.003f * AUDIO_SAMPLE_RATE_EXACT);
  // looping: a new window waits until the loop comes round (taken at the
  // seam, see below); once through: as described
  bool win_change = !loop_ && (nws != ws || nwe != we);
  if (win_change) {
    const double p0 = pos;
    const double safe_end = double(nwe) - double(h_l) - AUDIO_BLOCK_SAMPLES;
    if (win_g <= 0.0f || (p0 >= double(nws) && p0 < safe_end && p0 < double(we) - double(h_l) - AUDIO_BLOCK_SAMPLES)) {
      ws = nws; we = nwe; lt_a = nws; lt_b = nwe;
      if (pos < double(ws) || pos >= double(we)) pos = double(ws);
      win_change = false;
    }
  }
  // the crossfade half changes only while playing away from the seam and
  // the cached start (so the cache can be refilled before it is needed)
  // (the window crossed into too: a new START gets cached before the seam)
  {
    const size_t h = Half(ws, we, nws, nwe);
    const double p0 = pos;
    // (with two caches the new start is read while the old one plays; only
    // the last moments before the seam keep what they have)
    const size_t wlen = we - ws;
    const double margin = std::min<double>(AUDIO_BLOCK_SAMPLES * 4, wlen / 4);
    const bool stale = h_l > wlen / 2;  // the loop got shorter than its crossfade
    const bool away = stale || wrapped || wlen < AUDIO_BLOCK_SAMPLES * 8 ||
                      p0 < double(we) - double(std::max(h, size_t(h_l))) - margin;
    if ((h != h_l || nws != lt_a || nwe != lt_b) && away) { h_l = h; lt_a = nws; lt_b = nwe; }
    wrapped = false;
    if (!xfade_loop || !loop_) h_l = 0;
    if (!loop_) { lt_a = ws; lt_b = we; }
  }
  size_t a = ws, b = we;  // (the window changes at the seam, mid-block too)
  float wl = float(b - a);
  const float fade = std::min(float(seam), wl * 0.25f);
  // crossfade seam (looping): the last xl frames before END fade into the
  // loop's first xl frames (from the cache); then on from just after them
  const bool xon = xfade_loop && loop_;
  const size_t xh = h_l;                  // centred
  const size_t ta = lt_a, tb = lt_b;
  const size_t xl_ = xh ? 0 : XfLen(a, b, ta, tb); // else: the loop's own start
  double xs = double(b) - double(xh ? xh : xl_);
  double p = pos;
  // start / stop: 20 ms, eased (5 ms straight clicked on low notes); the
  // dip of a retrigger stays short (5 ms)
  const float run_step = 1.0f / ((retrig ? 0.005f : 0.02f) * AUDIO_SAMPLE_RATE_EXACT);
  const bool on = playing && !retrig;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    run_g = on ? std::min(1.0f, run_g + run_step) : std::max(0.0f, run_g - run_step);
    win_g = win_change ? std::max(0.0f, win_g - win_step) : std::min(1.0f, win_g + win_step);
    const size_t i = size_t(p);
    int16_t l0 = 0, r0 = 0, l1, r1;
    float yl = 0.0f, yr = 0.0f;
    const bool got_frame = Frame(i, l0, r0);
    if (got_frame) {
      if (!Frame(i + 1 < b || xh ? i + 1 : a, l1, r1)) { l1 = l0; r1 = r0; }
      const float fr = float(p - double(i));
      const float o = float(p - double(a));
      const float e = xon ? 1.0f : std::min(1.0f, std::min(o, wl - o) / fade);  // the seam
      yl = (l0 + fr * (l1 - l0)) * g * e;
      yr = (r0 + fr * (r1 - r0)) * g * e;
      if (xon && (xh || xl_ > 1) && p >= xs) {
        const double into = p - xs;
        const size_t k = ta - xh + size_t(into);
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
    // the card behind (or the buffers starting over after a jump): glide
    // down on the last sound instead of dropping to silence, back up after
    if (got_frame) { ur_g = std::min(1.0f, ur_g + 1.0f / (0.003f * AUDIO_SAMPLE_RATE_EXACT)); last_l = yl; last_r = yr; }
    else { ur_g = std::max(0.0f, ur_g - 1.0f / (0.002f * AUDIO_SAMPLE_RATE_EXACT)); yl = last_l; yr = last_r; }
    yl *= ur_g; yr *= ur_g;
    if (!loop_) {  // once through: the file's end eased out over ~20 ms
      const float left = std::min(1.0f, float(double(b) - p) * (1.0f / (0.02f * AUDIO_SAMPLE_RATE_EXACT)));
      const float ge = left > 0.0f ? left * left * (3.0f - 2.0f * left) : 0.0f;
      yl *= ge; yr *= ge;
    }
    const float rg = run_g * run_g * (3.0f - 2.0f * run_g);
    out[0]->data[n] = __SSAT(int32_t(yl * rg * win_g * 32767.0f), 16);
    out[1]->data[n] = __SSAT(int32_t(yr * rg * win_g * 32767.0f), 16);
    p += sp;
    if (xh && p >= double(b + xh)) {
      // centred crossfade: on from after it, in the window crossed into
      p = double(ta + xh) + (p - double(b + xh));
      ws = a = ta; we = b = tb;
      wrapped = true;
      wl = float(b - a); xs = double(b) - double(xh);
    } else if (!xh && p >= double(b)) {
      p = double(ta) + double(xon ? xl_ : 0) + (p - double(b));  // on from after the faded-in part
      ws = a = ta; we = b = tb;
      wrapped = true;
      wl = float(b - a); xs = double(b) - double(xl_);
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
