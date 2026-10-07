#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

// Stereo looper for the Loom SPOOL device. The loop lives in PSRAM (up to
// 16 s; less if memory is short). A press cycles REC -> PLAY -> DUB -> PLAY;
// the first REC sets the loop length. Stop keeps the loop (a press plays it
// again from the start), Clear empties it. SPEED plays it at any rate, backwards
// below zero; START / LEN pick the part that loops, with short fades at its
// ends so the seam doesn't click. MON passes the live input through.
class AudioEffectReel : public AudioStream {
public:
  enum State : uint8_t { EMPTY, REC, PLAY, DUB, STOPPED };
  static constexpr int HIST = 128;  // picture: level per 1/HIST of the buffer

  AudioEffectReel() : AudioStream(2, queue) {}

  // --- main loop, while the stream is not patched in ---
  // tries the longest that fits, in seconds; returns the length it got
  float Acquire(float secs) {
    Release();
    for (float s = secs; s >= 2.0f; s *= 0.5f) {
      const size_t n = size_t(s * AUDIO_SAMPLE_RATE_EXACT);
      int16_t* l = static_cast<int16_t*>(extmem_calloc(n, sizeof(int16_t)));
      int16_t* r = l ? static_cast<int16_t*>(extmem_calloc(n, sizeof(int16_t))) : nullptr;
      if (l && r) { buf[0] = l; buf[1] = r; cap = n; return s; }
      if (l) extmem_free(l);
    }
    return 0.0f;
  }
  // a buffer of exactly n frames, also while patched in: the new one is
  // swapped in between two audio updates (the loop starts over empty);
  // false if it doesn't fit (the old one is kept)
  bool Resize(size_t n) {
    if (n == cap && ready()) return true;
    int16_t* l = static_cast<int16_t*>(extmem_calloc(n, sizeof(int16_t)));
    int16_t* r = l ? static_cast<int16_t*>(extmem_calloc(n, sizeof(int16_t))) : nullptr;
    if (!l || !r) { if (l) extmem_free(l); return false; }
    int16_t* old[2];
    AudioNoInterrupts();
    old[0] = buf[0]; old[1] = buf[1];
    buf[0] = l; buf[1] = r;
    cap = n;
    Reset();
    AudioInterrupts();
    for (auto b : old) if (b) extmem_free(b);
    return true;
  }
  // --- SPOOL: memory only for what is recorded ---
  // Empty, it keeps just PARK frames (the live input still passes through);
  // REC first takes the longest buffer that fits (Grow, whole seconds down
  // to 1 s), and when the recording ends the loop moves into a buffer of its
  // own length (Shrink), so the rest goes back for the other track.
  static constexpr size_t PARK = 2 * AUDIO_BLOCK_SAMPLES;
  // UI thread, while EMPTY, right before the press that starts REC
  bool Grow(float secs) {
    for (int s = int(secs); s >= 1; s--) {
      const size_t n = size_t(s * AUDIO_SAMPLE_RATE_EXACT);
      if (n <= cap) return true;
      if (Swap(n, false)) return true;
    }
    return cap > PARK;
  }
  // main loop, while the stream is not patched in: just the small buffer
  bool Begin() { Release(); return Swap(PARK, true); }
  // main loop: empty again (cleared) -> back to the small buffer
  void Park() {
    if (st == EMPTY && !press && cap > PARK) Swap(PARK, true);
  }
  // main loop: after the first recording, keep just the loop
  void Shrink() {
    if ((st != PLAY && st != STOPPED) || !len || cap <= len + PARK) return;
    const size_t n = len;
    int16_t* l = static_cast<int16_t*>(extmem_malloc(n * sizeof(int16_t)));
    int16_t* r = l ? static_cast<int16_t*>(extmem_malloc(n * sizeof(int16_t))) : nullptr;
    if (!l || !r) { if (l) extmem_free(l); return; }
    memcpy(l, buf[0], n * sizeof(int16_t));
    memcpy(r, buf[1], n * sizeof(int16_t));
    int16_t* old[2] = {l, r};
    AudioNoInterrupts();
    if ((st == PLAY || st == STOPPED) && len == n) {  // not overdubbing meanwhile
      old[0] = buf[0]; old[1] = buf[1];
      buf[0] = l; buf[1] = r;
      cap = n;
    }
    AudioInterrupts();
    for (auto b : old) extmem_free(b);
  }
  void Release() {
    for (auto& b : buf) { if (b) extmem_free(b); b = nullptr; }
    cap = 0;
    Reset();
  }
  bool ready() const { return buf[0] && buf[1]; }
  void Debug(char* buf_, size_t n) const {
    snprintf(buf_, n, "MEM st %d win %u+%u set %u+%u pend %d %u+%u into %u+%u h %u f %u off %d",
             int(st), unsigned(ws), unsigned(wl), unsigned(ex_ws), unsigned(ex_wl), int(pend), unsigned(p_ws), unsigned(p_wl),
             unsigned(lt_ws), unsigned(lt_wl), unsigned(xh_l), unsigned(xf_l), int(off));
  }
  volatile uint8_t pick = 0;  // 0 stereo, 1 the left side on both, 2 the right side
  float CapSecs() const { return cap / AUDIO_SAMPLE_RATE_EXACT; }

  // --- UI thread; taken over at the next block ---
  void Press() { press = true; }
  // a press n frames from now, counted in the audio (start on a beat);
  // -1: none waiting
  void PressIn(int32_t frames) { press_in = std::max<int32_t>(0, frames); }
  void CancelPress() { press_in = -1; }
  bool PressWaiting() const { return press_in >= 0; }
  void Clear() { clear = true; }
  void Stop() { stop = true; }
  void Retrigger() { retrig = true; }  // back to the start, crossfaded
  void SetLoop(bool l) { loop_ = l; }  // off: play once and stop
  void SetSpeed(float s) { speed = s; }
  void SetWindow(float start, float len) { start_ = start; len_ = len; exact = false; }
  // the window in whole samples (the player sets its edges exactly)
  void SetWindowSamples(size_t start, size_t length) { ex_ws = start; ex_wl = length; exact = true; }
  void SetSeam(float samples) { seam = std::max(16.0f, samples); }  // fade at the loop ends
  // PLAYER: looping forward, the seam is a real crossfade (the loop's end
  // plays into what leads up to its start) instead of a dip
  void SetCrossfade(bool on) { xfade_loop = on; }
  // the crossfade, centred on the loop's edges: h frames either side of
  // END cross into h either side of START. Where the file has too little
  // before START or after END, h shrinks; when nothing is left (h 0) it
  // falls back to fading the last f frames into the loop's own first f.
  // nws / nwl: the window the loop goes on with (a new one waits for the seam)
  static void XfPlan(float seam, size_t ws, size_t wl, size_t len, size_t& h, size_t& f,
                     size_t nws = SIZE_MAX, size_t nwl = 0) {
    if (nws == SIZE_MAX) { nws = ws; nwl = wl; }
    h = std::min<size_t>({size_t(seam / 2), wl / 2, nwl / 2, nws, len > ws + wl ? len - ws - wl : 0});
    if (h < 16) h = 0;
    f = h ? 0 : std::min<size_t>({size_t(seam), wl / 2, nwl / 2});
  }
  // PLAYER: a new START / END waits until the loop comes round (no jump)
  void SetDeferWindow(bool on) { defer_win = on; }
  void SetDub(float keep) { keep_ = keep; }
  void SetMonitor(float m) { mon = m; }
  void SetLevel(float l) { level = l; }

  State state() const { return st; }
  size_t Length() const { return len; }
  size_t Capacity() const { return cap; }
  // --- the player loads into, and SPOOL saves from, the buffer directly
  // (main loop; while loading the engine stays silent)
  int16_t* Data(int c) { return buf[c]; }
  const int16_t* Data(int c) const { return buf[c]; }
  void BeginLoad() { st = EMPTY; len = 0; std::fill(hist, hist + HIST, 0); }
  void MarkRange(size_t from, size_t to) {  // picture levels of loaded audio
    for (size_t i = from; i < to; i++) Mark(i, buf[0][i], buf[1][i]);
  }
  void FinishLoad(size_t n) {
    off = 0.0f; xf = 0; ws = 0; wl = n;
    len = n;
    st = n ? STOPPED : EMPTY;
  }
  // loudest sample between two positions (reads every step-th), for a
  // zoomed picture
  int Peak(size_t from, size_t to, size_t step) const {
    int pk = 0;
    for (size_t i = from; i < to && i < len; i += step)
      pk = std::max(pk, std::max(abs(buf[0][i]), abs(buf[1][i])));
    return pk;
  }
  float LoopSecs() const { return len / AUDIO_SAMPLE_RATE_EXACT; }
  // where the play / record head is, as a fraction of the loop (REC: of
  // the buffer)
  float HeadPos() const {
    if (st == REC) return cap ? float(wpos) / cap : 0.0f;
    return len ? float((ws + size_t(off)) % len) / len : 0.0f;
  }
  // the looping part, as fractions of the loop
  void Window(float& a, float& b) const {
    a = len ? float(ws) / len : 0.0f;
    b = len ? float(ws + wl) / len : 1.0f;
  }
  // levels over the recorded loop (REC: over what is recorded so far),
  // HIST columns
  // the zoomed picture: cols columns of `span` frames from `from` (may be
  // < 0 or past the end: wraps round the loop when playing, blank while
  // recording), the lowest / highest point of each (L and R mixed)
  size_t HeadFrame() const {
    if (st == REC) return wpos;
    return len ? (ws + size_t(off)) % len : 0;
  }
  void ReadWave(long from, long span, int cols, int16_t* lo, int16_t* hi) const {
    const long n = long(st == REC ? wpos : len);
    for (int k = 0; k < cols; k++) {
      const long a = from + span * k / cols, b = std::max(a + 1, from + span * (k + 1) / cols);
      const long step = std::max(1L, (b - a) / 48);
      int l = 32767, h = -32768;
      for (long i = a; i < b && n > 0; i += step) {
        long j = i;
        if (st == REC) { if (j < 0 || j >= n) continue; }
        else j = ((j % n) + n) % n;
        const int v = (buf[0][j] + buf[1][j]) >> 1;
        l = std::min(l, v); h = std::max(h, v);
      }
      if (l > h) l = h = 0;
      lo[k] = int16_t(l); hi[k] = int16_t(h);
    }
  }
  // HIST columns, read from the audio itself (a short loop only covers a
  // slot or two of the coarse hist, which drew a solid block)
  void ReadHistory(uint8_t* dst) const {
    const size_t n = st == REC ? wpos : len;
    if (!n || !cap) { std::fill(dst, dst + HIST, 0); return; }
    const size_t span = std::max<size_t>(1, n / HIST);
    const size_t step = std::max<size_t>(1, span / 48);  // ~48 reads a column is plenty
    for (int k = 0; k < HIST; k++) {
      const size_t from = size_t(k) * n / HIST, to = std::min(n, from + span);
      int pk = 0;
      for (size_t i = from; i < to; i += step)
        pk = std::max(pk, std::max(abs(buf[0][i]), abs(buf[1][i])));
      dst[k] = uint8_t(std::min(255, pk >> 7));
    }
  }

  void update() override;

private:
  audio_block_t* queue[2];
  int16_t* buf[2] = {};
  size_t cap = 0, len = 0, wpos = 0, ws = 0, wl = 0;
  float off = 0.0f;  // play head, samples into the window
  // START / LEN changes: the old window keeps playing and fades out while
  // the new one fades in (one change per fade; the next waits for it)
  static constexpr int XFADE = 512;  // ~12 ms
  size_t o_ws = 0, o_wl = 0;
  float o_off = 0.0f;
  int xf = 0;
  float dub_g = 0.0f;  // overdub fading in / out (~10 ms), so it doesn't click
  float run_g = 0.0f;  // playback fading in / out (~5 ms) on start and stop
  volatile State st = EMPTY;
  volatile bool press = false, clear = false, stop = false, retrig = false, loop_ = true;
  volatile int32_t press_in = -1;
  volatile float speed = 1.0f, start_ = 0.0f, len_ = 1.0f, keep_ = 0.8f, mon = 1.0f, level = 1.0f;
  volatile float seam = 256.0f;
  volatile bool xfade_loop = false;
  size_t xh_l = 0, xf_l = 0;  // the crossfade plan in use (see update)
  size_t lt_ws = 0, lt_wl = 0;  // ... and the window it crosses into
  volatile bool defer_win = false;
  bool pend = false;            // a new window waiting for the seam
  bool wrapped = false;         // just came round: the next plan may change
  size_t p_ws = 0, p_wl = 0;
  volatile size_t ex_ws = 0, ex_wl = 0;
  volatile bool exact = false;
  uint8_t hist[HIST] = {};

  // a new, empty buffer of n frames swapped in between two audio updates
  // (cleared or not: REC writes before anything is read)
  bool Swap(size_t n, bool clear) {
    int16_t* l = static_cast<int16_t*>(clear ? extmem_calloc(n, sizeof(int16_t)) : extmem_malloc(n * sizeof(int16_t)));
    int16_t* r = l ? static_cast<int16_t*>(clear ? extmem_calloc(n, sizeof(int16_t)) : extmem_malloc(n * sizeof(int16_t))) : nullptr;
    if (!l || !r) { if (l) extmem_free(l); return false; }
    int16_t* old[2];
    AudioNoInterrupts();
    old[0] = buf[0]; old[1] = buf[1];
    buf[0] = l; buf[1] = r;
    cap = n;
    AudioInterrupts();
    for (auto b : old) if (b) extmem_free(b);
    return true;
  }

  void Reset() {
    st = EMPTY;
    len = wpos = ws = wl = 0;
    off = 0.0f;
    xf = 0;
    dub_g = 0.0f;
    run_g = 0.0f;
    std::fill(hist, hist + HIST, 0);
  }
  void Mark(size_t pos, int16_t a, int16_t b) {
    const int pk = std::max(abs(a), abs(b)) >> 7;
    uint8_t& h = hist[pos * HIST / cap];
    h = uint8_t(std::min(255, std::max<int>(h, pk)));
  }
};

FASTRUN void AudioEffectReel::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1] || !ready()) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }

  // a press waiting for its moment (start on a beat): in this block
  if (press_in >= 0) {
    const int32_t left = press_in - AUDIO_BLOCK_SAMPLES;
    if (left < 0) { press_in = -1; press = true; }
    else press_in = left;
  }
  // button presses from the UI
  if (clear) { clear = false; press = false; stop = false; Reset(); }
  if (stop) {
    stop = false;
    if (st == REC) { len = std::max<size_t>(wpos, AUDIO_BLOCK_SAMPLES); ws = 0; wl = len; }
    if (st != EMPTY) st = STOPPED;
  }
  if (press) {
    press = false;
    switch (st) {
      case EMPTY: wpos = 0; std::fill(hist, hist + HIST, 0); st = REC; break;
      case REC:
        len = std::max<size_t>(wpos, AUDIO_BLOCK_SAMPLES);
        ws = 0; wl = len; off = 0.0f; xf = 0;
        st = PLAY;
        break;
      case PLAY:  st = DUB; break;
      case DUB:   st = PLAY; break;
      case STOPPED:  // from the start of the window (its end, backwards)
        off = speed < 0.0f ? wl - 1.0f : 0.0f;
        xf = 0;
        st = PLAY;
        break;
    }
  }

  // back to the start: the old position fades out while the start fades in
  if (retrig) {
    retrig = false;
    if (len && wl) {
      if (st == PLAY || st == DUB) { o_ws = ws; o_wl = wl; o_off = off; xf = XFADE; }
      off = speed < 0.0f ? wl - 1.0f : 0.0f;
      if (st == STOPPED) st = PLAY;
    }
  }
  // the looping part of the loop (whole samples); a change starts a fade
  // from the old window to the new one, keeping the play position when it
  // is inside the new window
  if (len && st != REC) {
    const size_t nws = std::min<size_t>(len - 1, exact ? ex_ws : size_t(start_ * len));
    const size_t nwl = std::max<size_t>(AUDIO_BLOCK_SAMPLES,
                                        std::min<size_t>(len, exact ? ex_wl : size_t(len_ * len)));
    const bool forward_loop = defer_win && loop_ && speed > 0.0f && (st == PLAY || st == DUB);
    if (forward_loop && (nws != ws || nwl != wl)) {
      // playing on: the new window starts when the loop comes round
      pend = true; p_ws = nws; p_wl = nwl;
    } else if (forward_loop) {
      pend = false;
    } else if ((nws != ws || nwl != wl) && xf == 0) {
      pend = false;
      o_ws = ws; o_wl = wl; o_off = off;
      const size_t at = (ws + size_t(off)) % len;           // where we play now
      const size_t into = (at + len - nws) % len;            // ... in the new window
      ws = nws; wl = nwl;
      off = into < wl ? float(into) + (off - std::floor(off)) : (speed < 0.0f ? wl - 1.0f : 0.0f);
      // only a playing loop fades; a silent one takes the new window at once
      // (the fade counts down only while playing)
      const bool sounding = st == PLAY || st == DUB || run_g > 0.0f;
      xf = sounding ? XFADE : 0;
    }
  }
  const float sp = speed, g = level, m = mon, keep = keep_;
  constexpr float q = 1.0f / 32768.0f;
  // the crossfade seam (looping forward): over the last xl frames of the
  // window the loop's end fades out while the frames leading into START
  // (or the loop's own beginning) fade in, equal power
  const bool xon = xfade_loop && loop_ && sp > 0.0f && len && st != REC;
  // the plan only changes outside the crossfade (a new FADE / window in the
  // middle of one would jump)
  if (xon) {
    // crossing into the window the loop goes on with (a waiting one)
    const size_t tws = pend ? p_ws : ws, twl = pend ? p_wl : wl;
    size_t h, f;
    XfPlan(seam, ws, wl, len, h, f, tws, twl);
    const float zone = float(wl) - float(xh_l ? xh_l : xf_l);
    // a plan that doesn't fit this window any more (it got short) is
    // replaced at once, else it could never be replaced
    const bool stale = xh_l > wl / 2 || xf_l > wl / 2 || lt_wl == 0;
    // right after the seam it can always change (in a short loop with a long
    // fade the head is never outside the crossfade)
    if (off < zone || stale || wrapped || (h == xh_l && f == xf_l && tws == lt_ws && twl == lt_wl)) {
      wrapped = false;
      xh_l = h; xf_l = f; lt_ws = tws; lt_wl = twl;
    }
  } else {
    xh_l = xf_l = 0;
    lt_ws = ws; lt_wl = wl;
  }
  const size_t xh = xh_l, xf_in = xf_l;
  const float xhf = float(xh), xff = float(xf_in);
  // one play head: linear read, faded near its window's ends (the seam)
  auto read = [&](size_t w0, size_t wn, float o, float& l, float& r) {
    const size_t i0 = (w0 + size_t(o)) % len, i1 = (i0 + 1) % len;
    const float fr = o - std::floor(o);
    const float fade = std::min(float(seam), wn * 0.25f);
    const float e = xon && w0 == ws ? 1.0f : std::min(1.0f, std::min(o, wn - o) / fade);
    l = (buf[0][i0] + fr * (buf[0][i1] - buf[0][i0])) * q * e;
    r = (buf[1][i0] + fr * (buf[1][i1] - buf[1][i0])) * q * e;
  };
  auto step = [&](float& o, size_t wn) {  // true when it went round
    o += sp;
    bool wrapped = false;
    while (o >= wn) { o -= wn; wrapped = true; }
    while (o < 0.0f) { o += wn; wrapped = true; }
    return wrapped;
  };

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const int16_t xl = in[0] ? in[0]->data[n] : 0;
    const int16_t xr = in[1] ? in[1]->data[n] : 0;
    float yl = 0.0f, yr = 0.0f;
    if (st == REC) {
      buf[0][wpos] = xl;
      buf[1][wpos] = xr;
      Mark(wpos, xl, xr);
      if (++wpos >= cap) { len = cap; off = 0.0f; st = PLAY; }
    } else if ((st == PLAY || st == DUB || run_g > 0.0f) && len) {
      // start / stop glide instead of switching
      constexpr float run_step = 1.0f / (0.005f * AUDIO_SAMPLE_RATE_EXACT);
      run_g = (st == PLAY || st == DUB) ? std::min(1.0f, run_g + run_step) : std::max(0.0f, run_g - run_step);
      read(ws, wl, off, yl, yr);
      // in the crossfade: the incoming part, read like the main head
      // (centred: from h before START; else the loop's own start)
      const bool in_xf = xon && (xh ? off >= wl - xhf : (xf_in > 1 && off >= wl - xff));
      if (in_xf) {
        const float into = off - (wl - (xh ? xhf : xff));
        const float t = into / (xh ? 2.0f * xhf : xff);
        const float src = xh ? float(lt_ws) - xhf + into : float(lt_ws) + into;
        const size_t i0 = (size_t(src) + len) % len, i1 = (i0 + 1) % len;
        const float fr = src - std::floor(src);
        const float il = (buf[0][i0] + fr * (buf[0][i1] - buf[0][i0])) * q;
        const float ir = (buf[1][i0] + fr * (buf[1][i1] - buf[1][i0])) * q;
        const float go = cosf(t * 1.5707963f), gi = sinf(t * 1.5707963f);
        yl = yl * go + il * gi;
        yr = yr * go + ir * gi;
      }
      if (xf > 0) {  // the old window fading out
        float ol, orr;
        read(o_ws, o_wl, o_off, ol, orr);
        const float k = float(xf) / XFADE;
        yl += k * (ol - yl);
        yr += k * (orr - yr);
        step(o_off, o_wl);
        xf--;
      }
      yl *= run_g;
      yr *= run_g;
      const size_t i0 = (ws + size_t(off)) % len;
      // overdub: the input comes in and the old loop goes down to DUB,
      // both gliding with dub_g
      constexpr float dub_step = 1.0f / (0.01f * AUDIO_SAMPLE_RATE_EXACT);
      dub_g = st == DUB ? std::min(1.0f, dub_g + dub_step) : std::max(0.0f, dub_g - dub_step);
      if (dub_g > 0.0f) {
        const float k = 1.0f - dub_g * (1.0f - keep);
        const int16_t nl = __SSAT(int32_t(buf[0][i0] * k + xl * dub_g), 16);
        const int16_t nr = __SSAT(int32_t(buf[1][i0] * k + xr * dub_g), 16);
        buf[0][i0] = nl;
        buf[1][i0] = nr;
        Mark(i0, nl, nr);
      }
      // not looping: once round, stop (the seam has faded it out already)
      bool went_round;
      if (xon) {  // the crossfade seam: on from where the faded-in part ended
        off += sp;
        const float end = xh ? wl + xhf : wl;
        went_round = off >= end;
        if (went_round) {
          // into the window crossed into (a waiting START / END takes over here)
          const float over = off - end;
          if (ws != lt_ws || wl != lt_wl) { ws = lt_ws; wl = lt_wl; }
          wrapped = true;
          if (pend && p_ws == ws && p_wl == wl) pend = false;
          off = std::fmod((xh ? xhf : xff) + over, float(wl));
        }
      } else {
        went_round = step(off, wl);
        if (went_round && pend) {  // INOUT: the new window after the dip
          ws = p_ws; wl = p_wl; pend = false;
          off = std::fmod(std::max(0.0f, off), float(wl));
        }
      }
      if (went_round && !loop_ && st == PLAY) {
        st = STOPPED;
        run_g = 0.0f;
        off = sp < 0.0f ? wl - 1.0f : 0.0f;
      }
    }
    const float ol = yl * g + xl * q * m, orr = yr * g + xr * q * m;
    out[0]->data[n] = __SSAT(int32_t(ol * 32767.0f), 16);
    out[1]->data[n] = __SSAT(int32_t(orr * 32767.0f), 16);
  }

  // one side on both channels (linked PLAYERs: L takes the left, R the right)
  if (pick == 1) memcpy(out[1]->data, out[0]->data, sizeof(out[0]->data));
  else if (pick == 2) memcpy(out[0]->data, out[1]->data, sizeof(out[1]->data));
  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);
}
