#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>

// Stereo looper for the Sculpt REEL device. The loop lives in PSRAM (up to
// 16 s; less if memory is short). A press cycles REC -> PLAY -> DUB -> PLAY;
// the first REC sets the loop length. SPEED plays it at any rate, backwards
// below zero; START / LEN pick the part that loops, with short fades at its
// ends so the seam doesn't click. MON passes the live input through.
class AudioEffectReel : public AudioStream {
public:
  enum State : uint8_t { EMPTY, REC, PLAY, DUB };
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
  void Release() {
    for (auto& b : buf) { if (b) extmem_free(b); b = nullptr; }
    cap = 0;
    Reset();
  }
  bool ready() const { return buf[0] && buf[1]; }
  float CapSecs() const { return cap / AUDIO_SAMPLE_RATE_EXACT; }

  // --- UI thread; taken over at the next block ---
  void Press() { press = true; }
  void Clear() { clear = true; }
  void SetSpeed(float s) { speed = s; }
  void SetWindow(float start, float len) { start_ = start; len_ = len; }
  void SetDub(float keep) { keep_ = keep; }
  void SetMonitor(float m) { mon = m; }
  void SetLevel(float l) { level = l; }

  State state() const { return st; }
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
  void ReadHistory(uint8_t* dst) const {
    const size_t n = st == REC ? wpos : len;
    for (int k = 0; k < HIST; k++)
      dst[k] = n && cap ? hist[std::min<size_t>(HIST - 1, size_t(k) * n / cap)] : 0;
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
  volatile State st = EMPTY;
  volatile bool press = false, clear = false;
  volatile float speed = 1.0f, start_ = 0.0f, len_ = 1.0f, keep_ = 0.8f, mon = 1.0f, level = 1.0f;
  uint8_t hist[HIST] = {};

  void Reset() {
    st = EMPTY;
    len = wpos = ws = wl = 0;
    off = 0.0f;
    xf = 0;
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

  // button presses from the UI
  if (clear) { clear = false; press = false; Reset(); }
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
    }
  }

  // the looping part of the loop (whole samples); a change starts a fade
  // from the old window to the new one, keeping the play position when it
  // is inside the new window
  if (len && st != REC) {
    const size_t nws = std::min<size_t>(len - 1, size_t(start_ * len));
    const size_t nwl = std::max<size_t>(AUDIO_BLOCK_SAMPLES, std::min<size_t>(len, size_t(len_ * len)));
    if ((nws != ws || nwl != wl) && xf == 0) {
      o_ws = ws; o_wl = wl; o_off = off;
      const size_t at = (ws + size_t(off)) % len;           // where we play now
      const size_t into = (at + len - nws) % len;            // ... in the new window
      ws = nws; wl = nwl;
      off = into < wl ? float(into) + (off - std::floor(off)) : (speed < 0.0f ? wl - 1.0f : 0.0f);
      xf = XFADE;
    }
  }
  const float sp = speed, g = level, m = mon, keep = keep_;
  constexpr float q = 1.0f / 32768.0f;
  // one play head: linear read, faded near its window's ends (the seam)
  auto read = [&](size_t w0, size_t wn, float o, float& l, float& r) {
    const size_t i0 = (w0 + size_t(o)) % len, i1 = (i0 + 1) % len;
    const float fr = o - std::floor(o);
    const float fade = std::min(256.0f, wn * 0.25f);
    const float e = std::min(1.0f, std::min(o, wn - o) / fade);
    l = (buf[0][i0] + fr * (buf[0][i1] - buf[0][i0])) * q * e;
    r = (buf[1][i0] + fr * (buf[1][i1] - buf[1][i0])) * q * e;
  };
  auto step = [&](float& o, size_t wn) {
    o += sp;
    while (o >= wn) o -= wn;
    while (o < 0.0f) o += wn;
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
    } else if ((st == PLAY || st == DUB) && len) {
      read(ws, wl, off, yl, yr);
      if (xf > 0) {  // the old window fading out
        float ol, orr;
        read(o_ws, o_wl, o_off, ol, orr);
        const float k = float(xf) / XFADE;
        yl += k * (ol - yl);
        yr += k * (orr - yr);
        step(o_off, o_wl);
        xf--;
      }
      const size_t i0 = (ws + size_t(off)) % len;
      if (st == DUB) {
        const int16_t nl = __SSAT(int32_t(buf[0][i0] * keep) + xl, 16);
        const int16_t nr = __SSAT(int32_t(buf[1][i0] * keep) + xr, 16);
        buf[0][i0] = nl;
        buf[1][i0] = nr;
        Mark(i0, nl, nr);
      }
      step(off, wl);
    }
    const float ol = yl * g + xl * q * m, orr = yr * g + xr * q * m;
    out[0]->data[n] = __SSAT(int32_t(ol * 32767.0f), 16);
    out[1]->data[n] = __SSAT(int32_t(orr * 32767.0f), 16);
  }

  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);
}
