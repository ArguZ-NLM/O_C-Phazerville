// Copyright (c) 2026, Sascha A W Haber
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Modulators for the Loom app: WAVE / RANDOM / ADSR / FOLLOW / TREADLE / PICK. Four slots per track (MOD A-D); a device setting picks one as its
// modulation source. They run in the main loop about once per millisecond and
// output 0..1, like a 0..10 V CV; the setting's U/B choice decides whether
// that pushes one way or swings around the setting.
//
// Triggers come from the gate inputs, the CV inputs used as gates, or MIDI
// maps (notes / CCs over half way). Rising edges on the hardware inputs are
// latched in the app's ISR so short pulses aren't missed.

#pragma once

#include "LoomDevices.h"
// the drum map TREADLE reads (the one this build's DrumMap applet uses)
#ifdef DRUMMAP_GRIDS2
#include "../grids2_resources.h"
#else
#include "../grids_resources.h"
#endif

namespace loom {

class Modulator : public Device {
public:
  static constexpr int HIST = 64;
  float out = 0.0f;     // 0..1
  float hist[HIST] = {};
  uint8_t hist_pos = 0;
  float hist_t = 0.0f;
  float flash = 0.0f;   // trigger indicator, seconds left

  explicit Modulator(const DeviceDef& d);
  void Apply(int) override {}  // settings are read live in Tick()
  virtual void Tick(const ModCtx& ctx) = 0;
  // out is on notes, 4 octaves across: a CV output gives it as pitch (the
  // module's own octave: 1.2 V on the 2X0, 1 V on the SX0)
  virtual bool Pitched() const { return false; }
  // the value without a glide (RANDOM: its step as it is)
  virtual float Stepped() const { return out; }
  void Run(const ModCtx& ctx);
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;

protected:
  bool prev_gate[2] = {};
  uint8_t seen_starts = 0;
  // true once after each clock start (check every tick)
  bool Restarted() { const bool r = seen_starts != clock_starts; seen_starts = clock_starts; return r; }
  bool Synced(int p) const { return mode[p] != SYNC_FREE; }
  bool Gate(int src) const;
  bool Edge(int src, const ModCtx& ctx, int slot);
  float Seconds(int v, float lo, float hi) const; // 0..100 -> lo..hi, exp
  // RATE setting p in Hz: free (lo..hi) or one cycle per note value
  float RateHz(int p, float lo, float hi) const;
  int Step(int p, int v, int dir) const override;
  void FormatRate(int p, float hz, char* buf, size_t n) const;
};

FLASHMEM Modulator::Modulator(const DeviceDef& d) : Device(d) {}

FLASHMEM float Modulator::Seconds(int v, float lo, float hi) const {
  return lo * powf(hi / lo, constrain(v, 0, 100) * 0.01f);
}

FLASHMEM float Modulator::RateHz(int p, float lo, float hi) const {
  if (mode[p] == SYNC_FREE) return Seconds(cur[p], lo, hi);
  return 1.0f / DivSeconds(DivIndex(cur[p], 0, 100));  // (higher: shorter)
}

FLASHMEM int Modulator::Step(int p, int v, int dir) const {
  if (!strcmp(param(p).modes[1], "CLOCK") && mode[p] != SYNC_FREE) // RATE on the clock: note values
    return DivValue(DivIndex(v, 0, 100) + dir, 0, 100);
  return v + dir;
}

FLASHMEM void Modulator::FormatRate(int p, float hz, char* buf, size_t n) const {
  if (mode[p] != SYNC_FREE) FormatDiv(DivIndex(val[p], 0, 100), buf, n);
  else if (hz < 1.0f) snprintf(buf, n, "%ds", int(1.0f / hz + 0.5f));
  else snprintf(buf, n, "%dHz", int(hz + 0.5f));
}

FLASHMEM bool Modulator::Gate(int src) const { return SrcGate(src); }

// slot: which of this modulator's trigger settings (for MIDI edge tracking)
FLASHMEM bool Modulator::Edge(int src, const ModCtx& ctx, int slot) {
  return SrcEdge(src, ctx.edges, prev_gate[slot]);
}

FLASHMEM void Modulator::Run(const ModCtx& ctx) {
  Modulate(); // a modulator's own settings can be modulated too
  Tick(ctx);
  out = constrain(out, 0.0f, 1.0f);
  if (flash > 0.0f) flash -= ctx.dt;
  // history for the scope: ~1 s across the screen
  hist_t += ctx.dt;
  if (hist_t >= 1.0f / HIST) {
    hist_t = 0.0f;
    hist[hist_pos] = out;
    hist_pos = (hist_pos + 1) % HIST;
  }
}

FLASHMEM void Modulator::DrawViz(int x, int y, int w, int h) {
  // scrolling scope of the output, newest on the right
  int px = -1, py = 0;
  for (int i = 0; i < HIST; i++) {
    const float v = hist[(hist_pos + i) % HIST];
    const int xx = x + i * (w - 6) / (HIST - 1);
    const int yy = y + h - 1 - int(v * (h - 1));
    if (px >= 0) gfxLine(px, py, xx, yy);
    px = xx;
    py = yy;
  }
  gfxDottedLine(x, y + h - 1, x + w - 6, y + h - 1, 3);
  // current value as a bar on the right
  const int bh = int(out * h);
  gfxFrame(x + w - 4, y, 4, h);
  if (bh > 0) gfxRect(x + w - 4, y + h - bh, 4, bh);
  if (flash > 0.0f) gfxRect(x, y, 4, 4);
}

FLASHMEM void Modulator::DrawMini(int x, int y, int w, int h) {
  int px = -1, py = 0;
  for (int i = 0; i < w; i++) {
    const float v = hist[(hist_pos + HIST - w + i) % HIST];
    const int yy = y + h - 1 - int(v * (h - 1));
    if (px >= 0) gfxLine(px, py, x + i, yy);
    px = x + i;
    py = yy;
  }
  if (flash > 0.0f) gfxRect(x, y, 3, 3);
}

/////////////////////////////////////////////////////////////////
// WAVE: LFO
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef WAVE_DEF = {
  "WAVE", "WAV", 7, {"RST", ""},
  {
    {"RATE", 0, 100, 40, UNI, 0, 2, 0, {}, {"FREE", "CLOCK"}},
    {"AMNT", 0, 100, 100, UNI},
    {"PHASE", 0, 100, 0, UNI},
    {"OFFS", -50, 50, 0, BI},
    {"SHAPE", 0, 3, 0, LIST, 4, 0, 0, {"SIN", "TRI", "SQR", "SAW"}},
    {"SKEW", -50, 50, 0, BI},
    {"RESET", 0, TRIG_SOURCES - 1, 0, UNI},
  }
};

class WaveMod : public Modulator {
public:
  enum { P_RATE, P_AMT, P_PHASE, P_OFFS, P_SHAPE, P_SKEW, P_RESET };
  WaveMod() : Modulator(WAVE_DEF) {}
  void Tick(const ModCtx& ctx) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override { if (a == 0) { ph = 0.0f; flash = 0.1f; } }
  void DrawViz(int x, int y, int w, int h) override;
private:
  float ph = 0.0f;
  float Hz() const { return RateHz(P_RATE, 0.02f, 20.0f); }
  float At(float phase) const;  // the output at a point of the cycle, 0..1
};

FLASHMEM float WaveMod::At(float phase) const {
  float p = phase + cur[P_PHASE] * 0.01f;
  p -= floorf(p);
  // skew moves the half-way point, squashing the wave to one side
  const float k = 0.5f + cur[P_SKEW] * 0.0098f;
  p = p < k ? 0.5f * p / k : 0.5f + 0.5f * (p - k) / (1.0f - k);
  float wv;
  switch (cur[P_SHAPE]) {
    case 1:  wv = 1.0f - fabsf(2.0f * p - 1.0f); break;  // TRI
    case 2:  wv = p < 0.5f ? 1.0f : 0.0f; break;         // SQR
    case 3:  wv = p; break;                              // SAW
    default: wv = 0.5f - 0.5f * cosf(2.0f * float(M_PI) * p); break;
  }
  return constrain(0.5f + (wv - 0.5f) * cur[P_AMT] * 0.01f + cur[P_OFFS] * 0.01f, 0.0f, 1.0f);
}

FLASHMEM void WaveMod::Tick(const ModCtx& ctx) {
  if (Edge(cur[P_RESET], ctx, 0)) { ph = 0.0f; flash = 0.1f; }
  const bool synced = Synced(P_RATE);
  if (Restarted() && synced) ph = 0.0f;
  if (!synced || clock_running) ph += Hz() * ctx.dt;
  ph -= floorf(ph);
  out = At(ph);
}

// one cycle of the wave as it comes out, a dot running along it, the output
// level on the right
FLASHMEM void WaveMod::DrawViz(int x, int y, int w, int h) {
  const int ww = w - 8, top = y + 1, hh = h - 3;
  auto yy = [&](float v) { return top + hh - int(v * hh + 0.5f); };
  gfxDottedLine(x, yy(0.5f), x + ww - 1, yy(0.5f), 3);
  int py = -1;
  for (int i = 0; i < ww; i++) {
    const int cy = yy(At(float(i) / ww));
    if (py >= 0) gfxLine(x + i - 1, py, x + i, cy); else gfxPixel(x + i, cy);
    py = cy;
  }
  const int dx = x + int(ph * (ww - 1));
  gfxRect(dx - 1, yy(out) - 1, 3, 3);
  const int bh = int(out * h);
  gfxFrame(x + w - 4, y, 4, h);
  if (bh > 0) gfxRect(x + w - 4, y + h - bh, 4, bh);
  if (flash > 0.0f) gfxRect(x, y, 4, 4);  // reset: small corner mark
}

FLASHMEM void WaveMod::Format(int p, char* buf, size_t n) const {
  if (p == P_RATE) {
    FormatRate(p, Hz(), buf, n);
  } else if (p == P_RESET) {
    FormatTrig(val[p], buf, n);
  } else {
    Device::Format(p, buf, n);
  }
}

/////////////////////////////////////////////////////////////////
// RANDOM: stepped random, optionally a looping pattern
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef RANDOM_DEF = {
  "RANDOM", "RND", 8, {"STP", ""},
  {
    {"AMNT", 0, 100, 100, UNI},
    {"LEN", 0, 16, 0, UNI},
    // how often a step of the pattern changes (%); mode: how far it may
    // move, 5 .. 100 % (100: anything, a new value)
    {"VARI", 0, 100, 20, UNI, 0, 21, 20},
    {"OFFS", -50, 50, 0, BI},
    {"CLOCK", 0, TRIG_SOURCES - 1, 0, UNI},
    {"RATE", 0, 100, 45, UNI, 0, 2, 0, {}, {"FREE", "CLOCK"}},
    {"SMTH", 0, 100, 0, UNI},
    // the scale the output snaps to (off, then as TWILL's, with BLUES);
    // mode: its root note, C .. B
    {"QUANT", 0, 16, 0, UNI, 0, 12, 0},
  }
};

class RandomMod : public Modulator {
public:
  // (before 2026-10-14: RATE AMNT SMTH OFFS LEN VARI CLOCK, no QUANT)
  enum { P_AMT, P_LEN, P_VARI, P_OFFS, P_CLOCK, P_RATE, P_SMTH, P_QUANT };
  static constexpr int SCALES = 17, SCALES12 = 9;  // off, 8 on the 12 notes, 8 microtonal
  RandomMod() : Modulator(RANDOM_DEF) {
    for (auto& s : steps) s = random(1000) * 0.001f;
  }
  void Tick(const ModCtx& ctx) override;
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  void Action(int a) override { if (a == 0) Step(); }
  void DrawViz(int x, int y, int w, int h) override;
  bool Pitched() const override { return cur[P_QUANT] != 0; }
  float Stepped() const override { return Shaped(target); }
private:
  float steps[16];
  uint8_t pos = 0;
  float target = 0.5f, smooth = 0.5f, t = 0.0f;
  uint32_t step_at = 0;
  int64_t grid_at = -1;  // on the clock: the division it last stepped on
  float grid_div = 0.0f; // ... of which note value (in beats)
  float step_s = 0.5f;  // the time between steps, averaged (the picture's glides when CLOCK steps it)
  float Hz() const { return RateHz(P_RATE, 0.05f, 30.0f); }
  float GlideTau() const { const float s = cur[P_SMTH] * 0.01f; return 0.0005f + s * s * 1.0f; }
  void Step();
  float Shaped(float s) const;
  float Quantized(float v) const;
};

// a step through AMNT / OFFS, then onto the scale
FLASHMEM float RandomMod::Shaped(float s) const {
  const float v = constrain(0.5f + (s - 0.5f) * cur[P_AMT] * 0.01f + cur[P_OFFS] * 0.01f, 0.0f, 1.0f);
  return cur[P_QUANT] ? Quantized(v) : v;
}

// the whole output is four octaves (48 semitones), as a PITCH setting's
// range: AMNT 100 on a PITCH setting lands on the notes
FLASHMEM float RandomMod::Quantized(float v) const {
  static const uint16_t masks[SCALES12 - 1] = {
    0xFFF,                                                                        // CHROM
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 11),  // MAJOR
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 8) | (1 << 10),  // MINOR
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9),                         // PENTA
    (1 << 0) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 10),                        // minor pentatonic
    (1 << 0) | (1 << 3) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 10),             // BLUES
    (1 << 0) | (1 << 7),                                                          // FIFTH: root and fifth
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10),             // WHOLE tones
  };
  // as TWILL's: slendro, pelog (approximate), the 22 shrutis, the harmonic
  // series 8 .. 15
  static const float slendro[5] = {0, 240, 480, 720, 960};
  static const float pelog[7] = {0, 120, 270, 540, 670, 785, 950};
  static const float shruti[22] = {0, 90.2f, 111.7f, 182.4f, 203.9f, 294.1f, 315.6f, 386.3f, 407.8f, 498.0f, 519.6f,
                                   590.2f, 611.7f, 702.0f, 792.2f, 813.7f, 884.4f, 905.9f, 996.1f, 1017.6f, 1088.3f, 1109.8f};
  static const float harmonic[8] = {0, 203.91f, 386.31f, 551.32f, 701.96f, 840.53f, 968.83f, 1088.27f};
  float st[24];
  int ns = 0;
  float period = 1200.0f;
  auto equal = [&](int k, float per) { for (int i = 0; i < k; i++) st[i] = per * i / k; ns = k; period = per; };
  auto table = [&](const float* tb, int k) { for (int i = 0; i < k; i++) st[i] = tb[i]; ns = k; };
  const int sc = constrain(int(cur[P_QUANT]), 1, SCALES - 1);
  if (sc < SCALES12) {
    for (int i = 0; i < 12; i++) if (masks[sc - 1] >> i & 1) st[ns++] = i * 100.0f;
  } else switch (sc) {
    case 9: equal(24, 1200.0f); break;     // QUART: quarter tones
    case 10: equal(17, 1200.0f); break;    // 17ET
    case 11: equal(19, 1200.0f); break;    // 19ET
    case 12: table(slendro, 5); break;     // SLEND
    case 13: table(pelog, 7); break;       // PELOG
    case 14: equal(13, 1901.955f); break;  // BOHLN: Bohlen-Pierce, 13 steps of the twelfth
    case 15: table(shruti, 22); break;     // SHRUT
    default: table(harmonic, 8); break;    // HARMO
  }
  // the nearest note, in cents from the bottom (the root from mode)
  const float c = v * 4800.0f - mode[P_QUANT] * 100.0f;
  const int k = int(floorf(c / period));
  float best = 0.0f, gap = 1e9f;
  for (int o = k - 1; o <= k + 1; o++) {
    for (int i = 0; i < ns; i++) {
      const float nc = o * period + st[i];
      const float a = mode[P_QUANT] * 100.0f + nc;
      if (a < -0.5f || a > 4800.5f) continue;
      if (fabsf(nc - c) < gap) { gap = fabsf(nc - c); best = a; }
    }
  }
  return gap < 1e8f ? constrain(best / 4800.0f, 0.0f, 1.0f) : v;
}

FLASHMEM void RandomMod::Step() {
  const uint32_t now = millis();
  if (step_at && now - step_at < 20000) step_s += 0.25f * (std::max(0.005f, (now - step_at) * 0.001f) - step_s);
  step_at = now;
  const int len = cur[P_LEN];
  if (len == 0) {
    target = random(1000) * 0.001f;
  } else {
    pos = (pos + 1) % len;
    if (random(100) < cur[P_VARI]) {
      const float amount = mode[P_VARI] * 0.05f;
      if (amount >= 1.0f) steps[pos] = random(1000) * 0.001f;  // anything
      else {  // up or down by up to amount, folded back at the ends
        float v = steps[pos] + (random(2001) * 0.001f - 1.0f) * amount;
        if (v < 0.0f) v = -v;
        if (v > 1.0f) v = 2.0f - v;
        steps[pos] = constrain(v, 0.0f, 1.0f);
      }
    }
    target = steps[pos];
  }
  flash = 0.05f;
}

FLASHMEM void RandomMod::Tick(const ModCtx& ctx) {
  const bool synced = Synced(P_RATE) && !cur[P_CLOCK];
  if (Restarted() && synced) {
    // start: back to the first step
    t = 0.0f;
    grid_at = 0;
    pos = cur[P_LEN] > 0 ? cur[P_LEN] - 1 : 0;
    Step();
  } else if (cur[P_CLOCK]) {
    if (Edge(cur[P_CLOCK], ctx, 0)) Step();
  } else if (synced && clock_running) {
    // on the clock's grid (as FIBRE's grains on the clock count it)
    const float db = DivBeats(DivIndex(cur[P_RATE], 0, 100));
    const int64_t g = ClockGrid(db);
    // another note value: on its grid from here, its first step on its next tick
    if (db != grid_div) { grid_div = db; grid_at = g; }
    if (g != grid_at) { if (grid_at >= 0) Step(); grid_at = g; }
  } else if (!synced) {
    t += Hz() * ctx.dt;
    if (t >= 1.0f) { t -= floorf(t); Step(); }
  }
  // P_SMTH glides between steps (between the notes when quantized)
  const float tau = GlideTau();
  smooth += (Shaped(target) - smooth) * std::min(1.0f, ctx.dt / tau);
  out = smooth;
}

// LEN set: the sequence as bars, the current step lit; free: the last steps
// going by as a line
FLASHMEM void RandomMod::DrawViz(int x, int y, int w, int h) {
  const int ww = w - 8, top = y + 1, hh = h - 3;
  auto val = [&](float s) { return Shaped(s); };
  auto yy = [&](float v) { return top + hh - int(v * hh + 0.5f); };
  const int len = cur[P_LEN];
  if (len > 0) {
    const int bw = ww / len;
    const int bottom = top + hh;
    // SMTH: each block's top glides up / down from the step before, over
    // as much of the step as the glide takes
    const float tau = GlideTau();
    const bool glide = cur[P_SMTH] > 0;
    // a step's length: RATE's (steady), or when CLOCK steps it the gaps
    // between the last steps, averaged (so the curves don't twitch)
    const float st = cur[P_CLOCK] ? step_s : 1.0f / std::max(0.01f, Hz());
    for (int i = 0; i < len; i++) {
      const int bx = x + i * bw + 1, w1 = std::max(1, bw - 2);
      const float to = val(steps[i]), from = val(steps[(i + len - 1) % len]);
      if (!glide) {
        const int by = yy(to);
        if (i == pos) gfxRect(bx, by, w1, bottom - by + 1);
        else gfxFrame(bx, by, w1, bottom - by + 1);
        continue;
      }
      int py = -1;
      for (int c = 0; c < w1; c++) {
        const float tt = (c + 0.5f) / w1 * st;
        const int cy = yy(from + (to - from) * (1.0f - expf(-tt / tau)));
        if (i == pos) gfxLine(bx + c, cy, bx + c, bottom);
        else {
          if (py >= 0 && py != cy) gfxLine(bx + c, py, bx + c, cy);
          else gfxPixel(bx + c, cy);
          if (c == 0) gfxLine(bx, cy, bx, bottom);
          if (c == w1 - 1) gfxLine(bx + c, cy, bx + c, bottom);
        }
        py = cy;
      }
      if (i != pos) gfxLine(bx, bottom, bx + w1 - 1, bottom);
    }
  } else {
    // the history (~1 s across) as one stepped line
    int py = -1;
    for (int i = 0; i < HIST; i++) {
      const int cy = yy(hist[(hist_pos + i) % HIST]);
      const int x0 = x + i * ww / HIST, x1 = x + (i + 1) * ww / HIST - 1;
      if (py >= 0 && py != cy) gfxLine(x0, py, x0, cy);
      gfxLine(x0, cy, std::max(x0, x1), cy);
      py = cy;
    }
  }
  const int bh = int(out * h);
  gfxFrame(x + w - 4, y, 4, h);
  if (bh > 0) gfxRect(x + w - 4, y + h - bh, 4, bh);
}

FLASHMEM void RandomMod::Format(int p, char* buf, size_t n) const {
  if (p == P_RATE) {
    FormatRate(p, Hz(), buf, n);
  } else if (p == P_LEN) {
    if (val[P_LEN] == 0) snprintf(buf, n, "free");
    else snprintf(buf, n, "%d", val[P_LEN]);
  } else if (p == P_CLOCK) {
    if (val[P_CLOCK] == 0) snprintf(buf, n, "rate");
    else FormatTrig(val[P_CLOCK], buf, n);
  } else if (p == P_QUANT) {
    static const char* const scales[SCALES] = {"off", "CHROM", "MAJOR", "MINOR", "PENTA", "mPENT", "BLUES", "FIFTH",
                                               "WHOLE", "QUART", "17ET", "19ET", "SLEND", "PELOG", "BOHLN", "SHRUT", "HARMO"};
    snprintf(buf, n, "%s", scales[constrain(int(val[P_QUANT]), 0, SCALES - 1)]);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void RandomMod::FormatMode(int p, char* buf, size_t n) const {
  static const char* const names[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  if (p == P_QUANT) snprintf(buf, n, "%s", names[mode[P_QUANT] % 12]);
  else if (p == P_VARI) snprintf(buf, n, mode[P_VARI] >= 20 ? "ANY" : "%d%%", mode[P_VARI] * 5);
  else Modulator::FormatMode(p, buf, n);
}

/////////////////////////////////////////////////////////////////
// ADSR: envelope from a gate / trigger / MIDI note
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef ADSR_DEF = {
  "ADSR", "ENV", 7, {"TRG", ""},
  {
    {"ATT", 0, 100, 10, UNI},
    {"DEC", 0, 100, 45, UNI},
    {"SUS", 0, 100, 60, UNI},
    {"REL", 0, 100, 50, UNI},
    {"AMNT", 0, 100, 100, UNI},
    {"TRIG", 0, TRIG_SOURCES - 1, 1, UNI},
    {"CURVE", -50, 50, 0, BI},
  }
};

class AdsrMod : public Modulator {
public:
  enum { P_ATT, P_DEC, P_SUS, P_REL, P_AMT, P_TRIG, P_CURVE };
  AdsrMod() : Modulator(ADSR_DEF) {}
  void Tick(const ModCtx& ctx) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override { if (a == 0) { stage = ATTACK; manual = 0.15f; flash = 0.1f; } }
  void DrawViz(int x, int y, int w, int h) override;
private:
  enum Stage : uint8_t { IDLE, ATTACK, DECAY, SUSTAIN, RELEASE } stage = IDLE;
  float env = 0.0f;
  float rel_from = 1.0f;  // where the release started, for the picture
  float Shaped(float e) const;  // CURVE and AMNT applied
  float manual = 0.0f; // the action button holds the gate this long
  float Time(int p) const;
};

FLASHMEM float AdsrMod::Time(int p) const { return Seconds(cur[p], 0.001f, 10.0f); }

FLASHMEM void AdsrMod::Tick(const ModCtx& ctx) {
  if (Edge(cur[P_TRIG], ctx, 0)) { stage = ATTACK; flash = 0.1f; }
  if (manual > 0.0f) manual -= ctx.dt;
  const bool gate = Gate(cur[P_TRIG]) || manual > 0.0f;
  const float sus = cur[P_SUS] * 0.01f;
  switch (stage) {
    case ATTACK:
      env += ctx.dt / Time(P_ATT);
      if (env >= 1.0f) { env = 1.0f; stage = DECAY; }
      break;
    case DECAY:
      env -= ctx.dt / Time(P_DEC) * (1.0f - sus);
      if (env <= sus) { env = sus; stage = gate ? SUSTAIN : RELEASE; rel_from = env; }
      break;
    case SUSTAIN:
      env = sus;
      if (!gate) { stage = RELEASE; rel_from = env; }
      break;
    case RELEASE:
      env -= ctx.dt / Time(P_REL);
      if (env <= 0.0f) { env = 0.0f; stage = IDLE; }
      break;
    default: break;
  }
  // a short trigger (no gate held) plays attack-decay, then releases
  if (stage == DECAY && !gate && env <= sus + 0.0001f) { stage = RELEASE; rel_from = env; }
  out = Shaped(env);
}

// CURVE bends all stages: + exponential, - logarithmic; AMNT scales
FLASHMEM float AdsrMod::Shaped(float e) const {
  const float c = cur[P_CURVE] * 0.02f;
  const float e3 = e * e * e, l3 = 1.0f - (1.0f - e) * (1.0f - e) * (1.0f - e);
  const float sh = c >= 0.0f ? e + c * (e3 - e) : e - c * (l3 - e);
  return sh * cur[P_AMT] * 0.01f;
}

// the envelope as a line: attack, decay, a stretch of sustain, release, each
// as long as its setting; a dot rides along it while it plays
FLASHMEM void AdsrMod::DrawViz(int x, int y, int w, int h) {
  const int ww = w - 8, top = y + 1, hh = h - 3, seg = ww / 4;
  auto yy = [&](float e) { return top + hh - int(Shaped(e) * hh + 0.5f); };
  const int wa = 2 + cur[P_ATT] * (seg - 2) / 100, wd = 2 + cur[P_DEC] * (seg - 2) / 100;
  const int wr = 2 + cur[P_REL] * (seg - 2) / 100, wsus = ww - wa - wd - wr - 1;
  const float sus = cur[P_SUS] * 0.01f;
  gfxDottedLine(x, top + hh, x + ww - 1, top + hh, 3);
  // the shape, column by column
  int py = -1;
  for (int i = 0; i < ww; i++) {
    float e;
    if (i < wa) e = float(i) / wa;
    else if (i < wa + wd) e = 1.0f - (1.0f - sus) * float(i - wa) / wd;
    else if (i < wa + wd + wsus) e = sus;
    else e = sus * (1.0f - float(i - wa - wd - wsus) / wr);
    const int cy = yy(std::max(0.0f, e));
    if (py >= 0) gfxLine(x + i - 1, py, x + i, cy); else gfxPixel(x + i, cy);
    py = cy;
  }
  // where it is now
  int dx = -1;
  switch (stage) {
    case ATTACK:  dx = int(env * wa); break;
    case DECAY:   dx = wa + int((1.0f - env) / std::max(0.001f, 1.0f - sus) * wd); break;
    case SUSTAIN: dx = wa + wd + wsus / 2; break;
    case RELEASE: dx = wa + wd + wsus + int((1.0f - env / std::max(0.001f, rel_from)) * wr); break;
    default: break;
  }
  if (dx >= 0) gfxRect(x + constrain(dx, 0, ww - 1) - 1, top + hh - int(out * hh + 0.5f) - 1, 3, 3);
  const int bh = int(out * h);
  gfxFrame(x + w - 4, y, 4, h);
  if (bh > 0) gfxRect(x + w - 4, y + h - bh, 4, bh);
}

FLASHMEM void AdsrMod::Format(int p, char* buf, size_t n) const {
  if (p <= P_REL && p != P_SUS) {
    const float s = Time(p);
    if (s < 1.0f) snprintf(buf, n, "%dms", int(s * 1000.0f + 0.5f));
    else snprintf(buf, n, "%d.%ds", int(s), int(s * 10.0f) % 10);
  } else if (p == P_TRIG) {
    FormatTrig(val[P_TRIG], buf, n);
  } else {
    Device::Format(p, buf, n);
  }
}

/////////////////////////////////////////////////////////////////
// FOLLOW: envelope follower on the track's input or output
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef FOLLOW_DEF = {
  "FOLLOW", "FOL", 6, {"", ""},
  {
    {"INPUT", 0, 1, 0, LIST, 2, 0, 0, {"IN", "OUT"}},
    {"AMNT", 0, 100, 100, UNI},
    {"RISE", 0, 100, 10, UNI},
    {"FALL", 0, 100, 40, UNI},
    {"GAIN", 0, 200, 100, UNI},
    {"OFFS", -50, 50, 0, BI},
  }
};

class FollowMod : public Modulator {
public:
  enum { P_INPUT, P_AMT, P_RISE, P_FALL, P_GAIN, P_OFFS };
  FollowMod() : Modulator(FOLLOW_DEF) {}
  void Tick(const ModCtx& ctx) override;
  void DrawViz(int x, int y, int w, int h) override;
private:
  float env = 0.0f;
  float in_hist[HIST] = {};  // the level coming in, alongside hist (the output)
  float in_peak = 0.0f;
};

FLASHMEM void FollowMod::Tick(const ModCtx& ctx) {
  const float lvl = (cur[P_INPUT] ? ctx.out_level : ctx.in_level) * cur[P_GAIN] * 0.01f;
  const float tau = lvl > env ? Seconds(cur[P_RISE], 0.001f, 1.0f) : Seconds(cur[P_FALL], 0.005f, 3.0f);
  env += (lvl - env) * std::min(1.0f, ctx.dt / tau);
  out = env * cur[P_AMT] * 0.01f + cur[P_OFFS] * 0.01f;
  in_peak = std::max(in_peak, std::min(1.0f, lvl));
  // Run() moves hist_pos after this; keep the input in step with it
  if (hist_t + ctx.dt >= 1.0f / HIST) { in_hist[hist_pos] = in_peak; in_peak = 0.0f; }
}

// the level coming in as faint bars going by, the follower's output as a
// solid line over them (~1 s across)
FLASHMEM void FollowMod::DrawViz(int x, int y, int w, int h) {
  const int ww = w - 8, top = y + 1, hh = h - 3;
  auto yy = [&](float v) { return top + hh - int(constrain(v, 0.0f, 1.0f) * hh + 0.5f); };
  int py = -1;
  for (int i = 0; i < HIST; i++) {
    const int k = (hist_pos + i) % HIST;
    const int xx = x + i * ww / HIST;
    gfxDottedLine(xx, yy(in_hist[k]), xx, top + hh, 3);
    const int cy = yy(hist[k]);
    if (py >= 0) gfxLine(xx - ww / HIST, py, xx, cy); else gfxPixel(xx, cy);
    py = cy;
  }
  const int bh = int(out * h);
  gfxFrame(x + w - 4, y, 4, h);
  if (bh > 0) gfxRect(x + w - 4, y + h - bh, 4, bh);
}

/////////////////////////////////////////////////////////////////
// CLOCK: one tempo for the whole app
/////////////////////////////////////////////////////////////////
// MIDI clock pulses seen by the app's MIDI reader
volatile uint32_t midi_clocks = 0;

// source: 0 internal, 1 MIDI clock, 2+ trigger sources (TR, CV, MIDI map)
PROGMEM static const DeviceDef CLOCK_DEF = {
  "CLOCK", "CLK", 3, {"TAP", ""},
  {
    {"SOURC", 0, TRIG_SOURCES, 0, UNI},
    {"BPM", 30, 300, 120, UNI},
    {"PPQN", 0, 3, 0, LIST, 4, 0, 0, {"1", "2", "4", "24"}},
  }
};

class ClockDevice : public Modulator {
public:
  enum { P_SOURCE, P_BPM, P_PPQN };
  ClockDevice() : Modulator(CLOCK_DEF) {}
  void Tick(const ModCtx& ctx) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  bool external() const { return cur[P_SOURCE] != 0; }
  bool midi() const { return cur[P_SOURCE] == 1; }
  bool Waiting() const { return external() && !have_measure; }
  int Beat() const { return beat; }
private:
  uint8_t beat = 0;
  uint32_t last_edge_us = 0, tap_us = 0;
  uint32_t midi_mark = 0, midi_mark_us = 0;
  float beat_ph = 0.0f;
  float measured = 120.0f;
  bool have_measure = false;
  void Measured(float bpm);
};

FLASHMEM void ClockDevice::Measured(float bpm) {
  if (bpm < 20.0f || bpm > 400.0f) return;
  // follow quickly when far off, smooth out jitter when close
  measured = (have_measure && fabsf(bpm - measured) < measured * 0.1f)
    ? measured + 0.3f * (bpm - measured) : bpm;
  have_measure = true;
}

FLASHMEM void ClockDevice::Tick(const ModCtx& ctx) {
  const uint32_t now = micros();
  const int src = cur[P_SOURCE];
  if (src == 1) {
    // MIDI clock, 24 per beat: measure over whole beats
    const uint32_t c = midi_clocks;
    if (c - midi_mark >= 24) {
      if (midi_mark_us) Measured(60e6f * (c - midi_mark) / 24.0f / (now - midi_mark_us));
      midi_mark = c;
      midi_mark_us = now;
    }
  } else if (src >= 2) {
    if (SrcEdge(src - 1, ctx.edges, prev_gate[0])) {
      const uint32_t dt = now - last_edge_us;
      static const uint8_t ppqn[] = {1, 2, 4, 24};
      if (last_edge_us && dt > 2000 && dt < 4000000)
        Measured(60e6f / (dt * ppqn[constrain(cur[P_PPQN], 0, 3)]));
      last_edge_us = now;
    }
  }
  tempo_bpm = (src && have_measure) ? measured : float(cur[P_BPM]);

  // beat blink for the picture, and a 0..1 ramp per beat as output
  if (Restarted()) { beat_ph = 0.0f; beat = 0; flash = 0.08f; clock_beat_n = 0; }
  if (clock_running) beat_ph += ctx.dt * tempo_bpm / 60.0f;
  if (beat_ph >= 1.0f) { beat_ph -= floorf(beat_ph); beat = (beat + 1) % 4; flash = 0.08f; clock_beat_n++; }
  out = beat_ph;
  clock_beat_ph = beat_ph;  // for a PLAYER starting on the next beat
  clock_beat_us = micros();
}

// TAP: tap the tempo on the UP button (internal clock)
FLASHMEM void ClockDevice::Action(int a) {
  if (a != 0) return;
  const uint32_t now = micros();
  const uint32_t dt = now - tap_us;
  tap_us = now;
  if (dt > 200000 && dt < 2000000) {
    val[P_BPM] = constrain(int(60e6f / dt + 0.5f), 30, 300);
    Refresh(P_BPM);
  }
}

FLASHMEM void ClockDevice::Format(int p, char* buf, size_t n) const {
  if (p == P_SOURCE) {
    if (val[p] == 0) snprintf(buf, n, "INT");
    else if (val[p] == 1) snprintf(buf, n, "MIDI");
    else FormatTrig(val[p] - 1, buf, n);
  } else if (p == P_BPM && external()) {
    snprintf(buf, n, "%d", int(tempo_bpm + 0.5f));
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void ClockDevice::DrawViz(int x, int y, int w, int h) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", int(tempo_bpm + 0.5f));
  gfxPrint(x + 4, y + 4, buf);
  gfxPrint(x + 4 + 6 * int(strlen(buf)) + 2, y + 4, PSTR("BPM"));
  if (!clock_running) gfxPrint(x + 4, y + 16, PSTR("stopped  Z: start"));
  else if (external() && !have_measure) gfxPrint(x + 4, y + 16, PSTR("waiting for clock"));
  // four beats, the current one filled (none while stopped)
  for (int b = 0; b < 4; b++) {
    const int bx = x + w - 52 + b * 13;
    gfxFrame(bx, y + 4, 10, 10);
  }
  if (clock_running) gfxRect(x + w - 52 + beat * 13 + 2, y + 6, 6, 6);
}

FLASHMEM void ClockDevice::DrawMini(int x, int y, int w, int h) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", int(tempo_bpm + 0.5f));
  gfxPrint(x + 1, y + 2, buf);
  if (!clock_running) gfxPrint(x + 1, y + h / 2 + 2, PSTR("STP"));
  else if (flash > 0.0f) gfxRect(x + w / 2 - 3, y + h / 2 + 2, 6, 6);
  else gfxFrame(x + w / 2 - 3, y + h / 2 + 2, 6, 6);
}


/////////////////////////////////////////////////////////////////
// TREADLE: drum patterns. On a loom the treadling is the order the
// treadles are pressed in, the rhythm of the weave. A map of 25 drum
// patterns (32 steps each, for a low, a mid and a high part) after Grids
// (c) 2011-2012 Emilie Gillet, GPL 3 (the pattern data): X / Y move
// across the map, blending its four nearest patterns; FILL sets how many of
// the steps sound (its mode: the part), CHAOS stirs a little at random in
// each round. Each hit is a short pulse, accents full height, the others
// lower by ACCNT, so it strikes devices (MOD A-H as trigger sources) and
// plays drums from a CV output.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef TREADLE_DEF = {
  "TREADLE", "TRD", 8, {"RST", ""},
  {
    {"X", 0, 100, 50, UNI},
    {"Y", 0, 100, 50, UNI},
    {"FILL", 0, 100, 50, UNI, 0, 3, 0, {}, {"LOW", "MID", "HIGH"}},  // mode: the part
    {"CHAOS", 0, 100, 0, UNI},
    {"RATE", 0, 100, 74, UNI, 0, 2, 1, {}, {"FREE", "CLOCK"}},  // a step; on the clock (74: 1/16)
    {"RESET", 0, TRIG_SOURCES - 1, 0, UNI},
    {"ACCNT", 0, 100, 50, UNI},  // how much lower the plain hits are than the accents
    {"LEN", 1, 32, 32, UNI},
  }
};

class TreadleMod : public Modulator {
public:
  enum { P_X, P_Y, P_FILL, P_CHAOS, P_RATE, P_RESET, P_ACCNT, P_LEN };
  TreadleMod() : Modulator(TREADLE_DEF) {}
  void Tick(const ModCtx& ctx) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override { if (a == 0) Restart(); }
  void DrawViz(int x, int y, int w, int h) override;
private:
  int step = -1;           // the step playing (-1: none yet)
  int64_t grid_at = -1, grid_base = 0;
  float grid_div = 0.0f;
  float t = 0.0f, gate_s = 0.0f, step_s = 0.125f;
  uint8_t stir = 0;        // CHAOS: this round's push
  float hit_lvl = 0.0f;
  int Part() const { return constrain(int(mode[P_FILL]), 0, 2); }
  int Len() const { return constrain(int(cur[P_LEN]), 1, 32); }
  int Level(int s) const;  // the map's strength of step s, with CHAOS (0..255)
  bool Hits(int lv) const { return lv > 255 - cur[P_FILL] * 255 / 100; }
  void Restart() { grid_base = grid_at >= 0 ? grid_at + 1 : 0; step = -1; t = 0.0f; }
  void Advance(int s);
};

FLASHMEM int TreadleMod::Level(int s) const {
  const int x = cur[P_X] * 255 / 100, y = cur[P_Y] * 255 / 100;
  const int i = std::min(3, x >> 6), j = std::min(3, y >> 6);
  const int off = Part() * 32 + (s & 31);
  const int a = grids::drum_map[i][j][off], b = grids::drum_map[i + 1][j][off];
  const int c = grids::drum_map[i][j + 1][off], d = grids::drum_map[i + 1][j + 1][off];
  const int qx = (x << 2) & 255, qy = (y << 2) & 255;
  const int ab = (b * qx + a * (255 - qx)) >> 8, cd = (d * qx + c * (255 - qx)) >> 8;
  return std::min(255, ((cd * qy + ab * (255 - qy)) >> 8) + stir);
}

FLASHMEM void TreadleMod::Advance(int s) {
  step = s;
  if (s == 0) stir = uint8_t(random(0, cur[P_CHAOS] * 255 / 100 / 4 + 1));  // a new round
  const int lv = Level(s);
  if (Hits(lv)) {
    // accents (the map's strong steps) full height, the others lower by ACCNT
    hit_lvl = lv > 192 ? 1.0f : 1.0f - cur[P_ACCNT] * 0.0045f;
    gate_s = constrain(step_s * 0.25f, 0.005f, 0.03f);
    flash = 0.06f;
  }
}

FLASHMEM void TreadleMod::Tick(const ModCtx& ctx) {
  const bool synced = Synced(P_RATE);
  if (Restarted() && synced) { grid_at = -1; grid_base = 0; step = -1; }
  if (cur[P_RESET] && Edge(cur[P_RESET], ctx, 0)) Restart();
  const int len = Len();
  if (synced && clock_running) {
    const float db = DivBeats(DivIndex(cur[P_RATE], 0, 100));
    step_s = db * 60.0f / tempo_bpm;
    const int64_t g = ClockGrid(db);
    if (db != grid_div) { grid_div = db; grid_base = g - (step >= 0 ? step : 0); grid_at = g - 1; }
    if (g != grid_at) {
      grid_at = g;
      Advance(int(((g - grid_base) % len + len) % len));
    }
  } else if (!synced) {
    const float hz = RateHz(P_RATE, 0.5f, 30.0f);
    step_s = 1.0f / hz;
    t += hz * ctx.dt;
    if (t >= 1.0f || step < 0) { t -= floorf(t); Advance(step < 0 ? 0 : (step + 1) % len); }
  }
  if (gate_s > 0.0f) { gate_s -= ctx.dt; out = hit_lvl; }
  else out = 0.0f;
}

FLASHMEM void TreadleMod::Format(int p, char* buf, size_t n) const {
  if (p == P_RATE) FormatRate(p, RateHz(P_RATE, 0.5f, 30.0f), buf, n);
  else if (p == P_RESET) FormatTrig(cur[P_RESET], buf, n);
  else Device::Format(p, buf, n);
}

// the pattern: a bar per step as tall as the map's strength there; steps
// that sound solid (accents to the top), the rest a dotted outline, the
// line FILL draws across; the step playing marked underneath
FLASHMEM void TreadleMod::DrawViz(int x, int y, int w, int h) {
  const int len = Len(), ww = w - 2, hh = h - 4, base = y + hh;
  const int bw = std::max(2, ww / len);
  const int thr = 255 - cur[P_FILL] * 255 / 100;
  for (int s = 0; s < len; s++) {
    const int lv = Level(s), bx = x + 1 + s * bw;
    const int bh = std::max(1, lv * hh / 255);
    if (Hits(lv)) gfxRect(bx, base - bh + 1, std::max(1, bw - 1), bh);
    else gfxDottedLine(bx, base, bx, base - bh + 1, 2);
  }
  const int ty = base - thr * hh / 255;
  gfxDottedLine(x + 1, ty, x + 1 + len * bw - 1, ty, 3);
  if (step >= 0 && step < len) gfxRect(x + 1 + step * bw, base + 2, std::max(1, bw - 1), 2);
}

/////////////////////////////////////////////////////////////////
// PICK: a clock and two dividers. On a loom a pick is one pass of the
// weft; picks per inch is how dense the cloth is. RATE ticks; DIV A and
// DIV B count the ticks and strike every n-th (A full height, B lower);
// their L knob keeps the count to a set of numbers (all, odd, even,
// primes, triplets, Fibonacci, powers of two). A divider changed while
// running counts on from its last hit, so A and B on the same count can
// sit on different ticks; RESET (or the clock starting) lines them up.
// DENS leaves out hits at random; START plays a burst of BURST ticks on
// each trigger and then rests.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef PICK_DEF = {
  "PICK", "PCK", 8, {"RST", ""},
  {
    {"RATE", 0, 100, 74, UNI, 0, 2, 1, {}, {"FREE", "CLOCK"}},  // a tick; on the clock (74: 1/16)
    {"DIVA", 1, 144, 3, UNI, 0, 7, 0, {}, {}},  // mode: the set of numbers
    {"DIVB", 0, 144, 4, UNI, 0, 7, 0, {}, {}},  // (0: off)
    {"DENS", 0, 100, 100, UNI},
    {"START", 0, TRIG_SOURCES - 1, 0, UNI},  // none: runs all the time
    {"BURST", 1, 64, 8, UNI},  // ticks per START
    {"RESET", 0, TRIG_SOURCES - 1, 0, UNI},
    {"GATE", 0, 100, 0, UNI},  // 0: a short pulse; up to most of a tick
  }
};

class PickMod : public Modulator {
public:
  enum { P_RATE, P_DIVA, P_DIVB, P_DENS, P_START, P_BURST, P_RESET, P_GATE };
  enum { SET_ALL, SET_ODD, SET_EVEN, SET_PRIME, SET_TRIP, SET_FIB, SET_POW2, SETS };
  PickMod() : Modulator(PICK_DEF) {}
  void Tick(const ModCtx& ctx) override;
  int Step(int p, int v, int dir) const override;
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  // a divider's name is its set once one is chosen (PRIME, FIB ..)
  const char* Name(int p) const override {
    if ((p == P_DIVA || p == P_DIVB) && mode[p] > 0 && mode[p] < SETS) return SET_NAMES[mode[p]];
    return Modulator::Name(p);
  }
  void Action(int a) override { if (a == 0) Sync(); }
  void DrawViz(int x, int y, int w, int h) override;
private:
  static constexpr char SET_NAMES[SETS][6] = {"ALL", "ODD", "EVEN", "PRIME", "TRIP", "FIB", "2^N"};
  static constexpr int HITS = 40;
  uint8_t count[2] = {255, 255};  // ticks since each divider's last hit
  uint8_t hits[HITS] = {};        // the last ticks: bit 0 A, bit 1 B, bit 2 left out
  uint8_t hits_pos = 0;
  int64_t grid_at = -1;
  float t = 0.0f, gate_s = 0.0f, tick_s = 0.125f, hit_lvl = 0.0f;
  int burst_left = 0;
  void Sync() { count[0] = count[1] = 255; }
  void Pulse();
  static bool InSet(int set, int v);
  int Div(int p, int v) const;  // v snapped into p's set (0: off)
};

FLASHMEM bool PickMod::InSet(int set, int v) {
  if (v < 1) return false;
  switch (set) {
    case SET_ODD:  return v & 1;
    case SET_EVEN: return !(v & 1);
    case SET_PRIME:
      if (v < 2) return false;
      for (int d = 2; d * d <= v; d++) if (v % d == 0) return false;
      return true;
    case SET_TRIP: {  // 1 2 3 4 6 8 12 16 24 32 48 64 96 128
      int m = v;
      while (m > 4 && !(m & 1)) m >>= 1;
      return m <= 4;
    }
    case SET_FIB: { int a = 1, b = 2; while (b < v) { const int c = a + b; a = b; b = c; } return v == 1 || b == v; }
    case SET_POW2: return !(v & (v - 1));
    default: return true;
  }
}

FLASHMEM int PickMod::Div(int p, int v) const {
  const ParamDef& pd = param(p);
  v = constrain(v, int(pd.min), int(pd.max));
  if (v < 1) return 0;
  // the nearest member at or below, else the first above
  for (int d = v; d >= 1; d--) if (InSet(mode[p], d)) return d;
  for (int d = v + 1; d <= pd.max; d++) if (InSet(mode[p], d)) return d;
  return 1;
}

FLASHMEM int PickMod::Step(int p, int v, int dir) const {
  if (p != P_DIVA && p != P_DIVB) return Modulator::Step(p, v, dir);
  // from member to member of the set (DIV B: below the first is off)
  const ParamDef& pd = param(p);
  for (int d = v + dir; d >= pd.min && d <= pd.max; d += dir)
    if (d < 1 || InSet(mode[p], d)) return d;
  return v;
}

FLASHMEM void PickMod::Pulse() {
  int fired = 0;
  for (int k = 0; k < 2; k++) {
    const int p = k ? P_DIVB : P_DIVA, div = Div(p, cur[p]);
    if (!div) continue;
    if (count[k] < 255) count[k]++;
    if (count[k] >= div) { count[k] = 0; fired |= 1 << k; }
  }
  // DENS: at random, some hits are left out
  if (fired && int(random(100)) >= cur[P_DENS]) fired |= 4;
  hits[hits_pos] = uint8_t(fired);
  hits_pos = (hits_pos + 1) % HITS;
  if (fired && !(fired & 4)) {
    hit_lvl = (fired & 1) ? 1.0f : 0.6f;
    gate_s = std::max(0.005f, tick_s * cur[P_GATE] * 0.0095f);
    if (!cur[P_GATE]) gate_s = constrain(tick_s * 0.25f, 0.005f, 0.03f);
    flash = 0.06f;
  }
}

FLASHMEM void PickMod::Tick(const ModCtx& ctx) {
  const bool synced = Synced(P_RATE);
  if (Restarted() && synced) { grid_at = -1; Sync(); }
  if (cur[P_RESET] && Edge(cur[P_RESET], ctx, 0)) Sync();
  const bool bursts = cur[P_START] != 0;
  bool now = false;
  if (bursts && Edge(cur[P_START], ctx, 1)) {
    // a burst: from the top, its first tick right away
    Sync();
    burst_left = cur[P_BURST];
    t = 0.0f;
    now = true;
  }
  if (synced) {
    const float db = DivBeats(DivIndex(cur[P_RATE], 0, 100));
    tick_s = db * 60.0f / tempo_bpm;
    if (clock_running) {
      const int64_t g = ClockGrid(db);
      if (g != grid_at) { if (grid_at >= 0 && !now) now = true; grid_at = g; }
    }
  } else {
    const float hz = RateHz(P_RATE, 0.1f, 50.0f);
    tick_s = 1.0f / hz;
    t += hz * ctx.dt;
    if (t >= 1.0f) { t -= floorf(t); now = true; }
  }
  if (now && (!bursts || burst_left > 0)) {
    Pulse();
    if (bursts) burst_left--;
  }
  if (gate_s > 0.0f) { gate_s -= ctx.dt; out = hit_lvl; }
  else out = 0.0f;
}

FLASHMEM void PickMod::FormatMode(int p, char* buf, size_t n) const {
  if (p == P_DIVA || p == P_DIVB) {
    snprintf(buf, n, "%s", SET_NAMES[constrain(int(mode[p]), 0, SETS - 1)]);
  } else {
    Modulator::FormatMode(p, buf, n);
  }
}

FLASHMEM void PickMod::Format(int p, char* buf, size_t n) const {
  if (p == P_RATE) FormatRate(p, RateHz(P_RATE, 0.1f, 50.0f), buf, n);
  else if (p == P_START || p == P_RESET) FormatTrig(cur[p], buf, n);
  else if (p == P_DIVA || p == P_DIVB) {
    const int d = Div(p, val[p]);
    if (d) snprintf(buf, n, "/%d", d); else snprintf(buf, n, "OFF");
  } else if (p == P_GATE && !cur[P_GATE]) snprintf(buf, n, "SHORT");
  else Device::Format(p, buf, n);
}

// the last ticks going by, newest on the right: A hits full height, B hits
// half, both: A over B; hits DENS left out dotted; every tick a dot on
// the line; while a burst runs, what is left of it
FLASHMEM void PickMod::DrawViz(int x, int y, int w, int h) {
  const int ww = w - 8, base = y + h - 2, top = y + 1;
  const int cw = std::max(2, ww / HITS);
  for (int i = 0; i < HITS; i++) {
    const uint8_t f = hits[(hits_pos + i) % HITS];
    const int cx = x + i * cw;
    gfxPixel(cx, base + 1);
    if (!(f & 3)) continue;
    const int bh = (f & 1) ? base - top : (base - top) / 2;
    if (f & 4) gfxDottedLine(cx, base, cx, base - bh, 2);
    else gfxRect(cx, base - bh, std::max(1, cw - 1), bh + 1);
  }
  if (cur[P_START] && burst_left > 0) {
    char b[6];
    snprintf(b, sizeof(b), "%d", burst_left);
    gfxPrint(x + 1, y + 1, b);
  }
  const int bh = int(out * h);
  gfxFrame(x + w - 4, y, 4, h);
  if (bh > 0) gfxRect(x + w - 4, y + h - bh, 4, bh);
}

/////////////////////////////////////////////////////////////////
// MOD slots: type 0 = off
/////////////////////////////////////////////////////////////////
constexpr int NUM_MODS = 4;
constexpr int MOD_TYPES = 7;
// how many types a save file of this version holds (20 and older: no
// TREADLE; 21: no PICK)
constexpr int ModTypesIn(int version) { return version >= 22 ? 7 : version >= 21 ? 6 : 5; }

// by name: ADSR, FOLLOW, RANDOM, WAVE, then TREADLE and PICK (added later, at
// the end so saved files keep their numbers; files from before the sort used
// WAVE, RANDOM, ADSR, FOLLOW: OLD_MOD_TYPE maps them)
PROGMEM static const char MOD_TYPE_TAGS[MOD_TYPES][4] = {"-", "ENV", "FOL", "RND", "WAV", "TRD", "PCK"};
constexpr uint8_t OLD_MOD_TYPE[MOD_TYPES] = {0, 4, 3, 1, 2, 5, 6};

FLASHMEM static Modulator* MakeModulator(int type) {
  switch (type) {
    case 1: return new AdsrMod;
    case 2: return new FollowMod;
    case 3: return new RandomMod;
    case 4: return new WaveMod;
    case 5: return new TreadleMod;
    case 6: return new PickMod;
    default: return nullptr;
  }
}

} // namespace loom
