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
  float hist_t = 0.0f, hist_peak = 0.0f;
  float flash = 0.0f;   // trigger indicator, seconds left

  explicit Modulator(const DeviceDef& d);
  void Apply(int) override {}  // settings are read live in Tick()
  virtual void Tick(const ModCtx& ctx) = 0;
  // out is on notes, 4 octaves across: a CV output gives it as pitch (the
  // module's own octave: 1.2 V on the 2X0, 1 V on the SX0)
  virtual bool Pitched() const { return false; }
  // the value without a glide (RANDOM: its step as it is)
  virtual float Stepped() const { return out; }
  // several outputs (PICK, TREADLE): how many, each one's name (5 letters
  // at most) and value; own_out is the one its own box sends (out), the
  // other boxes can take any of them (see the MOD view)
  virtual int Outs() const { return 1; }
  virtual void OutName(int k, char* buf, size_t n) const { snprintf(buf, n, "OUT"); }
  virtual float OutVal(int k) const { return out; }
  uint8_t own_out = 0;
  // WAVE, PICK, TREADLE: A runs it, A held stops it (as SHUTTLE); stopped,
  // WAVE holds where it is and the others play no hits
  virtual bool Stops() const { return false; }
  bool stopped = false;
  bool ActionLit(int a) const override { return a == 0 && Stops() && !stopped; }
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
  if (Outs() > 1) out = OutVal(std::min<int>(own_out, Outs() - 1));
  out = constrain(out, 0.0f, 1.0f);
  if (flash > 0.0f) flash -= ctx.dt;
  // history for the scope: ~1 s across the screen
  // (the highest since the last point, so short pulses show)
  hist_t += ctx.dt;
  hist_peak = std::max(hist_peak, out);
  if (hist_t >= 1.0f / HIST) {
    hist_t = 0.0f;
    hist[hist_pos] = hist_peak;
    hist_peak = 0.0f;
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
  "WAVE", "WAV", 7, {"RUN", "RST"},  // (A held: stop)
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
  void Action(int a) override {
    if (a == 0) stopped = false;
    else if (a == 1) { ph = 0.0f; flash = 0.1f; }
  }
  bool Stops() const override { return true; }
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
  if (stopped) { out = At(ph); return; }
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
  "RANDOM", "RND", 8, {"STP", "RST"},  // B: the pattern from its first step
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
  void Action(int a) override {
    if (a == 1 && cur[P_LEN] > 0) pos = uint8_t(cur[P_LEN] - 1);  // (so the step lands on the first)
    Step();
  }
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
  "ADSR", "ENV", 7, {"TRG", "REL"},  // B: let go now
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
  void Action(int a) override {
    if (a == 0) { stage = ATTACK; manual = 0.15f; flash = 0.1f; }
    else if (stage != IDLE && stage != RELEASE) { stage = RELEASE; rel_from = env; manual = 0.0f; }
  }
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
  "FOLLOW", "FOL", 6, {"HOLD", "DROP"},  // A: holds where it is (again: free); B: down to nothing at once
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
  void Action(int a) override { if (a == 0) held = !held; else env = 0.0f; flash = 0.05f; }
  bool ActionLit(int a) const override { return a == 0 && held; }
  void DrawViz(int x, int y, int w, int h) override;
private:
  float env = 0.0f;
  float in_hist[HIST] = {};  // the level coming in, alongside hist (the output)
  float in_peak = 0.0f;
  bool held = false;
};

FLASHMEM void FollowMod::Tick(const ModCtx& ctx) {
  const float lvl = (cur[P_INPUT] ? ctx.out_level : ctx.in_level) * cur[P_GAIN] * 0.01f;
  const float tau = lvl > env ? Seconds(cur[P_RISE], 0.001f, 1.0f) : Seconds(cur[P_FALL], 0.005f, 3.0f);
  if (!held) env += (lvl - env) * std::min(1.0f, ctx.dt / tau);
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
  "CLOCK", "CLK", 3, {"TAP", "RST"},  // B: the beat from one again (all on the clock with it)
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
  if (a == 1) { ClockStart(); return; }
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
// treadles are pressed in, the rhythm of the weave. STYLE: ROCK, FUNK,
// HIPHP, HOUSE, BREAK, LATIN, DUB, four beats each (BEAT), or MAP: the 25
// drum patterns of Grids (c) 2011-2012 Emilie Gillet, GPL 3 (the pattern
// data), each 32 steps for a low, a mid and a high part; LOW / MID / HIGH set
// how many of each part's steps sound (their L knob: how many steps the
// part has, 1-32, so they can run against each other), CHAOS changes
// them as a drummer would, a variation held two run throughs: below 0
// it thins the groove out, above 0 it plays round it (pickups, ghost
// notes, a fill at the end of every other run through); its L knob
// swings the off steps; PLAY: forwards, backwards, back and
// forth, or a drunken walk. Six outputs: the three parts (each hit a short pulse,
// all full height), then each part's
// accents alone. Its own box sends one; the other boxes can take the rest.
/////////////////////////////////////////////////////////////////
// the styles' beats, two bars of sixteenths for each part (kick, snare,
// hi-hat): X an accent, x a hit, o an extra one (in from FILL ~55), - a
// rare one (~80), . none (the steps between come in last, the beats first)
PROGMEM static const char TRD_STYLES[8][6] = {"MAP", "ROCK", "FUNK", "HIPHP", "HOUSE", "BREAK", "LATIN", "DUB"};
PROGMEM static const char TRD_BEAT_NAMES[7][4][6] = {
  {"BEAT", "PUSH", "DRIVE", "SLOW"},  // ROCK
  {"JB", "GHOST", "SYNC", "LINE"},  // FUNK
  {"BOOM", "LAZY", "TRAP", "BOUNC"},  // HIPHP
  {"FOUR", "GARAG", "TECHN", "DISCO"},  // HOUSE
  {"AMEN", "FUNKY", "THINK", "DNB"},  // BREAK
  {"BOSSA", "SAMBA", "CLAVE", "DEMBW"},  // LATIN
  {"1DROP", "STEPR", "ROCKR", "SKA"},  // DUB
};
PROGMEM static const char TRD_BEATS[7][4][3][33] = {
  {  // ROCK
    {"X.......X.o.....X.......X.x...-.", "....X.......X..-....X.......X.o.", "X-x-X-x-X-x-X-x-X-x-X-x-X-x-X-x-"},  // BEAT
    {"X.....x.X.......X.....x.X.x.-...", "....X.......X.......X.......X..o", "X-x-X-x-X-x-X-x-X-x-X-x-X-x-X-x-"},  // PUSH
    {"X.x...x.X.x...x.X.x...x.X.x...x.", "....X.......X.......X.......X.o-", "X.o.X.o.X.o.X.o.X.o.X.o.X.o.X.o."},  // DRIVE
    {"X.........x.....X.....o...x.....", "........X...............X.....o.", "X.x.x.x.X.x.x.x.X.x.x.x.X.x.x.x."},  // SLOW
  },
  {  // FUNK
    {"X.x.......x..x..X.x.......x...o.", "....X..o.o..X..o....X..o.o..X.o.", "X-x-X-x-X-x-X-x-X-x-X-x-X-x-X-x-"},  // JB
    {"X.....x...x.....X.....x...x..o..", "....X.o.o.o.X.o.....X.o..o.oX.oo", "X.x.X.x.X.x.X.x.X.x.X.x.X.x.X.x."},  // GHOST
    {"X..x..x...x..x..X..x..x...x.....", "....X.......X.......X..o....X.o.", "x-x-X-x-x-x-X-x-x-x-X-x-x-x-X-x-"},  // SYNC
    {"X..x....x.x.....X..x....x.x..x..", "....X..o.o..X.......X..o.o..X..o", "..x...x...x...x...x...x...x...x."},  // LINE
  },
  {  // HIPHP
    {"X......x..X.....X......x..X..x..", "....X.......X.......X.......X..o", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // BOOM
    {"X.........x.x...X..o......x.....", "....X.......X..o....X.......X...", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // LAZY
    {"X.....x...x....xX.....x.......x.", "........X...............X.......", "x.x.x.x.x.x.xxxxx.x.x.x.x.xxx.x."},  // TRAP
    {"X..x..x...x.....X..x..x...x..o..", "....X.......X.......X.......X.o.", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // BOUNC
  },
  {  // HOUSE
    {"X...X...X...X...X...X...X...X...", "....X.......X.......X.......X...", "-.X.-.X.-.X.-.X.-.X.-.X.-.X.-.X."},  // FOUR
    {"X......x..x.....X......x..x...o.", "....X.......X.......X.......X..o", "..x.x.x...x.x.x...x.x.x...x.x.x."},  // GARAG
    {"X...X...X...X...X...X...X...X...", "........X...............X.....o.", "xxXxxxXxxxXxxxXxxxXxxxXxxxXxxxXx"},  // TECHN
    {"X...X...X...X...X...X...X...X...", "....X.......X.......X.......X.o.", "x.X.x.X.x.X.x.X.x.X.x.X.x.X.x.X."},  // DISCO
  },
  {  // BREAK
    {"X.X.......XX......XX......X.....", "....X..x.x..X..x.x..X..x.x....x.", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // AMEN
    {"X.X.......X..X..X.X.......X..x..", "....X..x.x.oX..x....X..x.x.oX.o.", "x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-"},  // FUNKY
    {"X......xX.X.....X......xX.x..x..", "....X.......X.......X..o....X.o.", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // THINK
    {"X.........X.......X.......X.....", "....X.......X.......X.......X..o", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // DNB
  },
  {  // LATIN
    {"X..xX..xX..xX..xX..xX..xX..xX..x", "x..x..x...x..x....x..x...x..x...", "x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-"},  // BOSSA
    {"X..xx..xX..xx..xX..xx..xX..xx..x", "x.xx.x.xx.x.x.x.x.xx.x.x.xx.x.x.", "XxxxXxxxXxxxXxxxXxxxXxxxXxxxXxxx"},  // SAMBA
    {"X.....x.......x.X.....x.......x.", "X..X..X...X.X...X..X..X...X.X...", "x.x.xx.x.xx.x.x.x.x.xx.x.xx.x.x."},  // CLAVE
    {"X...X...X...X...X...X...X...X...", "...X..X....X..X....X..X....X..X.", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // DEMBW
  },
  {  // DUB
    {"........X...............X.....o.", "........X...............X.....o.", "x.x.x.x.x.x.x.x.x.x.x.x.x.x.x.x."},  // 1DROP
    {"X...X...X...X...X...X...X...X...", "........X...............X.....o.", "..x...x...x...x...x...x...x...x."},  // STEPR
    {"X.......X.......X.......X.o.....", "....X.......X.......X.......X..o", "x.xxx.xxx.xxx.xxx.xxx.xxx.xxx.xx"},  // ROCKR
    {"X.......X.......X.......X.....o.", "....X.......X.......X.......X.o.", "..X...X...X...X...X...X...X...X."},  // SKA
  },
};
static constexpr int TRD_NSTYLES = sizeof(TRD_STYLES) / sizeof(TRD_STYLES[0]);

PROGMEM static const DeviceDef TREADLE_DEF = {
  "TREADLE", "TRD", 8, {"RUN", "RST"},  // (A held: stop; B resets)
  {
    {"LOW", 0, 100, 50, UNI, 0, 32, 31},    // each part's fill; mode: its length - 1
    {"MID", 0, 100, 50, UNI, 0, 32, 31},
    {"HIGH", 0, 100, 50, UNI, 0, 32, 31},
    {"PLAY", 0, 3, 0, LIST, 4, 0, 0, {"FWD", "REV", "PING", "DRUNK"}},
    {"STYLE", 0, TRD_NSTYLES - 1, 1, UNI},  // MAP, then the styles
    {"BEAT", 0, 24, 0, UNI},                 // the style's beat (MAP: one of its 25)
    {"CHAOS", -32, 32, 0, BI, 0, 13, 0},  // (-: thins out, +: plays round the groove; mode: the swing, SW50-SW74)
    {"RATE", 0, 100, 74, UNI, 0, 2, 1, {}, {"FREE", "CLOCK"}},  // a step; on the clock (74: 1/16)
  },
  {"FILL", "GROOVE"}
};

class TreadleMod : public Modulator {
public:
  enum { P_LOW, P_MID, P_HIGH, P_PLAY, P_STYLE, P_BEAT, P_CHAOS, P_RATE };
  enum { FWD, REV, PING, DRUNK };
  static constexpr int OUTS = 6;  // LOW MID HIGH, then their accents
  TreadleMod() : Modulator(TREADLE_DEF) {}
  void Tick(const ModCtx& ctx) override;
  void Format(int p, char* buf, size_t n) const override;
  int Step(int p, int v, int dir) const override {
    if (p == P_BEAT) return constrain(v + dir, 0, Beats(constrain(int(val[P_STYLE]), 0, TRD_NSTYLES - 1)) - 1);
    return Modulator::Step(p, v, dir);
  }
  void Action(int a) override { if (a == 0) stopped = false; else if (a == 1) Restart(); }
  bool Stops() const override { return true; }
  void DrawViz(int x, int y, int w, int h) override;
  int Outs() const override { return OUTS; }
  void OutName(int k, char* buf, size_t n) const override {
    static const char names[OUTS][6] = {"LOW", "MID", "HIGH", "ACC.L", "ACC.M", "ACC.H"};
    snprintf(buf, n, "%s", names[constrain(k, 0, OUTS - 1)]);
  }
  float OutVal(int k) const override { return k >= 0 && k < OUTS && gate[k] > 0.0f && wait[k] <= 0.0f ? lvl[k] : 0.0f; }
  void FormatMode(int p, char* buf, size_t n) const override {
    if (p >= P_LOW && p <= P_HIGH) snprintf(buf, n, "LEN%d", Len(p - P_LOW));
    else if (p == P_CHAOS) snprintf(buf, n, "SW%d", Swing());
    else Modulator::FormatMode(p, buf, n);
  }
  int BarStyle(int p) const override { return (p >= P_LOW && p <= P_HIGH) || p == P_CHAOS ? BAR_LEFT : Modulator::BarStyle(p); }
private:
  int step = -1;           // steps since the start (-1: none yet)
  int pos[3] = {};         // where each part is
  // each part's PLAY as it plays this round (a change waits for an end),
  // its direction (PING), steps into the round (DRUNK)
  uint8_t play_now[3] = {}, run[3] = {};
  int8_t dir[3] = {1, 1, 1};
  int64_t grid_at = -1, grid_base = 0;
  float grid_div = 0.0f;
  float t = 0.0f, step_s = 0.125f;
  uint32_t drop[3] = {};   // CHAOS: the steps each part leaves out this run through
  uint32_t add[3] = {};    // and the ones it adds
  uint8_t rounds[3] = {};  // each part's run throughs (a variation holds two)
  int8_t chaos_was[3] = {};
  bool fill_on = false;    // the snare's fill this run through (the hi-hat holds off)
  int fill_from = 0;
  float gate[OUTS] = {}, lvl[OUTS] = {}, wait[OUTS] = {};  // (wait: swung, the hit comes later)
  int Len(int part) const { return constrain(int(mode[P_LOW + part]) + 1, 1, 32); }
  static int Beats(int style) { return style == 0 ? 25 : 4; }
  int Style() const { return constrain(int(cur[P_STYLE]), 0, TRD_NSTYLES - 1); }
  int Beat() const { return constrain(int(cur[P_BEAT]), 0, Beats(Style()) - 1); }
  int Swing() const { return 50 + 2 * constrain(int(mode[P_CHAOS]), 0, 12); }  // % of two steps the first takes
  int Level(int part, int s) const;  // the map's strength of a part's step s, with CHAOS (0..255)
  bool Accent(int part, int s, int lv) const;
  mutable uint8_t lv_cache[3][32] = {};  // the styles' strengths (Level), for this style and beat
  mutable int lv_key = -1;
  bool Hits(int part, int lv) const { return lv > 255 - cur[P_LOW + part] * 255 / 100; }
  void Restart() { grid_base = grid_at >= 0 ? grid_at + 1 : 0; step = -1; t = 0.0f; }
  void Advance(int s);
  void Groove(int part, int len, int ch);
  void Thin(int part, int len, int ch);
};

// a step's strength (0..255): MAP's from its patterns, the styles' from
// their beats: each kind of step (X x o - .) has its own band (X 195-250,
// x 130-190: FILL 50 plays both), and within it every step its own
// strength, the beats above the eighths above the sixteenths, earlier
// before later, so FILL brings them in one at a time
FLASHMEM int TreadleMod::Level(int part, int s) const {
  s &= 31;
  const int st = Style(), bt = Beat();
  if (st == 0) return grids::drum_map[bt / 5][bt % 5][part * 32 + s];
  const int key = st * 32 + bt;
  if (key != lv_key) {
    static const char KINDS[5] = {'X', 'x', 'o', '-', '.'};
    static const uint8_t HI[5] = {250, 190, 128, 88, 48}, LO[5] = {195, 130, 90, 52, 2};
    for (int pt = 0; pt < 3; pt++) {
      const char* row = TRD_BEATS[st - 1][bt][pt];
      for (int kd = 0; kd < 5; kd++) {
        int idx[32], n = 0;
        for (int k = 0; k < 32; k++) {
          const char c = row[k];
          if (c == KINDS[kd] || (kd == 4 && c != 'X' && c != 'x' && c != 'o' && c != '-')) idx[n++] = k;
        }
        // the stronger places first (the one, the beats, the eighths ...), then earlier
        auto weight = [](int k) { const int b = k & 15; return b == 0 ? 5 : (b & 7) == 0 ? 4 : (b & 3) == 0 ? 3 : (b & 1) == 0 ? 2 : 1; };
        std::stable_sort(idx, idx + n, [&](int a, int b) { return weight(a) > weight(b); });
        for (int i = 0; i < n; i++)
          lv_cache[pt][idx[i]] = uint8_t(HI[kd] - (i * 2 + 1) * (HI[kd] - LO[kd]) / (2 * n));
      }
    }
    lv_key = key;
  }
  return lv_cache[part][s];
}

// an accent: MAP's strong steps, the styles' X
FLASHMEM bool TreadleMod::Accent(int part, int s, int lv) const {
  const int st = Style();
  if (st == 0) return lv > 192;
  return TRD_BEATS[st - 1][Beat()][part][s & 31] == 'X';
}

// CHAOS above 0: what a drummer plays round a groove, in the figures
// they play, as many steps as that share of the ones the part plays (as
// many again at 32: the fill sets how busy, CHAOS only varies it; no
// steps, nothing added), round a beat picked at random:
//   kick (LOW): the pickup into one of its hits, the push on the "and",
//     a double into its next hit, the "ba-dum" after one; never on the
//     snare's hits
//   snare (MID): a ghost note before or after its hit, a drag (two)
//     into it; between its hits, ghost notes on the "e" and the "a"
//   hi-hat (HIGH): the "and", an "and-a" pair, the skip on the "a", a
//     run of three sixteenths
FLASHMEM void TreadleMod::Groove(int part, int len, int ch) {
  auto at = [&](int k) { return ((k % len) + len) % len; };
  auto on = [&](int pt, int k) { return Hits(pt, Level(pt, at(k))); };
  int hits = 0;
  for (int k = 0; k < len; k++) if (on(part, k)) hits++;
  int budget = (hits * ch + 16) / 32;
  const int beats = std::max(1, len / 4);
  for (int tries = 0; budget > 0 && tries < 32; tries++) {
    const int bt = int(random(beats)) * 4, f = int(random(4));
    int k[3], n = 0;
    switch (part) {
      case 0:
        if (f == 0 && on(0, bt)) k[n++] = bt - 1;
        else if (f == 1) k[n++] = bt + 2;
        else if (f == 2 && on(0, bt + 4)) { k[n++] = bt + 2; k[n++] = bt + 3; }
        else if (f == 3 && on(0, bt)) k[n++] = bt + 1;
        break;
      case 1:
        if (on(1, bt)) {
          if (f == 0) k[n++] = bt - 1;
          else if (f == 1) { k[n++] = bt - 2; k[n++] = bt - 1; }
          else k[n++] = bt + (f == 2 ? 1 : 3);
        } else if (f >= 2) {
          k[n++] = bt + 1;
          if (f == 3) k[n++] = bt + 3;
        }
        break;
      default:
        if (f == 0) k[n++] = bt + 2;
        else if (f == 1) { k[n++] = bt + 2; k[n++] = bt + 3; }
        else if (f == 2) k[n++] = bt + 3;
        else { k[n++] = bt + 1; k[n++] = bt + 2; k[n++] = bt + 3; }
    }
    bool ok = n > 0;
    for (int i = 0; i < n && ok; i++)
      ok = !on(part, k[i]) && !(add[part] >> at(k[i]) & 1) && !(part == 0 && on(1, k[i]));
    if (!ok) continue;
    for (int i = 0; i < n; i++) add[part] |= 1u << at(k[i]);
    budget -= n;
  }
}

// CHAOS below 0: leaves out that share of the steps the part plays (all
// at -32), the least needed first: the sixteenths in between, then the
// eighths, then the beats, the one last; accents, and the snare's
// backbeat, hold on longer. A little chance in which
FLASHMEM void TreadleMod::Thin(int part, int len, int ch) {
  int sc[32], idx[32], n = 0;
  for (int k = 0; k < len; k++) {
    const int lv = Level(part, k);
    if (!Hits(part, lv)) continue;
    const int b = k & 15;
    int keep = b == 0 ? 160 : (b & 7) == 0 ? 120 : (b & 3) == 0 ? 90 : (b & 1) == 0 ? 55 : 20;
    keep += lv / 4 + (Accent(part, k, lv) ? 30 : 0);
    if (part == 1 && (b & 7) == 4) keep += 50;
    idx[n] = k;
    sc[n++] = keep + int(random(40));
  }
  const int want = (n * ch + 16) / 32;
  for (int i = 0; i < want && i < n; i++) {
    int worst = i;
    for (int j = i + 1; j < n; j++) if (sc[j] < sc[worst]) worst = j;
    std::swap(sc[i], sc[worst]);
    std::swap(idx[i], idx[worst]);
    drop[part] |= 1u << idx[i];
  }
}

// step s since the start: each part one step on. Its length counts at
// once, the step it is on stays: it plays on to the new end and turns
// there (shorter than where it is: it turns on the next step). A new PLAY
// waits for an end (FWD: past the last step, REV: past the first, PING:
// back at the start, DRUNK: as many steps as the part is long).
FLASHMEM void TreadleMod::Advance(int s) {
  step = s;
  const int play = constrain(int(cur[P_PLAY]), 0, 3);
  for (int part = 0; part < 3; part++) {
    const int len = Len(part);
    int& ps = pos[part];
    bool end = s == 0;  // (the start: everything new)
    if (!end) {
      switch (play_now[part]) {
        case REV:
          if (ps >= len) ps = len - 1;  // (past the new end: in at its last step)
          else if (--ps < 0) end = true;
          break;
        case PING:
          if (len < 2) { end = true; break; }
          if (ps >= len - 1) { ps = std::min(ps, len - 1); dir[part] = -1; }
          ps += dir[part];
          if (dir[part] < 0 && ps <= 0) end = true;  // (back at the start: the round is done)
          break;
        case DRUNK:
          ps = ((ps < len ? ps : len - 1) + (random(2) ? 1 : len - 1)) % len;
          if (++run[part] >= len) end = true;
          break;
        default: if (++ps >= len) end = true;
      }
    }
    if (!end) continue;
    // CHAOS: below 0 it thins out (Thin), above 0 it plays round the
    // groove (Groove); a variation holds two run throughs (anew at once
    // when CHAOS turns). At the end of the second, a chance (CHAOS in 32)
    // of a snare fill in the last beat: all four sixteenths, the "and-a",
    // the "e-and-a" or the one, "and" and "a"
    const int ch = cur[P_CHAOS];
    rounds[part]++;
    if (s == 0 || ch != chaos_was[part] || !(rounds[part] & 1)) {
      drop[part] = add[part] = 0;
      if (ch < 0) Thin(part, len, -ch);
      if (ch > 0) Groove(part, len, ch);
      chaos_was[part] = int8_t(ch);
      rounds[part] = 0;
    }
    if (part == 1) {
      fill_on = false;
      if (ch > 0 && (rounds[1] & 1) && len >= 4 && int(random(32)) < ch) {
        static const uint8_t FILLS[4] = {0b1111, 0b1100, 0b1110, 0b1101};  // (bit 0: the beat's first step)
        const int f = FILLS[random(4)];
        fill_from = len - 4;
        for (int i = 0; i < 4; i++) if (f >> i & 1) add[1] |= 1u << (fill_from + i);
        fill_on = true;
      }
    }
    const int was = play_now[part];
    play_now[part] = uint8_t(play);
    run[part] = 0;
    switch (play) {
      case REV: ps = len - 1; break;
      case PING: ps = 0; dir[part] = 1; break;
      case DRUNK: ps = s == 0 || was != DRUNK ? 0 : ps % len; break;  // (walking on from where it is)
      default: ps = 0;
    }
  }
  const float g = constrain(step_s * 0.25f, 0.005f, 0.03f);
  // swing: every other step a little late
  const float late = (s & 1) ? (Swing() - 50) * 0.02f * step_s : 0.0f;
  for (int part = 0; part < 3; part++) {
    if (part == 2 && fill_on && pos[1] >= fill_from) continue;  // (the snare's fill: the hi-hat holds off)
    const int lv = Level(part, pos[part]);
    const bool hit = Hits(part, lv) && !(drop[part] >> (pos[part] & 31) & 1);
    if (!hit && !(add[part] >> (pos[part] & 31) & 1)) continue;
    // every hit full height (triggers want 10 V); the accents (the map's
    // strong steps) also on their own outputs, to drive a level with
    const bool acc = hit && Accent(part, pos[part], lv);
    gate[part] = g;
    lvl[part] = 1.0f;
    wait[part] = late;
    if (acc) { gate[3 + part] = g; lvl[3 + part] = 1.0f; wait[3 + part] = late; }
    if (part == own_out % 3) flash = 0.06f;
  }
}

FLASHMEM void TreadleMod::Tick(const ModCtx& ctx) {
  const bool synced = Synced(P_RATE);
  if (Restarted() && synced) { grid_at = -1; grid_base = 0; step = -1; }
  for (int k = 0; k < OUTS; k++) {
    if (wait[k] > 0.0f) wait[k] -= ctx.dt;
    else if (gate[k] > 0.0f) gate[k] -= ctx.dt;
  }
  if (stopped) { for (float& g : gate) g = 0.0f; grid_at = -1; return; }
  if (synced && clock_running) {
    const float db = DivBeats(DivIndex(cur[P_RATE], 0, 100));
    step_s = db * 60.0f / tempo_bpm;
    const int64_t g = ClockGrid(db);
    if (db != grid_div) { grid_div = db; grid_base = g - (step >= 0 ? step : 0); grid_at = g - 1; }
    if (g != grid_at) {
      grid_at = g;
      Advance(int(std::max<int64_t>(0, g - grid_base)));
    }
  } else if (!synced) {
    const float hz = RateHz(P_RATE, 0.5f, 30.0f);
    step_s = 1.0f / hz;
    t += hz * ctx.dt;
    if (t >= 1.0f || step < 0) { t -= floorf(t); Advance(step < 0 ? 0 : step + 1); }
  }
}

FLASHMEM void TreadleMod::Format(int p, char* buf, size_t n) const {
  const int st = constrain(int(val[P_STYLE]), 0, TRD_NSTYLES - 1);
  if (p == P_RATE) FormatRate(p, RateHz(P_RATE, 0.5f, 30.0f), buf, n);
  else if (p == P_STYLE) snprintf(buf, n, "%s", TRD_STYLES[st]);
  else if (p == P_BEAT && st == 0) snprintf(buf, n, "%d", constrain(int(val[P_BEAT]), 0, 24) + 1);
  else if (p == P_BEAT) snprintf(buf, n, "%s", TRD_BEAT_NAMES[st - 1][constrain(int(val[P_BEAT]), 0, 3)]);
  else Device::Format(p, buf, n);
}

// the three parts in rows, LOW at the top, each as long as it is: a bar
// per step that sounds (accents the row's full height), a dot for the
// steps that don't, a line under the step playing; the part its own box
// sends marked by a line on the left. 4 pixels a step whatever the length
// (32 fill the width; the last one's gap falls off the edge)
FLASHMEM void TreadleMod::DrawViz(int x, int y, int w, int h) {
  const int rh = h / 3;
  const int bw = std::max(2, (w + 2) / 32);
  for (int part = 0; part < 3; part++) {
    const int top = y + part * rh, base = top + rh - 3, len = Len(part);
    if (part == own_out % 3) gfxLine(x, top + 1, x, base);
    for (int s = 0; s < len; s++) {
      const int lv = Level(part, s), bx = x + 1 + s * bw;
      if (Hits(part, lv)) {
        // (left out by CHAOS this run through: hollow)
        const int bh = Accent(part, s, lv) ? rh - 3 : (rh - 3) * 2 / 3;
        if (drop[part] >> s & 1) gfxFrame(bx, base - bh + 1, std::max(1, bw - 1), bh);
        else gfxRect(bx, base - bh + 1, std::max(1, bw - 1), bh);
      } else if (add[part] >> s & 1) {
        // (added by CHAOS this run through: a short bar)
        const int bh = std::max(2, (rh - 3) / 3);
        gfxRect(bx, base - bh + 1, std::max(1, bw - 1), bh);
      } else {
        gfxPixel(bx, base);
      }
    }
    if (step >= 0 && pos[part] < len) gfxLine(x + 1 + pos[part] * bw, base + 2, x + pos[part] * bw + std::max(1, bw - 1), base + 2);
  }
}

/////////////////////////////////////////////////////////////////
// PICK: a clock and its dividers. On a loom a pick is one pass of the
// weft; picks per inch is how dense the cloth is. RATE ticks (TICK); DENS
// lets each tick through at random (TOCK). The dividers count the ticks,
// or with DENS's mode on TOCK only the ones let through: DIV A and DIV B
// strike every n-th, their L knob keeping the count to a set of numbers
// (all, odd, even, primes, triplets, Fibonacci, powers of two), and eight
// fixed ones /2 /3 /4 /6 /8 /12 /16 /24. A divider changed while running
// counts on from its last hit, so A and B on the same count can sit on
// different ticks; RESET (or the clock starting) lines them all up. START
// plays a burst of BURST ticks on each trigger and then rests.
// Thirteen outputs: A+B (A full height, B lower), A, B, TICK, TOCK and the
// fixed ones. Its own box sends one; the other boxes can take the rest.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef PICK_DEF = {
  "PICK", "PCK", 8, {"RUN", "RST"},  // (A held: stop; B resets)
  {
    {"RATE", 0, 100, 74, UNI, 0, 2, 1, {}, {"FREE", "CLOCK"}},  // a tick; on the clock (74: 1/16)
    {"DIVA", 1, 144, 3, UNI, 0, 7, 0, {}, {}},  // mode: the set of numbers
    {"DIVB", 0, 144, 4, UNI, 0, 7, 0, {}, {}},  // (0: off)
    {"DENS", 0, 100, 100, UNI, 0, 2, 0, {}, {"TICK", "TOCK"}},  // mode: what the dividers count
    {"START", 0, TRIG_SOURCES - 1, 0, UNI},  // none: runs all the time
    {"BURST", 1, 64, 8, UNI},  // ticks per START
    {"RESET", 0, TRIG_SOURCES - 1, 0, UNI},
    {"GATE", 0, 100, 0, UNI},  // 0: a short pulse; then 1 ms .. 1 s (no longer than most of a tick)
  }
};

class PickMod : public Modulator {
public:
  enum { P_RATE, P_DIVA, P_DIVB, P_DENS, P_START, P_BURST, P_RESET, P_GATE };
  enum { SET_ALL, SET_ODD, SET_EVEN, SET_PRIME, SET_TRIP, SET_FIB, SET_POW2, SETS };
  enum { O_AB, O_A, O_B, O_TICK, O_TOCK, O_FIXED, OUTS = O_FIXED + 8 };
  PickMod() : Modulator(PICK_DEF) {}
  void Tick(const ModCtx& ctx) override;
  int Step(int p, int v, int dir) const override;
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  // a divider's name is its set once one is chosen (PRIME, FIB ..); DENS
  // keeps its name (its mode is what the dividers count)
  const char* Name(int p) const override {
    if ((p == P_DIVA || p == P_DIVB) && ModeNamed(p) && mode[p] < SETS) return SET_NAMES[mode[p]];
    if (p == P_DENS) return param(p).name;
    return Modulator::Name(p);
  }
  void Action(int a) override { if (a == 0) stopped = false; else if (a == 1) Sync(); }
  bool Stops() const override { return true; }
  void DrawViz(int x, int y, int w, int h) override;
  int Outs() const override { return OUTS; }
  void OutName(int k, char* buf, size_t n) const override;
  float OutVal(int k) const override { return k >= 0 && k < OUTS && og[k] > 0.0f ? olv[k] : 0.0f; }
private:
  static constexpr char SET_NAMES[SETS][6] = {"ALL", "ODD", "EVEN", "PRIME", "TRIP", "FIB", "2^N"};
  static constexpr uint8_t FIXED[8] = {2, 3, 4, 6, 8, 12, 16, 24};
  static constexpr int HITS = 30;  // (4 pixels each: B's bars hollow)
  uint8_t count[2] = {255, 255};  // ticks since each divider's last hit
  uint32_t counted = 0;           // ticks the dividers counted since the last sync
  uint8_t hits[HITS] = {};        // the last ticks: bit 0 A, bit 1 B, bit 2 TOCK, bit 3 resting
  uint8_t hits_pos = 0;
  int64_t grid_at = -1;
  float t = 0.0f, tick_s = 0.125f;
  float og[OUTS] = {}, olv[OUTS] = {};  // each output: its pulse's time left, its height
  int burst_left = 0;
  void Sync() { count[0] = count[1] = 255; counted = 0; }
  void Fire(int k, float lv = 1.0f);
  float GateSeconds(int v) const;
  void Pulse();
  static bool InSet(int set, int v);
  int Div(int p, int v) const;  // v snapped into p's set (0: off)
};

constexpr char PickMod::SET_NAMES[PickMod::SETS][6];
constexpr uint8_t PickMod::FIXED[8];

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

// how long a hit lasts: SHORT, a quarter of a tick (5 .. 30 ms); else
// 1 ms .. 1 s, but at most 95 % of a tick, so hits stay apart
FLASHMEM float PickMod::GateSeconds(int v) const {
  if (v <= 0) return constrain(tick_s * 0.25f, 0.005f, 0.03f);
  return std::min(Seconds(v, 0.001f, 1.0f), tick_s * 0.95f);
}

FLASHMEM void PickMod::Fire(int k, float lv) {
  og[k] = GateSeconds(cur[P_GATE]);
  olv[k] = lv;
}

FLASHMEM void PickMod::Pulse() {
  Fire(O_TICK);
  const bool tock = int(random(100)) < cur[P_DENS];
  if (tock) Fire(O_TOCK);
  int fired = tock ? 4 : 0;
  if (tock || !mode[P_DENS]) {  // the dividers count this one
    for (int k = 0; k < 2; k++) {
      const int p = k ? P_DIVB : P_DIVA, div = Div(p, cur[p]);
      if (!div) continue;
      if (count[k] < 255) count[k]++;
      if (count[k] >= div) { count[k] = 0; fired |= 1 << k; }
    }
    for (int j = 0; j < 8; j++) if (counted % FIXED[j] == 0) Fire(O_FIXED + j);
    counted++;
  }
  if (fired & 1) Fire(O_A);
  if (fired & 2) Fire(O_B);
  if (fired & 3) { Fire(O_AB, (fired & 1) ? 1.0f : 0.6f); flash = 0.06f; }
  hits[hits_pos] = uint8_t(fired);
  hits_pos = (hits_pos + 1) % HITS;
}

FLASHMEM void PickMod::Tick(const ModCtx& ctx) {
  const bool synced = Synced(P_RATE);
  if (Restarted() && synced) { grid_at = -1; Sync(); }
  if (cur[P_RESET] && Edge(cur[P_RESET], ctx, 0)) Sync();
  for (float& g : og) if (g > 0.0f) g -= ctx.dt;
  if (stopped) { for (float& g : og) g = 0.0f; grid_at = -1; return; }
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
  } else if (now) {  // resting between bursts: an empty tick goes by
    hits[hits_pos] = 8;
    hits_pos = (hits_pos + 1) % HITS;
  }
}

FLASHMEM void PickMod::OutName(int k, char* buf, size_t n) const {
  static const char names[O_FIXED][6] = {"A+B", "A", "B", "TICK", "TOCK"};
  if (k >= O_FIXED && k < OUTS) snprintf(buf, n, "/%d", FIXED[k - O_FIXED]);
  else snprintf(buf, n, "%s", names[constrain(k, 0, O_FIXED - 1)]);
}

FLASHMEM void PickMod::FormatMode(int p, char* buf, size_t n) const {
  if (p == P_DIVA || p == P_DIVB) snprintf(buf, n, "%s", SET_NAMES[constrain(int(mode[p]), 0, SETS - 1)]);
  else Modulator::FormatMode(p, buf, n);
}

FLASHMEM void PickMod::Format(int p, char* buf, size_t n) const {
  if (p == P_RATE) FormatRate(p, RateHz(P_RATE, 0.1f, 50.0f), buf, n);
  else if (p == P_START || p == P_RESET) FormatTrig(val[p], buf, n);
  else if (p == P_DIVA || p == P_DIVB) {
    const int d = Div(p, val[p]);
    if (d) snprintf(buf, n, "/%d", d); else snprintf(buf, n, "OFF");
  } else if (p == P_GATE) {
    // in ms as it plays now (held to most of a tick at fast rates)
    const float g = GateSeconds(val[P_GATE]);
    if (!val[P_GATE]) snprintf(buf, n, "SHORT");
    else if (g < 0.9995f) snprintf(buf, n, "%dms", std::max(1, int(g * 1000.0f + 0.5f)));
    else snprintf(buf, n, "1.0s");
  }
  else Device::Format(p, buf, n);
}

// the last ticks going by, newest on the right: A hits full height and
// solid, B hits half and hollow, both: A; under each tick a dot, two for the ones let
// through to TOCK (none between bursts); while a burst runs, what is left
FLASHMEM void PickMod::DrawViz(int x, int y, int w, int h) {
  const int ww = w - 8, base = y + h - 4, top = y + 1;
  const int cw = std::max(2, ww / HITS);
  for (int i = 0; i < HITS; i++) {
    const uint8_t f = hits[(hits_pos + i) % HITS];
    const int cx = x + i * cw;
    if (f & 8) continue;  // a rest between bursts
    gfxPixel(cx, base + 3);
    if (f & 4) gfxPixel(cx, base + 2);
    if (!(f & 3)) continue;
    if (f & 1) gfxRect(cx, top, std::max(1, cw - 1), base - top + 1);
    else gfxFrame(cx, base - (base - top) / 2, std::max(1, cw - 1), (base - top) / 2 + 1);
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
