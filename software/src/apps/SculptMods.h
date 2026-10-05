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

// Modulators for the Sculpt app: WAVE / RANDOM / ADSR / FOLLOW. Four slots per track (MOD A-D); a device setting picks one as its
// modulation source. They run in the main loop about once per millisecond and
// output 0..1, like a 0..10 V CV; the setting's U/B choice decides whether
// that pushes one way or swings around the setting.
//
// Triggers come from the gate inputs, the CV inputs used as gates, or MIDI
// maps (notes / CCs over half way). Rising edges on the hardware inputs are
// latched in the app's ISR so short pulses aren't missed.

#pragma once

#include "SculptDevices.h"

namespace sculpt {

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
  void Run(const ModCtx& ctx);
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;

protected:
  bool prev_gate[2] = {};
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
  return 1.0f / DivSeconds(NUM_DIVS - 1 - DivIndex(cur[p], 0, 100), mode[p]);
}

FLASHMEM int Modulator::Step(int p, int v, int dir) const {
  if (param(p).nmodes == 4 && mode[p] != SYNC_FREE) // synced RATE: whole notes
    return DivValue(DivIndex(v, 0, 100) + dir, 0, 100);
  return v + dir;
}

FLASHMEM void Modulator::FormatRate(int p, float hz, char* buf, size_t n) const {
  if (mode[p] != SYNC_FREE) FormatDiv(NUM_DIVS - 1 - DivIndex(val[p], 0, 100), mode[p], buf, n);
  else if (hz < 1.0f) snprintf(buf, n, "%ds", int(1.0f / hz + 0.5f));
  else snprintf(buf, n, "%dHz", int(hz + 0.5f));
}

FLASHMEM bool Modulator::Gate(int src) const { return TrigGate(src); }

// slot: which of this modulator's trigger settings (for MIDI edge tracking)
FLASHMEM bool Modulator::Edge(int src, const ModCtx& ctx, int slot) {
  return TrigEdge(src, ctx.edges, prev_gate[slot]);
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
    {"RATE", 0, 100, 40, UNI, 0, 4, 0, {}, {"FREE", "STR", "TRI", "DOT"}},
    {"AMT", 0, 100, 100, UNI},
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
  ph += Hz() * ctx.dt;
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
  if (flash > 0.0f) gfxInvert(x, y, ww, h);
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
  "RANDOM", "RND", 7, {"STP", ""},
  {
    {"RATE", 0, 100, 45, UNI, 0, 4, 0, {}, {"FREE", "STR", "TRI", "DOT"}},
    {"AMT", 0, 100, 100, UNI},
    {"SMTH", 0, 100, 0, UNI},
    {"OFFS", -50, 50, 0, BI},
    {"LEN", 0, 16, 0, UNI},
    {"VARI", 0, 100, 20, UNI},
    {"CLOCK", 0, TRIG_SOURCES - 1, 0, UNI},
  }
};

class RandomMod : public Modulator {
public:
  enum { P_RATE, P_AMT, P_SMTH, P_OFFS, P_LEN, P_VARI, P_CLOCK };
  RandomMod() : Modulator(RANDOM_DEF) {
    for (auto& s : steps) s = random(1000) * 0.001f;
  }
  void Tick(const ModCtx& ctx) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override { if (a == 0) Step(); }
private:
  float steps[16];
  uint8_t pos = 0;
  float target = 0.5f, smooth = 0.5f, t = 0.0f;
  float Hz() const { return RateHz(P_RATE, 0.05f, 30.0f); }
  void Step();
};

FLASHMEM void RandomMod::Step() {
  const int len = cur[P_LEN];
  if (len == 0) {
    target = random(1000) * 0.001f;
  } else {
    pos = (pos + 1) % len;
    if (random(100) < cur[P_VARI]) steps[pos] = random(1000) * 0.001f;
    target = steps[pos];
  }
  flash = 0.05f;
}

FLASHMEM void RandomMod::Tick(const ModCtx& ctx) {
  if (cur[P_CLOCK]) {
    if (Edge(cur[P_CLOCK], ctx, 0)) Step();
  } else {
    t += Hz() * ctx.dt;
    if (t >= 1.0f) { t -= floorf(t); Step(); }
  }
  // P_SMTH glides between steps
  const float s = cur[P_SMTH] * 0.01f;
  const float tau = 0.0005f + s * s * 1.0f;
  smooth += (target - smooth) * std::min(1.0f, ctx.dt / tau);
  out = 0.5f + (smooth - 0.5f) * cur[P_AMT] * 0.01f + cur[P_OFFS] * 0.01f;
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
  } else {
    Device::Format(p, buf, n);
  }
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
    {"AMT", 0, 100, 100, UNI},
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
  float Shaped(float e) const;  // CURVE and AMT applied
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

// CURVE bends all stages: + exponential, - logarithmic; AMT scales
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
    {"AMT", 0, 100, 100, UNI},
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
private:
  float env = 0.0f;
};

FLASHMEM void FollowMod::Tick(const ModCtx& ctx) {
  const float lvl = (cur[P_INPUT] ? ctx.out_level : ctx.in_level) * cur[P_GAIN] * 0.01f;
  const float tau = lvl > env ? Seconds(cur[P_RISE], 0.001f, 1.0f) : Seconds(cur[P_FALL], 0.005f, 3.0f);
  env += (lvl - env) * std::min(1.0f, ctx.dt / tau);
  out = env * cur[P_AMT] * 0.01f + cur[P_OFFS] * 0.01f;
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
private:
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
    if (TrigEdge(src - 1, ctx.edges, prev_gate[0])) {
      const uint32_t dt = now - last_edge_us;
      static const uint8_t ppqn[] = {1, 2, 4, 24};
      if (last_edge_us && dt > 2000 && dt < 4000000)
        Measured(60e6f / (dt * ppqn[constrain(cur[P_PPQN], 0, 3)]));
      last_edge_us = now;
    }
  }
  tempo_bpm = (src && have_measure) ? measured : float(cur[P_BPM]);

  // beat blink for the picture, and a 0..1 ramp per beat as output
  beat_ph += ctx.dt * tempo_bpm / 60.0f;
  if (beat_ph >= 1.0f) { beat_ph -= floorf(beat_ph); flash = 0.08f; }
  out = beat_ph;
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
  if (external() && !have_measure) gfxPrint(x + 4, y + 16, PSTR("waiting for clock"));
  // four beats, the current one filled
  for (int b = 0; b < 4; b++) {
    const int bx = x + w - 52 + b * 13;
    gfxFrame(bx, y + 4, 10, 10);
  }
  static uint8_t beat = 0;
  if (flash > 0.07f) beat = (beat + 1) % 4;
  gfxRect(x + w - 52 + beat * 13 + 2, y + 6, 6, 6);
}

FLASHMEM void ClockDevice::DrawMini(int x, int y, int w, int h) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", int(tempo_bpm + 0.5f));
  gfxPrint(x + 1, y + 2, buf);
  if (flash > 0.0f) gfxRect(x + w / 2 - 3, y + h / 2 + 2, 6, 6);
  else gfxFrame(x + w / 2 - 3, y + h / 2 + 2, 6, 6);
}

/////////////////////////////////////////////////////////////////
// MOD slots: type 0 = off
/////////////////////////////////////////////////////////////////
constexpr int NUM_MODS = 4;
constexpr int MOD_TYPES = 5;

PROGMEM static const char MOD_TYPE_TAGS[MOD_TYPES][4] = {"-", "WAV", "RND", "ENV", "FOL"};

FLASHMEM static Modulator* MakeModulator(int type) {
  switch (type) {
    case 1: return new WaveMod;
    case 2: return new RandomMod;
    case 3: return new AdsrMod;
    case 4: return new FollowMod;
    default: return nullptr;
  }
}

} // namespace sculpt
