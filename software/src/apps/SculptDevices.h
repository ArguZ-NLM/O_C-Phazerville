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

// Devices for the Sculpt app, modelled on the Torso S4 device chain.
//
// A device is a stereo audio stage with up to 8 parameters (2 pages of 4),
// up to 2 actions (UP / DOWN buttons) and a picture. Everything here runs
// from flash and keeps its tables in flash: RAM1 is full in T41_audio.
// Devices are created on first use and never deleted (Teensy audio streams
// can't be removed from the update list); a device that is not in a chain is
// disconnected, so it costs no CPU, and has released its buffers.

#pragma once

#include "../Audio/AudioDelayExt.h"
#include "../Audio/AudioEffectVast.h"

namespace sculpt {

enum ParamKind : uint8_t { UNI, BI, LIST };

struct ParamDef {
  char name[6];
  int16_t min, max, def;
  ParamKind kind;
  uint8_t nlabels;   // LIST values
  uint8_t nmodes;    // 0 = no modes
  uint8_t defmode;
  char labels[4][6];
  char modes[3][6];
};

struct DeviceDef {
  char name[8];
  char tag[4];       // 3 letters for the chain view
  uint8_t nparams;
  char actions[2][4];
  ParamDef params[8];
};

constexpr int MAX_PARAMS = 8;

class Device {
public:
  const DeviceDef& def;
  int16_t val[MAX_PARAMS] = {};
  uint8_t mode[MAX_PARAMS] = {};
  bool active = false;

  explicit Device(const DeviceDef& d) : def(d) {}

  const ParamDef& param(int p) const { return def.params[p]; }
  int num_params() const { return def.nparams; }

  // audio stage; nullptr means this device is not part of the audio chain
  virtual AudioStream* in() { return nullptr; }
  virtual AudioStream* out() { return nullptr; }
  // grab / free buffers and internal patch cords
  virtual bool Activate() { active = true; return true; }
  virtual void Deactivate() { active = false; }
  virtual bool ok() const { return true; }

  virtual void Apply(int p) = 0;
  virtual int Step(int p, int v, int dir) const { return v + dir; }
  virtual void Format(int p, char* buf, size_t n) const;
  virtual void Action(int a) {}
  virtual bool ActionLit(int a) const { return false; }
  virtual void DrawViz(int x, int y, int w, int h) {}
  virtual void DrawMini(int x, int y, int w, int h) {}

  void Defaults();
  void ApplyAll();
  void Nudge(int p, int dir);
  void NudgeMode(int p, int dir);
};

FLASHMEM void Device::Defaults() {
  for (int p = 0; p < num_params(); p++) {
    val[p] = param(p).def;
    mode[p] = param(p).defmode;
  }
}

FLASHMEM void Device::ApplyAll() {
  for (int p = 0; p < num_params(); p++) Apply(p);
}

FLASHMEM void Device::Nudge(int p, int dir) {
  const ParamDef& d = param(p);
  val[p] = constrain(Step(p, val[p], dir), d.min, d.max);
  Apply(p);
}

FLASHMEM void Device::NudgeMode(int p, int dir) {
  const ParamDef& d = param(p);
  if (!d.nmodes) return;
  mode[p] = constrain(mode[p] + dir, 0, d.nmodes - 1);
  Apply(p);
}

FLASHMEM void Device::Format(int p, char* buf, size_t n) const {
  const ParamDef& d = param(p);
  switch (d.kind) {
    case LIST: snprintf(buf, n, "%s", d.labels[constrain(val[p], 0, d.nlabels - 1)]); break;
    case BI:   snprintf(buf, n, val[p] > 0 ? "+%d" : "%d", val[p]); break;
    default:   snprintf(buf, n, "%d", val[p]); break;
  }
}

// a level that falls back slowly, for meters
struct Meter {
  float v = 0.0f;
  float Hit(float peak, float fall = 0.86f) {
    v = std::max(sqrtf(peak), v * fall);
    return v;
  }
};

FLASHMEM static void DrawVMeter(int x, int y, int w, int h, float lvl) {
  const int f = constrain(int(lvl * h), 0, h);
  if (f > 0) gfxRect(x, y + h - f, w, f);
}

FLASHMEM static void DrawHMeter(int x, int y, int w, int h, float lvl) {
  const int f = constrain(int(lvl * w), 0, w);
  if (f > 0) gfxRect(x, y, f, h);
}

/////////////////////////////////////////////////////////////////
// Track mixer: both tracks -> stereo out
/////////////////////////////////////////////////////////////////
class SculptMixer : public AudioStream {
public:
  SculptMixer() : AudioStream(4, queue) {}
  volatile float gain[2][2] = {{0.8f, 0.8f}, {0.8f, 0.8f}}; // [track][L/R]
  float peak[2][2] = {};  // held until read, [track][L/R]
  void update() override;
  void ReadPeaks(float out[2][2]) {
    AudioNoInterrupts();
    for (int t = 0; t < 2; t++)
      for (int c = 0; c < 2; c++) {
        out[t][c] = peak[t][c];
        peak[t][c] = 0.0f;
      }
    AudioInterrupts();
  }
private:
  audio_block_t* queue[4];
};

FLASHMEM void SculptMixer::update() {
  audio_block_t* in[4];
  for (int i = 0; i < 4; i++) in[i] = receiveReadOnly(i);
  audio_block_t* out[2] = {allocate(), allocate()};
  if (out[0] && out[1]) {
    float g[2][2];
    for (int t = 0; t < 2; t++)
      for (int c = 0; c < 2; c++) g[t][c] = gain[t][c];
    float pk[2][2] = {};
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      int32_t sum[2] = {0, 0};
      for (int t = 0; t < 2; t++) {
        for (int c = 0; c < 2; c++) {
          const audio_block_t* b = in[t * 2 + c];
          if (!b) continue;
          const int32_t s = static_cast<int32_t>(b->data[n] * g[t][c]);
          sum[c] += s;
          pk[t][c] = std::max(pk[t][c], fabsf(s * (1.0f / 32768.0f)));
        }
      }
      out[0]->data[n] = __SSAT(sum[0], 16);
      out[1]->data[n] = __SSAT(sum[1], 16);
    }
    for (int t = 0; t < 2; t++)
      for (int c = 0; c < 2; c++) peak[t][c] = std::max(peak[t][c], pk[t][c]);
    transmit(out[0], 0);
    transmit(out[1], 1);
  }
  for (auto b : out) if (b) release(b);
  for (auto b : in) if (b) release(b);
}

/////////////////////////////////////////////////////////////////
// MIX: level / pan / mute per track (not an audio stage itself)
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef MIX_DEF = {
  "MIX", "MIX", 3, {"", "MUT"},
  {
    {"LEVEL", 0, 100, 80, UNI},
    {"PAN", -50, 50, 0, BI},
    {"MUTE", 0, 1, 0, LIST, 2, 0, 0, {"off", "ON"}},
  }
};

class MixDevice : public Device {
public:
  MixDevice(SculptMixer& m, int t) : Device(MIX_DEF), mixer(m), track(t) {}
  void Apply(int p) override;
  void Action(int a) override;
  bool ActionLit(int a) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  float level[2] = {};  // set by the app from the mixer peaks
private:
  SculptMixer& mixer;
  int track;
  Meter meters[2];
};

FLASHMEM void MixDevice::Apply(int) {
  const float lvl = val[2] ? 0.0f : val[0] * 0.01f;
  const float pan = val[1] * 0.02f; // -1..1
  mixer.gain[track][0] = lvl * (pan > 0.0f ? 1.0f - pan : 1.0f);
  mixer.gain[track][1] = lvl * (pan < 0.0f ? 1.0f + pan : 1.0f);
}

FLASHMEM void MixDevice::Action(int a) {
  if (a == 1) { val[2] = !val[2]; Apply(2); }
}

FLASHMEM bool MixDevice::ActionLit(int a) const { return a == 1 && val[2]; }

FLASHMEM void MixDevice::DrawViz(int x, int y, int w, int h) {
  // two tall output meters with a pan marker underneath
  const float l = meters[0].Hit(level[0]), r = meters[1].Hit(level[1]);
  const int mw = 14, cx = x + w / 2;
  gfxFrame(cx - mw - 3, y, mw + 2, h - 4);
  gfxFrame(cx + 1, y, mw + 2, h - 4);
  DrawVMeter(cx - mw - 2, y + 1, mw, h - 6, l);
  DrawVMeter(cx + 2, y + 1, mw, h - 6, r);
  const int px = cx + val[1] * 40 / 50;
  gfxDottedLine(cx - 40, y + h - 2, cx + 40, y + h - 2, 2);
  gfxRect(px - 1, y + h - 3, 3, 3);
  gfxPrint(x + 2, y + 2, PSTR("L"));
  gfxPrint(x + w - 8, y + 2, PSTR("R"));
  if (val[2]) gfxPrint(cx - 12, y + h / 2 - 4, PSTR("MUTE"));
}

FLASHMEM void MixDevice::DrawMini(int x, int y, int w, int h) {
  if (val[2]) { gfxLine(x, y + h / 2, x + w - 1, y + h / 2); return; }
  const float l = meters[0].Hit(level[0]), r = meters[1].Hit(level[1]);
  DrawHMeter(x, y, w, h / 2, l);
  DrawHMeter(x, y + h / 2 + 1, w, h / 2, r);
}

/////////////////////////////////////////////////////////////////
// INPUT (Material): picks the track's source from the audio inputs
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef INPUT_DEF = {
  "INPUT", "IN", 3, {"", ""},
  {
    {"SOURC", 0, 3, 0, LIST, 4, 0, 0, {"STER", "IN1", "IN2", "OFF"}},
    {"GAIN", 0, 200, 100, UNI},
    {"WIDTH", 0, 200, 100, UNI},
  }
};

class InputDevice : public Device, public AudioStream {
public:
  enum Source { STEREO, IN1, IN2, OFF };
  InputDevice() : Device(INPUT_DEF), AudioStream(2, queue) {}
  AudioStream* in() override { return this; }
  AudioStream* out() override { return this; }
  void Apply(int p) override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void update() override;
private:
  audio_block_t* queue[2];
  volatile uint8_t source = STEREO;
  volatile float gain = 1.0f, width = 1.0f;
  float peak[2] = {};
  Meter meters[2];
  float wave[32] = {};   // rolling level history of the input
  uint8_t wave_pos = 0;
  void Poll();
};

FLASHMEM void InputDevice::Poll() {
  float pk[2];
  AudioNoInterrupts();
  pk[0] = peak[0]; pk[1] = peak[1];
  peak[0] = peak[1] = 0.0f;
  AudioInterrupts();
  meters[0].Hit(pk[0]);
  meters[1].Hit(pk[1]);
}

FLASHMEM void InputDevice::Apply(int p) {
  switch (p) {
    case 0: source = val[0]; break;
    case 1: gain = val[1] * 0.01f; break;
    case 2: width = val[2] * 0.01f; break;
  }
}

FLASHMEM void InputDevice::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (out[0] && out[1]) {
    const uint8_t src = source;
    const float g = gain, wd = width;
    float pk[2] = {};
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      const float a = in[0] ? in[0]->data[n] : 0.0f;
      const float b = in[1] ? in[1]->data[n] : 0.0f;
      float l, r;
      switch (src) {
        case IN1: l = r = a; break;
        case IN2: l = r = b; break;
        case OFF: l = r = 0.0f; break;
        default: {
          const float m = 0.5f * (a + b), s = 0.5f * (a - b) * wd;
          l = m + s;
          r = m - s;
        } break;
      }
      l *= g;
      r *= g;
      pk[0] = std::max(pk[0], fabsf(l));
      pk[1] = std::max(pk[1], fabsf(r));
      out[0]->data[n] = __SSAT(static_cast<int32_t>(l), 16);
      out[1]->data[n] = __SSAT(static_cast<int32_t>(r), 16);
    }
    for (int c = 0; c < 2; c++) peak[c] = std::max(peak[c], pk[c] * (1.0f / 32768.0f));
    transmit(out[0], 0);
    transmit(out[1], 1);
  }
  for (auto b : out) if (b) release(b);
  for (auto b : in) if (b) release(b);
}

FLASHMEM void InputDevice::DrawViz(int x, int y, int w, int h) {
  Poll();
  const float l = meters[0].v, r = meters[1].v;

  // scrolling level history, mirrored around the centre line like a waveform
  wave[wave_pos] = std::max(l, r);
  wave_pos = (wave_pos + 1) % 32;
  const int cy = y + h / 2, ww = w - 22;
  gfxDottedLine(x, cy, x + ww, cy, 3);
  for (int i = 0; i < 32; i++) {
    const float v = wave[(wave_pos + i) % 32];
    const int a = constrain(int(v * (h / 2 - 1)), 0, h / 2 - 1);
    const int xx = x + i * ww / 32;
    if (a) gfxLine(xx, cy - a, xx, cy + a);
  }
  // L / R meters at the right edge
  gfxFrame(x + w - 18, y, 8, h);
  gfxFrame(x + w - 9, y, 8, h);
  DrawVMeter(x + w - 17, y + 1, 6, h - 2, l);
  DrawVMeter(x + w - 8, y + 1, 6, h - 2, r);
}

FLASHMEM void InputDevice::DrawMini(int x, int y, int w, int h) {
  Poll();
  DrawHMeter(x, y, w, h / 2, meters[0].v);
  DrawHMeter(x, y + h / 2 + 1, w, h / 2, meters[1].v);
}

/////////////////////////////////////////////////////////////////
// VAST (Space): hybrid delay + reverb
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef VAST_DEF = {
  "VAST", "VST", 8, {"FRZ", "CLR"},
  {
    {"DELAY", 0, 100, 40, UNI},
    {"TIME", 20, 6000, 450, UNI},
    {"REVRB", 0, 100, 30, UNI},
    {"SIZE", 0, 100, 60, UNI},
    {"FDBK", 0, 100, 45, UNI},
    {"SPRD", 0, 100, 40, UNI, 0, 3, 1, {}, {"PHASE", "PING", "DIFF"}},
    {"TONE", -50, 50, -10, BI},
    {"DECAY", 0, 100, 50, UNI},
  }
};

class VastDevice : public Device {
public:
  enum { DELAY, TIME, REVERB, SIZE, FEEDBACK, SPREAD, TONE, DECAY };
  enum { PHASE, PING, DIFFUSE };
  using DelayStream = AudioDelayExt<9>; // same as the Delay applet: shared code
  static constexpr int TAPS = AudioEffectVast::TAPS;
  static constexpr float MAX_SECS = 6.0f;

  VastDevice();
  AudioStream* in() override { return &vast; }
  AudioStream* out() override { return &vast; }
  bool Activate() override;
  void Deactivate() override;
  bool ok() const override { return mem_ok; }
  void Apply(int p) override;
  int Step(int p, int v, int dir) const override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  bool ActionLit(int a) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;

private:
  // order matters: Vast updates before the delays it feeds
  AudioEffectVast vast;
  DelayStream delays[2];
  AudioConnection write[2];
  AudioConnection taps[2][TAPS];
  bool mem_ok = false;
  float tap_secs[2][TAPS] = {};
  Meter tap_m[2][TAPS], tail_m;

  static constexpr float TAP_POS[2][TAPS] = {
    {0.25f, 0.50f, 0.75f, 1.0f},
    {0.33f, 0.58f, 0.83f, 1.0f},
  };
  void SetTimes();
  void Poll();
};

constexpr size_t VAST_DELAY_SAMPLES
  = static_cast<size_t>(VastDevice::MAX_SECS * AUDIO_SAMPLE_RATE_EXACT) + 512;

FLASHMEM VastDevice::VastDevice()
  : Device(VAST_DEF)
  , delays{DelayStream(VAST_DELAY_SAMPLES), DelayStream(VAST_DELAY_SAMPLES)} {}

FLASHMEM bool VastDevice::Activate() {
  if (active) return mem_ok;
  AudioNoInterrupts();
  mem_ok = vast.Acquire();
  for (auto& d : delays) d.Acquire();
  mem_ok = mem_ok && delays[0].ready() && delays[1].ready();
  for (int c = 0; c < 2; c++) {
    delays[c].taps(TAPS);
    write[c].connect(vast, 2 + c, delays[c], 0);
    for (int t = 0; t < TAPS; t++) {
      taps[c][t].connect(delays[c], t, vast, 2 + c * TAPS + t);
    }
  }
  AudioInterrupts();
  ApplyAll();
  active = true;
  return mem_ok;
}

FLASHMEM void VastDevice::Deactivate() {
  if (!active) return;
  AudioNoInterrupts();
  for (int c = 0; c < 2; c++) {
    write[c].disconnect();
    for (auto& t : taps[c]) t.disconnect();
  }
  AudioInterrupts();
  // disconnected streams no longer update, so freeing is safe now
  for (auto& d : delays) d.Release();
  vast.Release();
  active = false;
}

FLASHMEM void VastDevice::SetTimes() {
  const float secs = val[TIME] * 0.001f;
  const float phase = mode[SPREAD] == PHASE ? 1.0f + 0.3f * val[SPREAD] * 0.01f : 1.0f;
  for (int c = 0; c < 2; c++) {
    for (int t = 0; t < TAPS; t++) {
      float s = secs * TAP_POS[c][t] * (c ? phase : 1.0f);
      s = std::min(s, MAX_SECS - 0.01f);
      tap_secs[c][t] = s;
      delays[c].delay(t, s - AudioEffectVast::LOOP_LATENCY_SECS);
    }
  }
}

FLASHMEM void VastDevice::Apply(int p) {
  const float v = val[p] * 0.01f;
  switch (p) {
    case DELAY:    vast.delay_mix(v); break;
    case TIME:     SetTimes(); break;
    case REVERB:   vast.reverb_mix(v); break;
    case SIZE:     vast.size(v); break;
    case FEEDBACK: vast.feedback(v); break;
    case SPREAD:
      vast.pingpong(mode[SPREAD] == PING ? v : 0.0f);
      vast.diffuse(mode[SPREAD] == DIFFUSE ? v : 0.0f);
      SetTimes();
      break;
    case TONE:     vast.tone(val[TONE] * 0.02f); break;
    case DECAY:    vast.decay(0.3f * powf(60.0f, v)); break; // 0.3 .. 18 s
  }
}

FLASHMEM int VastDevice::Step(int p, int v, int dir) const {
  if (p == TIME) return v + dir * std::max(1, v / 32); // ~3% per detent
  return v + dir;
}

FLASHMEM void VastDevice::Format(int p, char* buf, size_t n) const {
  if (p == TIME) {
    const int ms = val[TIME];
    if (ms < 1000) snprintf(buf, n, "%dms", ms);
    else snprintf(buf, n, "%d.%02ds", ms / 1000, (ms % 1000) / 10);
  } else if (p == DECAY) {
    const int ds = int(3.0f * powf(60.0f, val[DECAY] * 0.01f)); // tenths
    snprintf(buf, n, "%d.%ds", ds / 10, ds % 10);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void VastDevice::Action(int a) {
  if (a == 0) {
    vast.set_freeze(!vast.frozen());
  } else if (a == 1) {
    // CLEAR: empty the delay and reverb buffers
    AudioNoInterrupts();
    vast.Reset();
    AudioInterrupts();
    for (auto& d : delays) d.clear();
  }
}

FLASHMEM bool VastDevice::ActionLit(int a) const { return a == 0 && vast.frozen(); }

FLASHMEM void VastDevice::Poll() {
  AudioEffectVast::Viz v;
  vast.ReadViz(v);
  for (int c = 0; c < 2; c++)
    for (int t = 0; t < TAPS; t++) tap_m[c][t].Hit(v.tap[c][t]);
  tail_m.Hit(v.tail, 0.93f);
}

FLASHMEM void VastDevice::DrawViz(int x, int y, int w, int h) {
  Poll();
  if (!mem_ok) {
    gfxPrint(x + 22, y + h / 2 - 4, PSTR("out of memory"));
    return;
  }
  // echoes on a time line (L filled, R outlined), then the reverb tail as a
  // fading wedge whose length follows DECAY and height the tail level
  const int base = y + h - 1, tl_w = w * 2 / 3;
  float longest = 0.0f;
  for (auto& c : tap_secs) for (float s : c) longest = std::max(longest, s);
  gfxDottedLine(x, base, x + w - 1, base, 2);
  gfxLine(x, y + 4, x, base);  // the dry hit
  for (int c = 0; c < 2; c++) {
    for (int t = 0; t < TAPS; t++) {
      const float lvl = tap_m[c][t].v;
      const int bh = constrain(int(lvl * (h - 2)), 0, h - 2);
      const int bx = x + 3 + int(tap_secs[c][t] / longest * (tl_w - 6)) + c * 3 - 3;
      if (bh > 0) {
        if (c == 0) gfxRect(bx, base - bh, 2, bh);
        else gfxFrame(bx, base - bh, 2, bh);
      }
    }
  }
  const float tail = tail_m.v;
  const int th = constrain(int(tail * (h - 2)), 0, h - 2);
  const int tx = x + tl_w + 2;
  const int len = 12 + (w - tl_w - 14) * val[DECAY] / 100;
  for (int i = 0; i < len; i += 2) {
    const int hh = th * (len - i) / len;
    if (hh > 0) gfxLine(tx + i, base - hh, tx + i, base);
  }
  if (vast.frozen()) {
    gfxPrint(x + w - 25, y + 1, PSTR("FRZ"));
    gfxInvert(x + w - 26, y, 20, 9);
  }
}

FLASHMEM void VastDevice::DrawMini(int x, int y, int w, int h) {
  Poll();
  float m = 0.0f;
  for (auto& c : tap_m) for (auto& t : c) m = std::max(m, t.v);
  DrawHMeter(x, y, w, h / 2, m);
  DrawHMeter(x, y + h / 2 + 1, w, h / 2, tail_m.v);
  if (vast.frozen()) gfxInvert(x, y, w, h + 1);
}

/////////////////////////////////////////////////////////////////
// Slot catalogue, S4 order. Option 0 of the effect slots is BYPASS.
/////////////////////////////////////////////////////////////////
enum Slot : uint8_t { MATERIAL, GRANULAR, FILTER, COLOR, SPACE, SLOT_COUNT };
constexpr int MAX_OPTIONS = 2;

struct SlotDef {
  char name[9];
  char tag[4];
  uint8_t noptions;
};

PROGMEM static const SlotDef SLOT_DEFS[SLOT_COUNT] = {
  {"MATERIAL", "MAT", 1},  // INPUT (TAPE next)
  {"GRANULAR", "GRN", 1},  // BYPASS (MOSAIC to come)
  {"FILTER",   "FLT", 1},  // BYPASS (RING to come)
  {"COLOR",    "COL", 1},  // BYPASS (DEFORM to come)
  {"SPACE",    "SPC", 2},  // BYPASS, VAST
};

// nullptr = BYPASS. MATERIAL/INPUT is owned by the track, not made here.
FLASHMEM static Device* MakeDevice(int slot, int option) {
  if (slot == SPACE && option == 1) return new VastDevice;
  return nullptr;
}

} // namespace sculpt
