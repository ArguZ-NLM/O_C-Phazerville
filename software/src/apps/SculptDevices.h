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

#include "../Audio/AudioEffectVast.h"
#include "../Audio/AudioEffectDeform.h"
#include "../Audio/AudioEffectBandz.h"

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
  char modes[4][6];
};

struct DeviceDef {
  char name[8];
  char tag[4];       // 3 letters for the chain view
  uint8_t nparams;
  char actions[2][5];
  ParamDef params[8];
};

constexpr int MAX_PARAMS = 8;

// rising edges since the last modulator tick: bits 0-3 gate inputs,
// bits 4-11 CV inputs as gates. Set in the ISR, taken in the main loop.
volatile uint16_t trig_latch = 0;

struct ModCtx {
  float dt;           // seconds since the last tick
  uint16_t edges;     // latched rising edges, see trig_latch
  float in_level;     // the track's input level, 0..1
  float out_level;    // the track's output level, 0..1
};

// --- tempo ---
// set by the CLOCK device; synced times and rates follow it
float tempo_bpm = 120.0f;

// note values for synced times, in beats (quarter notes)
constexpr int NUM_DIVS = 9;
PROGMEM static const float DIV_BEATS[NUM_DIVS] = {0.125f, 0.25f, 0.5f, 1, 2, 4, 8, 16, 32};
PROGMEM static const char DIV_NAMES[NUM_DIVS][5] = {"1/32", "1/16", "1/8", "1/4", "1/2", "1", "2", "4", "8"};
enum SyncMode : uint8_t { SYNC_FREE, SYNC_STRAIGHT, SYNC_TRIPLET, SYNC_DOTTED };

// map a setting's value range onto the note values (so CV can pick them)
FLASHMEM static int DivIndex(int v, int lo, int hi) {
  return constrain((v - lo) * NUM_DIVS / (hi - lo + 1), 0, NUM_DIVS - 1);
}
FLASHMEM static int DivValue(int idx, int lo, int hi) {
  idx = constrain(idx, 0, NUM_DIVS - 1);
  return lo + ((hi - lo + 1) * (2 * idx + 1)) / (2 * NUM_DIVS);
}
FLASHMEM static float DivSeconds(int idx, uint8_t mode) {
  float beats = DIV_BEATS[constrain(idx, 0, NUM_DIVS - 1)];
  if (mode == SYNC_TRIPLET) beats *= 2.0f / 3.0f;
  if (mode == SYNC_DOTTED) beats *= 1.5f;
  return beats * 60.0f / tempo_bpm;
}
FLASHMEM static void FormatDiv(int idx, uint8_t mode, char* buf, size_t n) {
  snprintf(buf, n, "%s%s", DIV_NAMES[constrain(idx, 0, NUM_DIVS - 1)],
           mode == SYNC_TRIPLET ? "T" : mode == SYNC_DOTTED ? "D" : "");
}

// trigger source numbering: 0 off, 1-4 gate in, 5-12 CV in, 13+ MIDI map
constexpr int TRIG_SOURCES = 1 + 4 + 8 + MIDIMAP_MAX;

FLASHMEM static void FormatTrig(int src, char* buf, size_t n) {
  if (src <= 0) snprintf(buf, n, "-");
  else if (src <= 4) snprintf(buf, n, "TR%d", src);
  else if (src <= 12) snprintf(buf, n, "CV%d", src - 4);
  else snprintf(buf, n, "M%d", src - 12);
}

FLASHMEM static bool TrigGate(int src) {
  if (src <= 0) return false;
  if (src <= 12) return HS::frame.gate_high[src - 1];
  return HS::frame.MIDIState.mapping[src - 13].ViewOut() > GATE_THRESHOLD;
}

// hardware inputs use the latched edges; MIDI maps keep their own last state
FLASHMEM static bool TrigEdge(int src, uint16_t edges, bool& prev) {
  if (src <= 0) return false;
  if (src <= 12) return edges & (1 << (src - 1));
  const bool g = TrigGate(src);
  const bool e = g && !prev;
  prev = g;
  return e;
}

class Device {
public:
  const DeviceDef& def;
  int16_t val[MAX_PARAMS] = {};      // what the user set
  int16_t cur[MAX_PARAMS] = {};      // after modulation; Apply() uses this
  uint8_t mode[MAX_PARAMS] = {};
  int8_t mod_amt[MAX_PARAMS] = {};   // -100..100 % of the range, 0 = off
  // modulation source: the same picker the Quadrants applets use (CV inputs,
  // CV outputs, MIDI maps, noise)
  HS::CVInputMap mod_in[MAX_PARAMS];
  uint8_t mod_bus_sel[MAX_PARAMS] = {}; // 1..4 = the track's MOD A-D instead
  uint8_t act_trig[2] = {};              // trigger sources for the 2 actions
  bool act_prev[2] = {};
  const float* mod_bus = nullptr;        // the track's modulator outputs
  uint8_t mod_bi = 0;                // bit per param: bipolar around val
  bool active = false;
  uint8_t view_page = 0;  // which page of settings the device view shows

  explicit Device(const DeviceDef& d);

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
  // peak CPU % of this device's audio streams since the last call: parts[0]
  // is the device's own stream, parts[1] anything it owns (e.g. delay lines)
  virtual void CpuMax(float parts[2]) { parts[0] = parts[1] = 0.0f; }
  virtual void TempoChanged() {}

  void Defaults();
  void ApplyAll();
  void Nudge(int p, int dir);
  void NudgeMode(int p, int dir);
  bool Bipolar(int p) const { return mod_bi & (1 << p); }
  int Modulated(int p) const;
  void Refresh(int p);   // recompute cur[p] and apply it
  void Modulate();       // follow the CV inputs
  void RunTriggers(uint16_t edges);  // fire actions from their triggers
  void CopyFrom(const Device& d);
  void ChangeModSource(int p, int dir);
  void ModSourceName(int p, char* buf, size_t n) const;
  bool ModSourceOn(int p) const { return mod_bus_sel[p] || mod_in[p].enabled(); }
};

FLASHMEM Device::Device(const DeviceDef& d) : def(d) {}

FLASHMEM void Device::Defaults() {
  for (int p = 0; p < num_params(); p++) {
    val[p] = cur[p] = param(p).def;
    mode[p] = param(p).defmode;
    mod_amt[p] = 0;
    mod_in[p].SetInput(p); // setting n listens to CV n by default
  }
  mod_bi = 0;
}

FLASHMEM int Device::Modulated(int p) const {
  const ParamDef& d = param(p);
  if (!mod_amt[p] || !ModSourceOn(p)) return val[p];
  float c = (mod_bus_sel[p] && mod_bus) ? mod_bus[mod_bus_sel[p] - 1]
                                        : constrain(mod_in[p].InF(), -1.0f, 1.0f);
  if (Bipolar(p)) c = 2.0f * c - 1.0f;
  const int v = val[p] + int(mod_amt[p] * 0.01f * c * (d.max - d.min));
  return constrain(v, d.min, d.max);
}

FLASHMEM void Device::Refresh(int p) {
  cur[p] = Modulated(p);
  Apply(p);
}

FLASHMEM void Device::ApplyAll() {
  for (int p = 0; p < num_params(); p++) Refresh(p);
}

FLASHMEM void Device::Modulate() {
  for (int p = 0; p < num_params(); p++) {
    if (!mod_amt[p] && cur[p] == val[p]) continue;
    const int m = Modulated(p);
    if (m != cur[p]) {
      cur[p] = m;
      Apply(p);
    }
  }
}

FLASHMEM void Device::RunTriggers(uint16_t edges) {
  for (int a = 0; a < 2; a++) {
    if (act_trig[a] && TrigEdge(act_trig[a], edges, act_prev[a])) Action(a);
  }
}

FLASHMEM void Device::CopyFrom(const Device& d) {
  memcpy(val, d.val, sizeof(val));
  memcpy(mode, d.mode, sizeof(mode));
  memcpy(mod_amt, d.mod_amt, sizeof(mod_amt));
  for (int p = 0; p < MAX_PARAMS; p++) mod_in[p] = d.mod_in[p];
  memcpy(mod_bus_sel, d.mod_bus_sel, sizeof(mod_bus_sel));
  memcpy(act_trig, d.act_trig, sizeof(act_trig));
  mod_bi = d.mod_bi;
  ApplyAll();
}

// source list: off, MOD A-D, then the CVInputMap list (CV in, CV out, MIDI
// maps, noise)
FLASHMEM void Device::ChangeModSource(int p, int dir) {
  HS::CVInputMap& in = mod_in[p];
  uint8_t& bus = mod_bus_sel[p];
  if (bus) {
    const int b = bus + dir;
    if (b < 1) { bus = 0; in.Unpack(0); }            // -> off
    else if (b > 4) { bus = 0; in.SetInput(0); }     // -> CV 1
    else bus = b;
  } else if (!in.enabled()) {
    if (dir > 0) bus = 1;                            // off -> MOD A
  } else {
    in.ChangeSource(dir);
    if (!in.enabled()) bus = 4;                      // CV 1 -> MOD D
  }
  Refresh(p);
}

FLASHMEM void Device::ModSourceName(int p, char* buf, size_t n) const {
  if (mod_bus_sel[p]) snprintf(buf, n, "Md%c", 'A' + mod_bus_sel[p] - 1);
  else snprintf(buf, n, "%s", mod_in[p].InputName());
}

FLASHMEM void Device::Nudge(int p, int dir) {
  const ParamDef& d = param(p);
  val[p] = constrain(Step(p, val[p], dir), d.min, d.max);
  Refresh(p);
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
  volatile float level[2] = {}; // last block's peak per track, for FOLLOW
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
    int32_t acc[2][AUDIO_BLOCK_SAMPLES] = {};
    float blk[2] = {};
    for (int t = 0; t < 2; t++) {
      for (int c = 0; c < 2; c++) {
        const audio_block_t* b = in[t * 2 + c];
        if (!b) continue;
        const int32_t g = static_cast<int32_t>(gain[t][c] * 32767.0f); // Q15
        int32_t pk = 0;
        for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
          const int32_t v = (b->data[n] * g) >> 15;
          acc[c][n] += v;
          pk = std::max<int32_t>(pk, abs(v));
        }
        peak[t][c] = std::max(peak[t][c], pk * (1.0f / 32768.0f));
        blk[t] = std::max(blk[t], pk * (1.0f / 32768.0f));
      }
    }
    level[0] = blk[0];
    level[1] = blk[1];
    for (int c = 0; c < 2; c++)
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) out[c]->data[n] = __SSAT(acc[c][n], 16);
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
  "MIX", "MIX", 7, {"", "MUTE"},
  {
    {"LEVEL", 0, 100, 80, UNI},
    {"PAN", -50, 50, 0, BI},
    {"MUTE", 0, 1, 0, LIST, 2, 0, 0, {"off", "ON"}},
    {"BANDZ", 0, 100, 0, UNI},
    {"LOW", -24, 24, 0, BI, 0, 3, 1, {}, {"120", "250", "500"}},
    {"MID", -24, 24, 0, BI},
    {"HIGH", -24, 24, 0, BI, 0, 3, 1, {}, {"1.5k", "3k", "6k"}},
  }
};

// level / pan / mute and a 3-band tone + dynamics stage (after 3-Bandz).
// The track's audio runs through the band stage, then into the mixer.
class MixDevice : public Device {
public:
  enum { LEVEL, PAN, MUTE, BANDZ, B_LOW, B_MID, B_HIGH };
  explicit MixDevice(int t) : Device(MIX_DEF), track(t) {}
  AudioStream* in() override { return &bandz; }
  AudioStream* out() override { return &bandz; }
  void Apply(int p) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  bool ActionLit(int a) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = bandz.processorUsageMax();
    parts[1] = 0.0f;
    bandz.processorUsageMaxReset();
  }
  float level[2] = {};  // set by the app from the mixer peaks
  SculptMixer* mixer = nullptr;
private:
  AudioEffectBandz bandz;
  int track;
  Meter meters[2], band_m[3], gr_m[3];
  static constexpr int SPEC_BARS = 31; // 3 px each across the 3 left columns
  float spec[SPEC_BARS] = {};
  float* fft = nullptr; // 2 x CAPTURE floats, made when first drawn
  void DrawSpectrum(int x, int y, int w, int h);
};

// in-place radix-2 FFT, n a power of two
FLASHMEM static void Fft(float* re, float* im, int n) {
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { std::swap(re[i], re[j]); std::swap(im[i], im[j]); }
  }
  for (int len = 2; len <= n; len <<= 1) {
    const float a = -2.0f * float(M_PI) / len;
    const float wr = cosf(a), wi = sinf(a);
    for (int i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (int k = 0; k < len / 2; k++) {
        const int u = i + k, v = i + k + len / 2;
        const float tr = re[v] * cr - im[v] * ci, ti = re[v] * ci + im[v] * cr;
        re[v] = re[u] - tr; im[v] = im[u] - ti;
        re[u] += tr; im[u] += ti;
        const float nr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = nr;
      }
    }
  }
}

FLASHMEM void MixDevice::Apply(int p) {
  static const float lo_hz[3] = {120.0f, 250.0f, 500.0f};
  static const float hi_hz[3] = {1500.0f, 3000.0f, 6000.0f};
  switch (p) {
    case BANDZ: bandz.compress(cur[BANDZ] * 0.01f); return;
    case B_LOW:   bandz.band_gain(0, cur[B_LOW] * 0.5f); bandz.split_low(lo_hz[mode[B_LOW] % 3]); return;
    case B_MID:   bandz.band_gain(1, cur[B_MID] * 0.5f); return;
    case B_HIGH:  bandz.band_gain(2, cur[B_HIGH] * 0.5f); bandz.split_high(hi_hz[mode[B_HIGH] % 3]); return;
    default: break;
  }
  if (!mixer) return;
  const float lvl = cur[MUTE] ? 0.0f : cur[LEVEL] * 0.01f;
  const float pan = cur[PAN] * 0.02f; // -1..1
  mixer->gain[track][0] = lvl * (pan > 0.0f ? 1.0f - pan : 1.0f);
  mixer->gain[track][1] = lvl * (pan < 0.0f ? 1.0f + pan : 1.0f);
}

// band gains in 0.5 dB steps
FLASHMEM void MixDevice::Format(int p, char* buf, size_t n) const {
  if (p >= B_LOW) {
    const int v = val[p];
    snprintf(buf, n, "%s%d.%d", v < 0 ? "-" : v > 0 ? "+" : "", abs(v) / 2, abs(v) % 2 ? 5 : 0);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void MixDevice::Action(int a) {
  if (a == 1) { val[MUTE] = !val[MUTE]; Refresh(MUTE); }
}

FLASHMEM bool MixDevice::ActionLit(int a) const { return a == 1 && val[MUTE]; }

FLASHMEM void MixDevice::DrawViz(int x, int y, int w, int h) {
  AudioEffectBandz::Viz v;
  bandz.ReadViz(v);
  for (int b = 0; b < 3; b++) {
    band_m[b].Hit(v.level[b]);
    gr_m[b].Hit(std::min(1.0f, v.gr_db[b] / 18.0f), 0.9f);
  }
  const int cy = y + h / 2;
  // page 1: spectrum of the track's output; page 2: the three bands sit
  // above their LOW / MID / HIGH settings (columns of 32 px): gain as a bar
  // from the centre line, the band's level as a thin meter beside it,
  // compression hanging from the top
  if (view_page == 0) DrawSpectrum(x, y, 94, h);
  for (int b = 0; b < 3 && view_page == 1; b++) {
    const int bx = x + b * 32;
    const int gain = cur[B_LOW + b] * (h / 2 - 1) / 24;
    gfxDottedLine(bx + 3, cy, bx + 28, cy, 2);
    if (gain > 0) gfxRect(bx + 10, cy - gain, 8, gain);
    else if (gain < 0) gfxRect(bx + 10, cy, 8, -gain);
    else gfxLine(bx + 10, cy, bx + 17, cy);
    DrawVMeter(bx + 21, y, 2, h, band_m[b].v);
    const int gr = int(gr_m[b].v * h);
    if (gr > 0) gfxRect(bx + 25, y, 2, gr);
  }

  // fourth column: output meters with the pan position underneath
  const float l = meters[0].Hit(level[0]), r = meters[1].Hit(level[1]);
  const int ox = x + 96 + 6;
  gfxFrame(ox, y, 8, h - 4);
  gfxFrame(ox + 11, y, 8, h - 4);
  DrawVMeter(ox + 1, y + 1, 6, h - 6, l);
  DrawVMeter(ox + 12, y + 1, 6, h - 6, r);
  const int px = ox + 9 + cur[PAN] * 9 / 50;
  gfxDottedLine(ox, y + h - 2, ox + 19, y + h - 2, 2);
  gfxRect(px - 1, y + h - 3, 3, 3);
}

// 31 bars, log frequency 60 Hz .. 20 kHz, -60..0 dB, with the band splits
FLASHMEM void MixDevice::DrawSpectrum(int x, int y, int w, int h) {
  constexpr int N = AudioEffectBandz::CAPTURE;
  if (!fft) fft = new float[2 * N];
  if (!fft) return;
  float* re = fft;
  float* im = fft + N;
  int16_t samples[N];
  bandz.ReadCapture(samples);
  for (int i = 0; i < N; i++) {
    const float win = 0.5f - 0.5f * cosf(2.0f * float(M_PI) * i / (N - 1)); // Hann
    re[i] = samples[i] * (1.0f / 32768.0f) * win;
    im[i] = 0.0f;
  }
  Fft(re, im, N);

  constexpr float f0 = 60.0f, f1 = 20000.0f;
  const float bin_hz = AUDIO_SAMPLE_RATE_EXACT / N;
  const int bw = w / SPEC_BARS;
  for (int b = 0; b < SPEC_BARS; b++) {
    const float lo = f0 * powf(f1 / f0, float(b) / SPEC_BARS);
    const float hi = f0 * powf(f1 / f0, float(b + 1) / SPEC_BARS);
    int k0 = int(lo / bin_hz), k1 = int(hi / bin_hz);
    k0 = constrain(k0, 1, N / 2 - 1);
    k1 = constrain(std::max(k1, k0), 1, N / 2 - 1);
    float pk = 0.0f;
    for (int k = k0; k <= k1; k++) pk = std::max(pk, re[k] * re[k] + im[k] * im[k]);
    // magnitude in dB relative to a full-scale sine with the Hann window
    const float db = 10.0f * log10f(pk + 1e-12f) - 20.0f * log10f(N / 4.0f);
    const float v = constrain((db + 60.0f) / 60.0f, 0.0f, 1.0f);
    spec[b] = std::max(v, spec[b] - 0.04f);
    const int bh = int(spec[b] * h);
    if (bh > 0) gfxRect(x + b * bw, y + h - bh, bw - 1, bh);
  }
  // where the 3-band stage splits
  static const float lo_hz[3] = {120.0f, 250.0f, 500.0f};
  static const float hi_hz[3] = {1500.0f, 3000.0f, 6000.0f};
  for (float hz : {lo_hz[mode[B_LOW] % 3], hi_hz[mode[B_HIGH] % 3]}) {
    const int sx = x + int(log10f(hz / f0) / log10f(f1 / f0) * SPEC_BARS * bw);
    gfxDottedLine(sx, y, sx, y + h - 1, 3);
  }
}

FLASHMEM void MixDevice::DrawMini(int x, int y, int w, int h) {
  if (cur[MUTE]) { gfxLine(x, y + h / 2, x + w - 1, y + h / 2); return; }
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
  void CpuMax(float parts[2]) override {
    parts[0] = processorUsageMax();
    parts[1] = 0.0f;
    processorUsageMaxReset();
  }
  void update() override;
  volatile float level = 0.0f; // last block's peak, for FOLLOW
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
    case 0: source = cur[0]; break;
    case 1: gain = cur[1] * 0.01f; break;
    case 2: width = cur[2] * 0.01f; break;
  }
}

FLASHMEM void InputDevice::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (out[0] && out[1]) {
    static const int16_t silence[AUDIO_BLOCK_SAMPLES] = {};
    const int16_t* a = in[0] ? in[0]->data : silence;
    const int16_t* b = in[1] ? in[1]->data : silence;
    const uint8_t src = source;
    const int32_t g = static_cast<int32_t>(gain * 16384.0f);   // Q14, up to 2.0
    const int32_t wd = static_cast<int32_t>(width * 16384.0f); // Q14
    int16_t* l = out[0]->data;
    int16_t* r = out[1]->data;
    int32_t pk[2] = {0, 0};
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      int32_t x, y;
      switch (src) {
        case IN1: x = y = a[n]; break;
        case IN2: x = y = b[n]; break;
        case OFF: x = y = 0; break;
        default: {
          // mid/side width
          const int32_t m = (a[n] + b[n]) >> 1;
          const int32_t sd = (((a[n] - b[n]) >> 1) * wd) >> 14;
          x = m + sd;
          y = m - sd;
        } break;
      }
      l[n] = __SSAT((x * g) >> 14, 16);
      r[n] = __SSAT((y * g) >> 14, 16);
      pk[0] = std::max<int32_t>(pk[0], abs(l[n]));
      pk[1] = std::max<int32_t>(pk[1], abs(r[n]));
    }
    for (int c = 0; c < 2; c++) peak[c] = std::max(peak[c], pk[c] * (1.0f / 32768.0f));
    level = std::max(pk[0], pk[1]) * (1.0f / 32768.0f);
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
  "SPACE", "VST", 8, {"FRZ", "CLR"}, // the tag stays VST: saved files use it
  {
    {"DELAY", 0, 100, 40, UNI},
    {"TIME", 20, 6000, 450, UNI, 0, 4, 0, {}, {"FREE", "STR", "TRI", "DOT"}},
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
  void CpuMax(float parts[2]) override;
  void TempoChanged() override { if (mode[TIME] != SYNC_FREE) SetTimes(); }

private:
  AudioEffectVast vast;
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

FLASHMEM VastDevice::VastDevice() : Device(VAST_DEF), vast(VAST_DELAY_SAMPLES) {}

// called while the stream is not patched in, so it isn't updating
FLASHMEM bool VastDevice::Activate() {
  if (active) return mem_ok;
  mem_ok = vast.Acquire();
  ApplyAll();
  active = true;
  return mem_ok;
}

// called after the stream has been unpatched
FLASHMEM void VastDevice::Deactivate() {
  if (!active) return;
  vast.Release();
  active = false;
}

FLASHMEM void VastDevice::CpuMax(float parts[2]) {
  parts[0] = vast.processorUsageMax();
  parts[1] = 0.0f;
  vast.processorUsageMaxReset();
}

FLASHMEM void VastDevice::SetTimes() {
  const float secs = mode[TIME] == SYNC_FREE ? cur[TIME] * 0.001f
    : DivSeconds(DivIndex(cur[TIME], 20, 6000), mode[TIME]);
  const float phase = mode[SPREAD] == PHASE ? 1.0f + 0.3f * cur[SPREAD] * 0.01f : 1.0f;
  for (int c = 0; c < 2; c++) {
    for (int t = 0; t < TAPS; t++) {
      float s = secs * TAP_POS[c][t] * (c ? phase : 1.0f);
      s = std::min(s, MAX_SECS - 0.01f);
      tap_secs[c][t] = s;
      vast.tap(c, t, int(s * AUDIO_SAMPLE_RATE_EXACT));
    }
  }
}

FLASHMEM void VastDevice::Apply(int p) {
  const float v = cur[p] * 0.01f;
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
    case TONE:     vast.tone(cur[TONE] * 0.02f); break;
    case DECAY:    vast.decay(0.3f * powf(60.0f, v)); break; // 0.3 .. 18 s
  }
}

FLASHMEM int VastDevice::Step(int p, int v, int dir) const {
  if (p == TIME && mode[TIME] != SYNC_FREE)
    return DivValue(DivIndex(v, 20, 6000) + dir, 20, 6000);
  if (p == TIME) return v + dir * std::max(1, v / 32); // ~3% per detent
  return v + dir;
}

FLASHMEM void VastDevice::Format(int p, char* buf, size_t n) const {
  if (p == TIME && mode[TIME] != SYNC_FREE) {
    FormatDiv(DivIndex(val[TIME], 20, 6000), mode[TIME], buf, n);
  } else if (p == TIME) {
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
    if (!mem_ok) return;
    AudioNoInterrupts();
    vast.Reset();
    AudioInterrupts();
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
  const int len = 12 + (w - tl_w - 14) * cur[DECAY] / 100;
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
// DEFORM (Color): drive, compress, crush, tilt, envelope-following noise
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef DEFORM_DEF = {
  "DEFORM", "DEF", 8, {"", ""},
  {
    {"DRIVE", 0, 100, 30, UNI},
    {"FOLD", 0, 100, 0, UNI},
    {"CRSH", 0, 100, 0, UNI, 0, 3, 0, {}, {"BIT", "REDUX", "BOTH"}},
    {"TILT", -50, 50, 0, BI, 0, 3, 1, {}, {"LOW", "MID", "HIGH"}},
    {"NOIS", 0, 100, 0, UNI, 0, 3, 1, {}, {"WHITE", "PINK", "DUST"}},
    {"N.DCY", 0, 100, 40, UNI},
    {"N.TON", 0, 100, 50, UNI},
    {"WET", 0, 100, 100, UNI},
  }
};

class DeformDevice : public Device {
public:
  enum { DRIVE, FOLD, CRUSH, TILT, NOISE, NDECAY, NTONE, WET };
  DeformDevice() : Device(DEFORM_DEF) {}
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  void Apply(int p) override;
  void Format(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
private:
  AudioEffectDeform fx;
  Meter in_m, out_m, noise_m;
  void Poll();
  float NoiseDecaySecs() const { return 0.01f * powf(200.0f, cur[NDECAY] * 0.01f); } // 10 ms .. 2 s
};

FLASHMEM void DeformDevice::Apply(int p) {
  const float v = cur[p] * 0.01f;
  switch (p) {
    case DRIVE:  fx.drive(v); break;
    case FOLD:   fx.fold(v); break;
    case CRUSH:  fx.crush(v, mode[CRUSH]); break;
    case TILT:   fx.tilt(cur[TILT] * 0.02f, mode[TILT]); break;
    case NOISE:  fx.noise(v, mode[NOISE]); break;
    case NDECAY: fx.noise_decay(NoiseDecaySecs()); break;
    case NTONE:  fx.noise_tone(v); break;
    case WET:    fx.wet(v); break;
  }
}

FLASHMEM void DeformDevice::Format(int p, char* buf, size_t n) const {
  if (p == NDECAY) {
    const float s = 0.01f * powf(200.0f, val[NDECAY] * 0.01f);
    if (s < 1.0f) snprintf(buf, n, "%dms", int(s * 1000.0f + 0.5f));
    else snprintf(buf, n, "%d.%ds", int(s), int(s * 10.0f) % 10);
  } else if (p == NTONE) {
    const int hz = int(100.0f * powf(100.0f, val[NTONE] * 0.01f));
    if (hz < 1000) snprintf(buf, n, "%dHz", hz);
    else snprintf(buf, n, "%dk%d", hz / 1000, (hz % 1000) / 100);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void DeformDevice::Poll() {
  AudioEffectDeform::Viz v;
  fx.ReadViz(v);
  in_m.Hit(v.in);
  out_m.Hit(v.out);
  noise_m.Hit(v.noise, 0.8f);
}

FLASHMEM void DeformDevice::DrawViz(int x, int y, int w, int h) {
  Poll();
  // transfer curve: how FOLD and DRIVE bend the signal, with CRUSH's steps
  const int bs = h;
  gfxFrame(x, y, bs, bs);
  gfxDottedLine(x, y + bs / 2, x + bs - 1, y + bs / 2, 3);
  const float drv = cur[DRIVE] * 0.01f, crs = cur[CRUSH] * 0.01f, fld = cur[FOLD] * 0.01f;
  const bool bits = crs > 0.0f && mode[CRUSH] != AudioEffectDeform::CRUSH_REDUX;
  // the picture exaggerates the steps so they are visible at 30 px
  const int levels = bits ? std::max(2, 1 << std::min(5, AudioEffectDeform::CrushBits(crs) - 11)) : 0;
  int py = -1;
  for (int i = 1; i < bs - 1; i++) {
    const float xin = -1.0f + 2.0f * (i - 1) / (bs - 3);
    float yo = AudioEffectDeform::Shape(AudioEffectDeform::Fold(xin, fld), drv);
    if (levels) yo = roundf(yo * levels) / levels;
    const int yy = y + bs / 2 - int(yo * (bs / 2 - 2));
    if (py >= 0) gfxLine(x + i - 1, py, x + i, yy);
    py = yy;
  }

  // tilt as a frequency response, 20 Hz .. 20 kHz on a log scale: it turns
  // around the crossover (dot + frequency underneath), +-12 dB at the edges
  static const float xover_hz[3] = {80.0f, 650.0f, 2500.0f};
  static const char* const xover_name[3] = {"80", "650", "2k5"};
  const int tx = x + bs + 4, tw = 44, ch = h - 9, cy = y + ch / 2;
  const float t = cur[TILT] * 0.02f;
  const float gl = powf(10.0f, -12.0f * t / 20.0f), gh = powf(10.0f, 12.0f * t / 20.0f);
  const float pivot_hz = xover_hz[mode[TILT] % 3];
  const float fc = std::min(pivot_hz * gh, 18000.0f); // same corner as the effect
  const float px_per_db = (ch / 2 - 1) / 12.0f;
  gfxDottedLine(tx, cy, tx + tw - 1, cy, 3);
  int ly = -1;
  for (int i = 0; i < tw; i++) {
    const float f = 20.0f * powf(1000.0f, float(i) / (tw - 1));
    const float r = f / fc;
    const float mag = sqrtf((gl * gl + gh * gh * r * r) / (1.0f + r * r));
    const int yy = cy - int(20.0f * log10f(mag) * px_per_db);
    if (ly >= 0) gfxLine(tx + i - 1, ly, tx + i, yy);
    ly = yy;
  }
  // the pivot sits on the 0 dB line at the crossover
  const int pivot = tx + int(log10f(pivot_hz / 20.0f) / 3.0f * (tw - 1));
  gfxRect(pivot - 1, cy - 1, 3, 3);
  gfxLine(pivot, cy + 2, pivot, y + ch);
  const char* lbl = xover_name[mode[TILT] % 3];
  const int lx = constrain(pivot - 3 * int(strlen(lbl)), tx, tx + tw - 6 * int(strlen(lbl)));
  gfxPrint(lx, y + ch + 1, lbl);

  // noise: a box between the tilt curve and the meters, filled with grains
  // that look like the noise type; denser with more noise
  const int gx = x + w - 14;
  const int nx = tx + tw + 4, nw = gx - 3 - nx, nh = h - 9;
  gfxFrame(nx - 1, y, nw + 2, h, true);
  const float lvl = noise_m.v;
  switch (mode[NOISE]) {
    case AudioEffectDeform::NOISE_WHITE: {  // fine, even
      const int dots = int(lvl * 120.0f);
      for (int i = 0; i < dots; i++) gfxPixel(nx + random(nw), y + 1 + random(nh));
    } break;
    case AudioEffectDeform::NOISE_PINK: {   // chunkier, heavier at the bottom
      const int dots = int(lvl * 45.0f);
      for (int i = 0; i < dots; i++) {
        const int r = random(nh - 1);
        const int yy = y + 1 + (nh - 2) - (r * r) / (nh - 1); // more near the bottom
        gfxRect(nx + random(nw - 1), yy, 2, 2);
      }
    } break;
    default: {                              // dust: few big specks, in bursts
      const int specks = lvl > 0.02f && random(100) < int(lvl * 160.0f) ? 1 + random(3) : 0;
      for (int i = 0; i < specks; i++) {
        const int sx = nx + random(nw - 4), sy = y + 1 + random(nh - 3);
        if (random(2)) gfxRect(sx, sy, 3, 3);
        else gfxLine(sx, sy + 1, sx + 4, sy + 1);
      }
    } break;
  }
  static const char* const noise_name[3] = {"WHT", "PNK", "DST"};
  gfxPrint(nx + (nw - 18) / 2, y + h - 8, noise_name[mode[NOISE] % 3]);

  // input and output level
  gfxFrame(gx, y, 6, h);
  DrawVMeter(gx + 1, y + 1, 4, h - 2, in_m.v);
  gfxFrame(gx + 7, y, 6, h);
  DrawVMeter(gx + 8, y + 1, 4, h - 2, out_m.v);
}

FLASHMEM void DeformDevice::DrawMini(int x, int y, int w, int h) {
  Poll();
  DrawHMeter(x, y, w, h / 2, in_m.v);
  DrawHMeter(x, y + h / 2 + 1, w, h / 2, out_m.v);
}

/////////////////////////////////////////////////////////////////
// Slot catalogue, S4 order. Option 0 of the effect slots is BYPASS.
/////////////////////////////////////////////////////////////////
enum Slot : uint8_t { MATERIAL, GRANULAR, FILTER, COLOR, SPACE, SLOT_COUNT };
constexpr int MAX_OPTIONS = 2;

struct SlotDef {
  char name[9];
  char tag[4];
  uint8_t noptions;  // BYPASS + devices that exist now (MATERIAL: no bypass)
  uint8_t planned;   // devices this slot will offer (dots on the chain view)
};

PROGMEM static const SlotDef SLOT_DEFS[SLOT_COUNT] = {
  {"MATERIAL", "MAT", 1, 3},  // INPUT (TAPE and DISC to come)
  {"GRANULAR", "GRN", 1, 1},  // BYPASS (MOSAIC to come)
  {"FILTER",   "FLT", 1, 1},  // BYPASS (RING to come)
  {"COLOR",    "COL", 2, 1},  // BYPASS, DEFORM
  {"SPACE",    "SPC", 2, 1},  // BYPASS, SPACE
};

// nullptr = BYPASS. MATERIAL/INPUT is owned by the track, not made here.
FLASHMEM static Device* MakeDevice(int slot, int option) {
  if (slot == COLOR && option == 1) return new DeformDevice;
  if (slot == SPACE && option == 1) return new VastDevice;
  return nullptr;
}

} // namespace sculpt
