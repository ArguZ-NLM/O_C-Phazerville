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

// Devices for the Loom app.
//
// A device is a stereo audio stage with up to 8 parameters (2 pages of 4),
// up to 2 actions (UP / DOWN buttons) and a picture. The audio updates run
// from RAM1 (ITCM); the UI code runs from flash.
// Devices are created on first use and never deleted (Teensy audio streams
// can't be removed from the update list); a device that is not in a chain is
// disconnected, so it costs no CPU, and has released its buffers.

#pragma once

#include "../Audio/AudioEffectEcho.h"
#include "../Audio/AudioEffectDirt.h"
#include "../Audio/AudioEffectBandz.h"
#include "../Audio/AudioEffectResonator.h"
#include "../Audio/RingsVoice.h"
#include "../Audio/StretchVoice.h"
#include "../Audio/AudioEffectClouds.h"
#include "../Audio/AudioEffectReel.h"
#include "../Audio/AudioPlayStream.h"
#include <SD.h>

extern "C" uint8_t external_psram_size;  // MB of PSRAM found at start (Teensy core)

namespace loom {
// the longest SPOOL loop / PLAYER sample held in memory. They take only
// what they hold (SPOOL: what is free when REC starts, then just the loop),
// so on 8 MB of PSRAM they share what ECHO and SPRAY leave
inline float BigSecs() { return 16.0f; }

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
  char name[10];
  char tag[4];       // 3 letters for the chain view
  uint8_t nparams;
  char actions[2][5];
  ParamDef params[8];
  char page_title[2][6]; // shown after the name in the header, per page
};

constexpr int MAX_PARAMS = 8;
// modulator outputs a setting can pick: the track's MOD A-D, then the
// shared MOD E-H (the same for both tracks)
constexpr int MOD_BUSES = 8;

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
// where the clock is in its beat (0..1) and when that was (micros), for
// starting on the next beat
float clock_beat_ph = 0.0f;
uint32_t clock_beat_us = 0;
// Z starts and stops the clock (so does MIDI start/stop when the clock
// follows MIDI): synced modulators hold while stopped and begin again from
// the top on start
bool clock_running = true;
uint8_t clock_starts = 0;  // counts starts; each modulator notices a new one
inline void ClockStart() { clock_running = true; clock_starts++; }
inline void ClockStop() { clock_running = false; }

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
  if (src <= 0) snprintf(buf, n, "none");
  else if (src <= 4) snprintf(buf, n, "TR%d", src);
  else if (src <= 12) snprintf(buf, n, "CV%d", src - 4);
  else snprintf(buf, n, "MIDI%d", src - 12);
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
  int16_t last_delta[MAX_PARAMS] = {};  // the last change by hand (LINKED moves the twin by it)
  int16_t cur[MAX_PARAMS] = {};      // after modulation; Apply() uses this
  uint8_t mode[MAX_PARAMS] = {};
  int8_t mod_amt[MAX_PARAMS] = {};   // -100..100 % of the range, 0 = off
  // modulation source: the same picker the Quadrants applets use (CV inputs,
  // CV outputs, MIDI maps, noise)
  HS::CVInputMap mod_in[MAX_PARAMS];
  uint8_t mod_bus_sel[MAX_PARAMS] = {}; // 1..8 = MOD A-H instead
  // a second source on top of the first, so modulations can be mixed (e.g.
  // 40 % of MOD A plus 20 % of MOD G); only the modulators
  int8_t mod_amt2[MAX_PARAMS] = {};
  uint8_t mod_bus2[MAX_PARAMS] = {};    // 0 = off, 1..8 = MOD A-H
  uint8_t act_trig[2] = {};              // trigger sources for the 2 actions
  bool act_prev[2] = {};
  const float* mod_bus = nullptr;        // the track's modulator outputs
  uint8_t mod_bi = 0;                // bit per param: bipolar around val
  uint8_t mod_bi2 = 0;               // the same for the second source
  bool active = false;
  float wet = 1.0f;       // the device's share in its slot's fade (SPRAY: LEVEL)
  uint8_t view_page = 0;  // which page of settings the device view shows
  int8_t view_cursor = 0; // the setting under the cursor there

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
  // a setting's mode as text; modes past the 4 named ones are shown by the
  // device (e.g. a number)
  virtual void FormatMode(int p, char* buf, size_t n) const;
  // the name shown above a setting (some name themselves after their mode)
  virtual const char* Name(int p) const { return param(p).name; }
  virtual void Action(int a) {}
  virtual bool ActionLit(int a) const { return false; }
  virtual void DrawViz(int x, int y, int w, int h) {}
  virtual void DrawMini(int x, int y, int w, int h) {}
  virtual void Idle() {}  // main loop, while the device is in use
  // peak CPU % of this device's audio streams since the last call: parts[0]
  // is the device's own stream, parts[1] anything it owns (e.g. delay lines)
  virtual void CpuMax(float parts[2]) { parts[0] = parts[1] = 0.0f; }
  virtual void TempoChanged() {}
  // bypassed devices are unpatched (no CPU) once this long has passed, so
  // a tail can ring out first; Parked() runs then, with the stream idle
  virtual uint32_t TailMs() const;
  virtual void Parked() {}
  // parked (unpatched) or back in the chain: a device whose audio object
  // would otherwise keep computing on silence stops while parked
  virtual void Rest(bool on) {}
  // resting at WET 0 (nothing of it heard) like a bypassed device; only
  // where waking up costs nothing audible (no recorded buffer to go stale)
  virtual bool RestsWhenDry() const { return false; }
  // keeps its input while parked (it records, Rest() stops the rest): its
  // buffer has no gap of old audio when it comes back
  virtual bool FeedsWhileParked() const { return false; }

  void Defaults();
  void ApplyAll();
  // R held on the device page: back to the defaults (a device keeps what
  // is routing rather than sound: its file, input, the track's pan)
  virtual void Reset() { Defaults(); ApplyAll(); }
  // what a reset changes, kept so L held can undo it
  struct Saved {
    int16_t val[MAX_PARAMS];
    uint8_t mode[MAX_PARAMS];
    int8_t mod_amt[MAX_PARAMS], mod_amt2[MAX_PARAMS];
    HS::CVInputMap mod_in[MAX_PARAMS];
    uint8_t mod_bus_sel[MAX_PARAMS], mod_bus2[MAX_PARAMS];
    uint8_t mod_bi, mod_bi2;
    uint32_t ex[2];  // PLAYER: its exact edges
    bool ex_valid;
  };
  void Keep(Saved& k) const;
  void Restore(const Saved& k);
  virtual void KeepMore(Saved&) const {}
  // a short note for the page's header bar (false: none)
  virtual bool HeaderInfo(char*, size_t) const { return false; }
  virtual void RestoreMore(const Saved&) {}
  void Nudge(int p, int dir);
  void NudgeMode(int p, int dir);
  bool Bipolar(int p) const { return mod_bi & (1 << p); }
  bool Bipolar2(int p) const { return mod_bi2 & (1 << p); }
  bool Modded(int p) const { return mod_amt[p] || mod_amt2[p]; }
  // a source with an amount that really moves the setting (not off, not an
  // empty MOD slot)
  bool ModLive(int p, int k) const {
    if (k) return mod_amt2[p] && mod_bus2[p] && mod_bus && mod_bus[mod_bus2[p] - 1] >= 0.0f;
    return mod_amt[p] && ModSourceOn(p)
           && !(mod_bus_sel[p] && mod_bus && mod_bus[mod_bus_sel[p] - 1] < 0.0f);
  }
  bool ModsLive(int p) const { return ModLive(p, 0) || ModLive(p, 1); }
  // the lowest and highest value the live sources together can give
  void ModRange(int p, int& lo, int& hi) const {
    float a = 0.0f, b = 0.0f;  // percent
    for (int k = 0; k < 2; k++) {
      if (!ModLive(p, k)) continue;
      const int amt = k ? mod_amt2[p] : mod_amt[p];
      if (k ? Bipolar2(p) : Bipolar(p)) { a -= abs(amt); b += abs(amt); }
      else if (amt > 0) b += amt;
      else a += amt;
    }
    const ParamDef& d = param(p);
    lo = constrain(val[p] + int(a * 0.01f * (d.max - d.min)), d.min, d.max);
    hi = constrain(val[p] + int(b * 0.01f * (d.max - d.min)), d.min, d.max);
  }
  int Modulated(int p) const;
  void Refresh(int p);   // recompute cur[p] and apply it
  void Modulate();       // follow the CV inputs
  void RunTriggers(uint16_t edges);  // fire actions from their triggers
  void CopyFrom(const Device& d);
  void ChangeModSource(int p, int dir);
  void ChangeModSource2(int p, int dir);
  void ModSourceName(int p, char* buf, size_t n) const;
  bool ModSourceOn(int p) const { return mod_bus_sel[p] || mod_in[p].enabled(); }
};

FLASHMEM Device::Device(const DeviceDef& d) : def(d) {}

FLASHMEM void Device::Defaults() {
  for (int p = 0; p < num_params(); p++) {
    val[p] = cur[p] = param(p).def;
    mode[p] = param(p).defmode;
    mod_amt[p] = mod_amt2[p] = 0;
    mod_bus2[p] = 0;
    mod_in[p].SetInput(p); // setting n listens to CV n by default
  }
  mod_bi = mod_bi2 = 0;
}

FLASHMEM int Device::Modulated(int p) const {
  const ParamDef& d = param(p);
  // a MOD slot with no modulator in it (-1) counts as no source
  const bool one = ModLive(p, 0), two = ModLive(p, 1);
  if (!one && !two) return val[p];
  // the two sources add up, each by its own amount
  float sum = 0.0f;
  if (one) {
    float c = (mod_bus_sel[p] && mod_bus) ? mod_bus[mod_bus_sel[p] - 1]
                                          : constrain(mod_in[p].InF(), -1.0f, 1.0f);
    if (Bipolar(p)) c = 2.0f * c - 1.0f;
    sum += mod_amt[p] * c;
  }
  if (two) {
    float c = mod_bus[mod_bus2[p] - 1];
    if (Bipolar2(p)) c = 2.0f * c - 1.0f;
    sum += mod_amt2[p] * c;
  }
  const int v = val[p] + int(sum * 0.01f * (d.max - d.min));
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
    if (!Modded(p) && cur[p] == val[p]) continue;
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
  memcpy(mod_amt2, d.mod_amt2, sizeof(mod_amt2));
  memcpy(mod_bus2, d.mod_bus2, sizeof(mod_bus2));
  mod_bi2 = d.mod_bi2;
  memcpy(act_trig, d.act_trig, sizeof(act_trig));
  mod_bi = d.mod_bi;
  ApplyAll();
}

FLASHMEM void Device::Keep(Saved& k) const {
  memcpy(k.val, val, sizeof(val));
  memcpy(k.mode, mode, sizeof(mode));
  memcpy(k.mod_amt, mod_amt, sizeof(mod_amt));
  memcpy(k.mod_amt2, mod_amt2, sizeof(mod_amt2));
  for (int p = 0; p < MAX_PARAMS; p++) k.mod_in[p] = mod_in[p];
  memcpy(k.mod_bus_sel, mod_bus_sel, sizeof(mod_bus_sel));
  memcpy(k.mod_bus2, mod_bus2, sizeof(mod_bus2));
  k.mod_bi = mod_bi;
  k.mod_bi2 = mod_bi2;
  KeepMore(k);
}

FLASHMEM void Device::Restore(const Saved& k) {
  memcpy(val, k.val, sizeof(val));
  memcpy(mode, k.mode, sizeof(mode));
  memcpy(mod_amt, k.mod_amt, sizeof(mod_amt));
  memcpy(mod_amt2, k.mod_amt2, sizeof(mod_amt2));
  for (int p = 0; p < MAX_PARAMS; p++) mod_in[p] = k.mod_in[p];
  memcpy(mod_bus_sel, k.mod_bus_sel, sizeof(mod_bus_sel));
  memcpy(mod_bus2, k.mod_bus2, sizeof(mod_bus2));
  mod_bi = k.mod_bi;
  mod_bi2 = k.mod_bi2;
  RestoreMore(k);
  ApplyAll();
}

// source list: off, MOD A-H, then the CVInputMap list (CV in, CV out, MIDI
// maps, noise)
FLASHMEM void Device::ChangeModSource(int p, int dir) {
  HS::CVInputMap& in = mod_in[p];
  uint8_t& bus = mod_bus_sel[p];
  if (bus) {
    const int b = bus + dir;
    if (b < 1) { bus = 0; in.Unpack(0); }            // -> off
    else if (b > MOD_BUSES) { bus = 0; in.SetInput(0); }  // -> CV 1
    else bus = b;
  } else if (!in.enabled()) {
    if (dir > 0) bus = 1;                            // off -> MOD A
  } else {
    in.ChangeSource(dir);
    if (!in.enabled()) bus = MOD_BUSES;              // CV 1 -> MOD H
  }
  Refresh(p);
}

// the second source: off, MOD A-H
FLASHMEM void Device::ChangeModSource2(int p, int dir) {
  mod_bus2[p] = constrain(mod_bus2[p] + dir, 0, MOD_BUSES);
  Refresh(p);
}

FLASHMEM void Device::ModSourceName(int p, char* buf, size_t n) const {
  if (mod_bus_sel[p]) snprintf(buf, n, "Md%c", 'A' + mod_bus_sel[p] - 1);
  else snprintf(buf, n, "%s", mod_in[p].InputName());
}

FLASHMEM void Device::Nudge(int p, int dir) {
  const ParamDef& d = param(p);
  const int16_t was = val[p];
  val[p] = constrain(Step(p, val[p], dir), d.min, d.max);
  last_delta[p] = val[p] - was;  // for a LINKED twin
  Refresh(p);
}

FLASHMEM void Device::NudgeMode(int p, int dir) {
  const ParamDef& d = param(p);
  if (!d.nmodes) return;
  mode[p] = constrain(mode[p] + dir, 0, d.nmodes - 1);
  Apply(p);
}

FLASHMEM void Device::FormatMode(int p, char* buf, size_t n) const {
  const int m = mode[p];
  if (m < 4) snprintf(buf, n, "%s", param(p).modes[m]);
  else snprintf(buf, n, "%d", m);
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
class LoomMixer : public AudioStream {
public:
  LoomMixer() : AudioStream(4, queue) {}
  volatile float gain[2][2] = {{0.8f, 0.8f}, {0.8f, 0.8f}}; // [track][L/R]
  volatile float level[2] = {}; // last block's peak per track, for FOLLOW
  // STEREO link: a track's two channels summed into one side of the output
  // (0 both sides as usual, 1 all to the left, 2 all to the right)
  volatile uint8_t route[2] = {0, 0};
  float peak[2][2] = {};  // held until read, [track][L/R]
  volatile bool mute = false;   // the whole output, fading over ~15 ms
  float mute_g = 1.0f;
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

FASTRUN void LoomMixer::update() {
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
        const uint8_t rt = route[t];
        // routed: both channels at half into the one side
        const int to = rt ? rt - 1 : c;
        const int sh = rt ? 16 : 15;
        int32_t pk = 0;
        for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
          const int32_t v = (b->data[n] * g) >> sh;
          acc[to][n] += v;
          pk = std::max<int32_t>(pk, abs(v));
        }
        peak[t][c] = std::max(peak[t][c], pk * (1.0f / 32768.0f));
        blk[t] = std::max(blk[t], pk * (1.0f / 32768.0f));
      }
    }
    level[0] = blk[0];
    level[1] = blk[1];
    if (mute || mute_g < 1.0f) {  // fade the sum out / back in
      const float to = mute ? 0.0f : 1.0f, step = 1.0f / (0.015f * AUDIO_SAMPLE_RATE_EXACT);
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        mute_g = to > mute_g ? std::min(to, mute_g + step) : std::max(to, mute_g - step);
        acc[0][n] = int32_t(acc[0][n] * mute_g);
        acc[1][n] = int32_t(acc[1][n] * mute_g);
      }
    }
    for (int c = 0; c < 2; c++)
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) out[c]->data[n] = __SSAT(acc[c][n], 16);
    transmit(out[0], 0);
    transmit(out[1], 1);
  }
  for (auto b : out) if (b) release(b);
  for (auto b : in) if (b) release(b);
}

/////////////////////////////////////////////////////////////////
// Bypass fades. LoomFade sits after every effect slot and mixes the
// slot's input (dry) with the device's output (wet); LoomGate sits in
// front of a device whose tail rings out when bypassed (ECHO). Both glide
// to their targets in ~15 ms and pass blocks through untouched once there.
/////////////////////////////////////////////////////////////////
constexpr float FADE_STEP = 1.0f / (0.015f * AUDIO_SAMPLE_RATE_EXACT);

static inline float Glide(float g, float to) {
  return to > g ? std::min(to, g + FADE_STEP) : std::max(to, g - FADE_STEP);
}

class LoomFade : public AudioStream {
public:
  LoomFade() : AudioStream(4, queue) {}  // dry L/R, wet L/R
  void Set(float dry, float wet) { to_dry = dry; to_wet = wet; }
  void update() override;
private:
  audio_block_t* queue[4];
  volatile float to_dry = 0.0f, to_wet = 1.0f;
  float g_dry = 0.0f, g_wet = 1.0f;
};

FASTRUN void LoomFade::update() {
  audio_block_t* in[4];
  for (int i = 0; i < 4; i++) in[i] = receiveReadOnly(i);
  const float td = to_dry, tw = to_wet;
  const bool settled = g_dry == td && g_wet == tw;
  if (settled && (td == 0.0f || tw == 0.0f) && (td + tw) == 1.0f) {
    // all wet or all dry: pass that side on
    const int from = td == 0.0f ? 2 : 0;
    for (int c = 0; c < 2; c++) if (in[from + c]) transmit(in[from + c], c);
  } else {
    audio_block_t* out[2] = {allocate(), allocate()};
    if (out[0] && out[1]) {
      constexpr float q = 1.0f / 32768.0f;
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        g_dry = Glide(g_dry, td);
        g_wet = Glide(g_wet, tw);
        for (int c = 0; c < 2; c++) {
          const float d = in[c] ? in[c]->data[n] * q : 0.0f;
          const float w = in[2 + c] ? in[2 + c]->data[n] * q : 0.0f;
          out[c]->data[n] = __SSAT(int32_t((d * g_dry + w * g_wet) * 32767.0f), 16);
        }
      }
      transmit(out[0], 0);
      transmit(out[1], 1);
    }
    for (auto b : out) if (b) release(b);
  }
  for (auto b : in) if (b) release(b);
}

class LoomGate : public AudioStream {
public:
  LoomGate() : AudioStream(2, queue) {}
  void Set(float g) { to = g; }
  void update() override;
private:
  audio_block_t* queue[2];
  volatile float to = 1.0f;
  float g = 1.0f;
};

FASTRUN void LoomGate::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  const float t = to;
  if (g == t && (t == 0.0f || t == 1.0f)) {
    // open: pass on; closed: send nothing (silence)
    if (t == 1.0f) for (int c = 0; c < 2; c++) if (in[c]) transmit(in[c], c);
  } else {
    audio_block_t* out[2] = {allocate(), allocate()};
    if (out[0] && out[1]) {
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        g = Glide(g, t);
        for (int c = 0; c < 2; c++)
          out[c]->data[n] = in[c] ? int16_t(in[c]->data[n] * g) : 0;
      }
      transmit(out[0], 0);
      transmit(out[1], 1);
    }
    for (auto b : out) if (b) release(b);
  }
  for (auto b : in) if (b) release(b);
}

/////////////////////////////////////////////////////////////////
// MIX: level / pan / mute per track (not an audio stage itself)
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef MIX_DEF = {
  "OUTPUT", "MIX", 8, {"", ""},  // the tag stays MIX: saved files use it
  {
    // the chain against the track's source as it came in (0 = only the
    // source, 100 = only the chain): all the effects dialled back at once
    {"MIX", 0, 100, 100, UNI},
    {"LEVEL", 0, 100, 80, UNI},
    {"PAN", -50, 50, 0, BI},
    {"MUTE", 0, 1, 0, LIST, 2, 0, 0, {"off", "ON"}},
    {"LOW", -24, 24, 0, BI, 0, 3, 1, {}, {"120", "250", "500"}},
    {"MID", -24, 24, 0, BI},
    {"HIGH", -24, 24, 0, BI, 0, 3, 1, {}, {"1.5k", "3k", "6k"}},
    {"COMP", 0, 100, 0, UNI},  // one-knob 3-band compression (after 3-Bandz; was OTT)
  }
};

// level / pan / mute and a 3-band tone + dynamics stage (after 3-Bandz).
// The track's audio runs through the band stage, then into the mixer.
class MixDevice : public Device {
public:
  enum { DRYWET, LEVEL, PAN, MUTE, B_LOW, B_MID, B_HIGH, BANDZ };
  LoomFade* out_fade = nullptr;  // source (dry) against the chain (wet), before the band stage
  explicit MixDevice(int t) : Device(MIX_DEF), track(t) {}
  void Reset() override {
    const int16_t pan = val[PAN];
    Defaults();
    val[PAN] = cur[PAN] = pan;
    ApplyAll();
  }
  AudioStream* in() override { return &bandz; }
  AudioStream* out() override { return &bandz; }
  void Apply(int p) override;
  void Format(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  bool Muted() const { return cur[MUTE]; }
  void SetBypass(bool b) { bandz.set_bypass(b); }  // tone + dynamics stage
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = bandz.processorUsageMax();
    parts[1] = 0.0f;
    bandz.processorUsageMaxReset();
  }
  float level[2] = {};  // set by the app from the mixer peaks
  LoomMixer* mixer = nullptr;
private:
  AudioEffectBandz bandz;
  int track;
  Meter meters[2], band_m[3], gr_m[3];
  static constexpr int SPEC_BARS = 31; // 3 px each across the 3 left columns
  float spec[SPEC_BARS] = {};
  float* fft = nullptr; // 4 x 512 floats for the two FFTs, made when first drawn
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
    case DRYWET:
      if (out_fade) {  // equal power; exact at the ends (the fade then just passes one side on)
        const float m = cur[DRYWET] * 0.01f * 1.5707963f;
        if (cur[DRYWET] >= 100) out_fade->Set(0.0f, 1.0f);
        else if (cur[DRYWET] <= 0) out_fade->Set(1.0f, 0.0f);
        else out_fade->Set(cosf(m), sinf(m));
      }
      return;
    case BANDZ: bandz.compress(cur[BANDZ] * 0.01f); return;
    case B_LOW:   bandz.band_gain(0, cur[B_LOW] * 0.5f); bandz.split_low(lo_hz[mode[B_LOW] % 3]); return;
    case B_MID:   bandz.band_gain(1, cur[B_MID] * 0.5f); return;
    case B_HIGH:  bandz.band_gain(2, cur[B_HIGH] * 0.5f); bandz.split_high(hi_hz[mode[B_HIGH] % 3]); return;
    default: break;
  }
  if (!mixer) return;
  const float lvl = cur[MUTE] ? 0.0f : cur[LEVEL] * 0.01f;
  const float pan = mixer->route[track] ? 0.0f : cur[PAN] * 0.02f; // -1..1 (STEREO link: no pan, the side is fixed)
  mixer->gain[track][0] = lvl * (pan > 0.0f ? 1.0f - pan : 1.0f);
  mixer->gain[track][1] = lvl * (pan < 0.0f ? 1.0f + pan : 1.0f);
}

// band gains in 0.5 dB steps
FLASHMEM void MixDevice::Format(int p, char* buf, size_t n) const {
  if (p >= B_LOW && p <= B_HIGH) {
    const int v = val[p];
    snprintf(buf, n, "%s%d.%d", v < 0 ? "-" : v > 0 ? "+" : "", abs(v) / 2, abs(v) % 2 ? 5 : 0);
  } else {
    Device::Format(p, buf, n);
  }
}

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
    const int bx = x + b * 32;  // above LOW / MID / HIGH
    const int gain = cur[B_LOW + b] * (h / 2 - 1) / 24;
    gfxDottedLine(bx + 3, cy, bx + 28, cy, 2);
    if (gain > 0) gfxRect(bx + 10, cy - gain, 8, gain);
    else if (gain < 0) gfxRect(bx + 10, cy, 8, -gain);
    else gfxLine(bx + 10, cy, bx + 17, cy);
    DrawVMeter(bx + 21, y, 2, h, band_m[b].v);
    const int gr = int(gr_m[b].v * h);
    if (gr > 0) gfxRect(bx + 25, y, 2, gr);
  }

  // output meters with the pan position underneath, on the right (page 2:
  // above COMP)
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

// 31 bars, log frequency 60 Hz .. 20 kHz, -60..0 dB, with the band splits.
// Two 512-point FFTs: the newest 512 samples at full rate (94 Hz bins) for
// the top, and all 2048 samples taken down to 12 kHz (23 Hz bins) for the
// bars below 1 kHz. A bar narrower than a bin reads between two bins. The
// display tilts +3 dB/octave so music reads roughly level.
FLASHMEM void MixDevice::DrawSpectrum(int x, int y, int w, int h) {
  constexpr int N = 512, CAP = AudioEffectBandz::CAPTURE, DECIM = CAP / N;
  if (!fft) fft = new float[4 * N];
  if (!fft) return;
  float* re = fft;          // full rate
  float* im = fft + N;
  float* dre = fft + 2 * N; // decimated
  float* dim = fft + 3 * N;
  int16_t* samples = new int16_t[CAP];
  if (!samples) return;
  bandz.ReadCapture(samples);
  for (int i = 0; i < N; i++) {
    const float win = 0.5f - 0.5f * cosf(2.0f * float(M_PI) * i / (N - 1)); // Hann
    re[i] = samples[CAP - N + i] * (1.0f / 32768.0f) * win;
    im[i] = 0.0f;
    // average DECIM samples: a simple low-pass before taking every DECIM-th
    int32_t acc = 0;
    for (int k = 0; k < DECIM; k++) acc += samples[i * DECIM + k];
    dre[i] = acc * (1.0f / (32768.0f * DECIM)) * win;
    dim[i] = 0.0f;
  }
  delete[] samples;
  Fft(re, im, N);
  Fft(dre, dim, N);

  constexpr float f0 = 60.0f, f1 = 20000.0f, split_hz = 1000.0f;
  const int bw = w / SPEC_BARS;
  const float ref_db = 20.0f * log10f(N / 4.0f); // full-scale sine, Hann
  for (int b = 0; b < SPEC_BARS; b++) {
    const float lo = f0 * powf(f1 / f0, float(b) / SPEC_BARS);
    const float hi = f0 * powf(f1 / f0, float(b + 1) / SPEC_BARS);
    const bool low = hi <= split_hz;
    const float* r = low ? dre : re;
    const float* i2 = low ? dim : im;
    const float bin_hz = AUDIO_SAMPLE_RATE_EXACT / (low ? DECIM : 1) / N;
    float pk;
    if (hi - lo < bin_hz) {
      // narrower than a bin: interpolate at the bar's centre
      const float pos = constrain(sqrtf(lo * hi) / bin_hz, 1.0f, N / 2 - 2.0f);
      const int k = int(pos);
      const float t = pos - k;
      const float m0 = r[k] * r[k] + i2[k] * i2[k];
      const float m1 = r[k + 1] * r[k + 1] + i2[k + 1] * i2[k + 1];
      pk = m0 + t * (m1 - m0);
    } else {
      const int k0 = constrain(int(lo / bin_hz + 0.5f), 1, N / 2 - 1);
      const int k1 = constrain(int(hi / bin_hz), k0, N / 2 - 1);
      pk = 0.0f;
      for (int k = k0; k <= k1; k++) pk = std::max(pk, r[k] * r[k] + i2[k] * i2[k]);
    }
    const float tilt = 3.0f * log2f(sqrtf(lo * hi) / 1000.0f); // 0 dB at 1 kHz
    const float db = 10.0f * log10f(pk + 1e-12f) - ref_db + tilt;
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
  // standing L and R bars: where the track sits in the stereo mix
  const float l = meters[0].Hit(level[0]), r = meters[1].Hit(level[1]);
  const int bw = (w - 3) / 2;
  DrawVMeter(x, y, bw, h, l);
  DrawVMeter(x + w - bw, y, bw, h, r);
}

/////////////////////////////////////////////////////////////////
// INPUT (Source): picks the track's source from the audio inputs
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef INPUT_DEF = {
  "INPUT", "IN", 4, {"", ""},
  {
    {"SOURC", 0, 4, 0, UNI},  // L+R, L, R, OFF, and on track B: OUT A (see InputSourceName)
    {"GAIN", 0, 200, 100, UNI},
    {"WIDTH", 0, 200, 100, UNI},
    {"VIEW", 0, 8, 2, UNI},   // the picture only: how much time it shows (see VIEW_MS)
  }
};

// INPUT's sources: the jacks, or (track B only) track A's output, so both
// chains run one after the other
static const char* InputSourceName(int s) {
  static const char* const names[5] = {"L+R", "L", "R", "OFF", "OUT A"};
  return names[constrain(s, 0, 4)];
}

// INPUT's VIEW steps, in ms
PROGMEM static const uint16_t VIEW_MS[9] = {2, 5, 10, 20, 50, 100, 200, 500, 1000};

class InputDevice : public Device, public AudioStream {
  uint32_t heard_at = 0;  // last time the mini picture saw a signal
public:
  // inputs 0-1: the jacks; 2-3: track A's output (track B's INPUT only)
  enum Source { STEREO, IN1, IN2, OFF, OUT_A };
  InputDevice() : Device(INPUT_DEF), AudioStream(4, queue) {}
  void Reset() override {
    const int16_t src = val[0];
    Defaults();
    val[0] = cur[0] = src;
    ApplyAll();
  }
  bool chained = false;  // track B: may take OUT A
  int Step(int p, int v, int dir) const override {
    const int r = v + dir;
    return p == 0 && !chained ? std::min(r, int(OFF)) : r;
  }
  void Format(int p, char* buf, size_t n) const override {
    if (p == 0) snprintf(buf, n, "%s", InputSourceName(val[0]));
    else if (p == 3 && VIEW_MS[constrain(val[3], 0, 8)] >= 1000) snprintf(buf, n, "1s");
    else if (p == 3) snprintf(buf, n, "%dms", VIEW_MS[constrain(val[3], 0, 8)]);
    else Device::Format(p, buf, n);
  }
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
  audio_block_t* queue[4];
  volatile uint8_t source = STEREO;
  volatile float gain = 1.0f, width = 1.0f;
  float peak[2] = {};
  Meter meters[2];
  // the picture: a stretch of the input taken from a rising edge (like a
  // scope's trigger), so a steady sound stands still; with no edge for a
  // while it takes one anyway
  // Each column keeps the lowest and highest point of its stretch (and
  // starts from where the last one ended, so the line stays joined).
  static constexpr int SCOPE_HYST = 160; // the edge must cross -this .. +this
  int16_t scope_lo[128] = {}, scope_hi[128] = {};
  volatile int16_t scope_col = -1;      // recording into this column; -1: waiting for an edge
  volatile bool scope_ready = false;    // a full picture waits for the screen
  volatile uint16_t scope_ms = 10;      // VIEW
  volatile uint8_t scope_cols = 104;    // the picture's width (set by the screen)
  uint16_t scope_k = 0, scope_spc = 1;  // samples in this column so far / per column
  uint8_t scope_ncol = 104;             // columns in this picture
  int16_t scope_last = 0;
  bool scope_low = false;
  uint8_t scope_wait = 0;               // blocks without an edge
  int8_t col_lo[128] = {}, col_hi[128] = {};
  void ScopeStart(int16_t v);
  float scope_range = 1024.0f;          // auto range: follows the loudest point, slowly back down
  void Poll();
  void Scope(const int16_t* l, const int16_t* r);
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
    case 3: scope_ms = VIEW_MS[constrain(cur[3], 0, 8)]; break;
  }
}

FASTRUN void InputDevice::update() {
  audio_block_t* in[4];
  for (int i = 0; i < 4; i++) in[i] = receiveReadOnly(i);
  audio_block_t* out[2] = {allocate(), allocate()};
  if (out[0] && out[1]) {
    static const int16_t silence[AUDIO_BLOCK_SAMPLES] = {};
    uint8_t src = source;
    // OUT A: track A's output, taken like the stereo jacks
    const bool from_a = src == OUT_A;
    if (from_a) src = STEREO;
    const int base = from_a ? 2 : 0;
    // channel 0 is L (the input is resynced at start, see AudioIO)
    const int16_t* a = in[base] ? in[base]->data : silence;
    const int16_t* b = in[base + 1] ? in[base + 1]->data : silence;
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
      const int32_t gg = g;
      l[n] = __SSAT((x * gg) >> 14, 16);
      r[n] = __SSAT((y * gg) >> 14, 16);
      pk[0] = std::max<int32_t>(pk[0], abs(l[n]));
      pk[1] = std::max<int32_t>(pk[1], abs(r[n]));
    }
    for (int c = 0; c < 2; c++) peak[c] = std::max(peak[c], pk[c] * (1.0f / 32768.0f));
    level = std::max(pk[0], pk[1]) * (1.0f / 32768.0f);
    Scope(l, r);
    transmit(out[0], 0);
    transmit(out[1], 1);
  }
  for (auto b : out) if (b) release(b);
  for (auto b : in) if (b) release(b);
}

FASTRUN void InputDevice::ScopeStart(int16_t v) {
  const int total = std::max(1, int(scope_ms * (AUDIO_SAMPLE_RATE_EXACT / 1000.0f)));
  scope_ncol = std::min<int>(scope_cols, total);
  scope_spc = total / scope_ncol;
  scope_col = 0;
  scope_k = 0;
  scope_last = v;
  scope_wait = 0;
}

FASTRUN void InputDevice::Scope(const int16_t* l, const int16_t* r) {
  if (scope_ready) return;  // the screen hasn't taken the last one yet
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const int16_t v = (l[n] + r[n]) >> 1;
    if (scope_col >= 0) {
      const int c = scope_col;
      if (scope_k == 0) scope_lo[c] = scope_hi[c] = scope_last;
      scope_lo[c] = std::min(scope_lo[c], v);
      scope_hi[c] = std::max(scope_hi[c], v);
      scope_last = v;
      if (++scope_k >= scope_spc) {
        scope_k = 0;
        if (c + 1 >= scope_ncol) { scope_col = -1; scope_ready = true; return; }
        scope_col = c + 1;
      }
    } else if (v < -SCOPE_HYST) {
      scope_low = true;
    } else if (v > SCOPE_HYST && scope_low) {  // a rising edge
      scope_low = false;
      ScopeStart(v);
    }
  }
  if (scope_col < 0 && ++scope_wait > 30) ScopeStart(0);  // ~90 ms without an edge
}

FLASHMEM void InputDevice::DrawViz(int x, int y, int w, int h) {
  Poll();
  const float l = meters[0].v, r = meters[1].v;

  const int cy = y + h / 2, ww = std::min(w - 22, 128);
  const int half = h / 2 - 1;
  // a new picture: the lowest and highest point in each column (each one
  // reaching back to the column before, so the line stays joined)
  scope_cols = ww;
  if (scope_ready) {
    const int nc = scope_ncol;
    int pk = 0;
    for (int k = 0; k < nc; k++) pk = std::max({pk, abs(int(scope_lo[k])), abs(int(scope_hi[k]))});
    scope_range = std::max({float(pk), scope_range * 0.97f, 1024.0f});
    const int range = int(scope_range) + 1;
    // fewer columns than pixels (a very short VIEW): each one drawn wider
    for (int i = 0; i < ww; i++) {
      const int k = i * nc / ww;
      col_lo[i] = constrain(scope_lo[k] * half / range, -half, half);
      col_hi[i] = constrain(scope_hi[k] * half / range, -half, half);
    }
    scope_ready = false;  // armed again
  }
  gfxDottedLine(x, cy, x + ww, cy, 3);
  for (int i = 0; i < ww; i++)
    if (col_lo[i] || col_hi[i]) gfxLine(x + i, cy - col_hi[i], x + i, cy - col_lo[i]);
  // meters at the right edge: L and R for the stereo input, one for a
  // single input, none when off
  if (cur[0] == STEREO) {
    gfxFrame(x + w - 18, y, 8, h);
    gfxFrame(x + w - 9, y, 8, h);
    DrawVMeter(x + w - 17, y + 1, 6, h - 2, l);
    DrawVMeter(x + w - 8, y + 1, 6, h - 2, r);
  } else if (cur[0] != OFF) {
    gfxFrame(x + w - 18, y, 17, h);
    DrawVMeter(x + w - 17, y + 1, 15, h - 2, std::max(l, r));
  }
}

// standing bars on the side the signal comes from: both for L+R, the left
// one for L, the right one for R, none when off
FLASHMEM void InputDevice::DrawMini(int x, int y, int w, int h) {
  Poll();
  // nothing coming in for 2 s (or OFF): a microphone, like REC's circle /
  // PLY's arrow; sound brings the bars straight back
  if (std::max(meters[0].v, meters[1].v) >= 0.02f) heard_at = millis();
  if (cur[0] == OFF || millis() - heard_at > 2000) {
    const int cx = x + w / 2;
    gfxFrame(cx - 2, y, 5, 6);                // capsule
    gfxPixel(cx - 2, y); gfxPixel(cx + 2, y);  // (round its top)
    graphics.clearRect(cx - 2, y, 1, 1);
    graphics.clearRect(cx + 2, y, 1, 1);
    gfxLine(cx - 4, y + 3, cx - 4, y + 5);    // holder
    gfxLine(cx + 4, y + 3, cx + 4, y + 5);
    gfxLine(cx - 3, y + 6, cx + 3, y + 6);
    gfxLine(cx, y + 7, cx, y + 8);            // stand
    gfxLine(cx - 2, y + 9, cx + 2, y + 9);
    return;
  }
  const int bw = (w - 3) / 2;
  if (cur[0] == STEREO || cur[0] == IN1) DrawVMeter(x, y, bw, h, cur[0] == STEREO ? meters[0].v : std::max(meters[0].v, meters[1].v));
  if (cur[0] == STEREO || cur[0] == IN2) DrawVMeter(x + w - bw, y, bw, h, cur[0] == STEREO ? meters[1].v : std::max(meters[0].v, meters[1].v));
}

/////////////////////////////////////////////////////////////////
// SPOOL (Source): a looper in front of the chain. It records what comes in
// through the track's input; IN is the input's source, so it is set here too.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef SPOOL_DEF = {
  "SPOOL", "REL", 8, {"REC", "STP"},  // tag REL kept for saved files (was REEL)
  {
    {"IN", 0, 4, 1, UNI},  // like INPUT's SOURC
    {"SPEED", -200, 200, 100, BI},
    {"START", 0, 100, 0, UNI},
    {"LEN", 1, 100, 100, UNI},
    {"DUB", 0, 100, 80, UNI},  // how much of the old loop stays when overdubbing
    {"MON", 0, 100, 100, UNI}, // the live input passed through
    {"LEVEL", 0, 100, 100, UNI},
    {"VIEW", 0, 9, 0, UNI},  // the picture only: ALL, or a window round the play head (REEL_VIEW_MS)
  },
  {"", ""},
};

// SPOOL's VIEW steps after ALL, in ms
PROGMEM static const uint16_t REEL_VIEW_MS[9] = {2000, 1000, 500, 200, 100, 50, 20, 10, 5};

class ReelDevice : public Device {
public:
  enum { IN, SPEED, START, LEN, DUB, MON, LEVEL, VIEW };
  static float MaxSecs() { return BigSecs(); }
  explicit ReelDevice(InputDevice* i) : Device(SPOOL_DEF), input(i) {}
  void Reset() override {
    const int16_t in = val[IN];
    Defaults();
    val[IN] = cur[IN] = in;
    ApplyAll();
  }
  int Step(int p, int v, int dir) const override {  // OUT A only on track B
    return p == IN && input ? input->Step(0, v, dir) : v + dir;
  }
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  bool Activate() override;
  void Deactivate() override;
  bool ok() const override { return fx.ready(); }
  void Apply(int p) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  bool ActionLit(int a) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
  void ClearLoop() { fx.Clear(); cleared_at = millis() | 1; hold = 0.0f; }  // B held for 3 s
  void Idle() override;
  size_t HeldFrames() const { return fx.Capacity(); }
  uint32_t no_mem_at = 0;   // REC found no memory free (shown for a moment)
  // saving the loop to the SD card (the app writes it, A held for 2 s)
  const int16_t* LoopData(int c) const { return fx.Data(c); }
  size_t LoopLength() const {
    const auto st = fx.state();
    return st == AudioEffectReel::EMPTY || st == AudioEffectReel::REC ? 0 : fx.Length();
  }
  float save_hold = 0.0f;   // how far A has been held towards saving, 0..1
  float saving = -1.0f;     // writing: progress 0..1, else < 0
  char saved_name[16] = {}; // the file it went to ...
  uint32_t saved_at = 0;    // ... and when
  float hold = 0.0f;        // how far B has been held towards clearing, 0..1 (set by the app)
  uint32_t cleared_at = 0;  // when it was cleared, for the picture
private:
  AudioEffectReel fx;
  InputDevice* input;
  float view_range = 1024.0f;  // zoomed picture's auto range
  void DrawZoom(int x, int y, int w, int h);
};

// called while the stream is not patched in
FLASHMEM bool ReelDevice::Activate() {
  if (active) return fx.ready();
  fx.Begin();  // a small buffer: the big one comes with REC (see Action)
  // IN is the input's own choice: take it over rather than impose ours
  if (input) val[IN] = cur[IN] = input->val[0];
  ApplyAll();
  active = true;
  return fx.ready();
}

FLASHMEM void ReelDevice::Deactivate() {
  if (!active) return;
  fx.Release();
  active = false;
}

FLASHMEM void ReelDevice::Apply(int p) {
  const float v = cur[p] * 0.01f;
  switch (p) {
    case IN:
      if (input) { input->val[0] = cur[IN]; input->Refresh(0); }
      break;
    case SPEED: fx.SetSpeed(v); break;
    case START: case LEN: fx.SetWindow(cur[START] * 0.01f, cur[LEN] * 0.01f); break;
    case DUB:   fx.SetDub(v); break;
    case MON:   fx.SetMonitor(v); break;
    case LEVEL: fx.SetLevel(v); break;
  }
}

FLASHMEM void ReelDevice::Format(int p, char* buf, size_t n) const {
  if (p == SPEED || p == START || p == LEN) snprintf(buf, n, "%d%%", val[p]);
  else if (p == IN) snprintf(buf, n, "%s", InputSourceName(val[p]));
  else if (p == VIEW && val[p] <= 0) snprintf(buf, n, "ALL");
  else if (p == VIEW) {
    const int ms = REEL_VIEW_MS[constrain(val[p] - 1, 0, 8)];
    if (ms >= 1000) snprintf(buf, n, "%ds", ms / 1000);
    else snprintf(buf, n, "%dms", ms);
  }
  else Device::Format(p, buf, n);
}

// A: REC -> PLAY -> DUB -> PLAY (stopped: play from the start); B: stop
// (held for 3 s: clear, see the app)
FLASHMEM void ReelDevice::Action(int a) {
  if (a == 0) {
    // the first REC: as much memory as is free, up to MaxSecs
    if (fx.state() == AudioEffectReel::EMPTY && !fx.Grow(MaxSecs())) {
      no_mem_at = millis() | 1;
      return;
    }
    fx.Press();
  } else {
    fx.Stop();
  }
}

// main loop: a finished recording keeps just its length; cleared, the
// buffer goes back to the small one
FLASHMEM void ReelDevice::Idle() {
  if (!active) return;
  fx.Shrink();
  fx.Park();
}

FLASHMEM bool ReelDevice::ActionLit(int a) const {
  const auto st = fx.state();
  return a == 0 && (st == AudioEffectReel::REC || st == AudioEffectReel::DUB);
}

// the loop as a waveform, the looping part (START / LEN) bracketed, the
// play head running through it; REC / DUB in the corner
FLASHMEM void ReelDevice::DrawViz(int x, int y, int w, int h) {
  if (no_mem_at && millis() - no_mem_at < 1500) {
    gfxPrint(x + 22, y + h / 2 - 4, PSTR("memory full"));
    return;
  }
  if (!fx.ready()) {
    gfxPrint(x + 22, y + h / 2 - 4, PSTR("out of memory"));
    return;
  }
  const int base = y + h - 1, cy = y + h / 2;
  const auto st = fx.state();
  // just cleared: a big CLEARED for a second
  if (cleared_at && millis() - cleared_at < 1200) {
    gfxDottedLine(x, cy, x + w - 1, cy, 2);
    graphics.clearRect(x + w / 2 - 26, cy - 6, 52, 13);
    gfxFrame(x + w / 2 - 26, cy - 6, 52, 13);
    gfxPrint(x + w / 2 - 21, cy - 3, PSTR("CLEARED"));
    if ((millis() - cleared_at) / 200 % 2 == 0) gfxInvert(x + w / 2 - 25, cy - 5, 50, 11);
    return;
  }
  if (st == AudioEffectReel::EMPTY) {
    gfxDottedLine(x, cy, x + w - 1, cy, 2);
    gfxPrint(x + w / 2 - 21, cy - 12, PSTR("A: REC"));
    return;
  }
  if (cur[VIEW] > 0) {
    DrawZoom(x, y, w, h);
  } else {
  constexpr int HIST = AudioEffectReel::HIST;
  uint8_t hist[HIST];
  fx.ReadHistory(hist);
  // scaled to the loop's loudest point, so a quiet loop still has a shape
  int top = 32;
  for (int k = 0; k < HIST; k++) top = std::max<int>(top, hist[k]);
  for (int k = 0; k < HIST; k++) {
    const int cx = x + k * w / HIST;
    const int a = int(sqrtf(float(hist[k]) / top) * (h / 2 - 1));
    gfxLine(cx, cy - a, cx, cy + a);
  }
  if (st != AudioEffectReel::REC) {
    float a, b;
    fx.Window(a, b);
    const int wl = x + int(a * (w - 1)), wr = x + int(std::min(b, 1.0f) * (w - 1));
    for (int px = wl; px <= wr; px++) { gfxPixel(px, y); gfxPixel(px, base); }
    for (int k = 1; k < 5; k++) {
      gfxPixel(wl, y + k); gfxPixel(wr, y + k);
      gfxPixel(wl, base - k); gfxPixel(wr, base - k);
    }
  }
  // the head: recording at the end of what is there, playing in the loop
  const int hx = constrain(x + int(fx.HeadPos() * (w - 1)), x, x + w - 1);
  const int cl = std::max(x, hx - 1), cr = std::min(x + w - 1, hx + 1);
  graphics.clearRect(cl, y + 1, cr - cl + 1, h - 2);
  gfxLine(hx, y + 1, hx, base - 1);
  }
  // saving: A held fills a bar, then the file is written, then its name
  if (save_hold > 0.0f || saving >= 0.0f || (saved_at && millis() - saved_at < 1500)) {
    char t[24];
    int fill = 0;
    if (saving >= 0.0f) { snprintf(t, sizeof(t), "SAVING %d%%", int(saving * 100)); fill = int(saving * 76); }
    else if (save_hold > 0.0f) { snprintf(t, sizeof(t), "SAVE LOOP"); fill = int(save_hold * 76); }
    else snprintf(t, sizeof(t), "%s", saved_name[0] ? saved_name : "SD ERROR");
    graphics.clearRect(x + w / 2 - 40, cy - 7, 80, 15);
    gfxFrame(x + w / 2 - 40, cy - 7, 80, 15);
    gfxPrint(x + w / 2 - 3 * int(strlen(t)), cy - 3, t);
    if (fill) gfxInvert(x + w / 2 - 38, cy - 5, fill, 11);
    else gfxInvert(x + w / 2 - 39, cy - 6, 78, 13);
    return;
  }
  // B held: the loop is wiped away from the right as the 3 s run out
  if (hold > 0.0f) {
    const int wipe = std::min(w, int(hold * w + 0.5f));
    graphics.clearRect(x + w - wipe, y, wipe, h);
    if (wipe < w) gfxLine(x + w - wipe - 1, y, x + w - wipe - 1, base);
    char t[10];
    snprintf(t, sizeof(t), "CLEAR %d", 3 - std::min(2, int(hold * 3.0f)));
    graphics.clearRect(x + w / 2 - 26, cy - 6, 52, 13);
    gfxPrint(x + w / 2 - 21, cy - 3, t);
    gfxInvert(x + w / 2 - 25, cy - 5, 50, 11);
    return;
  }
  if (st == AudioEffectReel::REC || st == AudioEffectReel::DUB) {
    const bool rec = st == AudioEffectReel::REC;
    if (!rec || (millis() / 300) % 2) {
      graphics.clearRect(x + w - 27, y, 27, 9);
      gfxPrint(x + w - 25, y + 1, rec ? PSTR("REC") : PSTR("DUB"));
      gfxInvert(x + w - 26, y, 20, 9);
    }
  }
}

// VIEW zoomed: the waveform round the play head (in the middle; while
// recording, at the right edge), the loop's ends marked where they show
FLASHMEM void ReelDevice::DrawZoom(int x, int y, int w, int h) {
  const int cy = y + h / 2, base = y + h - 1, half = h / 2 - 1;
  const auto st = fx.state();
  const bool rec = st == AudioEffectReel::REC;
  const long span = long(REEL_VIEW_MS[constrain(cur[VIEW] - 1, 0, 8)] * (AUDIO_SAMPLE_RATE_EXACT / 1000.0f));
  const long head = long(fx.HeadFrame());
  const int hx = rec ? x + w - 1 : x + w / 2;
  const long from = head - span * (hx - x) / w;
  int16_t lo[128], hi[128];
  const int cols = std::min(w, 128);
  fx.ReadWave(from, span, cols, lo, hi);
  int pk = 0;
  for (int k = 0; k < cols; k++) pk = std::max({pk, abs(int(lo[k])), abs(int(hi[k]))});
  view_range = std::max({float(pk), view_range * 0.95f, 1024.0f});
  const int range = int(view_range) + 1;
  gfxDottedLine(x, cy, x + w - 1, cy, 3);
  int plo = 0, phi = 0;
  for (int k = 0; k < cols; k++) {
    int a = lo[k] * half / range, b = hi[k] * half / range;
    if (k) { a = std::min(a, phi); b = std::max(b, plo); }  // joined to the last column
    if (lo[k] || hi[k]) gfxLine(x + k, cy - b, x + k, cy - a);
    plo = lo[k] * half / range; phi = hi[k] * half / range;
  }
  // the looping part's ends, where they fall in view
  if (!rec && fx.Length()) {
    float fa, fb;
    fx.Window(fa, fb);
    const long n = long(fx.Length());
    for (long e : {long(fa * n), long(std::min(fb, 1.0f) * n)}) {
      for (long rep = -1; rep <= 1; rep++) {  // the view may wrap round the loop
        const long px = x + (e + rep * n - from) * w / span;
        if (px < x || px >= x + w) continue;
        for (int k = 0; k < 5; k++) { gfxPixel(px, y + k); gfxPixel(px, base - k); }
      }
    }
  }
  graphics.clearRect(std::max(x, hx - 1), y + 1, std::min(3, x + w - std::max(x, hx - 1)), h - 2);
  gfxLine(hx, y + 1, hx, base - 1);
}

FLASHMEM void ReelDevice::DrawMini(int x, int y, int w, int h) {
  const auto st = fx.state();
  const int cy = y + h / 2;
  if (cleared_at && millis() - cleared_at < 1200) {  // flashes when cleared
    if ((millis() - cleared_at) / 200 % 2 == 0) gfxRect(x, y, w, h);
    return;
  }
  if (st == AudioEffectReel::EMPTY) {  // nothing recorded: a record circle
    gfxCircle(x + w / 2, cy, std::min(w, h) / 2 - 1);
    return;
  }
  constexpr int HIST = AudioEffectReel::HIST;
  uint8_t hist[HIST];
  fx.ReadHistory(hist);
  for (int i = 0; i < w; i++) {
    const int a = int(sqrtf(hist[i * HIST / w] / 255.0f) * (h / 2));
    gfxLine(x + i, cy - a, x + i, cy + a);
  }
  const int hx = constrain(x + int(fx.HeadPos() * (w - 1)), x, x + w - 1);
  gfxInvert(hx, y, 1, h);
  if (st == AudioEffectReel::REC && (millis() / 300) % 2) gfxInvert(x, y, w, h);
}

/////////////////////////////////////////////////////////////////
// Loop files on the SD card: /LOOM/*.WAV, 16-bit PCM, mono or stereo.
// Reading and writing happen in small steps from the main loop, so the
// screen keeps going and the audio interrupt is never held up.
/////////////////////////////////////////////////////////////////
constexpr char LOOP_DIR[] = "/LOOM";
constexpr char REC_DIR[] = "/LOOM/RECORDINGS";  // SPOOL's saved loops
constexpr int MAX_FILES = 48, NAME_LEN = 80;
constexpr size_t IO_CHUNK = 2048;  // sample frames per step

// a file name for the screen: without .wav, and when longer than `fit`
// characters cut in the middle ("first..last"), so both ends stay readable
FLASHMEM static void ShortName(const char* nm, int fit, char* out, size_t cap) {
  int l = int(strlen(nm));
  if (l > 4 && !strcasecmp(nm + l - 4, ".WAV")) l -= 4;
  fit = std::max(3, std::min(fit, int(cap) - 1));
  if (l <= fit) { snprintf(out, cap, "%.*s", l, nm); return; }
  const int head = (fit - 2 + 1) / 2, tail = fit - 2 - head;
  snprintf(out, cap, "%.*s..%.*s", head, nm, tail, nm + l - tail);
}

FLASHMEM static void PutLE(uint8_t* p, uint32_t v, int n) {
  for (int i = 0; i < n; i++) p[i] = uint8_t(v >> (8 * i));
}

// writes a buffer pair as a stereo WAV, a step at a time
struct WavWriter {
  File f;
  const int16_t* src[2] = {};
  size_t n = 0, done = 0;
  bool busy = false;
  char name[NAME_LEN] = {};

  FLASHMEM bool Begin(const int16_t* l, const int16_t* r, size_t frames) {
    if (!SDcard_Ready || !frames) return false;
    if (!SD.exists(LOOP_DIR)) SD.mkdir(LOOP_DIR);
    if (!SD.exists(REC_DIR)) SD.mkdir(REC_DIR);
    char path[48];
    for (int k = 1; k < 1000; k++) {  // the next free Spool_nnn.wav
      snprintf(name, sizeof(name), "Spool_%03d.wav", k);
      snprintf(path, sizeof(path), "%s/%s", REC_DIR, name);
      if (!SD.exists(path)) break;
    }
    f = SD.open(path, FILE_WRITE);
    if (!f) return false;
    uint8_t h[44] = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E',
                     'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 2, 0};
    const uint32_t bytes = frames * 4;
    PutLE(h + 4, 36 + bytes, 4);
    // the rate it was really recorded at (the module runs at 48 kHz)
    PutLE(h + 24, uint32_t(AUDIO_SAMPLE_RATE_EXACT), 4);
    PutLE(h + 28, uint32_t(AUDIO_SAMPLE_RATE_EXACT) * 4, 4);
    PutLE(h + 32, 4, 2);
    PutLE(h + 34, 16, 2);
    memcpy(h + 36, "data", 4);
    PutLE(h + 40, bytes, 4);
    f.write(h, 44);
    src[0] = l; src[1] = r; n = frames; done = 0; busy = true;
    return true;
  }
  // returns true when finished
  FLASHMEM bool Step() {
    if (!busy) return true;
    static int16_t tmp[IO_CHUNK * 2];
    const size_t k = std::min(IO_CHUNK, n - done);
    for (size_t i = 0; i < k; i++) {
      tmp[2 * i] = src[0][done + i];
      tmp[2 * i + 1] = src[1][done + i];
    }
    f.write(reinterpret_cast<uint8_t*>(tmp), k * 4);
    done += k;
    if (done >= n) { f.close(); busy = false; return true; }
    return false;
  }
  float Progress() const { return n ? float(done) / n : 1.0f; }
};

/////////////////////////////////////////////////////////////////
// PLAYER (Source): plays a WAV from the SD card. The file is loaded into
// PSRAM (up to 16 s, like SPOOL) and played by the same engine: START / END
// pick the loop, SPEED plays it at any rate (backwards below zero), XFADE
// is the fade at the loop ends. START / END zoom the picture (their mode)
// and, with SNAP on, land on the quietest point nearby. Longer files stream
// from the card (FILE mode STRM); FILE mode MEM copies a section of them
// (START..END when switched, at most 16 s) into memory and plays that as
// the sample, START / END then within it; back to STRM puts them back.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef PLAYER_DEF = {
  "PLAYER", "PLY", 8, {"PLAY", "TOP"},
  {
    {"FILE", 0, MAX_FILES - 1, 0, UNI, 0, 2, 0, {}, {"STRM", "MEM"}},
    {"LOOP", 0, 1, 1, LIST, 2, 0, 0, {"OFF", "ON"}},
    {"LEVEL", 0, 100, 100, UNI},
    {"SPEED", 0, 200, 100, UNI, 0, 3, 1, {}, {"REV", "FWD", "SYNC"}},  // mode: direction, REV left; SYNC: forward at the clock's tempo
    {"START", 0, 10000, 0, UNI, 0, 10, 0},      // mode: zoom, all .. 0.3 ms across
    {"END", 0, 10000, 10000, UNI, 0, 10, 0},
    {"FADE", 0, 100, 20, UNI, 0, 2, 0, {}, {"CROSS", "INOUT"}},  // the loop's seam
    {"SNAP", 0, 1, 0, LIST, 2, 0, 0, {"OFF", "ON"}},
  },
  {"", ""},
};

class PlayerDevice : public Device {
public:
  enum { FILE_, LOOP, LEVEL, SPEED, START, END, XFADE, SNAP };
  enum { DIR_REV, DIR_FWD, DIR_SYNC };  // SPEED's modes
  static float MaxSecs() { return BigSecs(); }
  static constexpr size_t MIN_BUF = 11025;  // 0.25 s: the buffer before a file
  // FILE mode MEM on a long file: the section in memory, in file frames
  // (len 0: none chosen yet, the first 16 s); kept in the saved settings
  uint32_t sec_off = 0, sec_len = 0;
  bool clipped = false;     // what is in memory is a section of a long file
  int applied_file = -1, applied_mode = -1;
  // longer files stream from the card (AudioPlayStream) into fx's input,
  // which fx passes through (its MON), so the device has one output
  static constexpr int IDX_FRAMES = 256;  // frames per entry of the level index
  PlayerDevice() : Device(PLAYER_DEF), st_l(stream, 0, fx, 0), st_r(stream, 1, fx, 1) {}
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  bool Activate() override;
  void Deactivate() override;
  bool ok() const override { return fx.ready(); }
  void Apply(int p) override;
  int Step(int p, int v, int dir) const override;
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  // lit while playing, or while a start waits for its beat
  bool ActionLit(int a) const override { return a == 0 && (Playing() || stream.PlayWaiting() || fx.PressWaiting()); }
  int32_t BeatWait() const;
  bool HeaderInfo(char* buf, size_t n) const override;
  static constexpr size_t MIN_SPAN = 16;  // deepest view: 16 samples across
  int Samples(size_t a, size_t count, int16_t* mono) const;  // picture: (L+R)/2
  // zoom steps: how much time is across the picture, the same for short and
  // long files (0 = the whole file)
  static constexpr uint32_t ZOOM_US[10] = {0, 16000000, 4000000, 1000000, 250000, 60000, 15000, 4000, 1000, 300};
  size_t ZoomSpan(int m) const {  // in frames, for zoom step m
    const size_t n = Frames();
    const uint32_t us = ZOOM_US[constrain(m, 0, 9)];
    if (!us) return std::max<size_t>(MIN_SPAN, n);
    const size_t f = size_t(double(us) * AUDIO_SAMPLE_RATE_EXACT * rate / 1e6);
    return std::max<size_t>(MIN_SPAN, std::min(f, std::max<size_t>(MIN_SPAN, n)));
  }
  static void FormatZoom(int m, char* buf, size_t n) {
    const uint32_t us = ZOOM_US[constrain(m, 0, 9)];
    if (!us) snprintf(buf, n, "all");
    else if (us >= 1000000) snprintf(buf, n, "%lus", us / 1000000);
    else if (us >= 1000) snprintf(buf, n, "%lums", us / 1000);
    else snprintf(buf, n, ".%lums", us / 100);
  }
  // the zoomed picture's wave, lowest / highest per column, filled a bit
  // every frame (from the card when streaming) and started over when the
  // view moves
  mutable int zoom_edge = START;  // the edge the zoomed picture centres on
  mutable bool wv_any = false;  // something is cached (wv_v0 means something)
  mutable int64_t wv_v0 = 0;
  mutable size_t wv_span = 0;
  mutable int wv_fa = 0, wv_fb = 0;  // the columns read so far
  mutable int wv_file = -1;
  mutable int16_t wv_lo[128], wv_hi[128];
  bool FillWave(int64_t v0, size_t span, int w, int fi) const;  // true when complete
  // START / END to the sample (the settings hold them to 0.01%); kept with
  // the folder and file in the saved settings
  mutable uint32_t ex_pos[2] = {0, 0};
  mutable bool ex_valid = false;
  bool keep_exact = false;  // set when the saved settings bring them
  void RestoreEdges(uint32_t a, uint32_t b) { ex_pos[0] = a; ex_pos[1] = b; ex_valid = true; keep_exact = true; loaded = -1; }
  size_t EdgeAt(int p) const;  // START / END in frames
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
  // main loop: loading in steps
  void Idle();
  bool Busy() const { return loading || analysing; }  // reading the file in
  // the WAVs in the current folder, sorted; Select picks one in /LOOM by
  // name (after a save), SetFile any folder + name (from the settings)
  void Scan();
  void Select(const char* name) { SetFile(REC_DIR, name); }  // a loop SPOOL just saved
  void SetFile(const char* folder, const char* name);
  const char* Folder() const { return dir; }
  void SetPick(uint8_t p) { fx.pick = p; }
  void Debug(char* buf, size_t n) const { if (streaming) stream.Debug(buf, n); else fx.Debug(buf, n); }
  float XfadeFrames() const {
    const float v = cur[XFADE] * 0.01f;
    return 16.0f + v * v * AUDIO_SAMPLE_RATE_EXACT;
  }
  // linked: the same file, place and edges as the other track's PLAYER
  void LinkFrom(const PlayerDevice& a) {
    CopyFrom(a);
    ex_pos[0] = a.ex_pos[0]; ex_pos[1] = a.ex_pos[1];
    ex_valid = keep_exact = a.ex_valid;
    sec_off = a.sec_off; sec_len = a.sec_len;
    applied_file = a.applied_file; applied_mode = a.applied_mode;
    SetFile(a.Folder(), a.FileName());
  }
  const char* FileName() const { return nfiles ? names[std::min<int>(cur[FILE_], nfiles - 1)] : ""; }
  // the settings that go with a file: <name>.loom next to the WAV, read
  // when the file is picked (not when the saved settings bring it back),
  // written when they changed (leaving the page, another file, a save)
  void WriteMeta();
  void Reset() override {
    const int16_t f = val[FILE_];
    const uint8_t m = mode[FILE_];
    Defaults();
    val[FILE_] = cur[FILE_] = f;
    mode[FILE_] = m;
    ex_valid = false;  // the edges from START / END (the section in memory stays)
    ApplyAll();
  }
  void KeepMore(Saved& k) const override { k.ex[0] = ex_pos[0]; k.ex[1] = ex_pos[1]; k.ex_valid = ex_valid; }
  void RestoreMore(const Saved& k) override { ex_pos[0] = k.ex[0]; ex_pos[1] = k.ex[1]; ex_valid = k.ex_valid; }
  // the browser: folders and WAVs as a text list (R push on FILE)
  bool browsing = false;
  void OpenBrowser();
  void BrowseMove(int d) { bsel = constrain(bsel + d, 0, std::max(0, bcount - 1)); }
  void BrowseEnter();
  void BrowseBack();
  void DrawBrowser(int x, int y, int w, int h) const;

private:
  AudioPlayStream stream;  // before fx: the audio update order follows the signal
  AudioEffectReel fx;
  AudioConnection st_l, st_r;
  char names[MAX_FILES][NAME_LEN] = {};
  uint32_t scanned_at = 0;
  char dir[96] = "/LOOM";  // the folder FILE steps through
  // browser
  char bdir[96] = "/";
  char bn[MAX_FILES][NAME_LEN] = {};
  bool bfolder[MAX_FILES] = {};
  int bcount = 0, bsel = 0;
  void List();
  // streaming (files too long for fx)
  bool streaming = false, analysing = false;
  bool indexing = false;   // streaming, and the level index still being read
  uint32_t indexed_at = 0;
  size_t data_off = 0, done = 0;  // where the samples start in the file; analysis progress
  uint8_t* idx = nullptr;         // level per IDX_FRAMES frames, for the picture
  size_t idx_n = 0;
  bool Playing() const { return streaming ? stream.Playing() : fx.state() == AudioEffectReel::PLAY; }
  size_t Frames() const { return streaming ? frames : fx.Length(); }
  int16_t Sample(const uint8_t* p) const;
  size_t ReadFrames(int16_t* l, int16_t* r, size_t n);  // from lf, converted
  int PeakAt(size_t a, size_t b) const;                 // picture: loudest in a..b
  int nfiles = 0, loaded = -1, loaded_mode = -1;
  struct Meta {
    int16_t val[MAX_PARAMS];
    uint8_t mode[MAX_PARAMS];
    uint32_t ex[2], sec_off, sec_len;
    bool operator==(const Meta& o) const { return !memcmp(this, &o, sizeof(Meta)); }
  };
  char meta_path[200] = "";  // the .loom of the file loaded
  Meta meta_was = {};        // what it holds (or held when read)
  bool meta_known = false;   // meta_was is what is on the card
  void TakeMeta(Meta& m) const;
  bool ReadMeta();
  // the file is in: what it has now is the start (only changes get written,
  // so stepping through files leaves nothing behind)
  void MetaBaseline() { if (!meta_known) { TakeMeta(meta_was); meta_known = true; } BpmStart(); }
  // tempo: the hits' envelope (one level per IDX_FRAMES frames, up to
  // BPM_BINS of them from START) and how regularly it rises, worked out a
  // slice at a time from Idle() once the file is in
  static constexpr int BPM_BINS = 6000;  // ~32 s at 48 kHz
  float bpm = 0.0f;           // the file's tempo at SPEED 100 (0: none found)
  int bpm_got = -1;           // levels read so far (-1: no analysis running)
  int bpm_n = 0;
  size_t bpm_from = 0;
  float* bpm_env = nullptr;   // RAM2, only while it runs
  void BpmStart();
  void BpmIdle();
  void BpmSolve();
  float FileRate() const { return rate * AUDIO_SAMPLE_RATE_EXACT; }
  // SYNC: how much faster the file plays so its tempo is the clock's (1:
  // not synced, or no tempo found); SPEED then multiplies that
  float SyncRatio() const {
    return mode[SPEED] == DIR_SYNC && bpm > 0.0f ? tempo_bpm / bpm : 1.0f;
  }
  float synced_at = 0.0f, synced_bpm = 0.0f;  // the clock tempo / file tempo SPEED was set for
  // the head, in frames of the whole file
  size_t HeadFrame() const {
    const size_t n = Frames();
    const size_t at = size_t((streaming ? stream.HeadPos() : fx.HeadPos()) * n);
    return at + (clipped ? sec_off : 0);
  }
  // START / END settings from the exact edges, once the length is known
  void EdgesToSettings() {
    const size_t n = Frames();
    if (!ex_valid || !n) return;
    for (int k = 0; k < 2; k++) {
      ex_pos[k] = std::min<uint32_t>(ex_pos[k], uint32_t(n));
      val[k ? END : START] = int(uint64_t(ex_pos[k]) * 10000 / n);
      Refresh(k ? END : START);
    }
  }
  // loading
  File lf;
  bool loading = false, too_long = false;
  int chans = 2, bytes = 2;  // per sample
  bool is_float = false;
  size_t frames = 0, got = 0;
  float rate = 1.0f;  // file rate / the module's rate (48 kHz)
  void Load(int i);
  void Window();
  size_t SnapQuiet(size_t at) const;
};

// the WAVs in the current folder, sorted by name; a card put in after
// power-up is picked up here
FLASHMEM void PlayerDevice::Scan() {
  nfiles = 0;
  scanned_at = millis() | 1;
  if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
  if (!SDcard_Ready) return;
  // the folder was /SCULPT before the app became Loom: move it over once
  if (!SD.exists(LOOP_DIR) && SD.exists("/SCULPT")) SD.rename("/SCULPT", LOOP_DIR);
  File d = SD.open(dir);
  if (!d) { snprintf(dir, sizeof(dir), "/"); d = SD.open(dir); }
  if (!d) return;
  while (nfiles < MAX_FILES) {
    File e = d.openNextFile();
    if (!e) break;
    const char* nm = e.name();
    const size_t l = strlen(nm);
    if (!e.isDirectory() && nm[0] != '.' && l > 4 && l < NAME_LEN - 1 && !strcasecmp(nm + l - 4, ".WAV"))
      snprintf(names[nfiles++], NAME_LEN, "%s", nm);
    e.close();
  }
  d.close();
  for (int i = 1; i < nfiles; i++)
    for (int j = i; j > 0 && strcasecmp(names[j - 1], names[j]) > 0; j--) {
      char t[NAME_LEN];
      memcpy(t, names[j], NAME_LEN); memcpy(names[j], names[j - 1], NAME_LEN); memcpy(names[j - 1], t, NAME_LEN);
    }
  Serial.printf("PLAYER: %d WAV in %s\n", nfiles, dir);  // over USB
}

FLASHMEM void PlayerDevice::SetFile(const char* folder, const char* name) {
  snprintf(dir, sizeof(dir), "%s", folder && folder[0] ? folder : "/");
  Scan();
  for (int i = 0; i < nfiles; i++)
    if (!strcmp(names[i], name)) { val[FILE_] = cur[FILE_] = i; break; }
  loaded = -1;
  if (active) Load(cur[FILE_]);
}

FLASHMEM void PlayerDevice::TakeMeta(Meta& m) const {
  memset(&m, 0, sizeof(m));
  for (int p = 0; p < num_params(); p++) { m.val[p] = val[p]; m.mode[p] = mode[p]; }
  m.val[FILE_] = 0;  // the file is the one this is next to
  m.ex[0] = uint32_t(EdgeAt(START));
  m.ex[1] = uint32_t(EdgeAt(END));
  m.sec_off = sec_off;
  m.sec_len = sec_len;
}

// a text file, one setting per line: NAME value mode; EDGES start end (in
// frames of what is loaded); SECTION offset length (MEM on a long file)
FLASHMEM bool PlayerDevice::ReadMeta() {
  if (!SDcard_Ready) return false;
  File f = SD.open(meta_path, FILE_READ);
  if (!f) return false;
  char line[48];
  bool edges = false;
  while (f.available()) {
    int n = 0;
    while (f.available() && n < int(sizeof(line)) - 1) {
      const char c = f.read();
      if (c == '\n') break;
      if (c != '\r') line[n++] = c;
    }
    line[n] = 0;
    char name[8];
    long a = 0, b = 0;
    if (sscanf(line, "%7s %ld %ld", name, &a, &b) != 3) continue;
    if (!strcmp(name, "EDGES")) {
      ex_pos[0] = uint32_t(std::max(0L, a)); ex_pos[1] = uint32_t(std::max(0L, b));
      edges = true;
    } else if (!strcmp(name, "SECTION")) {
      sec_off = uint32_t(std::max(0L, a)); sec_len = uint32_t(std::max(0L, b));
    } else {
      for (int p = 0; p < num_params(); p++) {
        if (strcmp(name, param(p).name)) continue;
        const ParamDef& pd = param(p);
        if (p != FILE_) val[p] = int16_t(constrain(a, long(pd.min), long(pd.max)));
        mode[p] = pd.nmodes ? uint8_t(constrain(b, 0L, long(pd.nmodes - 1))) : 0;
      }
    }
  }
  f.close();
  ex_valid = edges;  // EdgesToSettings() turns them into START / END
  return true;
}

FLASHMEM void PlayerDevice::WriteMeta() {
  // only a file that is all there (its edges mean something)
  if (!active || !meta_path[0] || loading || analysing || !Frames() || !SDcard_Ready) return;
  Meta m;
  TakeMeta(m);
  if (!meta_known || m == meta_was) return;  // nothing changed since it came in
  SD.remove(meta_path);
  File f = SD.open(meta_path, FILE_WRITE);
  if (!f) return;
  for (int p = 0; p < num_params(); p++) f.printf("%s %d %d\n", param(p).name, m.val[p], m.mode[p]);
  f.printf("EDGES %lu %lu\n", (unsigned long)m.ex[0], (unsigned long)m.ex[1]);
  f.printf("SECTION %lu %lu\n", (unsigned long)m.sec_off, (unsigned long)m.sec_len);
  f.close();
  meta_was = m;
  meta_known = true;
}

FLASHMEM void PlayerDevice::BpmStart() {
  if (bpm_env) { free(bpm_env); bpm_env = nullptr; }
  bpm = 0.0f;
  bpm_got = -1;
  const size_t n = Frames();
  bpm_from = EdgeAt(START);
  if (bpm_from + 800 * IDX_FRAMES > n) bpm_from = 0;  // too little after START: from the beginning
  bpm_n = int(std::min<size_t>((n - bpm_from) / IDX_FRAMES, BPM_BINS));
  if (bpm_n < 800) return;  // under ~4 s: no tempo
  bpm_env = static_cast<float*>(malloc(sizeof(float) * bpm_n));
  if (bpm_env) bpm_got = 0;
}

FLASHMEM void PlayerDevice::BpmIdle() {
  if (bpm_got < 0 || !bpm_env || loading || analysing) return;
  if (streaming && indexing) return;  // the level index is still being read
  // the envelope, 400 levels at a time (in dB-like steps, so quiet hits
  // count as well as loud ones)
  for (int k = 0; k < 400 && bpm_got < bpm_n; k++, bpm_got++) {
    const size_t a = bpm_from + size_t(bpm_got) * IDX_FRAMES;
    bpm_env[bpm_got] = logf(1.0f + PeakAt(a, a + IDX_FRAMES) * (1.0f / 128.0f));
  }
  if (bpm_got < bpm_n) return;
  BpmSolve();
  free(bpm_env);
  bpm_env = nullptr;
  bpm_got = -1;
}

// where the envelope rises (the hits), then how alike it is to itself one,
// two, three and four beats later, for every tempo from 70 to 180 BPM in
// 0.1 steps; a gentle preference for tempos near 120 decides between a
// tempo and its half / double
FLASHMEM void PlayerDevice::BpmSolve() {
  float* o = bpm_env;
  const int n = bpm_n;
  for (int i = n - 1; i > 0; i--) o[i] = std::max(0.0f, o[i] - o[i - 1]);
  o[0] = 0.0f;
  float mean = 0.0f;
  for (int i = 0; i < n; i++) mean += o[i];
  mean /= n;
  for (int i = 0; i < n; i++) o[i] -= mean;
  const float bin_s = IDX_FRAMES / FileRate();
  const int lmax = std::min(n / 2, int(4.0f * 60.0f / (70.0f * bin_s)) + 2);
  float* r = static_cast<float*>(malloc(sizeof(float) * (lmax + 1)));
  if (!r) return;
  for (int l = 0; l <= lmax; l++) {
    float acc = 0.0f;
    for (int i = 0; i + l < n; i++) acc += o[i] * o[i + l];
    r[l] = acc / float(n - l);
  }
  if (r[0] <= 0.0f) { free(r); return; }
  auto at = [&](float l) {
    const int i = int(l);
    if (i + 1 > lmax) return 0.0f;
    const float f = l - float(i);
    return (r[i] + (r[i + 1] - r[i]) * f) / r[0];
  };
  float best = 0.0f, best_s = 0.0f;
  for (int t = 700; t <= 1800; t++) {
    const float b = t * 0.1f;
    const float l = 60.0f / (b * bin_s);
    float sc = 0.0f;
    for (int k = 1; k <= 4; k++) sc += at(k * l);
    const float oct = log2f(b / 120.0f) / 0.6f;
    sc *= expf(-0.5f * oct * oct);
    if (sc > best_s) { best_s = sc; best = b; }
  }
  free(r);
  bpm = best_s > 0.25f ? best : 0.0f;  // too little alike: no clear beat
}

// --- the browser: sub-folders first (with a /), then WAVs; ".." goes up
FLASHMEM void PlayerDevice::List() {
  bcount = 0;
  bsel = 0;
  if (strcmp(bdir, "/")) { snprintf(bn[bcount], NAME_LEN, ".."); bfolder[bcount++] = true; }
  File d = SD.open(bdir);
  if (!d) return;
  Serial.printf("BROWSER %s\n", bdir);  // the whole folder over USB, for checking
  for (int pass = 0; pass < 2; pass++) {  // folders, then files
    d.rewindDirectory();
    const int first = bcount;
    while (bcount < MAX_FILES) {
      File e = d.openNextFile();
      if (!e) break;
      const char* nm = e.name();
      const size_t l = strlen(nm);
      const bool isdir = e.isDirectory();
      const bool want = nm[0] != '.' && l < NAME_LEN - 1 &&
                        (pass == 0 ? isdir : (!isdir && l > 4 && !strcasecmp(nm + l - 4, ".WAV")));
      if (want) { bfolder[bcount] = isdir; snprintf(bn[bcount++], NAME_LEN, "%s", nm); }
      if (pass == 1) Serial.printf("  %s%s %lu%s\n", nm, isdir ? "/" : "", (unsigned long)e.size(), want ? "" : "  (not listed)");
      e.close();
    }
    for (int i = first + 1; i < bcount; i++)
      for (int j = i; j > first && strcasecmp(bn[j - 1], bn[j]) > 0; j--) {
        char t[NAME_LEN];
        memcpy(t, bn[j], NAME_LEN); memcpy(bn[j], bn[j - 1], NAME_LEN); memcpy(bn[j - 1], t, NAME_LEN);
        std::swap(bfolder[j], bfolder[j - 1]);
      }
  }
  d.close();
}

FLASHMEM void PlayerDevice::OpenBrowser() {
  if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
  snprintf(bdir, sizeof(bdir), "%s", dir);
  List();
  // start on the file that is playing
  for (int i = 0; i < bcount; i++)
    if (!bfolder[i] && !strcmp(bn[i], FileName())) { bsel = i; break; }
  browsing = true;
}

FLASHMEM void PlayerDevice::BrowseBack() {
  if (!strcmp(bdir, "/")) { browsing = false; return; }  // at the top: close
  char* slash = strrchr(bdir, '/');
  if (slash == bdir) bdir[1] = 0; else if (slash) *slash = 0;
  List();
}

FLASHMEM void PlayerDevice::BrowseEnter() {
  if (bsel >= bcount) return;
  if (bfolder[bsel]) {
    if (!strcmp(bn[bsel], "..")) { BrowseBack(); return; }
    const size_t l = strlen(bdir);
    if (l + strlen(bn[bsel]) + 2 < sizeof(bdir))
      snprintf(bdir + l, sizeof(bdir) - l, "%s%s", l > 1 ? "/" : "", bn[bsel]);
    List();
    return;
  }
  SetFile(bdir, bn[bsel]);  // a WAV: play from this folder
  browsing = false;
}

FLASHMEM void PlayerDevice::DrawBrowser(int x, int y, int w, int h) const {
  // the folder on top, then 5 entries around the selection
  char t[24];
  const size_t l = strlen(bdir);
  snprintf(t, sizeof(t), "%s", l > 21 ? bdir + l - 21 : bdir);
  gfxPrint(x, y + 1, t);
  gfxLine(x, y + 10, x + w - 1, y + 10);
  if (!bcount) { gfxPrint(x + 4, y + 16, PSTR("(empty)")); return; }
  constexpr int rows = 5;
  const int first = constrain(bsel - rows / 2, 0, std::max(0, bcount - rows));
  for (int r = 0; r < rows && first + r < bcount; r++) {
    const int i = first + r;
    if (bfolder[i]) {
      const bool up = !strcmp(bn[i], "..");
      char f[20];
      ShortName(bn[i], 19 - !up, f, sizeof(f));
      snprintf(t, sizeof(t), "%s%s", f, up ? "" : "/");
    } else {
      ShortName(bn[i], 20, t, sizeof(t));
    }
    const int ry = y + 13 + r * 8;
    gfxPrint(x + 2, ry, t);
    if (i == bsel) gfxInvert(x, ry - 1, w, 9);
  }
}

// called while the stream is not patched in
FLASHMEM bool PlayerDevice::Activate() {
  if (active) return fx.ready();
  // a small buffer to start with; a file gets one its own size when it is
  // opened (streamed files keep the small one)
  fx.Resize(MIN_BUF);
  fx.SetMonitor(1.0f);  // passes the stream through
  Scan();
  active = true;
  loaded = -1;
  ApplyAll();
  return fx.ready();
}

FLASHMEM void PlayerDevice::Deactivate() {
  if (!active) return;
  if (loading || streaming || analysing) lf.close();
  loading = streaming = analysing = false;
  stream.Release();
  if (idx) { extmem_free(idx); idx = nullptr; }
  if (bpm_env) { free(bpm_env); bpm_env = nullptr; }
  bpm_got = -1;
  fx.Release();
  loaded = -1;
  active = false;
}

// reads the WAV header, then Idle() fills the buffer
FLASHMEM void PlayerDevice::Load(int i) {
  WriteMeta();  // what the last file had, before it goes
  fx.CancelPress();  // a start waiting for its beat was for the old file
  stream.CancelPlay();
  if (bpm_env) { free(bpm_env); bpm_env = nullptr; }
  bpm_got = -1;
  bpm = 0.0f;
  if (loading || streaming || analysing) lf.close();
  loading = streaming = analysing = false;
  stream.Stop();
  stream.Open(0);
  fx.BeginLoad();
  loaded = i;
  loaded_mode = mode[FILE_];
  too_long = clipped = indexing = false;
  wv_any = false;  // the cached picture belongs to what was there
  const bool restoring = keep_exact;  // the saved settings bring their own
  if (!keep_exact) ex_valid = false;  // a new file: from the settings
  keep_exact = false;
  if (i >= nfiles || !fx.ready()) return;
  char path[192];
  snprintf(path, sizeof(path), "%s/%s", strcmp(dir, "/") ? dir : "", names[i]);
  char mp[200];
  snprintf(mp, sizeof(mp), "%s", path);
  if (char* dot = strrchr(mp, '.')) *dot = 0;
  strlcat(mp, ".loom", sizeof(mp));
  if (strcmp(mp, meta_path)) {  // another file (not just STRM / MEM)
    snprintf(meta_path, sizeof(meta_path), "%s", mp);
    meta_known = false;
    if (!restoring && ReadMeta()) {
      applied_file = cur[FILE_];  // not another file for Apply(FILE_): the
      applied_mode = mode[FILE_]; // section and edges just read stay
      ApplyAll();
      loaded_mode = mode[FILE_];
    }
  }
  lf = SD.open(path, FILE_READ);
  if (!lf) return;
  uint8_t h[12];
  if (lf.read(h, 12) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) { lf.close(); return; }
  int bits = 0; uint32_t srate = 44100; chans = 0;
  while (lf.available()) {  // walk the chunks to fmt and data
    uint8_t c[8];
    if (lf.read(c, 8) != 8) break;
    const uint32_t sz = c[4] | c[5] << 8 | c[6] << 16 | uint32_t(c[7]) << 24;
    if (!memcmp(c, "fmt ", 4)) {
      uint8_t fm[40] = {};
      const uint32_t take = std::min<uint32_t>(sz, 40);
      lf.read(fm, take);
      if (sz > take) lf.seek(lf.position() + sz - take);
      chans = fm[2];
      srate = fm[4] | fm[5] << 8 | fm[6] << 16 | uint32_t(fm[7]) << 24;
      bits = fm[14];
      // PCM (1), float (3), or "extensible" (0xFFFE), whose real format is
      // the sub-format at byte 24
      int tag = fm[0] | fm[1] << 8;
      if (tag == 0xFFFE && take >= 26) tag = fm[24] | fm[25] << 8;
      is_float = tag == 3;
      if (tag != 1 && tag != 3) bits = 0;
    } else if (!memcmp(c, "data", 4)) {
      const bool ok_bits = is_float ? bits == 32 : (bits == 8 || bits == 16 || bits == 24 || bits == 32);
      if (!ok_bits || chans < 1) break;
      bytes = bits / 8;
      frames = sz / (bytes * chans);
      rate = srate / AUDIO_SAMPLE_RATE_EXACT;  // the module runs at 48 kHz
      got = 0;
      // MEM on a long file: only the chosen section goes into memory
      const size_t max_frames = size_t(MaxSecs() * AUDIO_SAMPLE_RATE_EXACT);
      if (mode[FILE_] == 1 && frames > max_frames) {
        if (!sec_len || sec_off >= frames) { sec_off = 0; sec_len = max_frames; }
        sec_len = uint32_t(std::min<size_t>({size_t(sec_len), max_frames, frames - sec_off}));
        lf.seek(lf.position() + size_t(sec_off) * size_t(bytes) * chans);
        frames = sec_len;
        clipped = true;
      }
      // up to 16 s: into memory, in a buffer of the file's own length
      if (frames <= max_frames && fx.Resize(std::max<size_t>(frames, MIN_BUF))) {
        loading = true;
        return;
      }
      // too long for memory (or no room): stream it, after one pass for the
      // level index; memory only for the pass-through
      fx.Resize(MIN_BUF);
      data_off = lf.position();
      if (!stream.ready()) stream.Acquire(4.0f, 1.0f);
      if (idx) extmem_free(idx);
      idx_n = frames / IDX_FRAMES + 1;
      idx = static_cast<uint8_t*>(extmem_calloc(idx_n, 1));
      if (!stream.ready() || !idx) {  // no room to stream either: the start of it
        fx.Resize(size_t(MaxSecs() * AUDIO_SAMPLE_RATE_EXACT)) || fx.Resize(size_t(2 * AUDIO_SAMPLE_RATE_EXACT));
        too_long = true; frames = fx.Capacity(); loading = true; return;
      }
      // streams straight away; the level index (the overview picture) is
      // read alongside, a little at a time
      done = 0;
      indexing = true;
      streaming = true;
      stream.Open(frames);
      EdgesToSettings();
      ApplyAll();
      MetaBaseline();
      return;
    } else {
      lf.seek(lf.position() + sz + (sz & 1));
    }
  }
  lf.close();
}

// one sample of any size / float as 16 bit
FLASHMEM int16_t PlayerDevice::Sample(const uint8_t* p) const {
  switch (bytes) {
    case 1: return int16_t((p[0] - 128) << 8);  // 8-bit is unsigned
    case 2: return int16_t(p[0] | p[1] << 8);
    case 3: return int16_t(p[1] | p[2] << 8);
    default:
      if (is_float) {
        float f;
        memcpy(&f, p, 4);
        return int16_t(constrain(f, -1.0f, 1.0f) * 32767.0f);
      }
      return int16_t(p[2] | p[3] << 8);
  }
}

// up to n frames from the file's current position; the first two channels
// (mono: both)
FLASHMEM size_t PlayerDevice::ReadFrames(int16_t* l, int16_t* r, size_t n) {
  static uint8_t raw[IO_CHUNK * 8];
  const size_t frame = size_t(bytes) * chans;
  const size_t k = std::min(n, sizeof(raw) / frame);
  const size_t rd = lf.read(raw, k * frame) / frame;
  for (size_t i = 0; i < rd; i++) {
    const uint8_t* fp = raw + i * frame;
    l[i] = Sample(fp);
    r[i] = Sample(fp + (chans > 1 ? bytes : 0));
  }
  return rd;
}

FLASHMEM void PlayerDevice::Idle() {
  if (!active) return;
  BpmIdle();
  // SYNC: follow the clock's tempo (and the file's, once it is found)
  if (mode[SPEED] == DIR_SYNC && (synced_at != tempo_bpm || synced_bpm != bpm)) Refresh(SPEED);
  if (!nfiles && millis() - scanned_at > 3000) Scan();  // card / files may come later
  if ((loaded != cur[FILE_] || loaded_mode != mode[FILE_]) && !loading && !analysing) Load(cur[FILE_]);
  static int16_t tl[IO_CHUNK], tr[IO_CHUNK];
  if (loading) {  // short file: all of it into fx
    const size_t rd = ReadFrames(fx.Data(0) + got, fx.Data(1) + got, std::min(IO_CHUNK, frames - got));
    fx.MarkRange(got, got + rd);
    got += rd;
    if (!rd || got >= frames) {
      lf.close();
      loading = false;
      fx.FinishLoad(got);
      EdgesToSettings();
      ApplyAll();
      MetaBaseline();
    }
    return;
  }
  if (analysing) {  // long file, first pass: the level index
    const size_t rd = ReadFrames(tl, tr, std::min(IO_CHUNK, frames - done));
    for (size_t i = 0; i < rd; i++) {
      const size_t e = (done + i) / IDX_FRAMES;
      const uint8_t pk = uint8_t(std::min(255, std::max(abs(tl[i]), abs(tr[i])) >> 7));
      if (e < idx_n && pk > idx[e]) idx[e] = pk;
    }
    done += rd;
    if (!rd || done >= frames) {
      analysing = false;
      streaming = true;
      stream.Open(frames);
      EdgesToSettings();
      ApplyAll();
      MetaBaseline();
    }
    return;
  }
  if (streaming) {  // keep the stream's buffers topped up, a few chunks a time
    for (int k = 0; k < 4; k++) {
      int16_t *l, *r;
      size_t at, count;
      if (!stream.Next(l, r, at, count, IO_CHUNK)) break;
      lf.seek(data_off + at * size_t(bytes) * chans);
      const size_t rd = ReadFrames(l, r, count);
      stream.Filled(rd);
      if (rd < count) break;
    }
    // the level index, one chunk every 5 ms (~9x faster than it plays)
    if (indexing && millis() - indexed_at >= 5) {
      indexed_at = millis();
      lf.seek(data_off + done * size_t(bytes) * chans);
      const size_t rd = ReadFrames(tl, tr, std::min(IO_CHUNK, frames - done));
      for (size_t i = 0; i < rd; i++) {
        const size_t e = (done + i) / IDX_FRAMES;
        const uint8_t pk = uint8_t(std::min(255, std::max(abs(tl[i]), abs(tr[i])) >> 7));
        if (e < idx_n && pk > idx[e]) idx[e] = pk;
      }
      done += rd;
      if (!rd || done >= frames) indexing = false;
    }
  }
}

// the quietest sample within ~5 ms of a position (a soft place to cut)
FLASHMEM size_t PlayerDevice::SnapQuiet(size_t at) const {
  const size_t n = Frames();
  if (!n) return at;
  const size_t r = 220, lo = at > r ? at - r : 0, hi = std::min(n - 1, at + r);
  size_t best = at;
  int quiet = INT32_MAX;
  static int16_t sl[2 * 220 + 1], sr[2 * 220 + 1];
  const int16_t *pl, *pr;
  if (streaming) {  // read the spot from the card
    auto* self = const_cast<PlayerDevice*>(this);
    self->lf.seek(data_off + lo * size_t(bytes) * chans);
    const size_t rd = self->ReadFrames(sl, sr, hi - lo + 1);
    pl = sl - lo; pr = sr - lo;
    if (rd < hi - lo + 1) return at;
  } else {
    pl = fx.Data(0); pr = fx.Data(1);
  }
  for (size_t i = lo; i <= hi; i++) {
    const int a = abs(pl[i]) + abs(pr[i]);
    if (a < quiet) { quiet = a; best = i; }
  }
  return best;
}

// to the sample when set by hand; from the setting when it is modulated
// (or nothing finer is known)
FLASHMEM size_t PlayerDevice::EdgeAt(int p) const {
  const size_t n = Frames();
  if (ex_valid && cur[p] == val[p]) return std::min<size_t>(n, ex_pos[p == END]);
  return size_t(uint64_t(cur[p]) * n / 10000);
}

FLASHMEM void PlayerDevice::Window() {
  const size_t n = Frames();
  if (!n) return;
  size_t a = EdgeAt(START), b = EdgeAt(END);
  if (cur[SNAP]) { a = SnapQuiet(a); b = SnapQuiet(b); }
  if (b <= a + AUDIO_BLOCK_SAMPLES) b = std::min(n, a + AUDIO_BLOCK_SAMPLES * 2);
  if (streaming) stream.SetWindow(a, b);
  else fx.SetWindowSamples(a, b - a);
}

FLASHMEM void PlayerDevice::Apply(int p) {
  switch (p) {
    case FILE_: {  // Idle() loads it
      if (applied_file >= 0 && cur[FILE_] != applied_file) sec_len = 0;  // another file
      if (applied_mode >= 0 && mode[FILE_] != applied_mode && Frames()) {
        if (mode[FILE_] == 1 && streaming) {
          // STRM -> MEM: the loop (at most 16 s) becomes the sample
          const size_t a = EdgeAt(START), b = EdgeAt(END);
          sec_off = uint32_t(a);
          sec_len = uint32_t(std::min<size_t>(std::max<size_t>(b - std::min(a, b), 2 * AUDIO_BLOCK_SAMPLES),
                                              size_t(MaxSecs() * AUDIO_SAMPLE_RATE_EXACT)));
          ex_pos[0] = 0; ex_pos[1] = sec_len;
          ex_valid = keep_exact = true;
        } else if (mode[FILE_] == 0 && clipped) {
          // MEM -> STRM: the edges back to where they are in the whole file
          ex_pos[0] = sec_off + EdgeAt(START); ex_pos[1] = sec_off + EdgeAt(END);
          ex_valid = keep_exact = true;
        }
      }
      applied_file = cur[FILE_];
      applied_mode = mode[FILE_];
    } break;
    case SPEED: {  // mode REV (0) plays backwards (streamed files: forward only)
      const float sp = cur[SPEED] * 0.01f * rate * SyncRatio();
      synced_at = tempo_bpm;
      synced_bpm = bpm;
      fx.SetSpeed(mode[SPEED] == DIR_REV ? -sp : sp);
      stream.SetSpeed(sp);
    } break;
    case LOOP: fx.SetLoop(cur[LOOP]); stream.SetLoop(cur[LOOP]); break;
    case START: case END:
      mode[p == START ? END : START] = mode[p];  // one zoom for both
      Window();
      break;
    case SNAP: Window(); break;
    case XFADE:  // up to 1 s (finer at the short end); CROSS or INOUT (a dip)
      fx.SetSeam(XfadeFrames());
      stream.SetSeam(XfadeFrames());
      fx.SetCrossfade(mode[XFADE] == 0);
      fx.SetDeferWindow(true);  // a new START / END waits for the loop's end
      stream.SetCrossfade(mode[XFADE] == 0);
      break;
    case LEVEL: fx.SetLevel(cur[LEVEL] * 0.01f); stream.SetLevel(cur[LEVEL] * 0.01f); break;
  }
}

// START / END: steps get finer with the zoom (1% at 1x .. ~0.02% at 64x)
FLASHMEM int PlayerDevice::Step(int p, int v, int dir) const {
  if (p == START || p == END) {
    // one pixel of the picture at the current zoom (down to one sample)
    const size_t n = Frames();
    if (!n) return v + dir * 100;
    const size_t span = ZoomSpan(mode[p]);
    const int64_t px = std::max<int64_t>(1, int64_t(span / 128));
    if (!ex_valid) { ex_pos[0] = EdgeAt(START); ex_pos[1] = EdgeAt(END); ex_valid = true; }
    // they never cross: closer than GAP, the one moved pushes the other
    constexpr int64_t GAP = 2 * AUDIO_BLOCK_SAMPLES;
    const int64_t nn = int64_t(n);
    const bool end = p == END;
    uint32_t& e = ex_pos[end];
    uint32_t& o = ex_pos[!end];
    e = uint32_t(end ? constrain(int64_t(e) + dir * px, std::min(GAP, nn), nn)
                     : constrain(int64_t(e) + dir * px, int64_t(0), std::max<int64_t>(0, nn - GAP)));
    const int64_t want = end ? std::max<int64_t>(0, int64_t(e) - GAP) : std::min(nn, int64_t(e) + GAP);
    if (end ? int64_t(o) > want : int64_t(o) < want) {
      o = uint32_t(want);
      auto* self = const_cast<PlayerDevice*>(this);
      const int q = end ? START : END;
      self->val[q] = int(uint64_t(o) * 10000 / n);
      self->Refresh(q);
    }
    return int(uint64_t(e) * 10000 / n);
  }
  return v + dir;
}

FLASHMEM void PlayerDevice::Format(int p, char* buf, size_t n) const {
  if (p == FILE_) {
    // the name is in the picture; here how it plays: STRM / MEM
    if (!nfiles) snprintf(buf, n, "none");
    else snprintf(buf, n, "%s", mode[FILE_] ? "MEM" : "STRM");
  } else if (p == START || p == END) {  // seconds into the file, to the ms
    const size_t fr = Frames();
    if (!fr) { snprintf(buf, n, "%d%%", val[p] / 100); return; }
    const uint32_t ms = uint32_t(uint64_t(EdgeAt(p)) * 1000 / uint64_t(AUDIO_SAMPLE_RATE_EXACT * rate));
    if (ms < 10000) snprintf(buf, n, "%lu.%03lu", ms / 1000, ms % 1000);
    else snprintf(buf, n, "%lu.%02lu", ms / 1000, (ms % 1000) / 10);
  } else if (p == SPEED) {
    snprintf(buf, n, "%d%%", val[p]);
  } else {
    Device::Format(p, buf, n);
  }
}

// START / END: their zoom
FLASHMEM void PlayerDevice::FormatMode(int p, char* buf, size_t n) const {
  if (p == START || p == END) {
    FormatZoom(mode[p], buf, n);
    return;
  }
  else Device::FormatMode(p, buf, n);
}

// A: play / stop; B: back to the start
// SYNC, the clock running and the file's tempo known: frames until the
// clock's next beat (-1: start now)
FLASHMEM int32_t PlayerDevice::BeatWait() const {
  if (mode[SPEED] != DIR_SYNC || !clock_running || bpm <= 0.0f || tempo_bpm <= 0.0f) return -1;
  float ph = clock_beat_ph + (micros() - clock_beat_us) * 1e-6f * tempo_bpm / 60.0f;
  ph -= floorf(ph);
  return int32_t((1.0f - ph) * 60.0f / tempo_bpm * AUDIO_SAMPLE_RATE_EXACT);
}

FLASHMEM void PlayerDevice::Action(int a) {
  // A while a start waits for its beat: never mind
  if (a == 0 && (stream.PlayWaiting() || fx.PressWaiting())) {
    stream.CancelPlay();
    fx.CancelPress();
    return;
  }
  const int32_t wait = a == 0 ? BeatWait() : -1;  // SYNC: on the next beat
  if (streaming) {
    if (a == 0 && stream.Playing()) stream.Stop();
    else if (a == 0 && wait >= 0) stream.PlayIn(wait);
    else if (a == 0) stream.Play();  // A: from the start of the loop
    else stream.Retrigger();         // B: back to it, without a click
    return;
  }
  const auto st = fx.state();
  if (a == 0) {
    if (st == AudioEffectReel::PLAY) fx.Stop();
    else if (st == AudioEffectReel::STOPPED && wait >= 0) fx.PressIn(wait);
    else if (st == AudioEffectReel::STOPPED) fx.Press();
  } else if (st == AudioEffectReel::PLAY || st == AudioEffectReel::STOPPED) {
    fx.Retrigger();
  }
}

// the sample as a waveform with the loop bracketed and the play head; with
// the cursor on START or END the picture zooms in around that point
FLASHMEM void PlayerDevice::DrawViz(int x, int y, int w, int h) {
  const int base = y + h - 1, cy = y + h / 2;
  if (!fx.ready()) { gfxPrint(x + 22, cy - 4, PSTR("out of memory")); return; }
  if (!SDcard_Ready) { gfxPrint(x + 34, cy - 4, PSTR("no SD card")); return; }
  if (!nfiles) { gfxPrint(x + 28, cy - 4, PSTR("no WAV found")); return; }
  const int fi = std::min<int>(cur[FILE_], nfiles - 1);
  if (loading || analysing) {
    char t[24];
    if (loading) snprintf(t, sizeof(t), "loading %d%%", frames ? int(got * 100 / frames) : 0);
    else snprintf(t, sizeof(t), "reading %d%%", frames ? int(done * 100 / frames) : 0);
    gfxPrint(x + 28, cy - 4, t);
    return;
  }
  const size_t n = Frames();
  // what part of the sample is on screen: zoomed around START / END while
  // the cursor is on them
  // the whole second page (START END XFADE SNAP) keeps the zoom, centred on
  // START or END, whichever the cursor was on last
  if (view_cursor == START || view_cursor == END) zoom_edge = view_cursor;
  const bool zoomed = view_cursor == START || view_cursor == END || view_cursor == XFADE || view_cursor == SNAP;
  const int zm = zoomed ? mode[START] : 0;
  {
    // the top line: the name, then the zoom (time across) and SD at the right
    int right = x + w;
    if (streaming) right -= 14;
    else if (clipped) right -= 20;
    char zt[8] = {};
    if (zm > 0) {
      FormatZoom(zm, zt, sizeof(zt));
      right -= 6 * int(strlen(zt)) + 2;
      gfxPrint(right + 2, y, zt);
    }
    char nm[24];
    ShortName(names[fi], (right - x) / 6 - 1, nm, sizeof(nm));
    gfxPrint(x, y, nm);
  }
  if (too_long) { char t[6]; snprintf(t, sizeof(t), "%ds", int(MaxSecs())); gfxPrint(x + w - 6 * int(strlen(t)), y, t); }
  else if (clipped) gfxPrint(x + w - 18, y, PSTR("MEM"));
  if (streaming) {  // streamed from the card; blinks if the card fell behind
    const bool late = stream.Underran();
    if (!late || (millis() / 200) % 2) gfxPrint(x + w - 12, y, late ? PSTR("!!") : PSTR("SD"));
  }
  if (!n) { gfxPrint(x + 28, cy - 4, PSTR("can't read")); return; }
  // zoomed: the point being set stays in the middle (down to a sample per
  // pixel); otherwise the START..END section with a little room, or the whole
  // file. Past the file's ends there is a dotted line.
  const size_t ea0 = EdgeAt(START), eb0 = EdgeAt(END);
  size_t span;
  int64_t v0;
  if (zoomed) {
    span = ZoomSpan(zm);
    v0 = int64_t(EdgeAt(zoom_edge)) - int64_t(span / 2);
  } else if (eb0 > ea0 && (ea0 > 0 || eb0 < n)) {
    const size_t sec = eb0 - ea0, room = sec / 16;
    span = std::max<size_t>(w, sec + 2 * room);
    v0 = int64_t(ea0) - int64_t(room);
  } else {
    span = std::max<size_t>(w, n);
    v0 = 0;
  }
  auto sx = [&](size_t pos) { return x + int((int64_t(pos) - v0) * (w - 1) / int64_t(span)); };
  const int wy = y + 9, wh = h - 9, wc = wy + wh / 2;
  if (span <= size_t(2 * w)) {
    // close in: the wave itself, a line through the samples (each sample a
    // dot when several pixels wide)
    static int16_t mono[2 * 128 + 2];
    const int64_t first = std::max<int64_t>(0, v0), last = std::min<int64_t>(int64_t(n), v0 + int64_t(span) + 1);
    const int got_n = last > first ? Samples(size_t(first), size_t(last - first), mono) : 0;
    int py = -1;
    for (int c = 0; c < w; c++) {
      const float fpos = float(v0) + float(c) * float(span) / float(w);
      if (fpos < 0.0f || fpos >= float(n) || !got_n) {
        if (c % 2 == 0) gfxPixel(x + c, wc);
        py = -1;
        continue;
      }
      const int i0 = int(fpos - float(first));
      const int i1 = std::min(i0 + 1, got_n - 1);
      const float fr = fpos - std::floor(fpos);
      const float v = (mono[std::min(i0, got_n - 1)] + fr * (mono[i1] - mono[std::min(i0, got_n - 1)])) / 32768.0f;
      const int yy = wc - int(v * (wh / 2 - 1));
      if (py >= 0) gfxLine(x + c - 1, py, x + c, yy); else gfxPixel(x + c, yy);
      py = yy;
    }
    if (span < size_t(w / 2)) {  // samples wide apart: mark each one
      for (int64_t i = first; i < last; i++) {
        const int c = int((i - v0) * (w - 1) / int64_t(span));
        if (c >= 0 && c < w) gfxRect(x + c - 1, wc - int(mono[i - first] / 32768.0f * (wh / 2 - 1)) - 1, 3, 3);
      }
    }
  } else {
    // the wave's real shape (lowest to highest per column) where it has been
    // read; the level bars where not yet (or, streaming, more than ~45 s
    // across: too much to read from the card)
    const bool can = !streaming || span <= size_t(AUDIO_SAMPLE_RATE_EXACT * rate * 45.0f);
    if (can) FillWave(v0, span, w, fi);
    for (int c = 0; c < w; c++) {
      const int64_t a = v0 + int64_t(c) * int64_t(span) / w, b = v0 + int64_t(c + 1) * int64_t(span) / w;
      if (b <= 0 || a >= int64_t(n)) {
        if (c % 2 == 0) gfxPixel(x + c, wc);
        continue;
      }
      if (can && c >= wv_fa && c < wv_fb && wv_lo[c] <= wv_hi[c]) {
        const int y0 = wc - int(wv_hi[c] / 32768.0f * (wh / 2 - 1)), y1 = wc - int(wv_lo[c] / 32768.0f * (wh / 2 - 1));
        gfxLine(x + c, y0, x + c, y1);
        continue;
      }
      if (can) {  // not read yet: just the middle line, no level bars in between
        if (c % 2 == 0) gfxPixel(x + c, wc);
        continue;
      }
      const size_t ca = size_t(std::max<int64_t>(0, a)), cb = size_t(std::min<int64_t>(int64_t(n), b));
      const int pk = PeakAt(ca, std::max(ca + 1, cb));
      const int amp = int(sqrtf(pk / 32768.0f) * (wh / 2 - 1));
      gfxLine(x + c, wc - amp, x + c, wc + amp);
    }
  }
  // the loop's edges and the head
  // where they are set (a new loop waits for the old one to come round,
  // the lines show the new one at once); SNAP shown for files in memory
  size_t ea = ea0, eb = eb0;
  if (!streaming && cur[SNAP]) { ea = SnapQuiet(ea); eb = SnapQuiet(eb); }
  for (const size_t e : {ea, eb}) {
    const int ex = sx(e);
    if (ex < x || ex >= x + w) continue;
    graphics.clearRect(std::max(x, ex - 1), wy, std::min(3, x + w - std::max(x, ex - 1)), wh);
    gfxLine(ex, wy, ex, base);
  }
  // the seam: the part fading out (a falling line) and the part fading in
  // (a rising one). CROSS: centred on END and START (an X over each line),
  // or, with no room around them, before END / after START; INOUT: the dip
  if (eb > ea && cur[LOOP]) {
    size_t h = 0, f = 0;
    if (mode[XFADE] == 0) {
      if (streaming) { h = stream.Half(ea, eb); f = stream.XfLen(ea, eb); }
      else AudioEffectReel::XfPlan(XfadeFrames(), ea, eb - ea, n, h, f);
    } else {
      f = std::min<size_t>(size_t(XfadeFrames()), (eb - ea) / 4);
    }
    if (h > 1 || f > 1) {
      auto ramp = [&](size_t f0, size_t f1, bool up) {
        const int x0 = sx(f0), x1 = sx(f1);
        if (x1 <= x0) return;
        for (int px = std::max(x, x0); px <= std::min(x + w - 1, x1); px += 2) {
          const float t = float(px - x0) / float(x1 - x0);
          const int yy = wy + int((up ? 1.0f - t : t) * (wh - 1));
          gfxPixel(px, yy);
        }
      };
      if (h > 1) {
        ramp(eb - h, eb + h, false);  // out, centred on END
        ramp(ea - h, ea + h, true);   // in, centred on START
      } else {
        ramp(eb - f, eb, false);      // out, into END
        ramp(ea, ea + f, true);       // in, after START
      }
    }
  }
  if (Playing()) {
    const int hx = sx(size_t((streaming ? stream.HeadPos() : fx.HeadPos()) * n));
    if (hx >= x && hx < x + w) gfxInvert(hx, wy, 1, wh);
  }
}

// the header: playing, the head's place in the whole file (m:ss.t);
// stopped, the tempo as heard (with SPEED; ... while it is worked out)
FLASHMEM bool PlayerDevice::HeaderInfo(char* buf, size_t n) const {
  if (!active || !Frames() || loading || analysing) return false;
  if (Playing() && !too_long) {
    const uint32_t ds = uint32_t(HeadFrame() * 10.0f / FileRate());
    snprintf(buf, n, "%lu:%02lu.%lu", (unsigned long)(ds / 600), (unsigned long)(ds / 10 % 60), (unsigned long)(ds % 10));
  } else if (bpm_got >= 0) {
    snprintf(buf, n, "...BPM");
  } else if (bpm > 0.0f) {
    const int tb = int(bpm * SyncRatio() * cur[SPEED] + 5.0f) / 10;  // tenths
    snprintf(buf, n, "%d.%dBPM", tb / 10, tb % 10);
  } else {
    snprintf(buf, n, "-BPM");
  }
  return true;
}

// (L+R)/2 of a few frames, for the close-in picture: from memory, or from
// the card when streaming
FLASHMEM int PlayerDevice::Samples(size_t a, size_t count, int16_t* mono) const {
  count = std::min<size_t>(count, 2 * 128 + 2);
  if (!streaming) {
    for (size_t i = 0; i < count; i++) mono[i] = int16_t((fx.Data(0)[a + i] + fx.Data(1)[a + i]) / 2);
    return int(count);
  }
  static int16_t l[2 * 128 + 2], r[2 * 128 + 2];
  auto* self = const_cast<PlayerDevice*>(this);
  self->lf.seek(data_off + a * size_t(bytes) * chans);
  const size_t rd = self->ReadFrames(l, r, count);
  for (size_t i = 0; i < rd; i++) mono[i] = int16_t((l[i] + r[i]) / 2);
  return int(rd);
}

// fills wv_lo / wv_hi for this view, column by column (span / w frames
// each): from memory all at once, from the card ~16k frames per call. When
// the view only slides (START / END moved) the columns already read move
// along and only the new ones are read; a new zoom or file starts over.
// Columns wv_fa..wv_fb-1 are valid.
FLASHMEM bool PlayerDevice::FillWave(int64_t v0, size_t span, int w, int fi) const {
  const int64_t n = int64_t(Frames());
  const int64_t cw = std::max<int64_t>(1, int64_t(span) / w);
  const int64_t shift = wv_any ? v0 - wv_v0 : 0;
  if (!wv_any || span != wv_span || fi != wv_file || shift % cw || llabs(shift / cw) >= w) {
    wv_span = span; wv_file = fi; wv_fa = wv_fb = 0; wv_any = true;
  } else if (shift) {
    const int k = int(shift / cw);  // columns the picture moves left
    if (k > 0) {
      memmove(wv_lo, wv_lo + k, (w - k) * sizeof(int16_t));
      memmove(wv_hi, wv_hi + k, (w - k) * sizeof(int16_t));
    } else {
      memmove(wv_lo - k, wv_lo, (w + k) * sizeof(int16_t));
      memmove(wv_hi - k, wv_hi, (w + k) * sizeof(int16_t));
    }
    wv_fa = constrain(wv_fa - k, 0, w);
    wv_fb = constrain(wv_fb - k, 0, w);
    if (wv_fa >= wv_fb) wv_fa = wv_fb = 0;
  }
  wv_v0 = v0;
  // one column: its lowest / highest (empty past the file's ends)
  static int16_t l[256], r[256];
  auto* self = const_cast<PlayerDevice*>(this);
  auto fill = [&](int c) {
    const int64_t a = std::max<int64_t>(0, v0 + c * cw), b = std::min<int64_t>(n, v0 + (c + 1) * cw);
    int mn = 32767, mx = -32768;
    if (!streaming) {
      const int16_t* dl = fx.Data(0);
      const int16_t* dr = fx.Data(1);
      for (int64_t i = a; i < b; i++) {
        const int v = (dl[i] + dr[i]) / 2;
        mn = std::min(mn, v); mx = std::max(mx, v);
      }
    } else if (b > a) {
      self->lf.seek(data_off + size_t(a) * size_t(bytes) * chans);
      for (int64_t i = a; i < b;) {
        const size_t rd = self->ReadFrames(l, r, size_t(std::min<int64_t>(256, b - i)));
        if (!rd) break;
        for (size_t k = 0; k < rd; k++) {
          const int v = (l[k] + r[k]) / 2;
          mn = std::min(mn, v); mx = std::max(mx, v);
        }
        i += rd;
      }
    }
    wv_lo[c] = int16_t(mn); wv_hi[c] = int16_t(mx);
    return b > a ? b - a : 0;
  };
  // grow the valid stretch: to the right first, then to the left
  int64_t budget = streaming ? 16384 : INT64_MAX;
  while (budget > 0 && (wv_fb < w || wv_fa > 0)) {
    if (wv_fa == wv_fb) { wv_fa = wv_fb = 0; }
    if (wv_fb < w) budget -= std::max<int64_t>(1, fill(wv_fb++));
    else budget -= std::max<int64_t>(1, fill(--wv_fa));
  }
  return wv_fa == 0 && wv_fb == w;
}

// loudest between two frames, from memory or (streaming) the level index
FLASHMEM int PlayerDevice::PeakAt(size_t a, size_t b) const {
  if (!streaming) return fx.Peak(a, b, std::max<size_t>(1, (b - a) / 24));
  int pk = 0;
  for (size_t e = a / IDX_FRAMES; e <= (b - 1) / IDX_FRAMES && e < idx_n; e++) pk = std::max<int>(pk, idx[e]);
  return pk << 7;
}

FLASHMEM void PlayerDevice::DrawMini(int x, int y, int w, int h) {
  const int cy = y + h / 2;
  if (streaming) {
    for (int i = 0; i < w; i++) {
      const size_t a = size_t(i) * frames / w, b = size_t(i + 1) * frames / w;
      const int amp = int(sqrtf(PeakAt(a, std::max(a + 1, b)) / 32768.0f) * (h / 2));
      gfxLine(x + i, cy - amp, x + i, cy + amp);
    }
    if (stream.Playing()) gfxInvert(constrain(x + int(stream.HeadPos() * (w - 1)), x, x + w - 1), y, 1, h);
    return;
  }
  if (!fx.Length()) {  // nothing loaded: a play arrow
    const int cx = x + w / 2, r = h / 2 - 1;
    gfxLine(cx - r / 2 - 1, cy - r, cx + r / 2 + 2, cy);
    gfxLine(cx + r / 2 + 2, cy, cx - r / 2 - 1, cy + r);
    gfxLine(cx - r / 2 - 1, cy - r, cx - r / 2 - 1, cy + r);
    return;
  }
  constexpr int HIST = AudioEffectReel::HIST;
  uint8_t hist[HIST];
  fx.ReadHistory(hist);
  for (int i = 0; i < w; i++) {
    const int a = int(sqrtf(hist[i * HIST / w] / 255.0f) * (h / 2));
    gfxLine(x + i, cy - a, x + i, cy + a);
  }
  if (fx.state() == AudioEffectReel::PLAY)
    gfxInvert(constrain(x + int(fx.HeadPos() * (w - 1)), x, x + w - 1), y, 1, h);
}

/////////////////////////////////////////////////////////////////
// ECHO: hybrid delay + reverb
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef ECHO_DEF = {
  "ECHO", "ECO", 8, {"FRZ", "CLR"},
  {
    // page 1: the delay, page 2: the reverb and TONE
    {"LEVEL", 0, 100, 40, UNI, 0, 2, 0, {}, {"MIX", "SEND"}},  // the delay's; mode: crossfade / added
    {"TIME", 20, 6000, 450, UNI, 0, 4, 0, {}, {"FREE", "STR", "TRI", "DOT"}},
    {"FDBK", 0, 100, 45, UNI},
    {"SPRD", 0, 100, 40, UNI, 0, 3, 1, {}, {"PHASE", "PING", "DIFF"}},
    // the reverb, after the Tegeler Raummaschine: mix, pre-delay, room
    // (size and decay together), walls (soft: dark and dense .. hard:
    // bright with audible reflections; also the delay's tone)
    {"LEVEL", 0, 100, 30, UNI, 0, 2, 0, {}, {"MIX", "SEND"}},
    {"PRE", 0, 250, 20, UNI},
    {"ROOM", 0, 100, 50, UNI},
    {"WALL", -50, 50, -10, BI},
  },
  {"DELAY", "REVRB"},
};

class EchoDevice : public Device {
public:
  enum { DELAY, TIME, FEEDBACK, SPREAD, REVERB, PRE, ROOM, WALL };
  // ROOM: decay 0.4 .. 16 s
  float DecaySecs() const { return 0.4f * powf(40.0f, cur[ROOM] * 0.01f); }
  enum { PHASE, PING, DIFFUSE };
  static constexpr int TAPS = AudioEffectEcho::TAPS;
  static constexpr float MAX_SECS = 6.0f;

  EchoDevice();
  AudioStream* in() override { return &echo; }
  AudioStream* out() override { return &echo; }
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
  uint32_t TailMs() const override;
  void Parked() override { if (mem_ok) echo.Reset(); }  // no old tail on return

private:
  AudioEffectEcho echo;
  bool mem_ok = false;
  float tap_secs[2][TAPS] = {};
  Meter tap_m[2][TAPS], tail_m;

  static constexpr float TAP_POS[2][TAPS] = {
    {0.25f, 0.50f, 0.75f, 1.0f},
    {0.25f, 0.50f, 0.75f, 1.0f},  // same grid: SPRD makes the stereo spread
  };
  void SetTimes();
  void Poll();
};

constexpr size_t ECHO_DELAY_SAMPLES
  = static_cast<size_t>(EchoDevice::MAX_SECS * AUDIO_SAMPLE_RATE_EXACT) + 512;

FLASHMEM EchoDevice::EchoDevice() : Device(ECHO_DEF), echo(ECHO_DELAY_SAMPLES) {}

// called while the stream is not patched in, so it isn't updating
FLASHMEM bool EchoDevice::Activate() {
  if (active) return mem_ok;
  mem_ok = echo.Acquire();
  ApplyAll();
  active = true;
  return mem_ok;
}

// called after the stream has been unpatched
FLASHMEM void EchoDevice::Deactivate() {
  if (!active) return;
  echo.Release();
  active = false;
}

FLASHMEM uint32_t Device::TailMs() const { return 40; }

// the reverb's decay plus a few rounds of echoes; frozen: never
FLASHMEM uint32_t EchoDevice::TailMs() const {
  if (echo.frozen()) return UINT32_MAX;
  const float decay = DecaySecs() + cur[PRE] * 0.001f;
  const float echoes = tap_secs[0][TAPS - 1] * (1.0f + 8.0f * cur[FEEDBACK] * 0.01f);
  return uint32_t((decay + echoes) * 1000.0f) + 200;
}

FLASHMEM void EchoDevice::CpuMax(float parts[2]) {
  parts[0] = echo.processorUsageMax();
  parts[1] = 0.0f;
  echo.processorUsageMaxReset();
}

FLASHMEM void EchoDevice::SetTimes() {
  const float secs = mode[TIME] == SYNC_FREE ? cur[TIME] * 0.001f
    : DivSeconds(DivIndex(cur[TIME], 20, 6000), mode[TIME]);
  const float phase = mode[SPREAD] == PHASE ? 1.0f + 0.3f * cur[SPREAD] * 0.01f : 1.0f;
  for (int c = 0; c < 2; c++) {
    for (int t = 0; t < TAPS; t++) {
      float s = secs * TAP_POS[c][t] * (c ? phase : 1.0f);
      s = std::min(s, MAX_SECS - 0.01f);
      tap_secs[c][t] = s;
      echo.tap(c, t, int(s * AUDIO_SAMPLE_RATE_EXACT));
    }
  }
}

FLASHMEM void EchoDevice::Apply(int p) {
  const float v = cur[p] * 0.01f;
  switch (p) {
    case DELAY:    echo.delay_mix(v); echo.delay_send(mode[DELAY] == 1); break;
    case TIME:     SetTimes(); break;
    case REVERB:   echo.reverb_mix(v); echo.reverb_send(mode[REVERB] == 1); break;
    case PRE:      echo.pre_delay(cur[PRE]); break;
    case ROOM:     echo.size(v); echo.decay(DecaySecs()); break;
    case FEEDBACK: echo.feedback(v); break;
    case SPREAD:
      echo.pingpong(mode[SPREAD] == PING ? v : 0.0f);
      echo.diffuse(mode[SPREAD] == DIFFUSE ? v : 0.0f);
      SetTimes();
      break;
    case WALL:  // soft walls: dark and dense; hard: bright, sparse reflections
      echo.tone(cur[WALL] * 0.02f);
      echo.density(0.5f - cur[WALL] * 0.01f);
      break;
  }
}

FLASHMEM int EchoDevice::Step(int p, int v, int dir) const {
  if (p == TIME && mode[TIME] != SYNC_FREE)
    return DivValue(DivIndex(v, 20, 6000) + dir, 20, 6000);
  if (p == TIME) return v + dir * std::max(1, v / 32); // ~3% per detent
  return v + dir;
}

FLASHMEM void EchoDevice::Format(int p, char* buf, size_t n) const {
  if (p == TIME && mode[TIME] != SYNC_FREE) {
    FormatDiv(DivIndex(val[TIME], 20, 6000), mode[TIME], buf, n);
  } else if (p == TIME) {
    const int ms = val[TIME];
    if (ms < 1000) snprintf(buf, n, "%dms", ms);
    else snprintf(buf, n, "%d.%02ds", ms / 1000, (ms % 1000) / 10);
  } else if (p == PRE) {
    snprintf(buf, n, "%dms", val[PRE]);
  } else if (p == ROOM) {  // shown as its decay time
    const int ds = int(4.0f * powf(40.0f, val[ROOM] * 0.01f) + 0.5f);  // tenths
    snprintf(buf, n, "%d.%ds", ds / 10, ds % 10);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void EchoDevice::Action(int a) {
  if (a == 0) {
    echo.set_freeze(!echo.frozen());
  } else if (a == 1) {
    // CLEAR: empty the delay and reverb buffers
    if (!mem_ok) return;
    AudioNoInterrupts();
    echo.Reset();
    AudioInterrupts();
  }
}

FLASHMEM bool EchoDevice::ActionLit(int a) const { return a == 0 && echo.frozen(); }

FLASHMEM void EchoDevice::Poll() {
  AudioEffectEcho::Viz v;
  echo.ReadViz(v);
  for (int c = 0; c < 2; c++)
    for (int t = 0; t < TAPS; t++) tap_m[c][t].Hit(v.tap[c][t]);
  tail_m.Hit(v.tail, 0.93f);
}

FLASHMEM void EchoDevice::DrawViz(int x, int y, int w, int h) {
  Poll();
  if (!mem_ok) {
    gfxPrint(x + 22, y + h / 2 - 4, PSTR("out of memory"));
    return;
  }
  const int base = y + h - 1;
  gfxDottedLine(x, base, x + w - 1, base, 2);
  if (view_page == 0) {
    // page 1, the delay: a fixed time line from the left edge to 6 s (the
    // longest delay). Filled bars: the 4 echoes of the left side, as tall as
    // DELAY; with SPRD up an outlined bar next to each shows the right side.
    // Thin lines further on: the repeats through FDBK, each round shorter
    // by the FDBK amount; with SPRD up the right side's repeats next to them.
    if (cur[DELAY] > 0) {
      const int bh = std::max(1, (h - 2) * cur[DELAY] / 100);
      const int tw = w - 8;
      auto px = [&](float sec) { return x + int(constrain(sec / MAX_SECS, 0.0f, 1.0f) * tw); };
      const float period = tap_secs[0][TAPS - 1], period_r = tap_secs[1][TAPS - 1];
      const float fb = std::min(1.0f, cur[FEEDBACK] * 0.01f);
      for (int t = 0; t < TAPS; t++) {
        const int lx = px(tap_secs[0][t]);
        gfxRect(lx, base - bh, 2, bh);
        if (cur[SPREAD] > 0) gfxFrame(std::max(lx + 3, px(tap_secs[1][t])), base - bh, 2, bh);
      }
      // repeats: the whole pattern again after each period
      float g = fb;
      for (int round = 1; round < 16 && period > 0.0f; round++, g *= fb) {
        const int rh = int(bh * g);
        if (rh < 1) break;
        bool any = false;
        for (int t = 0; t < TAPS; t++) {
          const float sec = round * period + tap_secs[0][t];
          if (sec > MAX_SECS) break;
          const int lx = px(sec);
          gfxLine(lx, base - rh + 1, lx, base);
          if (cur[SPREAD] > 0) {
            const int rx = std::max(lx + 2, px(round * period_r + tap_secs[1][t]));
            gfxDottedLine(rx, base - rh + 1, rx, base, 2);
          }
          any = true;
        }
        if (!any) break;
      }
    }
  } else {
    // page 2, the reverb: two walls that ROOM moves apart from 30% to the
    // edges, and between them a wedge fading out from the middle to both
    // sides, as tall as LEVEL and reaching towards the walls with ROOM,
    // falling off like a real tail (quickly first, then a long low end);
    // PRE leaves a gap in the middle. WALL fills it in: sparse when soft,
    // solid when hard; and it makes the walls crisp (hard) or diffuse (soft).
    static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    const int cx = x + w / 2, hw = w / 2 - 1;
    const int wall = hw * (30 + 70 * cur[ROOM] / 100) / 100;
    const int gap = (wall - 2) * cur[PRE] / 250 / 2;  // the pre-delay
    const int th = int((h - 2) * sqrtf(cur[REVERB] * 0.01f));  // shows more early on (50% = 70% high)
    const int len = std::max(2, (wall - 1 - gap) * (20 + 80 * cur[ROOM] / 100) / 100);
    const int fill = 3 + (cur[WALL] + 50) * 13 / 100;  // 3..16 of 16
    for (int i = gap; i < gap + len; i++) {
      constexpr float k = 4.0f, end = 0.0183156f;  // e^-4
      const int hh = int(th * (expf(-k * (i - gap) / len) - end) / (1.0f - end) + 0.5f);
      for (int py = base - hh + 1; py <= base; py++) {
        const bool edge = py == base - hh + 1;  // keep the outline readable
        if (edge || bayer[py & 3][(cx + i) & 3] < fill) gfxPixel(cx + i, py);
        if (i && (edge || bayer[py & 3][(cx - i) & 3] < fill)) gfxPixel(cx - i, py);
      }
    }
    // the walls: hard (WALL +50) a crisp line; softer, it spreads up to 2
    // pixels each side, thinning out from the middle - a diffuse surface.
    // A fixed threshold per screen place, so nothing flickers while WALL
    // turns.
    const float soft = (50 - cur[WALL]) * 0.01f;  // 0 hard .. 1 soft
    const int spread = int(soft * 2.0f + 0.5f);
    for (int wx : {cx - wall, cx + wall}) {
      for (int d = -spread; d <= spread; d++) {
        const int px = wx + d;
        if (px < x || px >= x + w) continue;
        const float weight = spread ? (1.0f - float(abs(d)) / (spread + 1)) * (1.0f - 0.35f * soft) : 1.0f;
        for (int py = y; py <= base; py++) {
          const float t = px * 0.7548777f + py * 0.5698403f;
          if (t - floorf(t) < weight) gfxPixel(px, py);
        }
      }
    }
  }
  if (echo.frozen()) {
    gfxPrint(x + w - 25, y + 1, PSTR("FRZ"));
    gfxInvert(x + w - 26, y, 20, 9);
  }
}

// the delay page in small: the echoes on a 1 s line (longer times fill the
// width), as tall as the delay LEVEL, and a dot running through them in
// real time
FLASHMEM void EchoDevice::DrawMini(int x, int y, int w, int h) {
  Poll();
  const int base = y + h - 1;
  const int bh = std::max(1, h * cur[DELAY] / 100);
  const float period = tap_secs[0][TAPS - 1], span = std::max(1.0f, period);
  auto px = [&](float sec) { return x + int(constrain(sec / span, 0.0f, 1.0f) * (w - 1)); };
  for (int t = 0; t < TAPS; t++) gfxLine(px(tap_secs[0][t]), base - bh + 1, px(tap_secs[0][t]), base);
  if (period > 0.0f) {
    const float at = fmodf(millis() * 0.001f, period);
    gfxPixel(px(at), base);
  }
  if (echo.frozen()) gfxInvert(x, y, w, h);
}

/////////////////////////////////////////////////////////////////
// TEXTURE: fold, drive, crush, tilt, envelope-following noise
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef DIRT_DEF = {
  "TEXTURE", "TEX", 8, {"", ""},
  {
    {"DRIVE", 0, 100, 0, UNI},
    {"FOLD", 0, 100, 0, UNI},
    {"CRSH", 0, 100, 0, UNI, 0, 3, 0, {}, {"BIT", "REDUX", "BOTH"}},
    {"TILT", -50, 50, 0, BI, 0, 3, 1, {}, {"LOW", "MID", "HIGH"}},
    {"NOIS", 0, 100, 0, UNI, 0, 3, 1, {}, {"WHITE", "PINK", "DUST"}},
    {"N.DCY", 0, 100, 40, UNI},
    {"N.TON", 0, 100, 50, UNI},
    {"WET", 0, 100, 100, UNI},
  }
};

class DirtDevice : public Device {
public:
  enum { DRIVE, FOLD, CRUSH, TILT, NOISE, NDECAY, NTONE, WET };
  DirtDevice() : Device(DIRT_DEF) {}
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
  AudioEffectDirt fx;
  Meter in_m, out_m, noise_m;
  void Poll();
  // the bit steps for the picture (levels per side, 0: none): fewer all
  // the way up, and coarser than the sound's so they show at this size
  // REDUX and BOTH: how much of the line's pixels stay (1 at 0 .. about
  // half at 100, so the other settings still show)
  float RedPixels() const {
    const float crs = cur[CRUSH] * 0.01f;
    if (crs <= 0.0f || mode[CRUSH] == AudioEffectDirt::CRUSH_BIT) return 1.0f;
    return 1.0f - 0.475f * crs;
  }
  // a line through ys (one height per column from x0), every pixel of it
  // (steep parts too), keeping `keep` of them: each place on the screen has
  // its own fixed threshold from an even 2D pattern, so turning CRUSH only
  // takes pixels away (or brings the same ones back), none move
  static void ThinLine(int x0, const int* ys, int n, float keep) {
    for (int i = 0; i < n; i++) {
      const int a = i ? ys[i - 1] : ys[0], b = ys[i];
      const int step = b > a ? 1 : -1;
      for (int yv = (i && a != b) ? a + step : b;; yv += step) {
        const float t = (x0 + i) * 0.7548777f + yv * 0.5698403f;
        if (t - floorf(t) < keep) gfxPixel(x0 + i, yv);
        if (yv == b) break;
      }
    }
  }
  float PictureLevels() const {
    const float crs = cur[CRUSH] * 0.01f;
    if (crs <= 0.0f || mode[CRUSH] == AudioEffectDirt::CRUSH_REDUX) return 0.0f;
    // coarse early: 14 steps a side at the start (already visible at this
    // size) down to 2 at 100; whole steps, so the top one is the sine's top
    // (with 1.4 the line was cut off and looked folded)
    return roundf(2.0f + 12.0f * (1.0f - crs) * (1.0f - crs));
  }
  float NoiseDecaySecs() const { return 0.01f * powf(200.0f, cur[NDECAY] * 0.01f); } // 10 ms .. 2 s
};

FLASHMEM void DirtDevice::Apply(int p) {
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

FLASHMEM void DirtDevice::Format(int p, char* buf, size_t n) const {
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

FLASHMEM void DirtDevice::Poll() {
  AudioEffectDirt::Viz v;
  fx.ReadViz(v);
  in_m.Hit(v.in);
  out_m.Hit(v.out);
  noise_m.Hit(v.noise, 0.8f);
}

FLASHMEM void DirtDevice::DrawViz(int x, int y, int w, int h) {
  Poll();
  // only the section of the setting under the cursor gets an outline
  int sect;
  switch (view_cursor) {
    case DRIVE: case FOLD: case CRUSH: sect = 0; break;
    case TILT: sect = 1; break;
    case NOISE: case NDECAY: case NTONE: sect = 2; break;
    default: sect = 3; break; // WET: the level meters
  }
  // a sine over time, like the fold diagrams: the box's top and bottom are
  // the limits. Dotted: the input boosted by FOLD (where it is still inside
  // the box). Solid: what comes out - it folds back from the limits, then
  // DRIVE rounds it and CRUSH adds steps. One whole cycle, standing still.
  const int bs = 52, half = h / 2 - 1, cyw = y + h / 2, lim = half;
  if (sect == 0) gfxFrame(x, y, bs, h);
  const float drv = cur[DRIVE] * 0.01f, fld = cur[FOLD] * 0.01f;
  const float levels = PictureLevels();
  // BIT: the steps; REDUX: the whole line losing pixels as CRUSH goes up;
  // BOTH: the steps losing pixels
  const float keep = RedPixels();
  const float boost = 1.0f + 6.0f * fld * std::min(1.0f, fld * 4.0f);
  int ys[128];
  const int nc = std::min(128, bs - 2);
  for (int i = 1; i <= nc; i++) {
    const float ph = 2.0f * float(M_PI) * (i - 1) / (bs - 3);
    const float in = sinf(ph);
    const int gy = int(in * boost * lim);
    // dotted: with DRIVE up, how far it pushes the sound out, drawn the
    // other way - from the line inwards (on the line at 0); with DRIVE and
    // CRUSH at 0, the input as FOLD boosts it
    const float folded = AudioEffectDirt::Fold(in, fld);
    if (drv > 0.0f) {
      const float d = 2.0f * folded - AudioEffectDirt::Shape(folded, drv);
      if (i & 1) gfxPixel(x + i, cyw - int(constrain(d, -1.0f, 1.0f) * lim));
    } else if (cur[CRUSH] <= 0 && abs(gy) < half && (i & 1)) {
      gfxPixel(x + i, cyw - gy);
    }
    // solid: the sine with FOLD and CRUSH (not DRIVE - that is the dots)
    float yo = folded;
    if (levels > 0.0f) yo = roundf(constrain(yo, -1.0f, 1.0f) * levels) / levels;
    ys[i - 1] = cyw - int(constrain(yo, -1.0f, 1.0f) * lim);
  }
  ThinLine(x + 1, ys, nc, keep);

  // tilt as a frequency response, 20 Hz .. 20 kHz on a log scale: it turns
  // around the crossover (dot + frequency underneath), +-12 dB at the edges
  static const float xover_hz[3] = {80.0f, 650.0f, 2500.0f};
  static const char* const xover_name[3] = {"80", "650", "2k5"};
  const int tx = x + bs + 4, tw = 30, ch = h - 10, cy = y + ch / 2;
  if (sect == 1) gfxFrame(tx - 2, y, tw + 4, h);
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
  const int nx = tx + tw + 5, nw = gx - 3 - nx, nh = h - 10;
  if (sect == 2) gfxFrame(nx - 1, y, nw + 2, h);
  const float lvl = noise_m.v;
  switch (mode[NOISE]) {
    case AudioEffectDirt::NOISE_WHITE: {  // fine, even
      const int dots = int(lvl * 120.0f);
      for (int i = 0; i < dots; i++) gfxPixel(nx + random(nw), y + 1 + random(nh));
    } break;
    case AudioEffectDirt::NOISE_PINK: {   // chunkier, heavier at the bottom
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
  gfxPrint(nx + (nw - 18) / 2, y + h - 9, noise_name[mode[NOISE] % 3]);

  // input and output level
  if (sect == 3) gfxFrame(gx - 2, y, x + w - gx + 2, h);
  gfxFrame(gx, y, 6, h);
  DrawVMeter(gx + 1, y + 1, 4, h - 2, in_m.v);
  gfxFrame(gx + 7, y, 6, h);
  DrawVMeter(gx + 8, y + 1, 4, h - 2, out_m.v);
}

// the fold picture in small, standing still like on the page
FLASHMEM void DirtDevice::DrawMini(int x, int y, int w, int h) {
  Poll();
  const float drv = cur[DRIVE] * 0.01f, fld = cur[FOLD] * 0.01f;
  const float levels = PictureLevels();
  const float keep = RedPixels();
  const int cy = y + h / 2, lim = h / 2;
  int ys[128];
  const int nc = std::min(128, w);
  for (int i = 0; i < nc; i++) {
    const float in = sinf(2.0f * float(M_PI) * i / (w - 1));
    float yo = AudioEffectDirt::Shape(AudioEffectDirt::Fold(in, fld), drv);
    if (levels > 0.0f) yo = roundf(constrain(yo, -1.0f, 1.0f) * levels) / levels;
    ys[i] = cy - int(constrain(yo, -1.0f, 1.0f) * lim);
  }
  ThinLine(x, ys, nc, keep);
}

/////////////////////////////////////////////////////////////////
// RESONATOR: 48-band resonant filter bank
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef RESONATOR_DEF = {
  "RESONATOR", "RES", 8, {"", ""},
  {
    {"CUT", 0, 100, 70, UNI},
    {"SLOPE", -50, 50, -50, BI},
    {"RESO", 0, 100, 20, UNI},
    {"DCAY", 0, 100, 50, UNI, 0, 2, 0, {}, {"SUST", "CHOKE"}},  // Q 4 .. 1000: from the middle up, the bands ring as pitches
    {"PITCH", -24, 24, 0, BI},
    {"TONE", -50, 50, 0, BI},
    {"SCALE", 0, 3, 0, LIST, 4, 0, 0, {"CHROM", "MAJOR", "MINOR", "PENTA"}},
    {"WET", 0, 100, 100, UNI},
  }
};

class ResonatorDevice : public Device {
public:
  enum { CUT, SLOPE, RESO, DECAY, PITCH, TONE, SCALE, WET };
  static constexpr int BANDS = AudioEffectResonator::BANDS;
  ResonatorDevice() : Device(RESONATOR_DEF) {}
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  bool Activate() override { fx.Reset(); active = true; return true; }
  void Apply(int p) override { Rebuild(); }
  void Format(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
private:
  AudioEffectResonator fx;
  float band_m[BANDS] = {};
  Meter out_m;
  void Rebuild();
  float Base(float k) const;
  float Bump(float k) const;
  float View(float k) const;
  static float QOf(int v) { return 4.0f * powf(250.0f, v * 0.01f); }  // 4 .. 1000
  float Q() const { return QOf(cur[DECAY]); }
};

// band k sits 2 semitones above band k-1, from C1, moved by PITCH and
// snapped to SCALE; its gain follows CUT / SLOPE (12 dB per octave sides),
// a RESO bump at the cutoff and the TONE tilt
FLASHMEM void ResonatorDevice::Rebuild() {
  static const uint16_t scale_masks[4] = {
    0xFFF,                                                   // chromatic
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 11), // major
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 8) | (1 << 10), // minor
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9),    // pentatonic
  };
  const uint16_t mask = scale_masks[constrain(cur[SCALE], 0, 3)];
  const float q = Q();
  // a narrow band lets through less of the sound (its share goes with 1/Q):
  // made up with sqrt(Q), so ringing bands are as loud as wide ones (the
  // engine limits the sum softly)
  const float level = 0.5f * sqrtf(q / 4.0f);
  float f[BANDS], g[BANDS];
  for (int k = 0; k < BANDS; k++) {
    int note = 24 + 2 * k + cur[PITCH];  // MIDI note, C1 = 24
    for (int d = 0; d < 6; d++) {        // nearest note in the scale
      if (mask & (1 << (((note - d) % 12 + 12) % 12))) { note -= d; break; }
      if (mask & (1 << (((note + d) % 12 + 12) % 12))) { note += d; break; }
    }
    const float hz = 440.0f * powf(2.0f, (note - 69) / 12.0f);
    f[k] = hz < 8000.0f ? 2.0f * sinf(float(M_PI) * hz / AUDIO_SAMPLE_RATE_EXACT) : 0.0f;

    const float gk = Base(k) * (1.0f + cur[RESO] * 0.06f * Bump(k))
        * powf(4.0f, cur[TONE] * 0.02f * (k - BANDS / 2) / float(BANDS / 2));  // +-12 dB at the ends
    g[k] = gk * level;
  }
  fx.SetBank(f, g, 1.0f / q, mode[DECAY] == 1, cur[WET] * 0.01f);
}

// gain of a band (fractional k for the picture) from CUT / SLOPE (12 dB per
// octave sides, 6 bands an octave), before RESO and TONE
FLASHMEM float ResonatorDevice::Base(float k) const {
  const float d = k - cur[CUT] * 0.01f * (BANDS - 1);
  const float slope = cur[SLOPE] * 0.02f;
  const float g_lp = d <= 0.0f ? 1.0f : powf(10.0f, -2.0f * d / 20.0f);
  const float g_hp = d >= 0.0f ? 1.0f : powf(10.0f, 2.0f * d / 20.0f);
  const float g_bp = powf(10.0f, -2.0f * fabsf(d) / 20.0f);
  return slope < 0.0f ? g_bp + (-slope) * (g_lp - g_bp) : g_bp + slope * (g_hp - g_bp);
}

// the RESO bump around the cutoff, 0..1
FLASHMEM float ResonatorDevice::Bump(float k) const {
  const float d = k - cur[CUT] * 0.01f * (BANDS - 1);
  return expf(-(d * d) / 2.25f);
}

// height of the shape in the picture, 0..1: the pass band at the top, RESO
// lowers it to half with the peak staying at the top. TONE has its own box.
// Fixed scale, so nothing jumps while CUT moves between bands.
FLASHMEM float ResonatorDevice::View(float k) const {
  const float res = cur[RESO] * 0.01f;
  return std::min(1.0f, Base(k) * (1.0f - 0.5f * res + 0.5f * res * Bump(k)));
}

FLASHMEM void ResonatorDevice::Format(int p, char* buf, size_t n) const {
  if (p == CUT) {
    // the cutoff band's note, before PITCH
    static const char* const names[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    const int note = 24 + 2 * int(val[CUT] * 0.01f * (BANDS - 1) + 0.5f);
    snprintf(buf, n, "%s%d", names[note % 12], note / 12 - 1);
  } else if (p == DECAY) {
    // how long the band at the CUT note rings (to about -60 dB): T60 =
    // ln(1000) * Q / (pi * f), with PITCH
    const int note = 24 + 2 * int(val[CUT] * 0.01f * (BANDS - 1) + 0.5f) + cur[PITCH];
    const float hz = 440.0f * powf(2.0f, (note - 69) / 12.0f);
    const float t = 6.9078f * QOf(val[DECAY]) / (float(M_PI) * hz);
    if (t < 1.0f) snprintf(buf, n, "%dms", int(t * 1000.0f + 0.5f));
    else snprintf(buf, n, "%d.%ds", int(t), int(t * 10.0f) % 10);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void ResonatorDevice::DrawViz(int x, int y, int w, int h) {
  AudioEffectResonator::Viz v;
  fx.ReadViz(v);
  out_m.Hit(v.out);
  // 48 bands as thin bars (2 px each) across the three left columns, the
  // bank's shape drawn over them
  for (int k = 0; k < BANDS; k++) {
    band_m[k] = std::max(sqrtf(std::min(1.0f, v.band[k] * 4.0f)), band_m[k] * 0.9f);
    const int bx = x + k * 2;
    const int bh = int(band_m[k] * (h - 2));
    if (bh > 0) gfxLine(bx, y + h - 1, bx, y + h - 1 - bh);
  }
  // the shape as a solid line, worked out for every pixel
  int py = 0;
  for (int i = 0; i < BANDS * 2 - 1; i++) {
    const int sy = y + h - 1 - int(View(i * 0.5f) * (h - 3) + 0.5f);
    if (i) gfxLine(x + i - 1, py, x + i, sy);
    py = sy;
  }
  gfxDottedLine(x, y + h - 1, x + BANDS * 2, y + h - 1, 2);
  // output level on the right
  const int mx = x + w - 14;
  gfxFrame(mx, y, 12, h);
  DrawVMeter(mx + 1, y + 1, 10, h - 2, out_m.v);
  // TONE between the bands and PITCH: a box filled to its setting, empty
  // at the darkest, half (checkered) in the middle, solid at the brightest
  static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  const int tx = x + BANDS * 2 + 2, tw = mx - 7 - tx + 1;
  gfxFrame(tx, y, tw, h);
  const int fill = (cur[TONE] + 50) * 16 / 100;  // 0..16
  for (int py = y + 1; py < y + h - 1; py++)
    for (int px = tx + 1; px < tx + tw - 1; px++)
      if (bayer[py & 3][px & 3] < fill) gfxPixel(px, py);
  // PITCH left of the meter: a short line moving up and down, the middle
  // (no shift) marked by a dot
  const int pc = y + (h - 1) / 2;
  gfxPixel(mx - 5, pc);
  const int pyy = pc - cur[PITCH] * ((h - 3) / 2) / 24;
  gfxLine(mx - 4, pyy, mx - 2, pyy);
}

FLASHMEM void ResonatorDevice::DrawMini(int x, int y, int w, int h) {
  AudioEffectResonator::Viz v;
  fx.ReadViz(v);
  out_m.Hit(v.out);
  // a tiny bank: one column per 3 bands
  for (int i = 0; i < w; i++) {
    float m = 0.0f;
    for (int k = i * BANDS / w; k < (i + 1) * BANDS / w; k++) m = std::max(m, v.band[k]);
    // twice as tall as the page's bars, so the small picture reads
    band_m[i] = std::max(sqrtf(std::min(1.0f, m * 16.0f)), band_m[i] * 0.9f);
    const int bh = int(band_m[i] * h);
    if (bh > 0) gfxLine(x + i, y + h, x + i, y + h - bh);
  }
}

/////////////////////////////////////////////////////////////////
// SPRAY: grains from the last 4 s of the track, on Misty's
// engine (AudioEffectClouds, stereo). The engine gives the grains only; the
// slot's fade mixes them with the dry signal by LEVEL.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef SPRAY_DEF = {
  "SPRAY", "SPR", 8, {"FRZ", ""},
  {
    // left of the middle: P (periodic), right: R (random), middle: no grains
    {"RATE", -100, 100, 55, BI},
    {"SIZE", 0, 100, 50, UNI, 0, 21, 0},   // mode: size spread, 0..100% in 5% steps
    {"PITCH", -24, 24, 0, BI, 0, 25, 0},  // mode: pitch spread, 0..24 semitones
    // 0 = the oldest audio (left on the picture), 100 = now (right); mode:
    // position variation, 0..100% in 5% steps
    {"POS", 0, 100, 50, UNI, 0, 21, 4},
    {"SHAPE", 0, 100, 70, UNI},
    {"FDBK", 0, 100, 0, UNI},
    {"SPRD", 0, 100, 60, UNI},  // stereo spread: each grain's random place left / right
    {"MIX", 0, 100, 50, UNI},  // source <> grains
  },
  {"GRAIN", ""},
};

class SprayDevice : public Device {
public:
  enum { RATE, SIZE, PITCH, POS, SHAPE, FDBK, SPREAD, LEVEL };
  static constexpr float BUF_SECS = 4.0f;
  SprayDevice();
  void Rest(bool on) override { fx.rest = on; }
  bool FeedsWhileParked() const override { return true; }
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  bool Activate() override;
  void Deactivate() override;
  bool ok() const override { return fx.IsReady(); }
  void Apply(int p) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  bool ActionLit(int a) const override { return a == 0 && frozen; }
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
private:
  AudioEffectClouds fx;
  bool frozen = false;
  // 0.5 .. 40 Hz either side of the middle, 0 in it
  static float RateHz(int v) { return v ? 0.5f * powf(80.0f, (abs(v) - 1) / 99.0f) : 0.0f; }
  int SpreadSemis() const { return mode[PITCH]; }
  float SizeSpread() const { return mode[SIZE] * 0.05f; }
  float PosVariation() const { return mode[POS] * 0.025f; }  // up to half the buffer either way
  void FormatMode(int p, char* buf, size_t n) const override;
  float SizeSecs() const { return 0.01f * powf(100.0f, cur[SIZE] * 0.01f); } // 10 ms .. 1 s
};

FLASHMEM SprayDevice::SprayDevice()
  : Device(SPRAY_DEF), fx(size_t(BUF_SECS * AUDIO_SAMPLE_RATE_EXACT), 2) {}

// called while the stream is not patched in
FLASHMEM bool SprayDevice::Activate() {
  if (active) return fx.IsReady();
  fx.Acquire();
  fx.setGrainSource(0.0f);
  ApplyAll();
  active = true;
  return fx.IsReady();
}

FLASHMEM void SprayDevice::Deactivate() {
  if (!active) return;
  fx.Release();
  active = false;
}

FLASHMEM void SprayDevice::Apply(int p) {
  const float v = cur[p] * 0.01f;
  switch (p) {
    case RATE:  fx.setDensity(cur[RATE] < 0 ? -RateHz(cur[RATE]) : RateHz(cur[RATE])); break; // neg = periodic
    case SIZE:  fx.setSize(SizeSecs()); fx.setSizeSpread(SizeSpread()); break;
    case PITCH:
      fx.setPitch(powf(2.0f, cur[PITCH] / 12.0f));
      fx.setPitchSpread(SpreadSemis());
      break;
    case POS:   fx.setPosition(1.0f - v); fx.setSpray(PosVariation()); break;  // engine: 0 = now
    case SHAPE: fx.setTexture(v); break;
    case FDBK:  fx.setFeedback(0.9f * v * v); break;  // gentle start: 25 -> 6%, 50 -> 23%
    case SPREAD: fx.setStereoSpread(v); break;
    case LEVEL: wet = v; break;
  }
}

// the spreads: OFF, then +-semitones (PITCH) or percent (SIZE, POS)
FLASHMEM void SprayDevice::FormatMode(int p, char* buf, size_t n) const {
  if ((p == PITCH || p == SIZE || p == POS) && mode[p] == 0) snprintf(buf, n, "OFF");
  else if (p == PITCH) snprintf(buf, n, "+-%d", mode[p]);
  else if (p == SIZE || p == POS) snprintf(buf, n, "%d%%", mode[p] * 5);
  else Device::FormatMode(p, buf, n);
}

FLASHMEM void SprayDevice::Format(int p, char* buf, size_t n) const {
  if (p == RATE) {
    // P periodic / R random, in grains per second
    const float hz = RateHz(val[RATE]);
    const char side = val[RATE] < 0 ? 'P' : 'R';
    if (!val[RATE]) snprintf(buf, n, "OFF");
    else if (hz < 10.0f) snprintf(buf, n, "%c%d.%d", side, int(hz), int(hz * 10) % 10);
    else snprintf(buf, n, "%c%d", side, int(hz + 0.5f));
  } else if (p == SIZE) {
    const int ms = int(10.0f * powf(100.0f, val[SIZE] * 0.01f) + 0.5f);
    if (ms < 1000) snprintf(buf, n, "%dms", ms);
    else snprintf(buf, n, "1.0s");
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void SprayDevice::Action(int a) {
  if (a == 0) {
    frozen = !frozen;
    fx.setFreeze(frozen);
  }
}

FLASHMEM void SprayDevice::DrawViz(int x, int y, int w, int h) {
  if (!fx.IsReady()) {
    gfxPrint(x + 22, y + h / 2 - 4, PSTR("out of memory"));
    return;
  }
  const int base = y + h - 1;
  // the buffer as it sits in memory: the recording stays in place and the
  // record head (a line) runs across it, writing over the oldest audio, like
  // a tape loop; it stops when frozen
  const int cy = y + h / 2;
  constexpr int HIST = AudioEffectClouds::HIST;
  uint8_t hist[HIST];
  const int head = fx.ReadHistory(hist);
  auto col_x = [&](int k) { return x + ((k % HIST + HIST) % HIST) * w / HIST; };
  // MIX makes it see-through: solid with only the source heard (0), thinner
  // as the grains take over, gone at 100
  static const uint8_t wave_bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  const int wave_fill = 16 - cur[LEVEL] * 16 / 100;  // 16 .. 0 of 16
  for (int k = 0; k < HIST; k++) {
    const int a = int(sqrtf(hist[k] / 255.0f) * (h / 2 - 1));
    const int cx = col_x(k);
    for (int py = cy - a; py <= cy + a; py++)
      if (wave_bayer[py & 3][cx & 3] < wave_fill) gfxPixel(cx, py);
  }
  const int hx = col_x(head);
  const int cl = std::max(x, hx - 1), cr = std::min(x + w - 1, hx + 1);
  graphics.clearRect(cl, y, cr - cl + 1, h);  // kept inside the screen
  gfxLine(hx, y, hx, base);
  // the grain window: POS moves it from now (right) back to the oldest audio
  // (left); its variation widens it, only as far as the engine really
  // scatters (never past now or the oldest audio)
  const float pos = 1.0f - cur[POS] * 0.01f;  // back from now, like the engine
  const float eff = std::min(PosVariation(), std::min(pos, 1.0f - pos));
  const int mid = x + w - 1 - int(pos * (w - 1) + 0.5f), half = std::max(2, int(eff * w + 0.5f));
  const int wl = std::max(x, mid - half), wr = std::min(x + w - 1, mid + half);
  // brackets, their corners rounded by SHAPE (0 square .. 100 round, as the
  // grains' edges get softer); drawn pixel by pixel so the corners are exact
  const int r = std::min((cur[SHAPE] * 3 + 50) / 100, (wr - wl) / 2);
  constexpr int tick = 5;
  for (int px = wl + r; px <= wr - r; px++) { gfxPixel(px, y); gfxPixel(px, base); }
  for (int k = r; k < tick; k++) {
    gfxPixel(wl, y + k); gfxPixel(wr, y + k);
    gfxPixel(wl, base - k); gfxPixel(wr, base - k);
  }
  // the corner between them: r 2 = one pixel, r 3 = two, on the diagonal
  for (int i = 1; i < r; i++) {
    const int j = r - i;
    gfxPixel(wl + i, y + j); gfxPixel(wr - i, y + j);
    gfxPixel(wl + i, base - j); gfxPixel(wr - i, base - j);
  }
  // the grains as they play: across from the middle by their scatter, up
  // and down by their own pitch offset around PITCH (all on one line without
  // spread), sized by their length
  const int line = cy - cur[PITCH] * (h / 2 - 4) / 24;
  AudioEffectClouds::GrainView gv[AudioEffectClouds::MAX_GRAINS];
  const int n = fx.ReadGrains(gv, AudioEffectClouds::MAX_GRAINS);
  for (int i = 0; i < n; i++) {
    // squares, bigger for longer grains (3 .. 11 px)
    const int len_px = int(gv[i].secs / BUF_SECS * w + 0.5f);
    const int gh = constrain(3 + (len_px - 2) / 4, 3, 11), gw = gh;
    const int gx = constrain(mid + int(gv[i].scatter * w) - gw / 2, x + 1, x + w - gw - 1);
    const int gy = constrain(line - int(gv[i].semis * (h / 2 - 4) / 24) - gh / 2, y + 2, base - gh - 1);
    graphics.clearRect(gx - 1, gy - 1, gw + 2, gh + 2);  // a gap around it in the waveform
    // FDBK empties them gradually: solid at 0, a thinning dot pattern on the
    // way, just the outline at 100
    gfxFrame(gx, gy, gw, gh);
    static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    const int fill = 16 - cur[FDBK] * 16 / 100;
    for (int yy = gy + 1; yy < gy + gh - 1; yy++)
      for (int xx = gx + 1; xx < gx + gw - 1; xx++)
        if (bayer[yy & 3][xx & 3] < fill) gfxPixel(xx, yy);
  }
  if (frozen) {
    gfxPrint(x + w - 25, y + 1, PSTR("FRZ"));
    gfxInvert(x + w - 26, y, 20, 9);
  }
}

FLASHMEM void SprayDevice::DrawMini(int x, int y, int w, int h) {
  const int n = fx.ActiveGrainCount();
  uint32_t r = millis() / 60;
  for (int i = 0; i < n; i++) {
    r = r * 1103515245u + 12345u;
    gfxPixel(x + int((r >> 8) % uint32_t(w)), y + int((r >> 20) % uint32_t(h)));
  }
}

/////////////////////////////////////////////////////////////////
// Slot catalogue, in chain order. Option 0 of the effect slots is BYPASS.
/////////////////////////////////////////////////////////////////
enum Slot : uint8_t { SOURCE, SPRAY, RESONATOR, TEXTURE, ECHO, SLOT_COUNT };
constexpr int MAX_OPTIONS = 3;

/////////////////////////////////////////////////////////////////
// MODAL (Resonator slot, 2nd choice): Mutable Instruments Rings (its DSP,
// MIT, in src/extern/mi). MODEL picks the resonator: MODAL (a struck
// object, up to 32 partials), SYMP (sympathetic strings), STRNG
// (inharmonic string), FM (FM voice), SYMQ (sympathetic strings on chords).
// (Rings' string + reverb is left out: ECHO does the reverb.) The track (L+R) excites it and strums it by its
// onsets; A (HIT) strikes it by hand. Odd partials / strings left, even
// right, as far apart as SPRD.
/////////////////////////////////////////////////////////////////
class AudioRings : public AudioStream {
public:
  AudioRings() : AudioStream(1, queue) {}
  RingsVoice voice;
  volatile float spread = 0.7f;
  volatile float peak = 0.0f;
  volatile bool rest = false;  // parked: nothing to compute
  void update() override;
private:
  audio_block_t* queue[1];
};

FASTRUN void AudioRings::update() {
  audio_block_t* in = receiveReadOnly(0);
  if (!voice.ok() || rest) { if (in) release(in); return; }
  audio_block_t* o[2] = {allocate(), allocate()};
  if (o[0] && o[1]) {
    static float fi[AUDIO_BLOCK_SAMPLES], fo[AUDIO_BLOCK_SAMPLES], fa[AUDIO_BLOCK_SAMPLES];
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) fi[n] = in ? in->data[n] * (1.0f / 32768.0f) : 0.0f;
    voice.Process(fi, fo, fa, AUDIO_BLOCK_SAMPLES);
    const float a = 0.5f + 0.5f * spread, b = 1.0f - a;
    float pk = 0.0f;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      const float l = fo[n] * a + fa[n] * b, r = fa[n] * a + fo[n] * b;
      pk = std::max(pk, std::max(fabsf(l), fabsf(r)));
      o[0]->data[n] = __SSAT(int32_t(l * 32767.0f), 16);
      o[1]->data[n] = __SSAT(int32_t(r * 32767.0f), 16);
    }
    peak = std::max(float(peak), pk);
    transmit(o[0], 0);
    transmit(o[1], 1);
  }
  for (auto b : o) if (b) release(b);
  if (in) release(in);
}

PROGMEM static const DeviceDef MODAL_DEF = {
  "MODAL", "MDL", 8, {"HIT", ""},
  {
    {"NOTE", 0, 72, 36, UNI},      // C0 .. C6, C3 in the middle
    {"STRUC", 0, 100, 70, UNI},    // the module's STRUCTURE (SYMQ: the chord)
    {"BRITE", 0, 100, 60, UNI},
    {"DAMP", 0, 100, 65, UNI},
    {"POS", 0, 100, 25, UNI},      // where it is struck
    {"SPRD", 0, 100, 70, UNI},     // odd / even apart
    {"MODEL", 0, RingsVoice::MODELS - 1, 0, UNI},
    {"WET", 0, 100, 100, UNI},
  },
  {"", ""},
};

class ModalDevice : public Device {
public:
  enum { NOTE, STRUC, BRITE, DAMP, POS, SPRD, MODEL, WET };
  ModalDevice() : Device(MODAL_DEF), sum_l(in_mix, 0, fx, 0) {
    in_mix.gain(0, 0.5f); in_mix.gain(1, 0.5f); in_mix.gain(2, 0.0f); in_mix.gain(3, 0.0f);
  }
  AudioStream* in() override { return &in_mix; }   // L + R summed
  AudioStream* out() override { return &fx; }
  bool ok() const override { return fx.voice.ok(); }
  bool Activate() override {
    if (!fx.voice.Init()) return false;
    ApplyAll();
    active = true;
    return true;
  }
  void Deactivate() override {
    AudioNoInterrupts();
    fx.voice.Free();
    AudioInterrupts();
    active = false;
  }
  void Apply(int p) override;
  void Action(int a) override { if (a == 0) { fx.voice.Strike(); hit_at = millis() | 1; } }
  void Rest(bool on) override { fx.rest = on; }
  bool RestsWhenDry() const override { return true; }  // WET 0: rests (wakes ringing from silence)
  void Format(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
private:
  AudioMixer4 in_mix;
  AudioRings fx;
  AudioConnection sum_l;
  Meter out_m;
  uint32_t hit_at = 0;
  float Hz() const { return 16.352f * powf(2.0f, cur[NOTE] / 12.0f); }  // C0 = 16.35 Hz
  // the partials of a struck object (an approximation of Rings' modal
  // model), for the picture
  void Partials(float ratio[12], float gain[12]) const;
};

FLASHMEM void ModalDevice::Apply(int p) {
  switch (p) {
    case WET: wet = cur[WET] * 0.01f; return;
    case SPRD: fx.spread = cur[SPRD] * 0.01f; return;
    case MODEL: fx.voice.SetModel(cur[MODEL]); return;
    default:
      fx.voice.SetPatch(cur[NOTE], cur[STRUC] * 0.01f, cur[BRITE] * 0.01f, cur[DAMP] * 0.01f, cur[POS] * 0.01f);
  }
}

FLASHMEM void ModalDevice::Format(int p, char* buf, size_t n) const {
  if (p == NOTE) {
    static const char* const names[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    snprintf(buf, n, "%s%d", names[val[NOTE] % 12], val[NOTE] / 12);
  } else if (p == MODEL) {
    static const char* const models[RingsVoice::MODELS] = {"MODAL", "SYMP", "STRNG", "FM", "SYMQ"};
    snprintf(buf, n, "%s", models[constrain(val[MODEL], 0, RingsVoice::MODELS - 1)]);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void ModalDevice::Partials(float ratio[12], float gain[12]) const {
  const float structure = cur[STRUC] * 0.01f, bright = cur[BRITE] * 0.01f;
  float stiffness = structure * 1.55f - 0.05f, stretch = 1.0f;
  float q_loss = bright * (2.0f - bright) * 0.85f + 0.15f, q = 1.0f;
  const float q_rate = structure * (2.0f - structure) * 0.1f;
  const float pos = cur[POS] * 0.01f * 0.98f + 0.01f;
  for (int k = 0; k < 12; k++) {
    ratio[k] = stretch;
    const float sg = sinf(float(M_PI) * pos * (k + 1));
    gain[k] = sg * sg * q;
    stretch += stiffness;
    stiffness *= stiffness < 0.0f ? 0.93f : 0.98f;
    q_loss += q_rate * (1.0f - q_loss);
    q *= q_loss;
  }
}

// MODAL / FM: the partials as lines along a pitch scale (6 octaves across);
// the string models: their strings, shaking with the sound. Output meter
// on the right; a hit flashes the corner.
FLASHMEM void ModalDevice::DrawViz(int x, int y, int w, int h) {
  const float pk = fx.peak;
  fx.peak = 0.0f;
  out_m.Hit(pk);
  const int ww = w - 8;
  const int model = cur[MODEL];
  if (model == RingsVoice::MODAL || model == RingsVoice::FM) {
    float r[12], g[12];
    Partials(r, g);
    for (int k = 0; k < 12; k++) {
      const float pos = log2f(Hz() * r[k] / 16.352f) / 6.0f;
      if (pos < 0.0f || pos > 1.0f) continue;
      const int px = x + int(pos * (ww - 1));
      const int ph = std::max(1, int(sqrtf(std::min(1.0f, g[k])) * (h - 2)));
      gfxLine(px, y + h - 1, px, y + h - ph);
      if (k == 0) gfxRect(px - 1, y + h - ph - 2, 3, 2);  // the fundamental
    }
    gfxDottedLine(x, y + h - 1, x + ww - 1, y + h - 1, 3);
  } else {
    // strings: one (STRNG) or the sympathetic set, each a wave
    const int strings = (model == RingsVoice::SYMPATHETIC || model == RingsVoice::SYMPATHETIC_QUANTIZED) ? 4 : 1;
    const float amp = out_m.v * (h / (2.0f * strings) - 1);
    const float t = millis() * 0.02f;
    for (int s = 0; s < strings; s++) {
      const int cy = y + (2 * s + 1) * h / (2 * strings);
      int py = cy;
      for (int i = 0; i < ww; i++) {
        const float u = float(i) / (ww - 1);
        const int yy = cy + int(amp * sinf(float(M_PI) * u) * sinf(t * (1.0f + 0.37f * s) + u * 6.0f * (s + 1)));
        if (i) gfxLine(x + i - 1, py, x + i, yy);
        py = yy;
      }
    }
  }
  if (hit_at && millis() - hit_at < 120) gfxRect(x, y, 4, 4);
  gfxFrame(x + w - 4, y, 4, h);
  DrawVMeter(x + w - 4, y, 4, h, out_m.v);
}

FLASHMEM void ModalDevice::DrawMini(int x, int y, int w, int h) {
  const int model = cur[MODEL];
  if (model == RingsVoice::MODAL || model == RingsVoice::FM) {
    float r[12], g[12];
    Partials(r, g);
    for (int k = 0; k < 12; k++) {
      const float pos = log2f(r[k]) / 4.0f;
      if (pos > 1.0f) continue;
      const int px = x + int(pos * (w - 1));
      const int ph = std::max(1, int(sqrtf(std::min(1.0f, g[k])) * h));
      gfxLine(px, y + h - 1, px, y + h - ph);
    }
  } else {
    const int strings = (model == RingsVoice::SYMPATHETIC || model == RingsVoice::SYMPATHETIC_QUANTIZED) ? 4 : 1;
    for (int s = 0; s < strings; s++) {
      const int cy = y + (2 * s + 1) * h / (2 * strings);
      gfxLine(x, cy, x + w - 1, cy);
    }
  }
}

/////////////////////////////////////////////////////////////////
// STRETCH (Spray slot, 2nd choice): Mutable Instruments Clouds' pitch /
// time mode (WSOLA, MIT, in src/extern/mi). The track is recorded (4 s,
// unless frozen) and played back POS behind now, PITCH shifted without
// changing the speed; frozen (A), it plays through the buffer at SPEED
// without changing the pitch. SIZE: the window (short = grainy and quick,
// long = smooth). TONE: Clouds' filter (left lowpass, right highpass).
/////////////////////////////////////////////////////////////////
class AudioStretch : public AudioStream {
public:
  AudioStretch() : AudioStream(2, queue) {}
  StretchVoice voice;
  volatile float fdbk = 0.0f, lp_a = 1.0f, hp_a = 0.0f;
  volatile float peak = 0.0f;
  volatile bool rest = false;  // parked: nothing to compute
  void update() override;
private:
  audio_block_t* queue[2];
  float last[2 * AUDIO_BLOCK_SAMPLES] = {};
  float lp_z[2] = {}, hp_z[2] = {}, dc_z[2] = {};
  bool resting = false;
};

FASTRUN void AudioStretch::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  if (!voice.ok()) { for (auto b : in) if (b) release(b); return; }
  static float fi[2 * AUDIO_BLOCK_SAMPLES], fo[2 * AUDIO_BLOCK_SAMPLES];
  if (rest) {  // out of the chain: keep recording only
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++)
      for (int c = 0; c < 2; c++) fi[2 * n + c] = in[c] ? in[c]->data[n] * (1.0f / 32768.0f) : 0.0f;
    voice.Record(fi, AUDIO_BLOCK_SAMPLES);
    resting = true;
    for (auto b : in) if (b) release(b);
    return;
  }
  if (resting) {  // back: fresh windows, no old feedback or filter state
    resting = false;
    voice.Restart();
    memset(last, 0, sizeof(last));
    for (int c = 0; c < 2; c++) lp_z[c] = hp_z[c] = dc_z[c] = 0.0f;
  }
  audio_block_t* o[2] = {allocate(), allocate()};
  if (o[0] && o[1]) {
    const float fb = fdbk;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++)
      for (int c = 0; c < 2; c++) {
        float x = in[c] ? in[c]->data[n] * (1.0f / 32768.0f) : 0.0f;
        // feedback: last block's output, DC taken out, softly limited
        float f = last[2 * n + c];
        dc_z[c] += 0.002f * (f - dc_z[c]);
        f -= dc_z[c];
        x += fb * f;
        fi[2 * n + c] = x / (1.0f + fabsf(x) * 0.25f);
      }
    voice.Process(fi, fo, AUDIO_BLOCK_SAMPLES);
    const float la = lp_a, ha = hp_a;
    float pk = 0.0f;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++)
      for (int c = 0; c < 2; c++) {
        float y = fo[2 * n + c];
        lp_z[c] += la * (y - lp_z[c]);  // TONE: lowpass ...
        y = lp_z[c];
        hp_z[c] += ha * (y - hp_z[c]);  // ... or highpass
        y -= hp_z[c];
        last[2 * n + c] = y;
        pk = std::max(pk, fabsf(y));
        o[c]->data[n] = __SSAT(int32_t(y * 32767.0f), 16);
      }
    peak = std::max(float(peak), pk);
    transmit(o[0], 0);
    transmit(o[1], 1);
  }
  for (auto b : o) if (b) release(b);
  for (auto b : in) if (b) release(b);
}

PROGMEM static const DeviceDef STRETCH_DEF = {
  "STRETCH", "STR", 8, {"FRZ", ""},
  {
    // modes (like SPRAY's): random variation per window
    {"PITCH", -24, 24, 0, BI, 0, 25, 0},  // semitones, without changing the speed; mode: pitch spread 0..24
    {"POS", 0, 100, 80, UNI, 0, 21, 0},   // like SPRAY: 0 = the oldest audio (left), 100 = now (right); mode: position variation
    {"SIZE", 0, 100, 50, UNI, 0, 21, 0},  // the window: short = quick and grainy, long = smooth; mode: size spread
    {"SPEED", -200, 200, 100, BI}, // frozen: through the buffer (%, < 0 backwards)
    {"TONE", -50, 50, 0, BI},      // left lowpass, right highpass
    {"FDBK", 0, 100, 0, UNI},
    {"SPRD", 0, 100, 0, UNI},      // stereo spread: the windows wander left / right
    {"MIX", 0, 100, 100, UNI},
  },
  {"", ""},
};

class StretchDevice : public Device {
public:
  enum { PITCH, POS, SIZE, SPEED, TONE, FDBK, SPRD, MIX };
  static constexpr float BUF_SECS = 4.0f;
  void FormatMode(int p, char* buf, size_t n) const override;
  StretchDevice() : Device(STRETCH_DEF) {}
  void Rest(bool on) override { fx.rest = on; }
  bool FeedsWhileParked() const override { return true; }
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  bool ok() const override { return fx.voice.ok(); }
  bool Activate() override {
    if (!fx.voice.Init(BUF_SECS)) return false;
    ApplyAll();
    active = true;
    return true;
  }
  void Deactivate() override {
    AudioNoInterrupts();
    fx.voice.Free();
    AudioInterrupts();
    active = false;
  }
  // the correlator's search, from the main loop: each time a whole search
  // (~1 ms; a quarter at a time, as Clouds did, fell behind while the chain
  // view drew, and sounded different). From the audio it hung the module.
  void Idle() override { if (active && !fx.rest) fx.voice.Work(2000); }
  void Apply(int p) override;
  void Action(int a) override { if (a == 0) { frozen = !frozen; Apply(PITCH); } }
  bool ActionLit(int a) const override { return a == 0 && frozen; }
  void Format(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
private:
  AudioStretch fx;
  Meter out_m;
  bool frozen = false;
};

FLASHMEM void StretchDevice::Apply(int p) {
  switch (p) {
    case MIX: wet = cur[MIX] * 0.01f; return;
    case FDBK: fx.fdbk = 0.95f * cur[FDBK] * 0.01f; return;
    case TONE: {  // Clouds' texture filter: left closes a lowpass, right opens a highpass
      const float t = cur[TONE] * 0.02f;
      const float lp_hz = t < 0.0f ? 18000.0f * powf(150.0f / 18000.0f, -t) : 18000.0f;
      const float hp_hz = t > 0.0f ? 20.0f * powf(4000.0f / 20.0f, t) : 20.0f;
      fx.lp_a = 1.0f - expf(-2.0f * float(M_PI) * lp_hz / AUDIO_SAMPLE_RATE_EXACT);
      fx.hp_a = 1.0f - expf(-2.0f * float(M_PI) * hp_hz / AUDIO_SAMPLE_RATE_EXACT);
      return;
    }
    default:
      fx.voice.Set(1.0f - cur[POS] * 0.01f, cur[SIZE] * 0.01f, cur[PITCH], cur[SPEED] * 0.01f, frozen);
      fx.voice.SetSpread(mode[PITCH], mode[SIZE] * 0.05f, mode[POS] * 0.025f, cur[SPRD] * 0.01f);
  }
}

// the spreads, as on SPRAY: OFF, then +-semitones (PITCH) or percent
FLASHMEM void StretchDevice::FormatMode(int p, char* buf, size_t n) const {
  if ((p == PITCH || p == SIZE || p == POS) && mode[p] == 0) snprintf(buf, n, "OFF");
  else if (p == PITCH) snprintf(buf, n, "+-%d", mode[p]);
  else if (p == SIZE || p == POS) snprintf(buf, n, "%d%%", mode[p] * 5);
  else Device::FormatMode(p, buf, n);
}

FLASHMEM void StretchDevice::Format(int p, char* buf, size_t n) const {
  if (p == SPEED) snprintf(buf, n, "%d%%", val[SPEED]);
  else Device::Format(p, buf, n);
}

// the buffer as a line from the oldest (left) to now (right); the window
// being played as a box POS back from now, as wide as SIZE, raised or
// lowered by PITCH; frozen, it runs through the buffer at SPEED
FLASHMEM void StretchDevice::DrawViz(int x, int y, int w, int h) {
  const float pk = fx.peak;
  fx.peak = 0.0f;
  out_m.Hit(pk);
  const int ww = w - 8, cy = y + h / 2, base = y + h - 1;
  gfxDottedLine(x, cy, x + ww - 1, cy, 2);
  if (!frozen) gfxLine(x + ww - 1, y + 2, x + ww - 1, y + h - 3);  // now: the record head
  float pos = 1.0f - cur[POS] * 0.01f + (frozen ? fx.voice.Scrub() : 0.0f);  // back from now
  pos -= floorf(pos);
  auto pos_x = [&](float p) { p -= floorf(p); return x + int((1.0f - p) * (ww - 1)); };
  const float semis_px = float(h / 2 - 4) / 24.0f;
  // the variations' reach, like SPRAY's: POS as brackets either side, PITCH
  // as a dotted line up and down through the middle
  const float pv = mode[POS] * 0.025f;
  if (pv > 0.0f) {
    const int wl = std::max(x, pos_x(pos + pv)), wr = std::min(x + ww - 1, pos_x(pos - pv));
    for (int px = wl; px <= wr; px++) { gfxPixel(px, y); gfxPixel(px, base); }
    for (int k = 0; k < 5; k++) {
      gfxPixel(wl, y + k); gfxPixel(wr, y + k);
      gfxPixel(wl, base - k); gfxPixel(wr, base - k);
    }
  }
  const int pyc = cy - int(cur[PITCH] * semis_px);
  if (mode[PITCH] > 0) {
    const int a = int(mode[PITCH] * semis_px);
    for (int yy = std::max(y + 1, pyc - a); yy <= std::min(base - 1, pyc + a); yy += 2) gfxPixel(pos_x(pos), yy);
  }
  // the window playing now: POS back from now (plus its variation), as wide
  // as SIZE, raised or lowered by PITCH; frozen, it runs through the buffer
  const int px = pos_x(pos + fx.voice.DPos());
  const float sz = constrain(cur[SIZE] * 0.01f + fx.voice.DSize(), 0.0f, 1.0f);
  const int bw = 4 + int(sz * (ww / 4));
  const int py = cy - int((cur[PITCH] + fx.voice.DPitch()) * semis_px);
  const int bx = constrain(px - bw / 2, x, x + ww - bw);
  graphics.clearRect(bx, py - 3, bw, 7);
  gfxFrame(bx, py - 3, bw, 7);
  // SPRD: a dot inside the box, where it sits left / right
  if (cur[SPRD] > 0) gfxPixel(bx + 1 + int((fx.voice.Pan() + 1.0f) * 0.5f * (bw - 3)), py);
  if (frozen) {
    gfxPrint(x + 1, y + 1, PSTR("FRZ"));
    gfxInvert(x, y, 20, 9);
  }
  gfxFrame(x + w - 4, y, 4, h);
  DrawVMeter(x + w - 4, y, 4, h, out_m.v);
}

FLASHMEM void StretchDevice::DrawMini(int x, int y, int w, int h) {
  const int cy = y + h / 2;
  gfxLine(x, cy, x + w - 1, cy);
  float pos = 1.0f - cur[POS] * 0.01f + (frozen ? fx.voice.Scrub() : 0.0f);  // back from now
  pos -= floorf(pos);
  const int px = x + int((1.0f - pos) * (w - 1));
  const int py = cy - cur[PITCH] * (h / 2 - 2) / 24;
  gfxFrame(constrain(px - 2, x, x + w - 5), py - 2, 5, 5);
}

struct SlotDef {
  char name[10];
  char tag[5];       // chain box label, 3 letters (4 drawn tighter)
  uint8_t noptions;  // BYPASS + devices that exist now (SOURCE: no bypass)
  uint8_t planned;   // devices this slot will offer (dots on the chain view)
};

PROGMEM static const SlotDef SLOT_DEFS[SLOT_COUNT] = {
  {"SOURCE",   "SRC", 3, 3},  // INPUT, SPOOL, PLAYER (owned by the track, not made here)
  {"SPRAY",    "SPR", 3, 1},  // BYPASS, SPRAY (grains), STRETCH (Clouds pitch / time)
  {"RESONATOR", "RES", 3, 1},  // BYPASS, RESONATOR (filter bank), MODAL (struck object)
  {"TEXTURE",  "TEX", 2, 1},  // BYPASS, TEXTURE
  {"ECHO",     "ECO", 2, 1},  // BYPASS, ECHO
};

// nullptr = BYPASS. SOURCE/INPUT is owned by the track, not made here.
FLASHMEM static Device* MakeDevice(int slot, int option) {
  if (slot == SPRAY && option == 1) return new SprayDevice;
  if (slot == SPRAY && option == 2) return new StretchDevice;
  if (slot == RESONATOR && option == 1) return new ResonatorDevice;
  if (slot == RESONATOR && option == 2) return new ModalDevice;
  if (slot == TEXTURE && option == 1) return new DirtDevice;
  if (slot == ECHO && option == 1) return new EchoDevice;
  return nullptr;
}

} // namespace loom
