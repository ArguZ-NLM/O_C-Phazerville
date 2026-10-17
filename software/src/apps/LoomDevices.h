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
#include "../Audio/PsramAlloc.h"
#include "../Audio/AudioSample.h"
#include "../Audio/AudioEffectDirt.h"
#include "../Audio/AudioEffectBandz.h"
#include "../Audio/AudioEffectResonator.h"
#include "../Audio/AudioEffectChime.h"
#include "../Audio/AudioEffectWarp.h"
#include "../Audio/RingsVoice.h"
#include "../Audio/StretchVoice.h"
#include "../Audio/AudioEffectClouds.h"
#include "../Audio/AudioEffectReel.h"
#include "../Audio/AudioPlayStream.h"
#include <SD.h>

extern "C" uint8_t external_psram_size;  // MB of PSRAM found at start (Teensy core)

extern volatile bool g_noise_request;  // USB 'N' (Main.cpp)
extern volatile uint8_t g_heavy_request;  // USB 'H' / 'h' (Main.cpp)
extern volatile uint8_t g_capture_request;  // USB 'W' (1) / 'U' (2) (Main.cpp)
extern volatile uint8_t g_notes_request;    // USB 'M' / 'm' (Main.cpp)
extern volatile char g_set_request;         // USB '=' / ':' (Main.cpp)
extern char g_set_text[24];
extern volatile bool g_tape_request;        // USB 'T' (Main.cpp)
extern volatile bool g_midilog_request;     // USB 'J' (Main.cpp)
extern volatile bool g_droplog_request;     // USB 'O' (Main.cpp)
extern volatile uint32_t g_remote_at;       // the last remote-panel key (Main.cpp)
extern volatile int8_t g_move_request;      // USB 'Y' / 'Z' (Main.cpp)

extern char g_warp_report[200];  // what WARP loaded last (USB 'P', Main.cpp)

// the font's plus-minus sign (char 127), for the variations: LOOM_PM "%d"
#define LOOM_PM "\x7f"

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
  ParamDef params[12];   // up to three pages of four
  char page_title[3][8]; // shown after the name in the header, per page
};

constexpr int MAX_PARAMS = 12;
// modulator outputs a setting can pick: the track's MOD A-D, then the
// shared MOD E-H (the same for both tracks)
constexpr int MOD_BUSES = 8;
// a pitch setting's MOD1 / MOD2 source past MOD H: the last MIDI note
// played (any channel), C3 (60) = no change, a semitone a key at +100 %
constexpr uint8_t MIDI_BUS = MOD_BUSES + 1;

// rising edges since the last modulator tick: bits 0-3 gate inputs,
// bits 4-11 CV inputs as gates. Set in the ISR, taken in the main loop.
volatile uint16_t trig_latch = 0;

// MIDI notes in (from any MIDI input): the last note on per channel (index
// 0: any channel) and a count that moves on with every one, for devices
// that take a note (CHIME's NOTE)
inline uint8_t midi_note[17] = {};
inline uint32_t midi_note_seq[17] = {};
// ... and every note on / off as it came, for devices that play notes
// (WARP); each reads on from where it was
struct NoteEv { uint8_t ch, note, vel; };  // vel 0: note off
constexpr uint32_t NOTE_EVS = 64;
inline NoteEv note_ev[NOTE_EVS] = {};
inline uint32_t note_ev_n = 0;
// MIDI aftertouch, any channel: the channel's pressure, and each key's
// (poly; a key's starts at 0 with its note); at_seen counts them all
inline uint8_t chan_at = 0;
inline uint8_t poly_at[128] = {};
inline uint32_t at_seen = 0;
// the last MIDI messages in (USB 'J' prints them): when, what
struct MidiLogEv { uint32_t ms; uint8_t type, ch, d1, d2; };
constexpr int MIDI_LOG = 128;
inline MidiLogEv midi_log[MIDI_LOG] = {};
inline uint32_t midi_log_n = 0;
inline void MidiLogPrint() {
  const uint32_t n = std::min<uint32_t>(midi_log_n, MIDI_LOG);
  Serial.printf("--- MIDI log, last %lu ---\n", (unsigned long)n);
  for (uint32_t k = midi_log_n - n; k != midi_log_n; k++) {
    const MidiLogEv& e = midi_log[k % MIDI_LOG];
    const char* t = e.type == 0x90 ? "ON " : e.type == 0x80 ? "OFF" : e.type == 0xA0 ? "PAT" : e.type == 0xD0 ? "CAT"
                  : e.type == 0xE0 ? "BND" : e.type == 0xB0 ? "CC " : "?  ";
    Serial.printf("%8lu ch%2d %s %3d %3d\n", (unsigned long)e.ms, e.ch, t, e.d1, e.d2);
    delay(1);
  }
  Serial.println("--- end ---");
}

inline void NoteEvent(int ch, int note, int vel) {
  note_ev[note_ev_n % NOTE_EVS] = {uint8_t(ch), uint8_t(note), uint8_t(vel)};
  note_ev_n++;
}
inline void NoteIn(int ch, int note) {
  midi_note[0] = uint8_t(note);
  midi_note_seq[0]++;
  if (ch >= 1 && ch <= 16) { midi_note[ch] = uint8_t(note); midi_note_seq[ch]++; }
}

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
uint32_t clock_beat_n = 0;  // whole beats since the clock (re)started: with clock_beat_ph, where it is
// which division of div_beats the clock is in: modulators and devices on
// the clock count the same way, so their steps fall on the same tick
inline int64_t ClockGrid(float div_beats) {
  return int64_t(floorf((clock_beat_n + clock_beat_ph) / div_beats + 1e-4f));
}
// Z starts and stops the clock (so does MIDI start/stop when the clock
// follows MIDI): synced modulators hold while stopped and begin again from
// the top on start
bool clock_running = true;
uint8_t clock_starts = 0;  // counts starts; each modulator notices a new one
inline void ClockStart() { clock_running = true; clock_starts++; }
inline void ClockStop() { clock_running = false; }

// a setting on the clock (mode CLK): one of NUM_DIVS note values, 8 bars
// .. 1/64, each straight, dotted (D), triplet (T) and quintuplet (Q: five
// in the time of four), sorted from the longest; in beats (quarter = 1)
constexpr int NUM_DIVS = 40;
enum SyncMode : uint8_t { SYNC_FREE, SYNC_CLOCK };
FLASHMEM static void ClockDiv(int i, float* beats, char* name = nullptr, size_t n = 0) {
  static const char* const base[10] = {"8", "4", "2", "1", "1/2", "1/4", "1/8", "1/16", "1/32", "1/64"};
  static const float mult[4] = {1.5f, 1.0f, 0.8f, 2.0f / 3.0f};
  static const char suf[4] = {'D', 0, 'Q', 'T'};
  static float b[NUM_DIVS];
  static uint8_t order[NUM_DIVS];
  static bool made = false;
  if (!made) {  // all of them, sorted from the longest
    for (int k = 0; k < NUM_DIVS; k++) { b[k] = 32.0f / float(1 << (k / 4)) * mult[k % 4]; order[k] = uint8_t(k); }
    std::sort(order, order + NUM_DIVS, [](uint8_t x, uint8_t y) { return b[x] > b[y]; });
    made = true;
  }
  const int k = order[constrain(i, 0, NUM_DIVS - 1)];
  if (beats) *beats = b[k];
  if (name) {
    if (suf[k % 4]) snprintf(name, n, "%s%c", base[k / 4], suf[k % 4]);
    else snprintf(name, n, "%s", base[k / 4]);
  }
}
// map a setting's value range onto the note values, the longest at lo (so
// CV can pick them); a value in the middle of each one's stretch
FLASHMEM static int DivIndex(int v, int lo, int hi) {
  return constrain((v - lo) * NUM_DIVS / (hi - lo + 1), 0, NUM_DIVS - 1);
}
FLASHMEM static int DivValue(int idx, int lo, int hi) {
  idx = constrain(idx, 0, NUM_DIVS - 1);
  return lo + ((hi - lo + 1) * (2 * idx + 1)) / (2 * NUM_DIVS);
}
FLASHMEM static float DivBeats(int idx) { float b; ClockDiv(idx, &b); return b; }
FLASHMEM static float DivSeconds(int idx) { return DivBeats(idx) * 60.0f / tempo_bpm; }
FLASHMEM static void FormatDiv(int idx, char* buf, size_t n) { ClockDiv(idx, nullptr, buf, n); }

// trigger source numbering: 0 off, 1-4 gate in, 5-12 CV in, 13+ MIDI map
constexpr int TRIG_SOURCES = 1 + 4 + 8 + MIDIMAP_MAX + 8;  // ... then MOD A-H (a modulator past half way)
constexpr int TRIG_MOD0 = 1 + 4 + 8 + MIDIMAP_MAX;  // MOD A

FLASHMEM static void FormatTrig(int src, char* buf, size_t n) {
  if (src <= 0) snprintf(buf, n, "none");
  else if (src <= 4) snprintf(buf, n, "TR%d", src);
  else if (src <= 12) snprintf(buf, n, "CV%d", src - 4);
  else if (src < TRIG_MOD0) snprintf(buf, n, src - 12 < 10 ? "MIDI%d" : "MID%d", src - 12);  // (5 letters at most: MID10)
  else snprintf(buf, n, "MOD %c", 'A' + (src - TRIG_MOD0));
}

FLASHMEM static bool TrigGate(int src) {
  if (src <= 0) return false;
  if (src <= 12) return HS::frame.gate_high[src - 1];
  if (src >= TRIG_MOD0) return false;  // (MOD A-H: the device's own bus, see Device::SrcGate)
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
  int8_t ui_cursor = 0;  // the setting its page was left on (opens there again)
  uint8_t act_trig[2] = {};              // trigger sources for the 2 actions
  bool act_prev[2] = {};
  // Some modulators pick an action's trigger with one of their own settings
  // (ADSR's TRIG for TRG, WAVE's RESET for RST ...). For those the trigger
  // page and the setting are one and the same: this says which setting
  // (-1: none, the trigger page keeps its own choice in act_trig).
  virtual int TrigParam(int a) const { return -1; }
  // the trigger source of action a, wherever it is kept
  uint8_t TrigSource(int a) const {
    const int p = TrigParam(a);
    return p >= 0 ? uint8_t(val[p]) : act_trig[a];
  }
  // set it from the trigger page: into the setting if there is one (the
  // modulator reads that itself, so act_trig stays off and nothing fires
  // twice), else into act_trig
  void SetTrigSource(int a, int src) {
    src = constrain(src, 0, TRIG_SOURCES - 1);
    const int p = TrigParam(a);
    if (p >= 0) { act_trig[a] = 0; val[p] = int16_t(src); Refresh(p); }
    else act_trig[a] = uint8_t(src);
  }
  const float* mod_bus = nullptr;        // the track's modulator outputs
  // a trigger source as this device sees it: MOD A-H are its track's
  // modulators (a hit once one passes half way), the rest the inputs / MIDI
  bool SrcGate(int src) const {
    if (src >= TRIG_MOD0) return mod_bus && src - TRIG_MOD0 < MOD_BUSES && mod_bus[src - TRIG_MOD0] > 0.5f;
    return TrigGate(src);
  }
  bool SrcEdge(int src, uint16_t edges, bool& prev) const {
    if (src < TRIG_MOD0) return TrigEdge(src, edges, prev);
    const bool g = SrcGate(src), e = g && !prev;
    prev = g;
    return e;
  }
  uint16_t mod_bi = 0;               // bit per param: bipolar around val
  uint16_t mod_bi2 = 0;              // the same for the second source
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
  // the upper row: a setting with named modes reads its own name (what it
  // is for) until its mode is turned, then the mode it is on (SPEED: FWD /
  // REV / SYNC)
  virtual const char* Name(int p) const {
    const ParamDef& d = param(p);
    if (d.nmodes && d.nmodes <= 4 && mode[p] < 4 && d.modes[mode[p]][0] && ModeNamed(p)) return d.modes[mode[p]];
    return d.name;
  }
  bool ModeNamed(int p) const { return mode_named >> p & 1; }
  uint16_t mode_named = 0;  // bit per setting: its mode turned (by hand)
  // a setting taken once per event (a grain's PITCH): it follows a
  // modulator's step, not its glide
  virtual bool Sampled(int p) const { return false; }
  virtual void Action(int a) {}
  virtual bool ActionLit(int a) const { return false; }
  virtual bool LongA() { return false; }  // A held on the page: true if the device took it
  virtual void DrawViz(int x, int y, int w, int h) {}
  virtual void DrawMini(int x, int y, int w, int h) {}
  virtual void Idle() {}  // main loop, while the device is in use
  virtual void Clocked() {}  // each modulation tick, after the modulators and this device's settings
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
    uint16_t mod_bi, mod_bi2;
    uint32_t ex[2];  // PLAYER: its exact edges
    bool ex_valid;
  };
  void Keep(Saved& k) const;
  void Restore(const Saved& k);
  virtual void KeepMore(Saved&) const {}
  // a short note for the page's header bar (false: none)
  virtual bool HeaderInfo(char*, size_t) const { return false; }
  // the file list (LoomBrowser): R push on FileParam() opens it; a WAV
  // picked in it comes to Picked
  virtual int FileParam() const { return -1; }
  virtual void OpenBrowser() {}
  virtual void Picked(const char* dir, const char* name) {}
  // the WAV under the list's cursor is Picked as the cursor moves (WARP:
  // browse the thrums by ear)
  virtual bool PicksWhileListing() const { return false; }
  // the list opens on a double click of R (one click grabs the setting as
  // usual) instead of on a single one (WARP: R turns through the folder)
  virtual bool ListsOnDoubleClick() const { return false; }
  virtual void RestoreMore(const Saved&) {}
  void Nudge(int p, int dir);
  virtual void NudgeMode(int p, int dir);
  // the lowest mode the L knob can pick now (SHUTTLE streaming: no REV)
  virtual int ModeMin(int p) const { return 0; }
  // the upper row shows the mode instead of the name all the time
  virtual bool ModeShown(int p) const { return false; }
  // the setting's name already says which mode it is on, in the device's
  // own way (TWILL's SPRD names its bells, REPEAT's levels read SEND or
  // MIX): the upper row keeps that name and never swaps it for the mode
  virtual bool NameIsMode(int p) const { return false; }
  // a setting whose mode is a second value of its own (REPEAT: ROOM /
  // GLINT, WALL / PRSET): the L knob switches between the two (the bar's
  // left / right half), the R knob changes the one showing; its name here
  virtual const char* AltName(int p) const { return nullptr; }
  // the bar over the name: the mode's part (BAR_PARTS), which of the two
  // shows (BAR_HALVES), or a variation's amount growing from the left /
  // the middle (HEDDLE's FDBK / its ± STAGE)
  enum { BAR_PARTS, BAR_HALVES, BAR_LEFT, BAR_CENTRE };
  virtual int BarStyle(int p) const { return AltName(p) ? BAR_HALVES : Varies(p) ? BAR_CENTRE : BAR_PARTS; }
  // a variation up or down by a percentage: "±35%" (the font's ± is char 127, LOOM_PM)
  static void FormatVar(char* buf, size_t n, int pct) { snprintf(buf, n, LOOM_PM "%d%%", pct); }
  uint16_t alt_on = 0;  // which of those show their second value (the page's, not saved)
  bool AltShown(int p) const { return AltName(p) && (alt_on >> p & 1); }
  // a setting that goes round when turned past an end (DRAFT's POS: its
  // recording is a loop)
  virtual bool Wraps(int p) const { return false; }
  // a setting in semitones (PITCH, NOTE): it can follow MIDI notes
  bool PitchSetting(int p) const {
    const ParamDef& d = param(p);
    return d.kind != LIST && (!strcmp(d.name, "PITCH") || !strcmp(d.name, "NOTE"));
  }
  // MIDI as a source: keys from C3, scaled so +100 % is a semitone a key
  float MidiSource(int p) const {
    const int note = midi_note_seq[0] ? midi_note[0] : 60;  // (none yet: no change)
    return (note - 60) / float(std::max(1, param(p).max - param(p).min));
  }
  bool Bipolar(int p) const { return mod_bi & (1 << p); }
  bool Bipolar2(int p) const { return mod_bi2 & (1 << p); }
  bool Modded(int p) const { return mod_amt[p] || mod_amt2[p]; }
  // a source with an amount that really moves the setting (not off, not an
  // empty MOD slot)
  bool ModLive(int p, int k) const {
    if (k) return mod_amt2[p] && mod_bus2[p]
                  && (mod_bus2[p] == MIDI_BUS || (mod_bus && mod_bus[mod_bus2[p] - 1] >= 0.0f));
    return mod_amt[p] && ModSourceOn(p)
           && !(mod_bus_sel[p] && mod_bus_sel[p] != MIDI_BUS && mod_bus && mod_bus[mod_bus_sel[p] - 1] < 0.0f);
  }
  bool ModsLive(int p) const { return ModLive(p, 0) || ModLive(p, 1); }
  // the lowest and highest value the live sources together can give
  void ModRange(int p, int& lo, int& hi) const {
    float a = 0.0f, b = 0.0f;  // percent
    for (int k = 0; k < 2; k++) {
      if (!ModLive(p, k)) continue;
      const int amt = k ? mod_amt2[p] : mod_amt[p];
      const bool midi = (k ? mod_bus2[p] : mod_bus_sel[p]) == MIDI_BUS;  // (keys both ways from C3)
      if (midi || (k ? Bipolar2(p) : Bipolar(p))) { a -= abs(amt); b += abs(amt); }
      else if (amt > 0) b += amt;
      else a += amt;
    }
    const ParamDef& d = param(p);
    lo = constrain(val[p] + int(a * 0.01f * (d.max - d.min)), d.min, d.max);
    hi = constrain(val[p] + int(b * 0.01f * (d.max - d.min)), d.min, d.max);
  }
  int Modulated(int p) const;
  // a variation (the L knob, OFF .. +- 100 % of the range in 5 % steps):
  // each clock beat a new random offset, glided to, added like modulation;
  // a device says which settings vary (where the L knob is free)
  virtual bool Varies(int p) const { return false; }
  float var_now[MAX_PARAMS] = {}, var_to[MAX_PARAMS] = {};
  uint32_t var_beat = 0, var_ms = 0;
  void VarTick();
  void VarNew() { var_beat = loom::clock_beat_n + 1; }  // new variations at the next tick, as on a beat
  int VarAdd(int p, int v) const {
    if (!Varies(p) || var_now[p] == 0.0f) return v;
    const ParamDef& d = param(p);
    return constrain(v + int(lrintf(var_now[p] * 0.01f * (d.max - d.min))), int(d.min), int(d.max));
  }
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
  if (!one && !two) return VarAdd(p, val[p]);
  // the two sources add up, each by its own amount
  float sum = 0.0f;
  if (one) {
    const int at = Sampled(p) ? MOD_BUSES - 1 : -1;  // (the steps follow the outputs)
    float c;
    if (mod_bus_sel[p] == MIDI_BUS) c = MidiSource(p);
    else {
      c = (mod_bus_sel[p] && mod_bus) ? mod_bus[mod_bus_sel[p] + at]
                                      : constrain(mod_in[p].InF(), -1.0f, 1.0f);
      if (Bipolar(p)) c = 2.0f * c - 1.0f;
    }
    sum += mod_amt[p] * c;
  }
  if (two) {
    float c;
    if (mod_bus2[p] == MIDI_BUS) c = MidiSource(p);
    else {
      c = mod_bus[mod_bus2[p] + (Sampled(p) ? MOD_BUSES - 1 : -1)];
      if (Bipolar2(p)) c = 2.0f * c - 1.0f;
    }
    sum += mod_amt2[p] * c;
  }
  // (MIDI: exact semitones, rounded rather than cut)
  if ((one && mod_bus_sel[p] == MIDI_BUS) || (two && mod_bus2[p] == MIDI_BUS))
    return VarAdd(p, constrain(val[p] + int(lrintf(sum * 0.01f * (d.max - d.min))), int(d.min), int(d.max)));
  const int v = val[p] + int(sum * 0.01f * (d.max - d.min));
  return VarAdd(p, constrain(v, d.min, d.max));
}

FLASHMEM void Device::Refresh(int p) {
  cur[p] = Modulated(p);
  Apply(p);
}

FLASHMEM void Device::ApplyAll() {
  for (int p = 0; p < num_params(); p++) Refresh(p);
}

FLASHMEM void Device::Modulate() {
  VarTick();
  for (int p = 0; p < num_params(); p++) {
    if (!Modded(p) && cur[p] == val[p] && !(Varies(p) && var_now[p] != 0.0f)) continue;
    const int m = Modulated(p);
    if (m != cur[p]) {
      cur[p] = m;
      Apply(p);
    }
  }
}

FLASHMEM void Device::RunTriggers(uint16_t edges) {
  for (int a = 0; a < 2; a++) {
    if (TrigParam(a) >= 0) continue;  // (the modulator watches that source itself)
    if (act_trig[a] && SrcEdge(act_trig[a], edges, act_prev[a])) Action(a);
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
  if (bus == MIDI_BUS) {  // MIDI sits left of off
    if (dir > 0) { bus = 0; in.Unpack(0); }
  } else if (!bus && !in.enabled() && dir < 0) {
    if (PitchSetting(p)) bus = MIDI_BUS;
  } else if (bus) {
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
  if (mod_bus2[p] == MIDI_BUS) { if (dir > 0) mod_bus2[p] = 0; }       // MIDI sits left of off
  else if (!mod_bus2[p] && dir < 0) { if (PitchSetting(p)) mod_bus2[p] = MIDI_BUS; }
  else mod_bus2[p] = constrain(mod_bus2[p] + dir, 0, MOD_BUSES);
  Refresh(p);
}

FLASHMEM void Device::ModSourceName(int p, char* buf, size_t n) const {
  if (mod_bus_sel[p] == MIDI_BUS) snprintf(buf, n, "MIDI");
  else if (mod_bus_sel[p]) snprintf(buf, n, "Md%c", 'A' + mod_bus_sel[p] - 1);
  else snprintf(buf, n, "%s", mod_in[p].InputName());
}

FLASHMEM void Device::Nudge(int p, int dir) {
  const ParamDef& d = param(p);
  const int16_t was = val[p];
  int v = Step(p, val[p], dir);
  if (Wraps(p) && v > d.max) v = d.min + (v - d.max - 1);
  else if (Wraps(p) && v < d.min) v = d.max - (d.min - v - 1);
  val[p] = constrain(v, d.min, d.max);
  last_delta[p] = val[p] - was;  // for a LINKED twin
  Refresh(p);
}

FLASHMEM void Device::NudgeMode(int p, int dir) {
  const ParamDef& d = param(p);
  if (!d.nmodes) return;
  mode[p] = constrain(mode[p] + dir, ModeMin(p), d.nmodes - 1);
  mode_named |= uint16_t(1u << p);
  Apply(p);
}

// the variations: on each clock beat a new offset for every setting that
// varies, then a glide there (~50 ms; LEVEL, CUT ... no jumps)
FLASHMEM void Device::VarTick() {
  const uint32_t now = millis();
  if (now - var_ms < 2) return;
  const float dt = std::min(0.1f, (now - var_ms) * 0.001f);
  var_ms = now;
  const bool beat = loom::clock_beat_n != var_beat;
  var_beat = loom::clock_beat_n;
  const float k = std::min(1.0f, dt * 20.0f);
  for (int p = 0; p < num_params(); p++) {
    if (!Varies(p)) continue;
    if (!mode[p]) var_to[p] = 0.0f;
    else if (beat) var_to[p] = random(-1000, 1001) * 0.001f * mode[p] * 5.0f;
    if (var_now[p] != var_to[p]) {
      var_now[p] += (var_to[p] - var_now[p]) * k;
      if (fabsf(var_now[p] - var_to[p]) < 0.05f) var_now[p] = var_to[p];
    }
  }
}

FLASHMEM void Device::FormatMode(int p, char* buf, size_t n) const {
  if (Varies(p)) {  // the variation: OFF, +-5 .. +-100 %
    if (mode[p]) FormatVar(buf, n, mode[p] * 5);
    else snprintf(buf, n, "OFF");
    return;
  }
  const int m = mode[p];
  // unnamed modes (amounts, zoom steps): the number, never a blank
  if (m < 4 && param(p).modes[m][0]) snprintf(buf, n, "%s", param(p).modes[m]);
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
// BOLT (was TAPE): the whole output (both tracks after OUTPUT, not the file list's
// quick listen) recorded to the card as /LOOM/BOLTS/Bolt_S014_02.wav, for as
// long as it runs. The mixer hands every block over in the audio interrupt
// into a ring in PSRAM; the main loop writes the ring to the card, so the
// card's pauses (up to a second or so) don't lose anything. The header is
// brought up to date every few seconds, so a file cut off by the power
// still plays up to there. Started / stopped with A on OUTPUT.
/////////////////////////////////////////////////////////////////
class TapeRecorder {
public:
  static constexpr char DIR[] = "/LOOM/BOLTS";  // (was TAPES until 2026-10-10)
  static constexpr char OLD_DIR[] = "/LOOM/TAPES";
  enum Fault : uint8_t { OK, NO_CARD, NO_MEM };
  // A on OUTPUT (or its gate / MIDI): start or stop, done in Idle; both
  // tracks' OUTPUT may fire on the same trigger: that is one press
  void Toggle() {
    if (millis() - toggled < 100) return;
    toggled = millis();
    want = !want;
  }
  bool Wanted() const { return want; }
  bool Recording() const { return file_open; }
  void Idle();    // main loop: starts, stops, writes the ring out
  void Finish();  // at once, everything written (Loom pausing)
  uint32_t Seconds() const { return uint32_t(written / uint64_t(AUDIO_SAMPLE_RATE_EXACT)); }
  bool slow = false;      // the card fell behind: audio was lost
  Fault fault = OK;       // the last start that failed
  uint32_t fault_at = 0;
  char name[32] = "";
  // audio interrupt: one block of the output, before it is clipped
  void Push(const int32_t* l, const int32_t* r) {
    if (!on) return;
    if (cap - (wr - rd) < AUDIO_BLOCK_SAMPLES) { slow = true; return; }
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      int16_t* p = ring + 2 * ((wr + n) & (cap - 1));
      p[0] = SoftClip16(l[n]);
      p[1] = SoftClip16(r[n]);
    }
    wr += AUDIO_BLOCK_SAMPLES;
  }
private:
  int16_t* ring = nullptr;  // stereo frames, interleaved as in the file
  uint32_t cap = 0;         // frames, a power of two
  volatile uint32_t wr = 0, rd = 0;  // frames, running on (the ring wraps)
  volatile bool on = false;
  bool want = false, file_open = false;
  uint32_t toggled = 0, header_at = 0;
  uint64_t written = 0;  // frames
  File f;
  bool Begin();
  size_t Drain(size_t most);  // frames written
  void Header();
  void Close();
};
inline TapeRecorder* g_tape = nullptr;

/////////////////////////////////////////////////////////////////
// Noise test (USB 'N'): both tracks' INPUT takes a pure tone in place of
// the jacks while the output is muted. The final mix is matched with the
// best-fitting tone at that pitch, and what is left over (noise and
// distortion added on the way) is reported. All digital: the same result
// on any system, whatever its power supply.
/////////////////////////////////////////////////////////////////
class NoiseTest {
public:
  static constexpr double HZ = 997.0;  // not a whole fraction of 48 kHz
  // silence first (whatever played before has time to die), then up from
  // quiet, so each step's tail is below the next one
  static constexpr int STEPS = 5;
  static constexpr float STEP_DB[STEPS] = {-999.0f, -60.0f, -40.0f, -20.0f, -6.0f};
  static constexpr int SETTLE_BLOCKS = 3 * 375;  // ~3 s
  static constexpr int MEASURE_BLOCKS = 2 * 375; // ~2 s
  // where it listens (left channels only inside the tracks)
  enum { OUT_L, OUT_R, A_INPUT, B_INPUT, A_TO_MIX, B_TO_MIX, TAPS };
  void Start() {
    step = 0;
    blocks = 0;
    ClearSums();
    Level();
    Next();
    done = false;
    running = true;
  }
  bool Running() const { return running; }
  bool Done() const { return done; }
  void Report();  // main loop, once Done
  // INPUT (audio interrupt): this cycle's tone, both channels
  const int16_t* Tone() const { return tone; }
  // the mixer (audio interrupt): one block of the final mix, then the next
  // block of the tone
  void Measure(const int32_t* l, const int32_t* r) {
    if (!running) return;
    if (blocks >= SETTLE_BLOCKS) {
      const int32_t* y[2] = {l, r};
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        const double s = ref_s[n], c = ref_c[n];
        x.ss += s * s; x.cc += c * c; x.sc += s * c; x.s += s; x.c += c;
        for (int ch = 0; ch < 2; ch++) Add(ch_sum[ch], y[ch][n], s, c);
      }
      x.n += AUDIO_BLOCK_SAMPLES;
    }
    if (++blocks >= SETTLE_BLOCKS + MEASURE_BLOCKS) {
      result[step].x = x;
      for (int k = 0; k < TAPS; k++) result[step].ch[k] = ch_sum[k];
      ClearSums();
      blocks = 0;
      if (++step >= STEPS) { running = false; done = true; }
      else Level();
    }
    Next();
  }
  // a point inside the path (audio interrupt, before or after the mixer:
  // the tone's block stays the same until the mixer is done)
  void Tap(int k, const audio_block_t* b) {
    if (!running || blocks < SETTLE_BLOCKS) return;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) Add(ch_sum[k], b ? b->data[n] : 0, ref_s[n], ref_c[n]);
  }
private:
  struct XSums { double ss, cc, sc, s, c, n; };
  struct YSums { double yy, ys, yc, y; };
  struct Result { XSums x; YSums ch[TAPS]; };
  static void Add(YSums& y, double v, double s, double c) {
    y.yy += v * v; y.ys += v * s; y.yc += v * c; y.y += v;
  }
  volatile bool running = false, done = false;
  int step = 0, blocks = 0;
  double amp = 0.0;
  double re = 1.0, im = 0.0;  // the tone's phase, turned each sample
  double ref_s[AUDIO_BLOCK_SAMPLES] = {}, ref_c[AUDIO_BLOCK_SAMPLES] = {};
  int16_t tone[AUDIO_BLOCK_SAMPLES] = {};
  XSums x;
  YSums ch_sum[TAPS];
  Result result[STEPS];
  void ClearSums() { x = {}; for (auto& c : ch_sum) c = {}; }
  void Level() { amp = STEP_DB[step] < -200.0f ? 0.0 : 32767.0 * pow(10.0, STEP_DB[step] / 20.0); }
  void Next() {
    const double w = 2.0 * M_PI * HZ / AUDIO_SAMPLE_RATE_EXACT;
    const double cw = cos(w), sw = sin(w);
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      ref_s[n] = im;
      ref_c[n] = re;
      tone[n] = int16_t(lrint(amp * im));  // rounded to 16 bit, like a jack
      const double r2 = re * cw - im * sw;
      im = re * sw + im * cw;
      re = r2;
    }
    const double m = 1.0 / sqrt(re * re + im * im);  // keep it on the circle
    re *= m;
    im *= m;
  }
};
inline NoiseTest* g_noise = nullptr;

/////////////////////////////////////////////////////////////////
// Output capture (USB 'W'): the next 2 s of the output as it goes to the
// jacks and of the input jacks as they come in, held in PSRAM, then sent
// over USB in hex lines ("w" + 16 frames of out L, out R, in 1, in 2), a
// bit per main loop. For listening from the Mac.
/////////////////////////////////////////////////////////////////
class OutCapture {
public:
  static constexpr uint32_t FRAMES = 2 * 48000;
  static constexpr int CH = 4;
  bool track_b = false;  // USB 'U': track B on its way into the mixer instead of the jacks
  bool track_ab = false; // USB 'm': ... track A L and track B L instead
  // blocks a track did not deliver to the mixer while capturing (that
  // track silent for 2.7 ms)
  volatile uint32_t gaps[2] = {};
  // only: -1 all four; else that one channel (0 out L .. 3 in 2 / track B
  // R): a quarter of the memory, for when the PSRAM is full (USB 'M')
  bool Start(bool b_side = false, int only = -1, bool ab = false) {
    track_b = b_side || ab;
    track_ab = ab;
    gaps[0] = gaps[1] = 0;
    const int nch = only < 0 ? CH : 1;
    if (buf && nch != chans) { PsramFree(buf); buf = nullptr; }
    chans = nch;
    one = only;
    if (!buf) buf = static_cast<int16_t*>(PsramAlloc(FRAMES * chans * 2));
    if (!buf) return false;
    sent = 0;
    wr[0] = wr[1] = 0;
    on = true;
    return true;
  }
  // audio interrupt: one block of the output (part 0) or of the input jacks
  // (part 1)
  void Push(int part, const int16_t* a, const int16_t* b) {
    if (!on) return;
    uint32_t w = wr[part];
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES && w < FRAMES; n++, w++) {
      if (one < 0) {
        buf[CH * w + 2 * part] = a[n];
        buf[CH * w + 2 * part + 1] = b[n];
      } else if (one / 2 == part) {
        buf[w] = one & 1 ? b[n] : a[n];
      }
    }
    wr[part] = w;
    if (wr[0] >= FRAMES && wr[1] >= FRAMES) on = false;
  }
  bool Recording() const { return on; }
  // main loop: send what is captured, a few lines at a time
  void Idle() {
    if (!buf || on || sent >= FRAMES) return;
    if (sent == 0) {
      if (one < 0) Serial.printf("CAPTURE %lu frames 48000 Hz: out L, out R, %s (gaps A %lu B %lu)\n", (unsigned long)FRAMES,
                                 track_ab ? "track A L, track B L" : track_b ? "track B L, R" : "in 1, in 2",
                                 (unsigned long)gaps[0], (unsigned long)gaps[1]);
      else Serial.printf("CAPTURE %lu frames 48000 Hz, 1 channel: %d\n", (unsigned long)FRAMES, one);
    }
    char line[2 + 16 * CH * 4 + 2];
    // only while USB has room for a whole line, else on the next pass (a
    // full buffer made Serial drop whole lines: gaps in the capture)
    for (int k = 0; k < 8 && sent < FRAMES && Serial.availableForWrite() >= 2 * 64; k++) {
      int p = 0;
      line[p++] = 'w';
      for (int f = 0; f < 16 && sent < FRAMES; f++, sent++)
        for (int c = 0; c < chans; c++) p += sprintf(line + p, "%04X", uint16_t(buf[chans * sent + c]));
      line[p++] = '\n';
      line[p] = 0;
      Serial.write(line, p);
    }
    if (sent >= FRAMES) {
      Serial.println("--- end ---");
      if (one >= 0) { PsramFree(buf); buf = nullptr; }  // the room back
    }
  }
private:
  int16_t* buf = nullptr;
  int chans = CH, one = -1;
  volatile uint32_t wr[2] = {};
  volatile bool on = false;
  uint32_t sent = FRAMES;
};
inline OutCapture* g_capture = nullptr;

// dB against a full-scale sine (rms 32767 / sqrt 2)
static inline float NoiseDb(double rms) {
  return rms > 0.0 ? float(20.0 * log10(rms / (32767.0 / M_SQRT2))) : -999.0f;
}
// "-98.1" (this printf has no %f)
static inline const char* DbText(float db, char* buf) {
  const int t = int(lrintf(fabsf(db) * 10.0f));
  sprintf(buf, "%s%d.%d", db < 0.0f && t ? "-" : "", t / 10, t % 10);
  return buf;
}

FLASHMEM void NoiseTest::Report() {
  done = false;
  char d1[16];
  Serial.printf("NOISE TEST: a %d Hz tone into both tracks' INPUT (jacks ignored, output muted)\n", int(HZ));
  // one rounding to 16 bit: 1/sqrt(12) of a step
  Serial.printf("  dB against a full-scale sine; one rounding to 16 bit alone = %s dB\n",
                DbText(NoiseDb(1.0 / sqrt(12.0)), d1));
  static const char* const names[TAPS] = {"out L", "out R", "A input", "B input", "A to mix", "B to mix"};
  for (int table = 0; table < 2; table++) {
    Serial.printf(table ? "  LEFT OVER (noise + distortion)\n" : "  TONE LEVEL\n");
    Serial.printf("  tone in  ");
    for (auto nm : names) Serial.printf(" %9s", nm);
    Serial.println();
    for (int i = 0; i < STEPS; i++) {
      const Result& r = result[i];
      const XSums& s = r.x;
      if (s.n <= 0) continue;
      if (STEP_DB[i] < -200.0f) Serial.printf("  silence ");
      else Serial.printf("  %4d dB  ", int(STEP_DB[i]));
      for (int ch = 0; ch < TAPS; ch++) {
        const YSums& y = r.ch[ch];
        // least squares: y ~ a sin + b cos + d (the 3x3 normal equations)
        const double M[3][3] = {{s.ss, s.sc, s.s}, {s.sc, s.cc, s.c}, {s.s, s.c, s.n}};
        const double v[3] = {y.ys, y.yc, y.y};
        auto det3 = [](const double A[3][3]) {
          return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
               - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
               + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        };
        const double det = det3(M);
        double b[3];
        for (int k = 0; k < 3; k++) {  // Cramer's rule
          double A[3][3];
          for (int i2 = 0; i2 < 3; i2++)
            for (int j = 0; j < 3; j++) A[i2][j] = j == k ? v[i2] : M[i2][j];
          b[k] = det3(A) / det;
        }
        const double left = std::max(0.0, y.yy - (b[0] * v[0] + b[1] * v[1] + b[2] * v[2]));
        const double tone_rms = sqrt((b[0] * b[0] + b[1] * b[1]) / 2.0);
        Serial.printf(" %9s", DbText(NoiseDb(table ? sqrt(left / s.n) : tone_rms), d1));
      }
      Serial.println();
      delay(2);
    }
  }
  Serial.println("--- end ---");
}
// 12:34, or 1:02:03 past an hour
inline void FormatTapeTime(uint32_t s, char* buf, size_t n) {
  if (s >= 3600) snprintf(buf, n, "%lu:%02lu:%02lu", s / 3600, s / 60 % 60, s % 60);
  else snprintf(buf, n, "%lu:%02lu", s / 60, s % 60);
}

/////////////////////////////////////////////////////////////////
// Track mixer: both tracks -> stereo out
/////////////////////////////////////////////////////////////////
class LoomMixer : public AudioStream {
public:
  LoomMixer() : AudioStream(6, queue) {}  // 4, 5: the file list's quick listen
  volatile float gain[2][2] = {{0.8f, 0.8f}, {0.8f, 0.8f}}; // [track][L/R]
  float gain_now[2][2] = {{0.8f, 0.8f}, {0.8f, 0.8f}};      // ... as heard: glides there (~5 ms; MUTE clicked)
  volatile float level[2] = {}; // last block's peak per track, for FOLLOW
  // STEREO link: a track's two channels summed into one side of the output
  // (0 both sides as usual, 1 all to the left, 2 all to the right)
  volatile uint8_t route[2] = {0, 0};
  float peak[2][2] = {};  // held until read, [track][L/R]
  volatile bool mute = false;   // the whole output, fading over ~15 ms
  float mute_g = 1.0f;
  // a track faded out (~5 ms) and back, so its cords can change in the
  // quiet (an effect moved along the chain clicked)
  volatile float duck_to[2] = {1.0f, 1.0f};
  volatile float duck[2] = {1.0f, 1.0f};
  // short gaps: a track sent nothing for 1..8 blocks between sound (the
  // dropout log); miss: blocks missed in a row so far
  volatile uint32_t gaps[2] = {};
  uint8_t miss[2] = {};
  float lim_g = 1.0f;          // the output limiter's gain now
  volatile float lim_low = 1.0f; // ... its lowest since the status print took it
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
  audio_block_t* queue[6];
};

FASTRUN __attribute__((optimize("O2"))) void LoomMixer::update() {  // (O2: the build is -Os)
  audio_block_t* in[6];
  for (int i = 0; i < 6; i++) in[i] = receiveReadOnly(i);
  audio_block_t* out[2] = {allocate(), allocate()};
  if (out[0] && out[1]) {
    int32_t acc[2][AUDIO_BLOCK_SAMPLES] = {};
    float blk[2] = {};
    for (int t = 0; t < 2; t++) {
      if (!in[t * 2]) { if (miss[t] < 255) miss[t]++; }
      else { if (miss[t] >= 1 && miss[t] <= 8) gaps[t]++; miss[t] = 0; }
    }
    for (int t = 0; t < 2; t++) {
      for (int c = 0; c < 2; c++) {
        const audio_block_t* b = in[t * 2 + c];
        if (!b) continue;
        const float g_to = gain[t][c];
        const int32_t g = static_cast<int32_t>(g_to * 32767.0f); // Q15
        const uint8_t rt = route[t];
        // routed: both channels at half into the one side
        const int to = rt ? rt - 1 : c;
        const int sh = rt ? 16 : 15;
        int32_t pk = 0;
        if (fabsf(gain_now[t][c] - g_to) > 1e-4f) {
          // the gain moved (LEVEL, PAN, MUTE): glide to it, with the duck
          float gn = gain_now[t][c], dk = duck[t];
          const float k = 1.0f / (0.005f * AUDIO_SAMPLE_RATE_EXACT);
          const float step = 1.0f / (0.005f * AUDIO_SAMPLE_RATE_EXACT), aim = duck_to[t];
          const float sc = rt ? 0.5f : 1.0f;
          for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
            gn += k * (g_to - gn);
            dk = aim > dk ? std::min(aim, dk + step) : std::max(aim, dk - step);
            const int32_t v = int32_t(lrintf(b->data[n] * gn * dk * sc));
            acc[to][n] += v;
            pk = std::max<int32_t>(pk, abs(v));
          }
          gain_now[t][c] = fabsf(gn - g_to) > 1e-4f ? gn : g_to;
          if (c == 1 || !in[t * 2 + 1]) duck[t] = dk;
        } else if (duck[t] >= 1.0f && duck_to[t] >= 1.0f) {
          for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
            const int32_t v = (b->data[n] * g + (1 << (sh - 1))) >> sh;  // rounded
            acc[to][n] += v;
            pk = std::max<int32_t>(pk, abs(v));
          }
        } else {  // fading (both channels the same way: the duck moves once per track, below)
          float dk = duck[t];
          const float step = 1.0f / (0.005f * AUDIO_SAMPLE_RATE_EXACT), aim = duck_to[t];
          for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
            dk = aim > dk ? std::min(aim, dk + step) : std::max(aim, dk - step);
            const int32_t v = int32_t(((b->data[n] * g + (1 << (sh - 1))) >> sh) * dk);
            acc[to][n] += v;
            pk = std::max<int32_t>(pk, abs(v));
          }
          if (c == 1 || !in[t * 2 + 1]) duck[t] = dk;
        }
        peak[t][c] = std::max(peak[t][c], pk * (1.0f / 32768.0f));
        blk[t] = std::max(blk[t], pk * (1.0f / 32768.0f));
      }
    }
    level[0] = blk[0];
    level[1] = blk[1];
    if (g_noise) {
      g_noise->Tap(NoiseTest::A_TO_MIX, in[0]);
      g_noise->Tap(NoiseTest::B_TO_MIX, in[2]);
      g_noise->Measure(acc[0], acc[1]);
    }
    if (g_capture && g_capture->track_b) {  // USB 'U': track B as it comes in ('m': A and B)
      static const int16_t zero[AUDIO_BLOCK_SAMPLES] = {};
      if (g_capture->track_ab) {
        g_capture->Push(1, in[0] ? in[0]->data : zero, in[2] ? in[2]->data : zero);
        for (int t = 0; t < 2; t++) if (!in[t * 2] && g_capture->Recording()) g_capture->gaps[t]++;
      } else {
        g_capture->Push(1, in[2] ? in[2]->data : zero, in[3] ? in[3]->data : zero);
      }
    }  // (before the mute: it mutes the output)
    float g[AUDIO_BLOCK_SAMPLES];  // the mute fade, for the quick listen too
    if (mute || mute_g < 1.0f) {  // fade the sum out / back in
      const float to = mute ? 0.0f : 1.0f, step = 1.0f / (0.015f * AUDIO_SAMPLE_RATE_EXACT);
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        mute_g = to > mute_g ? std::min(to, mute_g + step) : std::max(to, mute_g - step);
        g[n] = mute_g;
        acc[0][n] = int32_t(acc[0][n] * mute_g);
        acc[1][n] = int32_t(acc[1][n] * mute_g);
      }
    } else {
      for (auto& v : g) v = 1.0f;
    }
    if (g_tape) g_tape->Push(acc[0], acc[1]);  // the tracks only
    // the quick listen: as it is, past the tracks (not in their levels)
    for (int c = 0; c < 2; c++)
      if (const audio_block_t* b = in[4 + c])
        for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) acc[c][n] += int32_t(((b->data[n] * 26214) >> 15) * g[n]);
    // limiter (both sides together): turned down at once (across the
    // block) when the sum would go past the soft clip's knee (90 %), back
    // up over ~150 ms; CLOTH's LEVEL can go to 200 and stay clean
    {
      constexpr float TH = 29490.0f;
      int32_t pk = 0;
      for (int c = 0; c < 2; c++)
        for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) pk = std::max<int32_t>(pk, abs(acc[c][n]));
      const float need = pk > TH ? TH / float(pk) : 1.0f;
      const float from = lim_g;
      lim_g = need < lim_g ? need : std::min(need, lim_g + (1.0f - lim_g) * 0.02f);
      if (from < 1.0f || lim_g < 1.0f) {
        const int ramp = need < from ? 32 : AUDIO_BLOCK_SAMPLES;  // down within 32 samples
        const float step = (lim_g - from) / ramp;
        for (int c = 0; c < 2; c++) {
          float gg = from;
          for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
            if (n < ramp) gg += step;
            acc[c][n] = int32_t(lrintf(acc[c][n] * gg));
          }
        }
      }
      if (lim_g < lim_low) lim_low = lim_g;
    }
    for (int c = 0; c < 2; c++)
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) out[c]->data[n] = SoftClip16(acc[c][n]);
    if (g_capture) g_capture->Push(0, out[0]->data, out[1]->data);
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

FASTRUN __attribute__((optimize("O2"))) void LoomFade::update() {  // (O2: the build is -Os)
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
          out[c]->data[n] = ToSample16(d * g_dry + w * g_wet);
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

FASTRUN __attribute__((optimize("O2"))) void LoomGate::update() {  // (O2: the build is -Os)
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
  "CLOTH", "MIX", 8, {"REC", "DUCK"},  // B (trigger page; the button is bypass): a dip, a kick pumping the track  // (was OUTPUT) the tag stays MIX: saved files use it; REC: BOLT
  {
    // the chain against the track's source as it came in (0 = only the
    // source, 100 = only the chain): all the effects dialled back at once
    {"MIX", 0, 100, 100, UNI, 0, 21, 0},
    {"LEVEL", 0, 200, 80, UNI, 0, 21, 0},  // (a double click: 100) 100: as it comes, 200: twice (a limiter on the output keeps it clean)
    {"PAN", -50, 50, 0, BI, 0, 21, 0},
    {"MUTE", 0, 1, 0, LIST, 2, 0, 0, {"off", "ON"}},
    // three bands: the band's level, -12 .. +12 dB; mode: its compressor,
    // how hard it squashes (off, 5 .. 100 %)
    {"LOW", -12, 12, 0, BI, 0, 21, 0},
    {"MID", -12, 12, 0, BI, 0, 21, 0},
    {"HIGH", -12, 12, 0, BI, 0, 21, 0},
    {"SPLIT", 0, 4, 2, UNI},  // where the bands divide: 120 / 1.5k .. 500 / 6k
  }
};

// level / pan / mute and a 3-band compressor (after 3-Bandz).
// The track's audio runs through the band stage, then into the mixer.
class MixDevice : public Device {
public:
  enum { DRYWET, LEVEL, PAN, MUTE, B_LOW, B_MID, B_HIGH, SPLIT };
  bool Varies(int p) const override { return p == DRYWET || p == LEVEL || p == PAN; }
  static constexpr float LO_HZ[5] = {120.0f, 180.0f, 250.0f, 350.0f, 500.0f};
  static constexpr float HI_HZ[5] = {1500.0f, 2000.0f, 3000.0f, 4000.0f, 6000.0f};
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
  void FormatMode(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  bool Muted() const { return cur[MUTE]; }
  // A: TAPE, the whole output to the card (the same on both tracks)
  void Action(int a) override {
    if (a == 0 && g_tape) g_tape->Toggle();
    if (a == 1) bandz.duck.Duck();
  }
  bool ActionLit(int a) const override { return a == 0 && g_tape && g_tape->Wanted(); }
  bool HeaderInfo(char* buf, size_t n) const override;
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
  switch (p) {
    case DRYWET:
      if (out_fade) {  // equal power; exact at the ends (the fade then just passes one side on)
        const float m = cur[DRYWET] * 0.01f * 1.5707963f;
        if (cur[DRYWET] >= 100) out_fade->Set(0.0f, 1.0f);
        else if (cur[DRYWET] <= 0) out_fade->Set(1.0f, 0.0f);
        else out_fade->Set(cosf(m), sinf(m));
      }
      return;
    case B_LOW: case B_MID: case B_HIGH:
      bandz.compress(p - B_LOW, mode[p] * 0.05f);
      bandz.band_gain(p - B_LOW, float(cur[p]));
      return;
    case SPLIT:
      bandz.split_low(LO_HZ[constrain(cur[SPLIT], 0, 4)]);
      bandz.split_high(HI_HZ[constrain(cur[SPLIT], 0, 4)]);
      return;
    default: break;
  }
  if (!mixer) return;
  const float lvl = cur[MUTE] ? 0.0f : cur[LEVEL] * 0.01f;
  const float pan = mixer->route[track] ? 0.0f : cur[PAN] * 0.02f; // -1..1 (STEREO link: no pan, the side is fixed)
  // equal power: -3 dB each side in the middle, so a sound keeps its
  // loudness wherever it is placed (was a linear balance until 2026-10-10)
  const float a = (pan + 1.0f) * 0.25f * float(M_PI);
  mixer->gain[track][0] = lvl * cosf(a);
  mixer->gain[track][1] = lvl * sinf(a);
}

// SPLIT shows where the low band ends (the picture above shows both)
FLASHMEM void MixDevice::Format(int p, char* buf, size_t n) const {
  if (p == SPLIT) snprintf(buf, n, "%d", int(LO_HZ[constrain(val[SPLIT], 0, 4)]));
  else if (p >= B_LOW && p <= B_HIGH) snprintf(buf, n, val[p] > 0 ? "+%ddB" : "%ddB", val[p]);
  else Device::Format(p, buf, n);
}

// a band's compressor: how hard it squashes
FLASHMEM void MixDevice::FormatMode(int p, char* buf, size_t n) const {
  if (p >= B_LOW && p <= B_HIGH) {
    if (mode[p]) snprintf(buf, n, "C%d%%", mode[p] * 5);
    else snprintf(buf, n, "C off");
  } else {
    Device::FormatMode(p, buf, n);
  }
}

FLASHMEM static void PrintHz(int x, int y, float hz) {
  char b[8];
  if (hz < 1000.0f) snprintf(b, sizeof(b), "%d", int(hz));
  else if (int(hz) % 1000) snprintf(b, sizeof(b), "%d.%dk", int(hz) / 1000, int(hz) % 1000 / 100);
  else snprintf(b, sizeof(b), "%dk", int(hz) / 1000);
  gfxPrint(x + (24 - 6 * int(strlen(b))) / 2, y, b);
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
  // above their LOW / MID / HIGH settings (columns of 32 px): the level
  // (R knob) as a bar from the centre line, and beside it how much the
  // band is squashed right now, filling a framed column from the top down
  // (full = 18 dB)
  if (view_page == 0) DrawSpectrum(x, y, 94, h);
  for (int b = 0; b < 3 && view_page == 1; b++) {
    const int bx = x + b * 32;  // above LOW / MID / HIGH
    const int gain = cur[B_LOW + b] * (h / 2 - 1) / 12;
    gfxDottedLine(bx + 3, cy, bx + 28, cy, 2);
    if (gain > 0) gfxRect(bx + 10, cy - gain, 8, gain);
    else if (gain < 0) gfxRect(bx + 10, cy, 8, -gain);
    else gfxLine(bx + 10, cy, bx + 17, cy);
    gfxFrame(bx + 21, y, 6, h);
    const int gr = int(gr_m[b].v * (h - 2) + 0.5f);
    if (gr > 0) gfxRect(bx + 22, y + 1, 4, gr);
  }

  // page 2, above SPLIT: where the bands divide, the high split on top
  if (view_page == 1) {
    const int sx = x + 96 + 4;
    PrintHz(sx, y + 1, HI_HZ[constrain(cur[SPLIT], 0, 4)]);
    gfxDottedLine(sx, y + h / 2, sx + 23, y + h / 2, 2);
    PrintHz(sx, y + h - 8, LO_HZ[constrain(cur[SPLIT], 0, 4)]);
    return;
  }
  // output meters with the pan position underneath, on the right
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
  // its buffers taken once and kept (malloc: none free, no picture; a
  // plain new can't say so and a 4 KB one each frame failed at a boot)
  if (!fft) fft = static_cast<float*>(malloc(sizeof(float) * 4 * N + sizeof(int16_t) * CAP));
  if (!fft) return;
  float* re = fft;          // full rate
  float* im = fft + N;
  float* dre = fft + 2 * N; // decimated
  float* dim = fft + 3 * N;
  int16_t* samples = reinterpret_cast<int16_t*>(fft + 4 * N);
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
  for (float hz : {LO_HZ[constrain(cur[SPLIT], 0, 4)], HI_HZ[constrain(cur[SPLIT], 0, 4)]}) {
    const int sx = x + int(log10f(hz / f0) / log10f(f1 / f0) * SPEC_BARS * bw);
    gfxDottedLine(sx, y, sx, y + h - 1, 3);
  }
}

// TAPE in the header: its time while it runs (SLOW blinking in when the
// card fell behind), or for 3 s why it could not start
FLASHMEM bool MixDevice::HeaderInfo(char* buf, size_t n) const {
  const TapeRecorder* t = g_tape;
  if (!t) return false;
  if (t->Recording()) {
    if (t->slow && (millis() / 500) % 2) { snprintf(buf, n, "SLOW"); return true; }
    FormatTapeTime(t->Seconds(), buf, n);
    return true;
  }
  if (t->fault && t->fault_at && millis() - t->fault_at < 3000) {
    snprintf(buf, n, t->fault == TapeRecorder::NO_MEM ? "NO MEM" : "NO CARD");
    return true;
  }
  return false;
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
  "INPUT", "IN", 4, {"MUTE", "DUCK"},  // A: off / on (faded); B: a dip, a kick pumping the input
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
  void Action(int a) override { if (a == 0) muted = !muted; else duck.Duck(); }
  bool ActionLit(int a) const override { return a == 0 && muted; }
  volatile float level = 0.0f; // last block's peak, for FOLLOW
private:
  audio_block_t* queue[4];
  volatile bool muted = false;
  float mute_g = 1.0f;  // faded (~3 ms)
  DuckEnv duck;
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
  // clipping: at the jack (the converter's limit: turn down what comes in)
  // or at GAIN (turn GAIN down); shown for a while after the last one
  volatile bool jack_clip = false, gain_clip = false;
  uint32_t jack_clip_at = 0, gain_clip_at = 0;
  static constexpr uint32_t CLIP_SHOW_MS = 1500;
public:
  int Clipping();  // 0 none, 1 at the jack, 2 at GAIN
  // the jacks' level as they come in (before GAIN), dB below the
  // converter's limit: up at once, back down slowly (readable); main loop,
  // once per screen
  float JackDb();
private:
  volatile int32_t jack_pk = 0;  // since the last JackDb
  float jack_db = -99.0f;
  uint32_t jack_db_at = 0, jack_peak_at = 0, jack_low_since = 0;
  int jack_shown = -99;  // the whole dB on screen (moves only 1 dB past it)
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

// USB 'o' test: a steady 440 Hz sine (-12 dB) in place of track B's
// jacks, so any chopping on its way shows
inline volatile uint8_t g_b_tone = 0;  // 1: the tone, 2: white noise (USB 'q')

FASTRUN __attribute__((optimize("O2"))) void InputDevice::update() {  // (O2: the build is -Os)
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
    if (!from_a && g_noise && g_noise->Running()) a = b = g_noise->Tone();  // the noise test
    static int16_t b_tone[AUDIO_BLOCK_SAMPLES];
    if (!from_a && chained && g_b_tone) {
      static float ph = 0.0f;
      static uint32_t rn = 22222;
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        rn ^= rn << 13; rn ^= rn >> 17; rn ^= rn << 5;
        if (g_b_tone == 2) { b_tone[n] = int16_t(int32_t(rn) >> 19); continue; }  // white, about -12 dB
        b_tone[n] = int16_t(lrintf(8192.0f * sinf(ph)));
        ph += 2.0f * float(M_PI) * 440.0f / AUDIO_SAMPLE_RATE_EXACT;
        if (ph > 2.0f * float(M_PI)) ph -= 2.0f * float(M_PI);
      }
      a = b = b_tone;
    }
    if (!chained && g_capture && !g_capture->track_b) g_capture->Push(1, a, b);  // the jacks as they come in (USB 'W')
    const int32_t g = static_cast<int32_t>(gain * 16384.0f);   // Q14, up to 2.0
    const int32_t wd = static_cast<int32_t>(width * 16384.0f); // Q14
    int16_t* l = out[0]->data;
    int16_t* r = out[1]->data;
    int32_t pk[2] = {0, 0};
    // the jacks used at the converter's limit (not the test tone, not OUT A)
    if (!from_a && !(g_noise && g_noise->Running()) && src != OFF) {
      int32_t jp = 0;
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        if (src != IN2) jp = std::max<int32_t>(jp, abs(a[n]));
        if (src != IN1) jp = std::max<int32_t>(jp, abs(b[n]));
      }
      if (jp >= 32760) jack_clip = true;
      if (jp > jack_pk) jack_pk = jp;
    }
    bool gc = false;
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
      int32_t gg = g;
      if (mute_g != (muted ? 0.0f : 1.0f) || duck.Active()) {  // MUTE / DUCK
        mute_g += ((muted ? 0.0f : 1.0f) - mute_g) * 0.008f;
        if (fabsf(mute_g - (muted ? 0.0f : 1.0f)) < 1e-4f) mute_g = muted ? 0.0f : 1.0f;
        gg = int32_t(g * mute_g * (duck.Active() ? duck.Next() : 1.0f));
      }
      const int32_t xl = (x * gg + 8192) >> 14, yr = (y * gg + 8192) >> 14;  // rounded
      gc |= xl > 32767 || xl < -32768 || yr > 32767 || yr < -32768;
      l[n] = SoftClip16(xl);
      r[n] = SoftClip16(yr);
      pk[0] = std::max<int32_t>(pk[0], abs(l[n]));
      pk[1] = std::max<int32_t>(pk[1], abs(r[n]));
    }
    if (gc) gain_clip = true;
    for (int c = 0; c < 2; c++) peak[c] = std::max(peak[c], pk[c] * (1.0f / 32768.0f));
    level = std::max(pk[0], pk[1]) * (1.0f / 32768.0f);
    Scope(l, r);
    if (g_noise) g_noise->Tap(chained ? NoiseTest::B_INPUT : NoiseTest::A_INPUT, out[0]);
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

FLASHMEM int InputDevice::Clipping() {
  const uint32_t now = millis();
  if (jack_clip) { jack_clip = false; jack_clip_at = now | 1; }
  if (gain_clip) { gain_clip = false; gain_clip_at = now | 1; }
  if (jack_clip_at && now - jack_clip_at < CLIP_SHOW_MS) return 1;
  if (gain_clip_at && now - gain_clip_at < CLIP_SHOW_MS) return 2;
  return 0;
}

FLASHMEM float InputDevice::JackDb() {
  AudioNoInterrupts();
  const int32_t pk = jack_pk;
  jack_pk = 0;
  AudioInterrupts();
  const uint32_t now = millis();
  const float dt = jack_db_at ? (now - jack_db_at) * 0.001f : 0.0f;
  jack_db_at = now;
  const float db = pk > 0 ? 20.0f * log10f(pk / 32767.0f) : -99.0f;
  // up at once; down: a peak is held 1.5 s (a level wobbling with an LFO
  // reads steady), then falls 10 dB a second;
  // a real drop (more than 6 dB below for 0.2 s) is followed quickly, the
  // faster the bigger the gap (a wobbling level must not set that off)
  const float gap = jack_db - db;
  if (gap <= 6.0f) jack_low_since = 0;
  else if (!jack_low_since) jack_low_since = now | 1;
  if (gap <= 0.0f) { jack_db = db; jack_peak_at = now; }
  else if (jack_low_since && now - jack_low_since > 200) jack_db = std::max(db, jack_db - 8.0f * gap * dt);
  else if (now - jack_peak_at > 1500) jack_db = std::max(db, jack_db - 10.0f * dt);
  // the number moves only once the reading is 1 dB past it (no flicker
  // between two values); up to a new peak at once
  const int v = int(lrintf(jack_db));
  if (v > jack_shown || fabsf(jack_db - jack_shown) >= 1.0f) jack_shown = v;
  return float(jack_shown);
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
  // clipping: the box blinks
  if (Clipping() && (millis() / 125) % 2) {
    gfxRect(x, y, w, h);
    return;
  }
  if (cur[0] == STEREO || cur[0] == IN1) DrawVMeter(x, y, bw, h, cur[0] == STEREO ? meters[0].v : std::max(meters[0].v, meters[1].v));
  if (cur[0] == STEREO || cur[0] == IN2) DrawVMeter(x + w - bw, y, bw, h, cur[0] == STEREO ? meters[1].v : std::max(meters[0].v, meters[1].v));
}

/////////////////////////////////////////////////////////////////
// SPOOL (Source): a looper in front of the chain. It records what comes in
// through the track's input; IN is the input's source, so it is set here too.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef SPOOL_DEF = {
  "SPOOL", "REL", 8, {"REC", "PLAY"},  // tag REL kept for saved files (was REEL); A: REC / DUB, B: PLAY / STOP
  {
    // mode (L): LOOP (round and round) or PING (back and forth)
    {"SPEED", -200, 200, 100, BI, 0, 2, 0, {}, {"LOOP", "PING"}},
    // the loop's edges, as PLAYER's: 0..10000 of the recording; mode (L):
    // the picture zooms in on the edge (SPOOL_ZOOM_MS)
    {"START", 0, 10000, 0, UNI, 0, 8, 0},
    {"END", 0, 10000, 10000, UNI, 0, 8, 0},
    {"FADE", 0, 100, 20, UNI, 0, 2, 0, {}, {"CROSS", "INOUT"}},  // the loop's seam, as PLAYER's
    {"IN", 0, 4, 1, UNI},  // like INPUT's SOURC
    // the live input passed through; mode (L): DUB, how much of the old loop
    // stays when overdubbing, 0..100 % in 5 % steps
    {"MON", 0, 100, 100, UNI, 0, 21, 16},
    {"LEVEL", 0, 100, 100, UNI, 0, 21, 0},
    // the picture only. Mode (L): ALL, LOOP (just START..END), or how much
    // time a closer view shows (REEL_VIEW_MS); value (R): where it looks,
    // HEAD (-1: follows the play head) or a place in the loop, 0..100 %
    {"VIEW", -1, 100, -1, UNI, 0, 11, 0},
  },
  {"", ""},
};

// SPOOL's VIEW steps after ALL, in ms
PROGMEM static const uint16_t REEL_VIEW_MS[9] = {2000, 1000, 500, 200, 100, 50, 20, 10, 5};
// START / END zoom steps after the whole loop, in ms
PROGMEM static const uint16_t SPOOL_ZOOM_MS[7] = {2000, 500, 100, 20, 5, 2, 1};
static void FormatMs(int ms, char* buf, size_t n) {
  if (ms >= 1000) snprintf(buf, n, "%ds", ms / 1000);
  else snprintf(buf, n, "%dms", ms);
}
// START / END in seconds, at most four digits so the columns don't run
// together: 5.04, 12.3, 125
static void FormatSeconds(uint32_t ms, char* buf, size_t n) {
  const unsigned long s = ms / 1000, r = ms % 1000;
  if (ms < 10000) snprintf(buf, n, "%lu.%02lu", s, r / 10);
  else if (ms < 100000) snprintf(buf, n, "%lu.%lu", s, r / 100);
  else snprintf(buf, n, "%lu", s);
}

class ReelDevice : public Device {
public:
  enum { SPEED, START, END, XFADE, IN, MON, LEVEL, VIEW };
  bool Varies(int p) const override { return p == LEVEL; }
  static float MaxSecs() { return BigSecs(); }
  explicit ReelDevice(InputDevice* i) : Device(SPOOL_DEF), input(i) {}
  void Reset() override {
    const int16_t in = val[IN];
    Defaults();
    val[IN] = cur[IN] = in;
    ApplyAll();
  }
  int Step(int p, int v, int dir) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
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
  Meter in_m[2];            // the input level, as on INPUT
  // saving the loop to the SD card (the app writes it, A held for 2 s)
  const int16_t* LoopData(int c) const { return fx.Data(c); }
  AudioEffectReel::State LoopState() const { return fx.state(); }
  void WriteMeta(const char* wav_name) const;  // the loop's settings for PLAYER
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
  int rec_top = 4096;          // recording picture: the loudest so far
  // a close view: span frames; centre -1 rides with the head
  void DrawZoom(int x, int y, int w, int h, long span, long centre, const char* label);
};

// IN: OUT A only on track B. START / END: one pixel of the picture at their
// zoom (the whole loop: 1/128 of it) a step; they never cross, the one
// moved pushes the other
FLASHMEM int ReelDevice::Step(int p, int v, int dir) const {
  if (p == IN) return input ? input->Step(0, v, dir) : v + dir;
  if (p != START && p != END) return v + dir;
  const long n = long(fx.Length());
  int step = 10000 / 128;
  if (n > 0 && mode[p] > 0) {
    const long span = long(SPOOL_ZOOM_MS[mode[p] - 1] * (AUDIO_SAMPLE_RATE_EXACT / 1000.0f));
    step = int(std::max(1L, std::min(10000L / 128, span * 10000L / (128L * n))));
  }
  constexpr int GAP = 50;  // 0.5 % at least between them
  const bool end = p == END;
  const int e = end ? constrain(v + dir * step, GAP, 10000) : constrain(v + dir * step, 0, 10000 - GAP);
  const int q = end ? START : END;
  const int want = end ? e - GAP : e + GAP;
  if (end ? val[q] > want : val[q] < want) {
    auto* self = const_cast<ReelDevice*>(this);
    self->val[q] = want;
    self->Refresh(q);
  }
  return e;
}

// START / END: their zoom
FLASHMEM void ReelDevice::FormatMode(int p, char* buf, size_t n) const {
  if (p == START || p == END) {
    if (mode[p] == 0) snprintf(buf, n, "all");
    else FormatMs(SPOOL_ZOOM_MS[mode[p] - 1], buf, n);
  } else if (p == MON) {
    snprintf(buf, n, "DUB%d", mode[p] * 5);
  } else if (p == VIEW) {  // how close the view is (it had no name: blank)
    if (mode[p] == 0) snprintf(buf, n, "ALL");
    else if (mode[p] == 1) snprintf(buf, n, "LOOP");
    else FormatMs(REEL_VIEW_MS[constrain(mode[p] - 2, 0, 8)], buf, n);
  } else {
    Device::FormatMode(p, buf, n);
  }
}

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
    case IN:  // only while SPOOL is the source (else its IN is an old copy: loading
              // the settings put it over INPUT's own choice)
      if (input && active) { input->val[0] = cur[IN]; input->Refresh(0); }
      break;
    case SPEED:
      fx.SetSpeed(v);
      fx.SetPingPong(mode[SPEED] == 1);
      break;
    case START: case END:
      fx.SetWindow(cur[START] * 0.0001f, std::max(0.005f, (cur[END] - cur[START]) * 0.0001f));
      break;
    case XFADE: {  // as PLAYER: up to 1 s (finer at the short end); CROSS or INOUT (a dip)
      const float f = cur[XFADE] * 0.01f;
      fx.SetSeam(16.0f + f * f * AUDIO_SAMPLE_RATE_EXACT);
      fx.SetCrossfade(mode[XFADE] == 0);
      // START / END move at once (a short fade from the old window), not
      // when the loop comes round as on PLAYER
      fx.SetDeferWindow(false);
      break;
    }
    case MON:  // and its mode: DUB
      fx.SetMonitor(v);
      fx.SetDub(mode[MON] * 0.05f);
      break;
    case LEVEL: fx.SetLevel(v); break;
  }
}

FLASHMEM void ReelDevice::Format(int p, char* buf, size_t n) const {
  if (p == SPEED) snprintf(buf, n, "%d%%", val[p]);
  else if (p == START || p == END) {  // seconds into the loop
    const size_t len = fx.Length();
    if (!len) { snprintf(buf, n, "%d%%", val[p] / 100); return; }
    const uint32_t ms = uint32_t(uint64_t(len) * val[p] / 10000 * 1000 / uint64_t(AUDIO_SAMPLE_RATE_EXACT));
    FormatSeconds(ms, buf, n);
  }
  else if (p == IN) snprintf(buf, n, "%s", InputSourceName(val[p]));
  else if (p == VIEW && mode[p] == 0) snprintf(buf, n, "ALL");
  else if (p == VIEW && mode[p] == 1) snprintf(buf, n, "LOOP");
  else if (p == VIEW && val[p] < 0) snprintf(buf, n, "HEAD");
  else if (p == VIEW) snprintf(buf, n, "%d%%", val[p]);
  else Device::Format(p, buf, n);
}

// A: REC -> PLAY -> DUB -> PLAY (stopped: play from the start); B: stop
// (held for 3 s: clear, see the app)
// A: REC when empty, then DUB (closing the loop), DUB <-> PLAY;
// B: PLAY / STOP (while recording: closes the loop and plays)
FLASHMEM void ReelDevice::Action(int a) {
  const auto st = fx.state();
  if (a == 0) {
    // the first REC: as much memory as is free, up to MaxSecs
    if (st == AudioEffectReel::EMPTY && !fx.Grow(MaxSecs())) {
      no_mem_at = millis() | 1;
      return;
    }
    if (st == AudioEffectReel::EMPTY) {  // a new loop: every setting from
      // its default (values and L modes), except IN, what is recorded
      for (int p = 0; p < num_params(); p++) {
        if (p == IN) continue;
        val[p] = param(p).def;
        mode[p] = param(p).defmode;
        Refresh(p);
      }
    }
    fx.Dub();
  } else if (st == AudioEffectReel::PLAY || st == AudioEffectReel::DUB) {
    fx.Stop();
  } else if (st != AudioEffectReel::EMPTY) {
    fx.Press();  // REC: the loop closes and plays; STOPPED: from the start
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
  if (a == 0) return st == AudioEffectReel::REC || st == AudioEffectReel::DUB;
  return st == AudioEffectReel::PLAY || st == AudioEffectReel::DUB;
}

// the loop as a waveform, the looping part (START / LEN) bracketed, the
// play head running through it; REC / DUB in the corner
FLASHMEM void ReelDevice::DrawViz(int x, int y, int w, int h) {
  // the input level at the right edge, as on INPUT (what REC takes in);
  // the loop's picture to the left of it
  {
    float pl, pr;
    fx.TakeInPeak(pl, pr);
    const int mx = x + w - 18;
    gfxFrame(mx, y, 8, h);
    gfxFrame(mx + 9, y, 8, h);
    DrawVMeter(mx + 1, y + 1, 6, h - 2, in_m[0].Hit(pl));
    DrawVMeter(mx + 10, y + 1, 6, h - 2, in_m[1].Hit(pr));
    w -= 22;
  }
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
  // the cursor on START / END with a zoom: around that edge, as PLAYER does
  const bool edge_zoom = (view_cursor == START || view_cursor == END) && mode[view_cursor] > 0
                         && st != AudioEffectReel::REC && fx.Length();
  if (edge_zoom) {
    const int ms = SPOOL_ZOOM_MS[mode[view_cursor] - 1];
    char t[8];
    FormatMs(ms, t, sizeof(t));
    DrawZoom(x, y, w, h, long(ms * (AUDIO_SAMPLE_RATE_EXACT / 1000.0f)),
             long(uint64_t(fx.Length()) * cur[view_cursor] / 10000), t);
  } else if (mode[VIEW] == 1 && st != AudioEffectReel::REC && fx.Length()) {
    // LOOP: just what plays, START to END across the width (2 pixels of room
    // either side so their lines show)
    float a, b;
    fx.Window(a, b);
    const long n = long(fx.Length());
    const long ws = long(a * n), wl = std::max(1L, long((std::min(b, 1.0f) - a) * n));
    const long span = wl * w / std::max(1, w - 4);
    DrawZoom(x, y, w, h, span, ws - span * 2 / w + span / 2, "LOOP");
  } else if (mode[VIEW] > 1) {
    const int ms = REEL_VIEW_MS[constrain(mode[VIEW] - 2, 0, 8)];
    char t[8];
    FormatMs(ms, t, sizeof(t));
    const long n = long(st == AudioEffectReel::REC ? fx.HeadFrame() : fx.Length());
    DrawZoom(x, y, w, h, long(ms * (AUDIO_SAMPLE_RATE_EXACT / 1000.0f)),
             cur[VIEW] < 0 ? -1L : n * cur[VIEW] / 100, t);
  } else {
  constexpr int HIST = AudioEffectReel::HIST;
  uint8_t hist[HIST];
  fx.ReadHistory(hist);
  // scaled to the loop's loudest point, so a quiet loop still has a shape
  int top = 32;
  for (int k = 0; k < HIST; k++) top = std::max<int>(top, hist[k]);
  // recording: the wave grows from the left on a steady scale (4 s across,
  // doubling whenever the recording reaches the right edge) instead of
  // squeezing into the whole width
  // squeezing into the whole width. Each pixel is a fixed stretch of time,
  // so what is drawn stays put as the recording grows; the height scale only
  // grows (the loudest so far)
  int span_w = w;
  if (st == AudioEffectReel::REC) {
    const float got = fx.HeadFrame() / AUDIO_SAMPLE_RATE_EXACT;
    float across = 4.0f;
    while (across < got && across < MaxSecs()) across *= 2.0f;
    span_w = std::max(1, std::min(w, int(w * got / across)));
    gfxDottedLine(x + span_w, cy, x + w - 1, cy, 2);
    int16_t lo[128], hi[128];
    const int cols = std::min(w, 128);
    fx.ReadWave(0, long(across * AUDIO_SAMPLE_RATE_EXACT), cols, lo, hi);
    for (int k = 0; k < cols && k < span_w; k++) rec_top = std::max({rec_top, abs(int(lo[k])), abs(int(hi[k]))});
    for (int k = 0; k < cols && k < span_w; k++) {
      const int pk = std::max(abs(int(lo[k])), abs(int(hi[k])));
      const int a = int(sqrtf(float(pk) / rec_top) * (h / 2 - 1));
      gfxLine(x + k, cy - a, x + k, cy + a);
    }
  } else {
    rec_top = 4096;
    for (int k = 0; k < HIST; k++) {
      const int cx = x + k * span_w / HIST;
      const int a = int(sqrtf(float(hist[k]) / top) * (h / 2 - 1));
      gfxLine(cx, cy - a, cx, cy + a);
    }
  }
  // START and END: solid lines (as on PLAYER), joined along the top and
  // bottom
  if (st != AudioEffectReel::REC) {
    float a, b;
    fx.Window(a, b);
    const int wl = x + int(a * (w - 1)), wr = x + int(std::min(b, 1.0f) * (w - 1));
    for (int px = wl; px <= wr; px++) { gfxPixel(px, y); gfxPixel(px, base); }
    for (const int ex : {wl, wr}) {
      const int cl = std::max(x, ex - 1), cr = std::min(x + w - 1, ex + 1);
      graphics.clearRect(cl, y + 1, cr - cl + 1, h - 2);
      gfxLine(ex, y, ex, base);
    }
  }
  // the head: recording, a solid line at the end of what is there; playing,
  // a thin gap through the wave
  if (st == AudioEffectReel::REC) {
    const int hx = x + span_w - 1;
    const int cl = std::max(x, hx - 1), cr = std::min(x + w - 1, hx + 1);
    graphics.clearRect(cl, y + 1, cr - cl + 1, h - 2);
    gfxLine(hx, y + 1, hx, base - 1);
  } else {
    gfxInvert(constrain(x + int(fx.HeadPos() * (w - 1)), x, x + w - 1), y + 1, 1, h - 2);
  }
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
FLASHMEM void ReelDevice::DrawZoom(int x, int y, int w, int h, long span, long centre, const char* label) {
  const int cy = y + h / 2, base = y + h - 1, half = h / 2 - 1;
  const auto st = fx.state();
  const bool rec = st == AudioEffectReel::REC;
  const long head = long(fx.HeadFrame());
  // HEAD: the view rides with the play head (recording: at the right edge);
  // a place: the view stands still, centred there, and the head passes by
  const bool follow = centre < 0;
  long from;
  int hx;
  if (follow) {
    hx = rec ? x + w - 1 : x + w / 2;
    from = head - span * (hx - x) / w;
  } else {
    const long n = long(rec ? fx.HeadFrame() : fx.Length());
    from = centre - span / 2;
    long rel = head - from;
    if (!rec && n > 0) rel = ((rel % n) + n) % n;  // the loop wraps
    hx = rel >= 0 && rel < span ? x + int(rel * w / span) : -1;
  }
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
        // a solid line, as on PLAYER: the point being set stands out
        graphics.clearRect(std::max(long(x), px - 1), y, std::min(3L, x + w - std::max(long(x), px - 1)), h);
        gfxLine(px, y, px, base);
      }
    }
  }
  // the head: playing, a thin gap through the wave (the edges are the solid
  // lines); recording, the solid line where it records
  if (hx >= x) {
    if (rec) {
      graphics.clearRect(std::max(x, hx - 1), y + 1, std::min(3, x + w - std::max(x, hx - 1)), h - 2);
      gfxLine(hx, y + 1, hx, base - 1);
    } else {
      gfxInvert(hx, y + 1, 1, h - 2);
    }
  }
  // how much time the view shows, top right
  const int tw = 6 * int(strlen(label));
  graphics.clearRect(x + w - tw - 1, y, tw + 1, 8);
  gfxPrint(x + w - tw, y, label);
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
constexpr char REC_DIR[] = "/LOOM/SPOOLS";  // SPOOL's saved loops
constexpr char OLD_REC_DIR[] = "/LOOM/RECORDINGS";  // ... called so until 2026-10
// file lists: alphabetical, but Loom's own folders first: BOLTS, SPOOLS, THRUMS
FLASHMEM static int ListOrder(const char* a, const char* b) {
  // BOLTS, SPOOLS, THRUMS (in that order), then the rest by name
  auto rank = [](const char* n) {
    return !strcasecmp(n, "BOLTS") ? 0 : !strcasecmp(n, "SPOOLS") ? 1 : !strcasecmp(n, "THRUMS") ? 2 : 3;
  };
  if (rank(a) != rank(b)) return rank(a) - rank(b);
  return strcasecmp(a, b);
}
constexpr int MAX_FILES = 48, NAME_LEN = 80;
constexpr size_t IO_CHUNK = 2048;  // sample frames per step
// one sample of any size / float as 16 bit
FLASHMEM static int16_t WavSample(const uint8_t* p, int bytes, bool is_float) {
  switch (bytes) {
    case 1: return int16_t((p[0] - 128) << 8);  // 8-bit is unsigned
    case 2: return int16_t(p[0] | p[1] << 8);
    case 3: return int16_t(p[1] | p[2] << 8);
    default:
      if (is_float) {
        float f;
        memcpy(&f, p, 4);
        return ToSample16(f);  // (overs bent, not cut off)
      }
      return int16_t(p[2] | p[3] << 8);
  }
}

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
// a saved loop's settings, next to it as PLAYER keeps them (name.loom), so
// PLAYER opens it the way it was set up in SPOOL: START / END (to the
// frame), FADE, LEVEL, SPEED (backwards: REV), looping
FLASHMEM void ReelDevice::WriteMeta(const char* wav_name) const {
  char path[64];
  snprintf(path, sizeof(path), "%s/%s", REC_DIR, wav_name);
  if (char* dot = strrchr(path, '.')) *dot = 0;
  strlcat(path, ".loom", sizeof(path));
  const uint64_t n = LoopLength();
  SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return;
  const int sp = val[SPEED];
  f.printf("LOOP 1 0\n");
  f.printf("LEVEL %d 0\n", int(val[LEVEL]));
  f.printf("SPEED %d %d\n", std::min(200, abs(sp)), sp < 0 ? 0 : 1);  // REV / FWD
  f.printf("START %d 0\n", int(val[START]));
  f.printf("END %d 0\n", int(val[END]));
  f.printf("FADE %d %d\n", int(val[XFADE]), int(mode[XFADE]));
  f.printf("SNAP 0 0\n");
  f.printf("EDGES %lu %lu\n", (unsigned long)(n * val[START] / 10000), (unsigned long)(n * val[END] / 10000));
  f.close();
}

// the session: counted on the card (/LOOM/SESSION.TXT), one up the first
// time something is saved after a power-up; 0 until then
inline int g_session = 0;
FLASHMEM static int Session() {
  if (g_session) return g_session;
  int n = 0;
  if (File f = SD.open("/LOOM/SESSION.TXT", FILE_READ)) {
    char b[12] = {};
    f.read(b, sizeof(b) - 1);
    f.close();
    n = atoi(b);
  }
  g_session = constrain(n + 1, 1, 999);
  SD.remove("/LOOM/SESSION.TXT");
  if (File f = SD.open("/LOOM/SESSION.TXT", FILE_WRITE)) {
    f.printf("%d\n", g_session);
    f.close();
  }
  return g_session;
}

// a free name in dir: Spool_S014_01.wav, the session and the n-th save in
// it (no dates: without a battery the module's clock starts in 2019 at
// every power-up)
FLASHMEM static void StampName(const char* dir, const char* prefix, char* name, size_t n, char* path, size_t pn) {
  const int s = Session();
  for (int k = 1; k < 1000; k++) {
    snprintf(name, n, "%s_S%03d_%02d.wav", prefix, s, k);
    snprintf(path, pn, "%s/%s", dir, name);
    if (!SD.exists(path)) return;
  }
}

// WavWriter's interleaving buffer (RAM2: RAM1 is short, the stack needs it)
DMAMEM static int16_t wav_tmp[IO_CHUNK * 2];
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
    char path[64];
    StampName(REC_DIR, "Spool", name, sizeof(name), path, sizeof(path));
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
    int16_t* const tmp = wav_tmp;
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

constexpr char TapeRecorder::DIR[];

FLASHMEM bool TapeRecorder::Begin() {
  fault_at = millis() | 1;
  if (!SDcard_Ready) { fault = NO_CARD; return false; }
  // the ring: 5.5 s, less when memory is short (SPOOL holds what it records)
  for (const uint32_t frames : {262144u, 131072u, 65536u}) {
    if (ring) break;
    ring = static_cast<int16_t*>(PsramAlloc(frames * 4));
    cap = ring ? frames : 0;
  }
  if (!ring) { fault = NO_MEM; return false; }
  if (!SD.exists(LOOP_DIR)) SD.mkdir(LOOP_DIR);
  if (!SD.exists(DIR) && SD.exists(OLD_DIR)) SD.rename(OLD_DIR, DIR);
  if (!SD.exists(DIR)) SD.mkdir(DIR);
  char path[64];
  StampName(DIR, "Bolt", name, sizeof(name), path, sizeof(path));
  f = SD.open(path, FILE_WRITE);
  if (!f) { fault = NO_CARD; PsramFree(ring); ring = nullptr; return false; }
  written = 0;
  slow = false;
  fault = OK;
  fault_at = 0;
  Header();
  header_at = millis();
  file_open = true;
  __disable_irq();
  wr = rd = 0;
  on = true;
  __enable_irq();
  return true;
}

// the sizes in the header as far as written; the file is then flushed, so
// the card's directory has its length too
FLASHMEM void TapeRecorder::Header() {
  const uint32_t bytes = uint32_t(std::min<uint64_t>(written * 4, 0xFFFFFFFFu - 36));
  uint8_t h[44] = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E',
                   'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 2, 0};
  PutLE(h + 4, 36 + bytes, 4);
  PutLE(h + 24, uint32_t(AUDIO_SAMPLE_RATE_EXACT), 4);
  PutLE(h + 28, uint32_t(AUDIO_SAMPLE_RATE_EXACT) * 4, 4);
  PutLE(h + 32, 4, 2);
  PutLE(h + 34, 16, 2);
  memcpy(h + 36, "data", 4);
  PutLE(h + 40, bytes, 4);
  const uint64_t end = f.position();
  f.seek(0);
  f.write(h, 44);
  if (end > 44) f.seek(end);
  f.flush();
}

// the ring to the card in 16 KB writes (all of it: also the last bit)
FLASHMEM size_t TapeRecorder::Drain(size_t most) {
  constexpr uint32_t CHUNK = 4096;  // frames
  DMAMEM static uint32_t buf[CHUNK];  // (RAM2: RAM1 is short)
  const uint32_t* src = reinterpret_cast<const uint32_t*>(ring);
  size_t done = 0;
  while (done < most) {
    const uint32_t avail = wr - rd;
    const uint32_t k = std::min<uint32_t>({avail, CHUNK, uint32_t(most - done)});
    if (!k || (k < CHUNK && on)) break;  // only whole chunks while it runs
    for (uint32_t i = 0; i < k; i++) buf[i] = src[(rd + i) & (cap - 1)];
    if (f.write(reinterpret_cast<uint8_t*>(buf), k * 4) != k * 4) {  // card full / gone
      fault = NO_CARD;
      fault_at = millis() | 1;
      want = false;
      break;
    }
    rd += k;
    written += k;
    done += k;
  }
  return done;
}

FLASHMEM void TapeRecorder::Close() {
  on = false;
  if (fault == OK) Drain(SIZE_MAX);
  Header();
  f.close();
  file_open = false;
  PsramFree(ring);  // the memory back to SPOOL / PLAYER
  ring = nullptr;
  cap = 0;
}

FLASHMEM void TapeRecorder::Idle() {
  if (want && !file_open && !Begin()) want = false;
  if (!file_open) return;
  Drain(8 * 4096);
  // a WAV holds at most 4 GB (about 6 hours): the take ends there
  if (written * 4 > 4000000000ull) want = false;
  if (!want) Close();
  else if (millis() - header_at > 5000) { Header(); header_at = millis(); }
}

FLASHMEM void TapeRecorder::Finish() {
  want = false;
  if (file_open) Close();
}

/////////////////////////////////////////////////////////////////
// PLAYER (Source): plays a WAV from the SD card. The file is loaded into
// PSRAM (up to 16 s, like SPOOL) and played by the same engine: START / END
// pick the loop, SPEED plays it at any rate (backwards below zero), XFADE
// is the fade at the loop ends. START / END zoom the picture (their mode)
// and SNAP lands them on the quietest point nearby (ZERO) or on the nearest
// beat when a tempo was found (BEAT). Longer files stream
// from the card (FILE mode STRM); FILE mode MEM copies a section of them
// (START..END when switched, at most 16 s) into memory and plays that as
// the sample, START / END then within it; back to STRM puts them back.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef PLAYER_DEF = {
  "SHUTTLE", "SHU", 8, {"PLAY", "TOP"},  // (was PLAYER / PLY until 2026-10-09)
  {
    {"FILE", 0, MAX_FILES - 1, 0, UNI, 0, 2, 0, {}, {"STRM", "MEM"}},
    {"LOOP", 0, 1, 1, LIST, 2, 0, 0, {"OFF", "ON"}},
    {"LEVEL", 0, 100, 100, UNI, 0, 21, 0},
    {"SPEED", 0, 200, 100, UNI, 0, 3, 1, {}, {"REV", "FWD", "SYNC"}},  // mode: direction, REV left; SYNC: forward at the clock's tempo
    {"START", 0, 10000, 0, UNI, 0, 10, 0},      // mode: zoom, all .. 0.3 ms across
    {"END", 0, 10000, 10000, UNI, 0, 10, 0},
    {"FADE", 0, 100, 20, UNI, 0, 2, 0, {}, {"CROSS", "INOUT"}},  // the loop's seam
    {"SNAP", 0, 2, 0, LIST, 3, 0, 0, {"OFF", "ZERO", "BEAT"}},  // ZERO: the quietest point; BEAT: the nearest beat
  },
  {"", ""},
};

// The file list's quick listen (B): a file straight from the card, once
// through, into the output past both tracks' effects. One for both tracks;
// its memory only while it plays. A new file waits for the last one to fade
// out (20 ms; cut off, it clicked).
class LoomPreview {
public:
  AudioPlayStream stream;
  bool Start(const char* folder, const char* name);
  void Stop();
  void Halt();  // at once (Loom suspended: no audio running)
  void Idle(bool keep);  // keep: the memory stays (the list is open)
  bool Playing() const { return want || (on && stream.Playing()); }
  float level = 1.0f;
  bool Is(const char* folder, const char* name) const {
    if (want) return !strcmp(next_dir, folder) && !strcmp(next_name, name);
    return on && stream.Playing() && !strcmp(dir_, folder) && !strcmp(name_, name);
  }
private:
  File f;
  bool on = false, want = false, is_float = false;
  int chans = 0, bytes = 0;
  size_t frames = 0, data_off = 0;
  char dir_[96] = "", name_[NAME_LEN] = "";
  char next_dir[96] = "", next_name[NAME_LEN] = "";  // waiting for the fade
  char path_[192] = "";  // of the file playing, to open it again
  bool Reopen() {
    if (f) f.close();
    f = SD.open(path_, FILE_READ);
    return bool(f);
  }
  bool Begin();
  void Close();
  size_t Fill(int chunks);
  size_t ReadFrames(int16_t* l, int16_t* r, size_t n);
  void LoopSection(const char* wav_path, size_t& a, size_t& b) const;  // from its .loom
};

// The file list (R push on FILE: PLAYER, WARP): sub-folders first (with a
// /), then WAVs; ".." goes up. One for the app, open for one device at a
// time. B listens to the file under the cursor (the quick listen), B held
// turns AUTO on: every file the cursor rests on plays.
class LoomBrowser {
public:
  const void* owner = nullptr;  // the device whose list is open
  LoomPreview* preview = nullptr;
  bool IsOpen(const void* who) const { return owner && owner == who; }
  // extra: a first row in every folder that picks no file (WARP: the
  // other track's SPOOL loop); select nullptr / "": on it
  void Open(const void* who, const char* dir, const char* select, const char* extra = nullptr);
  void Close() { owner = nullptr; autoplay = false; if (preview) preview->Stop(); }
  void Move(int d) {
    const int was = bsel;
    bsel = constrain(bsel + d, 0, std::max(0, bcount - 1));
    if (autoplay && bsel != was) { auto_due = true; auto_at = millis(); }
  }
  // R: a folder goes in (".." up); a WAV: true (the list closes; Dir() /
  // Name() are the one picked; the extra row: both "")
  bool Enter();
  void Back();  // at the top: closes
  void Play();
  void ToggleAutoplay();
  void AutoplayIdle();
  const char* Dir() const { return OnExtra() ? "" : bdir; }
  const char* Name() const { return bsel < bcount && !OnExtra() ? bn[bsel] : ""; }
  bool OnWav() const { return bsel < bcount && !OnExtra() && !bfolder[bsel]; }  // the cursor on a WAV
  void Draw(int x, int y, int w, int h) const;

private:
  char bdir[96] = "/";
  char bn[MAX_FILES][NAME_LEN] = {};
  bool bfolder[MAX_FILES] = {};
  int bcount = 0, bsel = 0;
  char extra_[16] = "";
  int extra_at = -1;  // its row
  bool OnExtra() const { return bsel == extra_at; }
  bool autoplay = false, auto_due = false;
  uint32_t auto_at = 0;
  void List();
};
inline LoomBrowser* g_browser = nullptr;  // made with Loom (on the heap: RAM2)

// the one quick listen (made with Loom, before the audio output; CARD
// borrows it while Loom is paused)
inline LoomPreview* g_preview = nullptr;

// --- the browser: sub-folders first (with a /), then WAVs; ".." goes up
FLASHMEM void LoomBrowser::List() {
  bcount = 0;
  bsel = 0;
  extra_at = -1;
  if (extra_[0]) { extra_at = bcount; snprintf(bn[bcount], NAME_LEN, "%s", extra_); bfolder[bcount++] = false; }
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
      for (int j = i; j > first && ListOrder(bn[j - 1], bn[j]) > 0; j--) {  // (the extra row and ".." stay first)
        char t[NAME_LEN];
        memcpy(t, bn[j], NAME_LEN); memcpy(bn[j], bn[j - 1], NAME_LEN); memcpy(bn[j - 1], t, NAME_LEN);
        std::swap(bfolder[j], bfolder[j - 1]);
      }
  }
  d.close();
}

FLASHMEM void LoomBrowser::Open(const void* who, const char* dir, const char* select, const char* extra) {
  if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
  snprintf(bdir, sizeof(bdir), "%s", dir && dir[0] ? dir : "/");
  snprintf(extra_, sizeof(extra_), "%s", extra ? extra : "");
  List();
  if (!select || !select[0]) { if (extra_at >= 0) bsel = extra_at; }
  else
    for (int i = 0; i < bcount; i++)
      if (!bfolder[i] && i != extra_at && !strcmp(bn[i], select)) { bsel = i; break; }
  owner = who;
}

FLASHMEM void LoomBrowser::Back() {
  if (!strcmp(bdir, "/")) { Close(); return; }  // at the top
  char* slash = strrchr(bdir, '/');
  if (slash == bdir) bdir[1] = 0; else if (slash) *slash = 0;
  List();
}

FLASHMEM bool LoomBrowser::Enter() {
  if (bsel >= bcount) return false;
  if (bfolder[bsel]) {
    if (!strcmp(bn[bsel], "..")) { Back(); return false; }
    const size_t l = strlen(bdir);
    if (l + strlen(bn[bsel]) + 2 < sizeof(bdir))
      snprintf(bdir + l, sizeof(bdir) - l, "%s%s", l > 1 ? "/" : "", bn[bsel]);
    List();
    if (autoplay) { auto_due = true; auto_at = millis(); }
    return false;
  }
  // a WAV (the quick listen ends)
  Close();
  return true;
}

// B in the list: a quick listen to the file under the cursor, straight
// from the card and past the effects, once through (the device's file
// stays); B again stops it
FLASHMEM void LoomBrowser::Play() {
  if (bsel >= bcount || bfolder[bsel] || OnExtra() || !preview) return;
  if (preview->Is(bdir, bn[bsel])) preview->Stop();
  else preview->Start(bdir, bn[bsel]);
}

FLASHMEM void LoomBrowser::ToggleAutoplay() {
  autoplay = !autoplay;
  auto_due = false;
  if (!preview) return;
  if (!autoplay) { preview->Stop(); return; }
  if (bsel < bcount && !bfolder[bsel] && !OnExtra() && !preview->Is(bdir, bn[bsel])) preview->Start(bdir, bn[bsel]);
}

// AUTO: once the cursor has rested on a file for a moment, it plays (a
// folder: quiet), so scrolling fast doesn't start every file on the way
FLASHMEM void LoomBrowser::AutoplayIdle() {
  if (!owner) autoplay = false;
  if (!autoplay || !auto_due || !preview || millis() - auto_at < 100) return;
  auto_due = false;
  if (bsel >= bcount || bfolder[bsel] || OnExtra()) preview->Stop();
  else if (!preview->Is(bdir, bn[bsel])) preview->Start(bdir, bn[bsel]);
}

FLASHMEM void LoomBrowser::Draw(int x, int y, int w, int h) const {
  // the folder on top, then 5 entries around the selection
  char t[24];
  const size_t l = strlen(bdir);
  const size_t room = autoplay ? 16 : 21;  // AUTO takes the right end
  snprintf(t, sizeof(t), "%s", l > room ? bdir + l - room : bdir);
  gfxPrint(x, y + 1, t);
  if (autoplay) {
    gfxPrint(x + w - 25, y + 1, "AUTO");
    gfxInvert(x + w - 26, y, 26, 9);
  }
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
    // the quick listen playing: a small arrow at the right
    if (!bfolder[i] && preview && preview->Is(bdir, bn[i])) {
      graphics.clearRect(x + w - 6, ry - 1, 6, 9);  // over a long name
      for (int k = 0; k < 3; k++) gfxLine(x + w - 5 + k, ry + k, x + w - 5 + k, ry + 6 - k);
    }
    if (i == bsel) gfxInvert(x, ry - 1, w, 9);
  }
}

FLASHMEM bool LoomPreview::Start(const char* folder, const char* name) {
  snprintf(next_dir, sizeof(next_dir), "%s", folder);
  snprintf(next_name, sizeof(next_name), "%s", name);
  want = true;
  stream.Stop();  // the one playing fades out first (see Idle)
  if (stream.Silent()) Idle(true);
  return true;
}

FLASHMEM void LoomPreview::Stop() {
  want = false;
  stream.Stop();  // fades out; the memory goes once it has (Idle)
}

FLASHMEM void LoomPreview::Halt() {
  want = false;
  stream.Stop();
  Close();
  AudioNoInterrupts();
  stream.Release();
  AudioInterrupts();
}

FLASHMEM void LoomPreview::Close() {
  if (f) f.close();
  on = false;
}

// main loop: the buffers topped up; once faded out, the next file; the
// memory given back once the list is closed. (Not with the audio held up
// for it: the stream is silent by then and doesn't touch its buffers, and
// holding up the audio while that much memory was freed clicked loudly.)
FLASHMEM void LoomPreview::Idle(bool keep) {
  if (on && stream.Playing()) { Fill(4); return; }
  if (!stream.Silent()) return;  // fading out
  if (on) Close();
  if (want) {
    want = false;
    if (Begin()) return;
  }
  if (!keep && stream.ready()) stream.Release();
}

// opens next_name and starts it
FLASHMEM bool LoomPreview::Begin() {
  char path[192];
  snprintf(path, sizeof(path), "%s/%s", strcmp(next_dir, "/") ? next_dir : "", next_name);
  snprintf(path_, sizeof(path_), "%s", path);
  f = SD.open(path, FILE_READ);
  if (!f) return false;
  uint8_t h[12];
  if (f.read(h, 12) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) { f.close(); return false; }
  int bits = 0;
  uint32_t srate = 44100;
  frames = 0;
  while (f.available()) {  // walk the chunks to fmt and data (as PLAYER)
    uint8_t c[8];
    if (f.read(c, 8) != 8) break;
    const uint32_t sz = c[4] | c[5] << 8 | c[6] << 16 | uint32_t(c[7]) << 24;
    if (!memcmp(c, "fmt ", 4)) {
      uint8_t fm[40] = {};
      const uint32_t take = std::min<uint32_t>(sz, 40);
      f.read(fm, take);
      if (sz > take) f.seek(f.position() + sz - take);
      chans = fm[2];
      srate = fm[4] | fm[5] << 8 | fm[6] << 16 | uint32_t(fm[7]) << 24;
      bits = fm[14];
      int tag = fm[0] | fm[1] << 8;
      if (tag == 0xFFFE && take >= 26) tag = fm[24] | fm[25] << 8;
      is_float = tag == 3;
      if (tag != 1 && tag != 3) bits = 0;
    } else if (!memcmp(c, "data", 4)) {
      const bool ok_bits = is_float ? bits == 32 : (bits == 8 || bits == 16 || bits == 24 || bits == 32);
      if (ok_bits && chans >= 1) {
        bytes = bits / 8;
        frames = sz / (bytes * chans);
        data_off = f.position();
      }
      break;
    } else {
      f.seek(f.position() + sz + (sz & 1));
    }
  }
  if (!frames || (!stream.ready() && !stream.Acquire(2.0f, 0.25f))) { f.close(); return false; }
  stream.SetLoop(false);
  stream.SetCrossfade(false);
  stream.SetLevel(level);
  stream.SetSpeed(srate / AUDIO_SAMPLE_RATE_EXACT);  // the module runs at 48 kHz
  stream.Open(frames);
  size_t a = 0, b = frames;
  if (!strcasecmp(next_dir, REC_DIR)) LoopSection(path, a, b);  // a saved loop: its loop only
  stream.SetWindow(a, b);
  Fill(8);  // a start buffered before it plays
  stream.Play();
  snprintf(dir_, sizeof(dir_), "%s", next_dir);
  snprintf(name_, sizeof(name_), "%s", next_name);
  on = true;
  return true;
}

FLASHMEM size_t LoomPreview::Fill(int chunks) {
  size_t all = 0;
  for (int k = 0; k < chunks; k++) {
    int16_t *l, *r;
    size_t at, count;
    if (!stream.Next(l, r, at, count, IO_CHUNK)) break;
    f.seek(data_off + at * size_t(bytes) * chans);
    size_t rd = ReadFrames(l, r, count);
    if (!rd && Reopen()) {  // the file handle went stale (card mode: the
      f.seek(data_off + at * size_t(bytes) * chans);  // computer's side
      rd = ReadFrames(l, r, count);                   // re-opens the card)
    }
    stream.Filled(rd);
    all += rd;
    if (rd < count) break;
  }
  return all;
}

// the loop START .. END saved next to the file (EDGES: in frames, else
// START / END in 1/10000 of the file); the whole file when there is none
FLASHMEM void LoomPreview::LoopSection(const char* wav_path, size_t& a, size_t& b) const {
  char mp[200];
  snprintf(mp, sizeof(mp), "%s", wav_path);
  if (char* dot = strrchr(mp, '.')) *dot = 0;
  strlcat(mp, ".loom", sizeof(mp));
  File m = SD.open(mp, FILE_READ);
  if (!m) return;
  long s = -1, e = -1, es = -1, ee = -1;
  char line[48];
  while (m.available()) {
    int n = 0;
    while (m.available() && n < int(sizeof(line)) - 1) {
      const char c = m.read();
      if (c == '\n') break;
      if (c != '\r') line[n++] = c;
    }
    line[n] = 0;
    char key[8];
    long x = 0, y = 0;
    if (sscanf(line, "%7s %ld %ld", key, &x, &y) != 3) continue;
    if (!strcmp(key, "EDGES")) { es = x; ee = y; }
    else if (!strcmp(key, "START")) s = x;
    else if (!strcmp(key, "END")) e = x;
  }
  m.close();
  size_t na = a, nb = b;
  if (es >= 0 && ee > es) { na = size_t(es); nb = size_t(ee); }
  else if (s >= 0 && e > s) { na = size_t(uint64_t(frames) * s / 10000); nb = size_t(uint64_t(frames) * e / 10000); }
  nb = std::min(nb, frames);
  if (na + AUDIO_BLOCK_SAMPLES <= nb) { a = na; b = nb; }
}

FLASHMEM size_t LoomPreview::ReadFrames(int16_t* l, int16_t* r, size_t n) {
  DMAMEM static uint8_t raw[IO_CHUNK * 8];  // (RAM2: RAM1 is short)
  const size_t frame = size_t(bytes) * chans;
  const size_t k = std::min(n, sizeof(raw) / frame);
  // (a failed read gives -1: as a size that ran the loop off the end of
  // memory and crashed the module)
  const int got = f.read(raw, k * frame);
  if (got <= 0) return 0;
  const size_t rd = size_t(got) / frame;
  for (size_t i = 0; i < rd; i++) {
    const uint8_t* fp = raw + i * frame;
    l[i] = WavSample(fp, bytes, is_float);
    r[i] = WavSample(fp + (chans > 1 ? bytes : 0), bytes, is_float);
  }
  return rd;
}

class PlayerDevice : public Device {
public:
  enum { FILE_, LOOP, LEVEL, SPEED, START, END, XFADE, SNAP };
  bool Varies(int p) const override { return p == LEVEL; }
  enum { SNAP_OFF, SNAP_ZERO, SNAP_BEAT };
  enum { DIR_REV, DIR_FWD, DIR_SYNC };  // SPEED's modes
  // STRM reads the card forward only: no REV (MEM has it)
  int ModeMin(int p) const override { return p == SPEED && mode[FILE_] == 0 ? DIR_FWD : 0; }
  // FILE's number row already says STRM / MEM (the file's name is above)
  const char* Name(int p) const override { return p == FILE_ ? "MODE" : Device::Name(p); }
  bool NameIsMode(int p) const override { return p == FILE_; }  // (the number row shows STRM / MEM)
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
  // a loop SPOOL just saved: PLAYER's file stays, but the file list opens
  // next time in SPOOLS on it (once)
  void Select(const char* name) { snprintf(fresh, sizeof(fresh), "%s", name); }
  char fresh[NAME_LEN] = "";
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
  // WARP on the other track: the file's path and its loop section in the
  // file's frames (false: no file in)
  bool Section(char* path, size_t n, uint32_t& a, uint32_t& b) const {
    if (!nfiles || loading || analysing || !Frames()) return false;
    snprintf(path, n, "%s/%s", strcmp(dir, "/") ? dir : "", FileName());
    const size_t off = clipped ? sec_off : 0;
    a = uint32_t(EdgeAt(START) + off);
    b = uint32_t(EdgeAt(END) + off);
    if (b < a) std::swap(a, b);
    return true;
  }
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
  // the file list (R push on FILE)
  int FileParam() const override { return FILE_; }
  void OpenBrowser() override;
  void Picked(const char* folder, const char* name) override;

private:
  AudioPlayStream stream;  // before fx: the audio update order follows the signal
  AudioEffectReel fx;
  AudioConnection st_l, st_r;
  char names[MAX_FILES][NAME_LEN] = {};
  uint32_t scanned_at = 0;
  char dir[96] = "/LOOM";  // the folder FILE steps through
  // streaming (files too long for fx)
  bool streaming = false, analysing = false;
  bool indexing = false;   // streaming, and the level index still being read
  uint32_t indexed_at = 0;
  size_t data_off = 0, done = 0;  // where the samples start in the file; analysis progress
  uint8_t* idx = nullptr;         // level per IDX_FRAMES frames, for the picture
  // and per IDX2 entries of it (after the fine one, in the same block), so
  // a picture of a whole long file doesn't read every entry (13 ms a frame)
  static constexpr size_t IDX2 = 64;
  uint8_t* idx2 = nullptr;
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
  float bpm_keep = 0.0f;      // the same file again (STRM <-> MEM): its tempo, not measured again
  double phase_keep = 0.0;    // ... and where its beats fall, in frames of the whole file
  int bpm_got = -1;           // levels read so far (-1: no analysis running)
  int bpm_n = 0;
  bool bpm_loop = false;      // the loop in memory, read round and round (a short one works too)
  size_t bpm_from = 0;
  float* bpm_env = nullptr;   // RAM2, only while it runs
  void BpmStart();
  void BpmIdle();
  void BpmSolve();
  double bpm_phase = 0.0;     // a frame a beat falls on (with bpm)
  double BeatFrames() const { return 60.0 * FileRate() / bpm; }
  size_t SnapPoint(size_t at) const;
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
  // and SPOOL's loops were in RECORDINGS: now SPOOLS (a file playing from
  // there goes along)
  if (!SD.exists(REC_DIR) && SD.exists(OLD_REC_DIR)) SD.rename(OLD_REC_DIR, REC_DIR);
  // the thrums were in WAVES for a day: now THRUMS
  if (!SD.exists("/LOOM/THRUMS") && SD.exists("/LOOM/WAVES")) SD.rename("/LOOM/WAVES", "/LOOM/THRUMS");
  // and the recordings were in TAPES: now BOLTS
  if (!SD.exists("/LOOM/BOLTS") && SD.exists("/LOOM/TAPES")) SD.rename("/LOOM/TAPES", "/LOOM/BOLTS");
  if (!strcasecmp(dir, OLD_REC_DIR)) snprintf(dir, sizeof(dir), "%s", REC_DIR);
  if (!strcasecmp(dir, "/LOOM/TAPES")) snprintf(dir, sizeof(dir), "/LOOM/BOLTS");
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
  bpm_got = -1;
  // the same file in the other mode: memory and the card measure a
  // different stretch, so a new measure would come out a little apart
  // (on SYNC: another speed, another pitch)
  if (bpm_keep > 0.0f) {
    bpm = bpm_keep;
    bpm_keep = 0.0f;
    bpm_phase = phase_keep - (clipped ? double(sec_off) : 0.0);
    Window();  // SNAP can use the beats
    Serial.printf("BPM: kept %d.%d for the same file\n", int(bpm), int(bpm * 10) % 10);
    if (mode[SPEED] == DIR_SYNC) Refresh(SPEED);
    return;
  }
  bpm = 0.0f;
  const size_t n = Frames();
  // in memory (MEM): the loop, START to END, as a loop: it comes round, so
  // ~1.5 s of it is enough. Streamed: ~32 s from START (or the beginning)
  bpm_loop = !streaming;
  if (bpm_loop) {
    bpm_from = EdgeAt(START);
    const size_t e = EdgeAt(END);
    bpm_n = e > bpm_from ? int(std::min<size_t>((e - bpm_from) / IDX_FRAMES, BPM_BINS)) : 0;
  } else {
    bpm_from = EdgeAt(START);
    if (bpm_from + 800 * IDX_FRAMES > n) bpm_from = 0;  // too little after START: from the beginning
    bpm_n = int(std::min<size_t>((n - bpm_from) / IDX_FRAMES, BPM_BINS));
  }
  if (bpm_n < (bpm_loop ? 280 : 800)) {  // under ~1.5 s (a loop) / ~4 s: no tempo
    Serial.printf("BPM: %d levels from %u of %u, too short\n", bpm_n, unsigned(bpm_from), unsigned(n));
    return;
  }
  bpm_env = static_cast<float*>(malloc(sizeof(float) * bpm_n));
  if (bpm_env) bpm_got = 0;
  Serial.printf("BPM: start, %d levels from frame %u of %u%s\n", bpm_n, unsigned(bpm_from), unsigned(n), bpm_env ? "" : ", NO MEMORY");  // over USB
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
  const float first = o[0], last = o[n - 1];
  for (int i = n - 1; i > 0; i--) o[i] = std::max(0.0f, o[i] - o[i - 1]);
  o[0] = bpm_loop ? std::max(0.0f, first - last) : 0.0f;  // (a loop: its end leads into its start)
  float mean = 0.0f;
  for (int i = 0; i < n; i++) mean += o[i];
  mean /= n;
  for (int i = 0; i < n; i++) o[i] -= mean;
  const float bin_s = IDX_FRAMES / FileRate();
  const int lmax = std::min(bpm_loop ? n - 1 : n / 2, int(4.0f * 60.0f / (70.0f * bin_s)) + 2);
  float* r = static_cast<float*>(malloc(sizeof(float) * (lmax + 1)));
  if (!r) { Serial.println("BPM: no memory to solve"); return; }
  for (int l = 0; l <= lmax; l++) {
    float acc = 0.0f;
    if (bpm_loop) {  // round the loop
      for (int i = 0; i < n; i++) acc += o[i] * o[(i + l) % n];
      r[l] = acc / float(n);
    } else {
      for (int i = 0; i + l < n; i++) acc += o[i] * o[i + l];
      r[l] = acc / float(n - l);
    }
  }
  if (r[0] <= 0.0f) { free(r); Serial.println("BPM: flat (no level changes)"); return; }
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
  Serial.printf("BPM: best %d.%d, score %d%% (needs 25%%)\n", int(best), int(best * 10) % 10, int(best_s * 100));
  if (bpm <= 0.0f) return;
  // where the beats fall: the hits folded onto one beat as angles, their
  // weighted average (finer than a level entry). A rise shows in the entry
  // the hit is in, so the middle of it.
  const float l = 60.0f / (bpm * bin_s);
  float sc = 0.0f, cc = 0.0f;
  for (int i = 0; i < n; i++) {
    if (o[i] <= 0.0f) continue;
    const float an = 2.0f * float(M_PI) * fmodf(float(i), l) / l;
    sc += o[i] * sinf(an);
    cc += o[i] * cosf(an);
  }
  float ph = atan2f(sc, cc) / (2.0f * float(M_PI)) * l;
  if (ph < 0.0f) ph += l;
  bpm_phase = double(bpm_from) + (double(ph) + 0.5) * IDX_FRAMES;
  Window();  // SNAP can use the beats now
}

FLASHMEM void PlayerDevice::OpenBrowser() {
  // just after SPOOL saved a loop: on that one, in SPOOLS; else on the file
  // that is playing
  const bool on_fresh = fresh[0] != 0;
  g_browser->Open(this, on_fresh ? REC_DIR : dir, on_fresh ? fresh : FileName());
  fresh[0] = 0;
}

// a WAV picked in the list: play from that folder
FLASHMEM void PlayerDevice::Picked(const char* folder, const char* name) {
  const bool same = nfiles && !strcmp(dir, folder) && !strcmp(names[std::min<int>(cur[FILE_], nfiles - 1)], name);
  if (!same) SetFile(folder, name);
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
  if (idx) { PsramFree(idx); idx = nullptr; idx2 = nullptr; }
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
  bpm_keep = bpm;  // (dropped below if it is another file)
  phase_keep = bpm_phase + (clipped ? double(sec_off) : 0.0);  // (MEM on a long file: a section of it)
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
    bpm_keep = 0.0f;
    snprintf(meta_path, sizeof(meta_path), "%s", mp);
    meta_known = false;
    if (!restoring && ReadMeta()) {
      applied_file = cur[FILE_];  // not another file for Apply(FILE_): the
      applied_mode = mode[FILE_]; // section and edges just read stay
      ApplyAll();
      loaded_mode = mode[FILE_];
    } else if (!restoring) {  // nothing saved for it: the whole file
      val[START] = 0;
      val[END] = 10000;
      sec_off = sec_len = 0;  // (MEM on a long file: from its start)
      Refresh(START);
      Refresh(END);
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
      if (idx) PsramFree(idx);
      idx_n = frames / IDX_FRAMES + 1;
      idx = static_cast<uint8_t*>(PsramAlloc(idx_n + idx_n / IDX2 + 1));
      idx2 = idx ? idx + idx_n : nullptr;
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

FLASHMEM int16_t PlayerDevice::Sample(const uint8_t* p) const { return WavSample(p, bytes, is_float); }

// up to n frames from the file's current position; the first two channels
// (mono: both)
FLASHMEM size_t PlayerDevice::ReadFrames(int16_t* l, int16_t* r, size_t n) {
  DMAMEM static uint8_t raw[IO_CHUNK * 8];  // (RAM2: RAM1 is short, the stack needs it)
  const size_t frame = size_t(bytes) * chans;
  const size_t k = std::min(n, sizeof(raw) / frame);
  const int got = lf.read(raw, k * frame);  // -1 on an error: nothing
  if (got <= 0) return 0;
  const size_t rd = size_t(got) / frame;
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
  DMAMEM static int16_t tl[IO_CHUNK], tr[IO_CHUNK];  // (RAM2)
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
      if (e < idx_n && pk > idx[e]) { idx[e] = pk; if (pk > idx2[e / IDX2]) idx2[e / IDX2] = pk; }
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
        if (e < idx_n && pk > idx[e]) { idx[e] = pk; if (pk > idx2[e / IDX2]) idx2[e / IDX2] = pk; }
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

// SNAP: ZERO the quietest sample nearby; BEAT (with a tempo found) the
// nearest beat, then the quietest sample just around it
FLASHMEM size_t PlayerDevice::SnapPoint(size_t at) const {
  const size_t n = Frames();
  if (cur[SNAP] != SNAP_BEAT || bpm <= 0.0f || !n) return SnapQuiet(at);
  const double beat = BeatFrames();
  const double g = bpm_phase + std::round((double(at) - bpm_phase) / beat) * beat;
  return SnapQuiet(size_t(std::clamp(g, 0.0, double(n - 1))));
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
  if (cur[SNAP]) {
    a = SnapPoint(a); b = SnapPoint(b);
    if (cur[SNAP] == SNAP_BEAT && bpm > 0.0f && b <= a) b = std::min(n, SnapQuiet(std::min(n - 1, a + size_t(BeatFrames()))));  // at least a beat
  }
  if (b <= a + AUDIO_BLOCK_SAMPLES) b = std::min(n, a + AUDIO_BLOCK_SAMPLES * 2);
  if (streaming) stream.SetWindow(a, b);
  else fx.SetWindowSamples(a, b - a);
}

FLASHMEM void PlayerDevice::Apply(int p) {
  if (mode[SPEED] < ModeMin(SPEED)) mode[SPEED] = DIR_FWD;  // (STRM plays forward only)
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
    case SPEED: {  // mode REV (0) plays backwards (MEM only: STRM can't pick it)
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
  } else if (p == START || p == END) {  // seconds into the file
    const size_t fr = Frames();
    if (!fr) { snprintf(buf, n, "%d%%", val[p] / 100); return; }
    const uint32_t ms = uint32_t(uint64_t(EdgeAt(p)) * 1000 / uint64_t(AUDIO_SAMPLE_RATE_EXACT * rate));
    FormatSeconds(ms, buf, n);
  } else if (p == SPEED) {  // SYNC: as it really plays (the file's tempo to the clock's)
    snprintf(buf, n, "%d%%", int(val[p] * SyncRatio() + 0.5f));
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
  // nothing yet: the file isn't read in until Idle() gets to it (start-up,
  // a new FILE); "can't read" only once a load of this file has failed
  if (!n) { gfxPrint(x + 28, cy - 4, loaded == fi ? PSTR("can't read") : PSTR("loading")); return; }
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
  if (!streaming && cur[SNAP]) {
    ea = SnapPoint(ea); eb = SnapPoint(eb);
    if (cur[SNAP] == SNAP_BEAT && bpm > 0.0f && eb <= ea) eb = std::min(n, SnapQuiet(std::min(n - 1, ea + size_t(BeatFrames()))));
  }
  // SNAP BEAT with a tempo found: the beats as ticks along the bottom (when
  // they are at least 4 pixels apart)
  if (cur[SNAP] == SNAP_BEAT && bpm > 0.0f) {
    const double beat = BeatFrames();
    if (beat * (w - 1) / double(span) >= 4.0) {
      double k = std::ceil((double(v0) - bpm_phase) / beat);
      for (double pos = bpm_phase + k * beat; pos < double(v0) + double(span); pos += beat) {
        if (pos < 0.0 || pos >= double(n)) continue;
        const int bx = sx(size_t(pos));
        if (bx >= x && bx < x + w) gfxLine(bx, base - 2, bx, base);
      }
    }
  }
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
// stopped, the file's own tempo in BPM (SPEED shows how it plays; ... while it is worked
// out, - when there is none)
FLASHMEM bool PlayerDevice::HeaderInfo(char* buf, size_t n) const {
  if (!active || !Frames() || loading || analysing) return false;
  if (Playing() && !too_long) {
    const uint32_t ds = uint32_t(HeadFrame() * 10.0f / FileRate());
    snprintf(buf, n, "%lu:%02lu.%lu", (unsigned long)(ds / 600), (unsigned long)(ds / 10 % 60), (unsigned long)(ds % 10));
  } else if (bpm_got >= 0) {
    snprintf(buf, n, "...");
  } else if (bpm > 0.0f) {
    const int tb = int(bpm * 10.0f + 0.5f);  // the file's own, in tenths (SPEED shows how it plays)
    snprintf(buf, n, "%d.%d", tb / 10, tb % 10);
  } else {
    snprintf(buf, n, "-");
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
  const size_t e1 = std::min((b - 1) / IDX_FRAMES, idx_n - 1);
  for (size_t e = a / IDX_FRAMES; e <= e1 && idx_n;) {
    if (e % IDX2 == 0 && e + IDX2 - 1 <= e1) { pk = std::max<int>(pk, idx2[e / IDX2]); e += IDX2; }
    else pk = std::max<int>(pk, idx[e++]);
  }
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
  "REPEAT", "ECO", 8, {"FRZ", "CLR"},  // (was ECHO; the tags stay: saved files use them)
  {
    // page 1: the delay, page 2: the reverb and TONE
    {"LEVEL", 0, 100, 40, UNI, 0, 2, 1, {}, {"MIX", "SEND"}},  // the delay's; mode: crossfade / added (SEND first)
    {"TIME", 20, 6000, 450, UNI, 0, 2, 0, {}, {"FREE", "CLOCK"}},
    // mode: the playback heads, one of HEAD_PATTERNS (named there)
    {"FDBK", 0, 100, 45, UNI, 0, 13, 0},
    {"SPRD", 0, 100, 40, UNI, 0, 3, 1, {}, {"PHASE", "PING", "DIFF"}},
    // the reverb, after the Tegeler Raummaschine: mix, pre-delay, room
    // (size and decay together), walls (soft: dark and dense .. hard:
    // bright with audible reflections; also the delay's tone)
    {"LEVEL", 0, 100, 30, UNI, 0, 2, 1, {}, {"MIX", "SEND"}},
    {"PRE", 0, 150, 20, UNI, 0, 21, 0},
    {"ROOM", 0, 100, 50, UNI, 0, 21, 0}, // mode: GLINT (a shimmer), the tail pitched back into itself, off .. 100 %
    {"WALL", -50, 50, -10, BI, 0, 20, 13},  // mode: GLINT's recipe (see ROOM; 13: OCT)
  },
  {"DELAY", "REVRB"},
};

// REPEAT's playback heads, picked with FDBK's L knob. The last head always
// sits at TIME, at full level, and is the one that feeds back, so the trail
// repeats every TIME whatever the pattern. Up to three more heads play
// before it, each at a share of TIME (0 = not there) and a level of its
// own, a little quieter, so the echo at TIME stays the main one.
struct HeadPattern {
  char name[6];
  float pos[3];   // where, as a share of TIME (0: no head)
  float gain[3];  // how loud, 0..1
};
static const HeadPattern HEAD_PATTERNS[] = {
  {"ONE",   {},                        {}},                    // a plain delay
  {"HALF",  {0.5f},                    {0.6f}},                // eighths on a quarter TIME
  {"DOTTD", {0.75f},                   {0.6f}},                // a dotted eighth, then the quarter
  {"TRIPL", {0.667f},                  {0.6f}},                // a triplet feel: long, short
  {"3RDS",  {0.333f, 0.667f},          {0.45f, 0.6f}},         // even triplets
  {"QUART", {0.25f, 0.5f, 0.75f},      {0.35f, 0.45f, 0.6f}},  // even sixteenths
  {"TAPE",  {0.34f, 0.63f},            {0.5f, 0.65f}},         // three heads a little uneven, like an old tape echo
  {"GOLD",  {0.382f, 0.618f},          {0.5f, 0.6f}},          // the golden section: never quite on the grid
  {"CLAVE", {0.375f, 0.75f},           {0.55f, 0.65f}},        // 3 + 3 + 2, the tresillo
  {"QUINT", {0.4f, 0.8f},              {0.5f, 0.65f}},         // quintuplet steps
  {"SLAP",  {0.125f},                  {0.7f}},                // a slapback just after the sound, then the echo
  {"FLAM",  {0.9f},                    {0.5f}},                // the echo doubled, a flam
  {"OPEN",  {0.125f, 0.25f, 0.5f},     {0.4f, 0.5f, 0.6f}},    // the gaps open up: each twice the last
};
constexpr int HEAD_N = sizeof(HEAD_PATTERNS) / sizeof(HEAD_PATTERNS[0]);

class EchoDevice : public Device {
public:
  enum { DELAY, TIME, FEEDBACK, SPREAD, REVERB, PRE, ROOM, WALL };
  bool Varies(int p) const override { return p == PRE; }
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
  void Shimmer();  // GLINT to the engine
  const char* AltName(int p) const override { return p == ROOM ? "GLINT" : p == WALL ? "PRSET" : nullptr; }
  int Step(int p, int v, int dir) const override;
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  // the LEVELs are named by their mode: SEND or MIX
  const char* Name(int p) const override {
    return p == DELAY || p == REVERB ? param(p).modes[mode[p] & 1] : param(p).name;
  }
  bool NameIsMode(int p) const override { return p == DELAY || p == REVERB; }
  void Action(int a) override;
  bool ActionLit(int a) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override;
  void TempoChanged() override { if (mode[TIME] != SYNC_FREE) SetTimes(); }
  uint32_t TailMs() const override;
  // TIME's mode turned: FREE takes the time it was on in sync (in ms), and
  // back on the clock it snaps to the note value nearest it
  void NudgeMode(int p, int dir) override {
    const int was = mode[TIME];
    Device::NudgeMode(p, dir);
    if (p != TIME || mode[TIME] == was) return;
    if (mode[TIME] == SYNC_FREE) {
      const float secs = DivSeconds(NUM_DIVS - 1 - DivIndex(val[TIME], 20, 6000));
      val[TIME] = int16_t(constrain(int(secs * 1000.0f + 0.5f), 20, 6000));
    } else {
      const float secs = val[TIME] * 0.001f;
      int best = NUM_DIVS - 1;
      float err = 1e9f;
      for (int i = 0; i < NUM_DIVS; i++) {
        const float d = DivSeconds(i);
        if (d > MAX_SECS - 0.01f && i < NUM_DIVS - 1) continue;  // (longer than the delay holds)
        const float e = fabsf(log2f(d / secs));
        if (e < err) { err = e; best = i; }
      }
      val[TIME] = int16_t(DivValue(NUM_DIVS - 1 - best, 20, 6000));
    }
    Refresh(TIME);
  }
  void Parked() override { if (mem_ok) echo.Reset(); }  // no old tail on return
  // a part switched off on its own page (B): bit 0 the delay, bit 1 the
  // reverb; the slot's bypass (chain view) leaves these as they are
  uint8_t off = 0;
  void SetOff(uint8_t o) {
    off = o & 3;
    echo.delay_open(!(off & 1));
    echo.reverb_open(!(off & 2));
  }

private:
  AudioEffectEcho echo;
  bool mem_ok = false;
  float tap_secs[2][TAPS] = {};
  Meter tap_m[2][TAPS], tail_m;
  // The playback heads (see HEAD_PATTERNS): the engine has four, the last
  // at TIME; the pattern on FDBK's L knob says which of the others play,
  // where and how loud. The silent ones aren't even read.
  const HeadPattern& Heads() const { return HEAD_PATTERNS[constrain(int(mode[FEEDBACK]), 0, HEAD_N - 1)]; }
  bool HeadOn(int t) const { return t == TAPS - 1 || (t < TAPS - 1 && Heads().pos[t] > 0.0f); }
  float HeadPos(int t) const { return HeadOn(t) && t < TAPS - 1 ? Heads().pos[t] : 1.0f; }
  float HeadGain(int t) const { return t == TAPS - 1 ? 1.0f : HeadOn(t) ? Heads().gain[t] : 0.0f; }
  // FDBK's knob to the loop gain: a curve, so the middle of the knob
  // already gives long trails (50 -> 0.67, 75 -> 0.89, 100 -> 1.0: held,
  // the soft clip and WALL's filter keep it from running away)
  float FbGain() const { return 1.0f - powf(1.0f - constrain(cur[FEEDBACK], 0, 100) * 0.01f, 1.6f); }
  // what one round really keeps, for the picture: the loop gain and about
  // what WALL's filter takes away on each pass (a rough figure, ~1 dB)
  float FbRound() const { return FbGain() * 0.89f; }

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
  const float decay = DecaySecs() * (1.0f + mode[ROOM] * 0.1f) + cur[PRE] * 0.001f;  // (GLINT rings on)
  // rounds until it is ~60 dB down (at most 40)
  const float r = FbRound();
  const float rounds = r > 0.001f ? std::min(40.0f, logf(0.001f) / logf(std::min(r, 0.99f))) : 1.0f;
  const float echoes = tap_secs[0][TAPS - 1] * (1.0f + rounds);
  return uint32_t((decay + echoes) * 1000.0f) + 200;
}

FLASHMEM void EchoDevice::CpuMax(float parts[2]) {
  parts[0] = echo.processorUsageMax();
  parts[1] = 0.0f;
  echo.processorUsageMaxReset();
}

FLASHMEM void EchoDevice::SetTimes() {
  const float secs = mode[TIME] == SYNC_FREE ? cur[TIME] * 0.001f
    : DivSeconds(NUM_DIVS - 1 - DivIndex(cur[TIME], 20, 6000));  // (the longest at the top)
  // PHASE: the right side's heads up to half as late again (a wide stereo echo)
  const float phase = mode[SPREAD] == PHASE ? 1.0f + 0.5f * cur[SPREAD] * 0.01f : 1.0f;
  for (int c = 0; c < 2; c++) {
    for (int t = 0; t < TAPS; t++) {
      // each head at its share of TIME (the silent ones simply at TIME)
      float s = secs * HeadPos(t) * (c ? phase : 1.0f);
      s = std::min(s, MAX_SECS - 0.01f);
      tap_secs[c][t] = s;
      echo.tap(c, t, int(s * AUDIO_SAMPLE_RATE_EXACT));
      echo.tap_gain(t, HeadGain(t));
    }
  }
}

// GLINT: ROOM's mode the amount, WALL's a recipe taken from Eventide's
// ShimmerVerb presets: left / right pitch (cents), the wait before the
// shifters (ms), and the low / mid / high bands that go round again
// (the lows kept down: they made it massive)
struct GlintRecipe { char name[6]; int16_t cl, cr, ms; float lo, mid, hi, fb; };
constexpr int GLINT_N = 20;
// A to Z (after the preset named; fb: how strongly it goes round)
PROGMEM static const GlintRecipe GLINTS[GLINT_N] = {
  {"2OCT", 1200, 2400, 120, 0.4f, 0.9f, 0.8f, 0.75f},  // Happy Trailing
  {"4TH5", 504, 702, 60, 0.6f, 0.7f, 0.75f, 0.75f},    // Impressions: fourth / fifth
  {"5TH", 691, 695, 40, 0.2f, 0.5f, 0.7f, 0.8f},       // Glint of Steel
  {"12TH", 1899, 1905, 100, 0.3f, 0.85f, 0.6f, 0.9f},  // Ice Amphitheater
  {"ANGEL", 3, 2399, 80, 0.1f, 0.66f, 0.73f, 0.9f},    // Solarium Sunset: as it is / two octaves
  {"DEEP", -1200, 1200, 40, 1.0f, 0.2f, 0.0f, 0.85f},  // Strata: dark, both ways
  {"DOVE", 509, -1202, 380, 0.3f, 0.3f, 1.0f, 1.0f},   // Dove Line: slow, up a fourth / down an octave
  {"DRIFT", -30, -27, 60, 0.7f, 0.95f, 0.75f, 1.0f},   // Mercury Tinctures: sinking slowly
  {"DUST", -1203, 1205, 50, 0.6f, 0.6f, 1.0f, 1.0f},   // Majestic Dust: octave down / up
  {"GLASS", 2410, 710, 60, 0.15f, 0.55f, 0.55f, 0.7f}, // High Harmonics Too
  {"HALL", -7, 6, 30, 0.6f, 0.6f, 0.55f, 1.0f},        // MicroPitch Down the Hall
  {"HALO", 702, 1209, 90, 0.4f, 0.45f, 0.67f, 0.8f},   // Opiate Halo
  {"MICRO", -25, 23, 15, 0.8f, 0.9f, 0.7f, 1.1f},      // Radiant Caverns: no climb, a lush moving room
  {"OCT", 1204, 1193, 50, 0.4f, 0.9f, 0.8f, 0.8f},     // the Default
  {"OCT+", 1200, 1209, 90, 0.5f, 0.9f, 0.8f, 1.0f},    // Touched By an H9: more of it
  {"ORGAN", 700, 1200, 20, 0.3f, 0.5f, 0.3f, 1.0f},    // Pipe Organ Decays
  {"SKY", 2390, 2410, 350, 0.5f, 0.95f, 0.75f, 0.9f},  // Heaven Can't Wait
  {"SPRNG", 1206, 1193, 66, 0.0f, 0.5f, 0.9f, 0.85f},  // Celestial Springs: no lows, bright
  {"STAIR", -703, 502, 40, 1.0f, 1.0f, 0.25f, 0.85f},  // Ambiguous Stairway
  {"STARS", 1208, 1903, 300, 0.3f, 0.85f, 0.9f, 0.9f}, // The Pleiades
};
FLASHMEM void EchoDevice::Shimmer() {
  const GlintRecipe& r = GLINTS[constrain(int(mode[WALL]), 0, GLINT_N - 1)];
  echo.shimmer_recipe(r.cl, r.cr, r.ms, r.lo, r.mid, r.hi, r.fb);
  echo.shimmer(mode[ROOM] * 0.05f);
}

FLASHMEM void EchoDevice::Apply(int p) {
  const float v = cur[p] * 0.01f;
  switch (p) {
    case DELAY:    echo.delay_mix(v); echo.delay_send(mode[DELAY] == 1); break;
    case TIME:     SetTimes(); break;
    case REVERB:   echo.reverb_mix(v); echo.reverb_send(mode[REVERB] == 1); break;
    case PRE:
      echo.pre_delay(cur[PRE]);
      Shimmer();
      break;
    case ROOM:     echo.size(v); echo.decay(DecaySecs()); Shimmer(); break;
    case FEEDBACK: echo.feedback(FbGain()); SetTimes(); break;  // (its mode: the head pattern)
    case SPREAD:
      echo.pingpong(mode[SPREAD] == PING ? v : 0.0f);
      echo.diffuse(mode[SPREAD] == DIFFUSE ? v : 0.0f);
      SetTimes();
      break;
    case WALL:  // soft walls: dark and dense; hard: bright, sparse reflections
      echo.tone(cur[WALL] * 0.02f);
      echo.density(0.5f - cur[WALL] * 0.01f);
      Shimmer();  // (its mode: GLINT's interval)
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
    FormatDiv(NUM_DIVS - 1 - DivIndex(val[TIME], 20, 6000), buf, n);
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

// WALL's mode: GLINT's recipe; ROOM's: how much of it
FLASHMEM void EchoDevice::FormatMode(int p, char* buf, size_t n) const {
  if (p == WALL) snprintf(buf, n, "%s", GLINTS[constrain(int(mode[WALL]), 0, GLINT_N - 1)].name);
  else if (p == ROOM) snprintf(buf, n, mode[ROOM] ? "%d" : "off", mode[ROOM] * 5);
  else if (p == FEEDBACK) snprintf(buf, n, "%s", Heads().name);
  else Device::FormatMode(p, buf, n);
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
    // page 1, the delay: one time line, 20 ms at the left edge to 6 s (the
    // longest delay) at the right, spaced logarithmically, so a short delay
    // and a long one both have room, and the repeats crowd together towards
    // the right as they would on a tape. Small ticks on the base line mark
    // 0.1 s and 1 s.
    // Filled bars: the heads that play, as tall as DELAY times their level.
    // With SPRD up an outlined bar shows the right side: on top of the left
    // one at first, moving out a pixel at a time as SPRD goes up (PHASE:
    // where the right side's echo really is; PING: as tall as that side plays).
    // Thin lines further on: the repeats through FDBK, each round lower by
    // the FDBK amount; with SPRD up the right side's repeats dotted next to them.
    {
      constexpr float T0 = 0.02f;
      const float lspan = logf(MAX_SECS / T0);
      const int tw = w - 4;
      auto px = [&](float sec) {
        return x + int(constrain(logf(std::max(sec, T0) / T0) / lspan, 0.0f, 1.0f) * tw);
      };
      for (float tick : {0.1f, 1.0f}) gfxLine(px(tick), base - 2, px(tick), base);
      if (cur[DELAY] > 0) {
        const int bh = std::max(1, (h - 2) * cur[DELAY] / 100);
        const float period = tap_secs[0][TAPS - 1], period_r = tap_secs[1][TAPS - 1];
        const float fb = std::min(1.0f, FbRound());  // what each round keeps, as heard
        const float sp = cur[SPREAD] * 0.01f;
        // PING: the repeats bounce - each round mostly on one side, the
        // other side down by SPRD (left on the even rounds, right on the odd)
        const bool ping = mode[SPREAD] == PING && sp > 0.0f;
        auto side_h = [&](int h0, int round, bool right) {
          if (!ping) return h0;
          const bool away = ((round & 1) != 0) != right;  // not this round's side
          return away ? int(h0 * (1.0f - sp)) : h0;
        };
        // how far right of the left side's echo the right side's is drawn:
        // PHASE where it really is (up to about 9 pixels on this line),
        // PING and DIFF (same time, other place) the same way, by SPRD
        auto SideOffset = [&](int t, int round) {
          if (mode[SPREAD] == PHASE)
            return std::max(0, px(round * period_r + tap_secs[1][t]) - px(round * period + tap_secs[0][t]));
          return int(sp * 9.0f + 0.5f);
        };
        for (int t = 0; t < TAPS; t++) {
          if (!HeadOn(t)) continue;
          const int th = std::max(1, int(bh * HeadGain(t)));
          const int lx = px(tap_secs[0][t]);
          gfxRect(lx, base - th, 2, th);
          if (sp <= 0.0f) continue;
          // the right side: an outlined bar that sits on the left one at
          // SPRD 0 and moves out a pixel at a time as SPRD goes up
          const int hr = side_h(th, 0, true);
          if (hr > 0) gfxFrame(lx + SideOffset(t, 0), base - hr, 2, hr);
        }
        // repeats: the whole pattern again after each period
        float g = fb;
        for (int round = 1; round < 24 && period > 0.0f; round++, g *= fb) {
          if (g < 0.016f || int(bh * g) < 1) break;  // quieter than ~-36 dB: not heard in the mix
          bool any = false;
          for (int t = 0; t < TAPS; t++) {
            if (!HeadOn(t)) continue;
            const int rh = int(bh * g * HeadGain(t));
            if (rh < 1) continue;
            const float sec = round * period + tap_secs[0][t];
            if (sec > MAX_SECS) break;
            const int lx = px(sec);
            const int hl = side_h(rh, round, false), hr = side_h(rh, round, true);
            if (hl > 0) gfxLine(lx, base - hl + 1, lx, base);
            if (sp > 0.0f && hr > 0) {
              const int rx = lx + SideOffset(t, round);
              if (rx < x + w) gfxDottedLine(rx, base - hr + 1, rx, base, 2);
            }
            any = true;
          }
          if (!any) break;
        }
      }
    }
  } else {
    // page 2, the reverb: two walls that ROOM moves apart from 30% to the
    // edges, and between them a wedge fading out from the middle to both
    // sides, as tall as LEVEL and reaching towards the walls with ROOM,
    // falling off like a real tail (quickly first, then a long low end);
    // PRE leaves a gap in the middle. WALL makes the walls crisp (hard) or
    // diffuse (soft); the tail stays solid.
    const int cx = x + w / 2, hw = w / 2 - 1;
    const int wall = hw * (30 + 70 * cur[ROOM] / 100) / 100;
    const int gap = (wall - 2) * cur[PRE] / 150 / 4;  // the pre-delay, at most a quarter of the way to each wall
    const int th = int((h - 2) * sqrtf(cur[REVERB] * 0.01f));  // shows more early on (50% = 70% high)
    const int len = std::max(2, (wall - 1 - gap) * (20 + 80 * cur[ROOM] / 100) / 100);
    constexpr float k = 4.0f, end = 0.0183156f;  // e^-4
    auto tail = [&](int i, int l, int t) {  // the tail's height i columns out
      return int(t * (expf(-k * (i - gap) / l) - end) / (1.0f - end) + 0.5f);
    };
    // GLINT (ROOM's mode): the tail pitched back into itself, ringing on. A
    // soft halo of dots round the tail, longer and taller with more GLINT,
    // and dark sparkles in the tail, both drifting up as the
    // shimmer climbs; the dots sit on a fixed pattern that slides, so they
    // glide instead of twinkling
    const float glint = mode[ROOM] * 0.05f;  // 0 .. 1
    // (whole pixels: slid by fractions the dots twinkled)
    const float drift = float(int((millis() % 100000) * 0.012f));  // px
    auto spark = [&](int px, int py) {  // 0..1, fixed per place on the sliding pattern
      const float t = px * 0.7548777f + (py + drift) * 0.5698403f;
      return t - floorf(t);
    };
    const int len2 = glint > 0.0f ? std::min(wall - 1 - gap, int(len * (1.0f + glint))) : 0;
    const int th2 = std::min(h - 2, int(th * (1.0f + 0.5f * glint) + 2.0f * glint + 0.5f));
    for (int i = gap; i < gap + std::max(len, len2); i++) {
      const int hh = i < gap + len ? tail(i, len, th) : 0;
      for (int sd : {1, -1}) {
        if (sd < 0 && !i) continue;
        const int px = cx + sd * i;
        if (hh > 0) {
          if (glint > 0.0f)  // the tail, with dark sparkles
            for (int py = base - hh + 1; py <= base; py++) {
              if (spark(px, py) >= 0.12f * glint) gfxPixel(px, py);
            }
          else
            gfxLine(px, base - hh + 1, px, base);
        }
        if (len2) {  // the halo above it, thinning out upwards and outwards
          const int h2 = tail(i, len2, th2);
          const float out = float(i - gap) / len2;
          for (int py = base - h2 + 1; py <= base - hh; py++) {
            const float up = float(base - py) / std::max(1, h2);
            if (spark(px, py) < glint * 0.45f * (1.0f - 0.6f * out) * (1.0f - 0.5f * up)) gfxPixel(px, py);
          }
        }
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
  for (int t = 0; t < TAPS; t++) {  // the heads that play
    if (!HeadOn(t)) continue;
    const int th = std::max(1, int(bh * HeadGain(t)));
    gfxLine(px(tap_secs[0][t]), base - th + 1, px(tap_secs[0][t]), base);
  }
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
  "TEXTURE", "TEX", 8, {"NEW", "NOIS"},  // A: new variations now; B: a burst of its noise
  {
    {"DRIVE", 0, 100, 0, UNI, 0, 21, 0},
    {"FOLD", 0, 100, 0, UNI, 0, 21, 0},
    {"CRSH", 0, 100, 0, UNI, 0, 3, 0, {}, {"BIT", "REDUX", "BOTH"}},
    {"TILT", -50, 50, 0, BI, 0, 3, 1, {}, {"LOW", "MID", "HIGH"}},
    {"NOICE", 0, 100, 0, UNI, 0, 3, 1, {}, {"WHITE", "PINK", "DUST"}},
    {"N.DCY", 0, 100, 40, UNI, 0, 21, 0},
    {"N.TON", 0, 100, 50, UNI, 0, 21, 0},
    {"WET", 0, 100, 100, UNI, 0, 21, 0},
  },
  {"", ""},
};

// HEDDLE, in TEXTURE's slot instead of it. Mode: SHIFT, a frequency
// shifter (as Bode's): in Hz, down left of the middle, up right; or PHASE,
// an endless phaser (a barber pole): its notches rise (right) or fall
// (left) for ever, in octaves a second; SHCLK / PHCLK: the same on the
// clock (a turn / an octave a note value). SHIFT: SIDE, which way the
// sound goes (BOTH: up left, down right); PHASE: STAGE, 4 .. 20
// all-passes, and RESO. SIDE / STAGE, FDBK / RESO and MIX: modes are
// variations, drawn anew on each beat (SIDE: the chance it changes; STAGE:
// +- 2 n stages; FDBK / RESO, MIX: +- 5 n %)
PROGMEM static const DeviceDef HEDDLE_DEF = {
  "HEDDLE", "HDL", 4, {"NEW", "FLIP"},  // A: new variations now; B: the shift / the phaser the other way (again: back)
  {
    {"SHIFT", -100, 100, 0, BI, 0, 4, 0, {}, {"SHIFT", "SHCLK", "PHASE", "PHCLK"}},
    {"SIDE", 0, 16, 0, UNI, 0, 11, 0},
    {"FDBK", 0, 100, 0, UNI, 0, 21, 0},
    {"MIX", 0, 100, 100, UNI, 0, 21, 0},
  },
  {""},
};

class DirtDevice : public Device {
public:
  enum { DRIVE, FOLD, CRUSH, TILT, NOISE, NDECAY, NTONE, WET };
  bool Varies(int p) const override { return p == DRIVE || p == FOLD || p == NDECAY || p == NTONE || p == WET; }
  DirtDevice() : Device(DIRT_DEF) {}
  void Action(int a) override { if (a == 0) VarNew(); else fx.noise_burst(); }
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
    // TILT tips the picture: brighter rises to the right, darker to the left
    const int tilt = int(cur[TILT] * 0.02f * 3.0f * (2.0f * i / std::max(1, nc - 1) - 1.0f));
    ys[i] = constrain(cy - int(constrain(yo, -1.0f, 1.0f) * lim) - tilt, y, y + h - 1);
  }
  ThinLine(x, ys, nc, keep);
  // NOICE: grains over it as loud as the noise is now, looking like its type
  // (as on the page): WHITE fine and even, PINK bigger and sinking to the
  // bottom, DUST the odd speck in bursts
  const float lvl = noise_m.v;
  switch (mode[NOISE]) {
    case AudioEffectDirt::NOISE_WHITE:
      for (int i = 0, n = int(lvl * 30.0f); i < n; i++) gfxPixel(x + random(w), y + random(h));
      break;
    case AudioEffectDirt::NOISE_PINK:
      for (int i = 0, n = int(lvl * 12.0f); i < n; i++) {
        const int r = random(h - 1);
        gfxRect(x + random(w - 1), y + (h - 2) - (r * r) / (h - 1), 2, 2);
      }
      break;
    default:
      if (lvl > 0.02f && random(100) < int(lvl * 160.0f))
        gfxRect(x + random(w - 2), y + random(h - 2), 2, 2);
  }
}

class HeddleDevice : public Device {
public:
  enum { SHIFT, SIDE, HFDBK, HMIX };
  HeddleDevice() : Device(HEDDLE_DEF) {}
  // HEDDLE: SHIFT or PHASE, each free or on the clock
  bool Phasing() const { return mode[SHIFT] >= 2; }
  bool OnClk() const { return mode[SHIFT] & 1; }
  // SHIFT in Hz: 0.05 .. 2000, either way; PHASE in octaves a second,
  // 0.01 .. 2.5 (the clock: a turn / an octave a note value)
  float ShiftHz(int v) const {
    if (!v) return 0.0f;
    const float r = OnClk() ? 1.0f / DivSeconds(DivIndex(abs(v), 1, 100))
                  : Phasing() ? 0.01f * powf(250.0f, (abs(v) - 1) / 99.0f)
                              : 0.05f * powf(40000.0f, abs(v) * 0.01f);
    return v < 0 ? -r : r;
  }
  // as heard: the settings with this beat's variations
  int Stages() const { return 4 + 2 * constrain(int(cur[SIDE]) + v_stage, 0, 8); }  // 4, 6 .. 20
  int MaxStages() const { return 4 + 2 * constrain(int(cur[SIDE]) + (Phasing() ? mode[SIDE] : 0), 0, 8); }  // (the CPU)
  int Side() const { return v_side >= 0 ? v_side : std::min<int>(cur[SIDE], 2); }
  float Fdbk() const { return constrain(cur[HFDBK] + v_fb, 0.0f, 100.0f); }
  float Mix() const { return constrain(cur[HMIX] + v_mix, 0.0f, 100.0f); }
  const char* Name(int p) const override {
    if (Phasing() && p == SIDE) return "STAGE";
    if (Phasing() && p == HFDBK) return "RESO";
    return Device::Name(p);
  }
  int8_t was_phase = -1;  // (switching SHIFT <-> PHASE: SIDE / STAGE start sensible)
  void TempoChanged() override { if (OnClk()) Apply(SHIFT); }
  void Idle() override;
  void FormatMode(int p, char* buf, size_t n) const override;
  // the variations (the L knob, straight): the bar over the name shows how
  // much: +- ones out from the middle, SIDE's chance (no +-) from the left
  int BarStyle(int p) const override {
    if (p == SIDE) return Phasing() ? BAR_CENTRE : BAR_LEFT;
    return p == HFDBK || p == HMIX ? BAR_CENTRE : BAR_PARTS;
  }
  int Step(int p, int v, int dir) const override {
    if (p == SIDE) return constrain(v + dir, 0, Phasing() ? 8 : 2);
    if (p != SHIFT || !OnClk()) return v + dir;
    // on the clock: one note value a turn (the longest next to the middle)
    const int s = v > 0 ? 1 : v < 0 ? -1 : 0;
    if (!s) return dir > 0 ? DivValue(0, 1, 100) : -DivValue(0, 1, 100);
    const int idx = DivIndex(abs(v), 1, 100) + dir * s;
    return idx < 0 ? 0 : s * DivValue(idx, 1, 100);
  }
  float pole = 0.0f;           // the picture's barber pole, where it is (in stripes)
  float per_now = 16.0f;       // its stripes' spacing, gliding to the new one
  float slide = 0.0f;          // how far the dither has moved (px, round 4)
  uint32_t pole_at = 0;
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  void Apply(int p) override;
  void Action(int a) override {
    if (a == 0) v_beat = loom::clock_beat_n + 1;  // (Idle draws them)
    else { flipped = !flipped; Send(); }
  }
  bool ActionLit(int a) const override { return a == 1 && flipped; }
  void Format(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override { DrawPole(x, y, w, h); }
  void DrawMini(int x, int y, int w, int h) override { DrawPole(x, y, w, h); }
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
private:
  AudioEffectDirt fx;  // (TEXTURE's engine with its own part left clean)
  void DrawPole(int x, int y, int w, int h);
  void Send();
  // the variations: drawn on each beat; FDBK and MIX glide there
  int8_t v_stage = 0, v_side = -1;  // (-1: SIDE as set)
  float v_fb = 0.0f, v_fb_to = 0.0f, v_mix = 0.0f, v_mix_to = 0.0f;
  uint32_t v_beat = 0, v_ms = 0;
  bool flipped = false;  // FLIP: the other way
};

FLASHMEM void HeddleDevice::Apply(int p) {
  switch (p) {
    case SHIFT: case SIDE: case HFDBK: case HMIX:
      if (was_phase != int8_t(Phasing())) {  // SIDE <-> STAGE: from where it makes sense
        if (was_phase >= 0) val[SIDE] = cur[SIDE] = Phasing() ? 4 : 0;  // 12 stages / UP
        was_phase = int8_t(Phasing());
      }
      Send();
      break;
  }
}

FLASHMEM void HeddleDevice::Send() {
  const float hz = ShiftHz(cur[SHIFT]) * (flipped ? -1.0f : 1.0f);
  if (Phasing()) fx.phase(hz, Stages(), Fdbk() * 0.009f, Mix() * 0.01f);
  else fx.shift(hz, Side(), Fdbk() * 0.0095f, Mix() * 0.01f);
}

// the variations: new ones on each beat, FDBK / RESO and MIX gliding
// there (~50 ms), SIDE and STAGE at once (the engine glides the side)
FLASHMEM void HeddleDevice::Idle() {
  if (!active) return;
  const uint32_t now = millis();
  if (now - v_ms < 4) return;
  const float dt = std::min(0.1f, (now - v_ms) * 0.001f);
  v_ms = now;
  bool send = false;
  if (loom::clock_beat_n != v_beat) {
    v_beat = loom::clock_beat_n;
    const auto rnd = [] { return random(-1000, 1001) * 0.001f; };
    v_fb_to = mode[HFDBK] ? rnd() * mode[HFDBK] * 5.0f : 0.0f;
    v_mix_to = mode[HMIX] ? rnd() * mode[HMIX] * 5.0f : 0.0f;
    const int8_t st = Phasing() && mode[SIDE] ? int8_t(random(-mode[SIDE], mode[SIDE] + 1)) : 0;
    int8_t sd = -1;
    if (!Phasing() && mode[SIDE]) sd = random(10) < mode[SIDE] ? int8_t(random(3)) : v_side;
    send = st != v_stage || sd != v_side;
    v_stage = st;
    v_side = sd;
  }
  if (!mode[SIDE] || Phasing()) { send |= v_side >= 0; v_side = -1; }
  if (!mode[SIDE] || !Phasing()) { send |= v_stage != 0; v_stage = 0; }
  if (!mode[HFDBK]) v_fb_to = 0.0f;
  if (!mode[HMIX]) v_mix_to = 0.0f;
  const float k = std::min(1.0f, dt * 20.0f);
  if (v_fb != v_fb_to || v_mix != v_mix_to) {
    v_fb += (v_fb_to - v_fb) * k;
    v_mix += (v_mix_to - v_mix) * k;
    if (fabsf(v_fb - v_fb_to) < 0.05f) v_fb = v_fb_to;
    if (fabsf(v_mix - v_mix_to) < 0.05f) v_mix = v_mix_to;
    send = true;
  }
  if (send) Send();
}

// the variations: OFF, then SIDE: the chance a beat it changes, STAGE: +-
// stages, FDBK / RESO, MIX: +- percent
FLASHMEM void HeddleDevice::FormatMode(int p, char* buf, size_t n) const {
  if (p == SHIFT) Device::FormatMode(p, buf, n);
  else if (mode[p] == 0) snprintf(buf, n, "OFF");
  else if (p == SIDE && Phasing()) snprintf(buf, n, LOOM_PM "%d", 2 * mode[p]);
  else if (p == SIDE) snprintf(buf, n, "%d%%", mode[p] * 10);
  else FormatVar(buf, n, mode[p] * 5);  // FDBK / RESO, MIX: up or down
}

FLASHMEM void HeddleDevice::Format(int p, char* buf, size_t n) const {
  if (p == SIDE) {  // SHIFT: the side; PHASE: the stages
    static const char* const sides[3] = {"UP", "DOWN", "BOTH"};
    if (Phasing()) snprintf(buf, n, "%d", 4 + 2 * constrain(int(val[SIDE]), 0, 8));
    else snprintf(buf, n, "%s", sides[constrain(int(val[SIDE]), 0, 2)]);
  } else if (p == SHIFT && Phasing() && !OnClk()) {  // octaves a second
    const float r = fabsf(ShiftHz(val[SHIFT]));
    const char sg = val[SHIFT] < 0 ? '-' : val[SHIFT] > 0 ? '+' : ' ';
    if (!val[SHIFT]) snprintf(buf, n, "0");
    else snprintf(buf, n, "%c%d.%02d", sg, int(r), int(r * 100.0f + 0.5f) % 100);
  } else if (p == SHIFT && OnClk() && val[SHIFT]) {
    char d[8];
    FormatDiv(DivIndex(abs(val[SHIFT]), 1, 100), d, sizeof(d));
    // with its sign: a 1/x note value without the 1 (+/16Q), so it fits
    snprintf(buf, n, "%c%s", val[SHIFT] < 0 ? '-' : '+', d[0] == '1' && d[1] == '/' ? d + 1 : d);
  } else if (p == SHIFT) {  // Hz, finer near 0
    const float hz = fabsf(ShiftHz(val[SHIFT]));
    const char sg = val[SHIFT] < 0 ? '-' : val[SHIFT] > 0 ? '+' : ' ';
    if (!val[SHIFT]) snprintf(buf, n, "0");
    else if (hz < 10.0f) snprintf(buf, n, "%c%d.%02d", sg, int(hz), int(hz * 100.0f) % 100);
    else if (hz < 1000.0f) snprintf(buf, n, "%c%d", sg, int(hz + 0.5f));
    else snprintf(buf, n, "%c%dk%d", sg, int(hz) / 1000, int(hz) % 1000 / 100);
  } else {
    Device::Format(p, buf, n);
  }
}

FLASHMEM void HeddleDevice::DrawPole(int x, int y, int w, int h) {
  // HEDDLE: a barber pole. Its stripes roll up (shifted up) or down,
  // faster the further SHIFT goes; BOTH: up on the left, down on the
  // right. SHIFT / SHCLK: level stripes, PHASE / PHCLK: slanting ones.
  // FDBK / RESO thickens them; MIX fills them in, a dither as FIBRE's
  // wave (0: a faint trace, 100: solid)
  const uint32_t now = millis();
  const float dt = pole_at ? std::min(0.1f, (now - pole_at) * 0.001f) : 0.0f;
  pole_at = now;
  const float hz = ShiftHz(cur[SHIFT]);
  // px a second: lively, up to ~60 (the page is drawn ~500 times a second)
  const float raw = Phasing() ? hz * 24.0f  // (octaves a second)
                              : (hz < 0.0f ? -1.0f : 1.0f) * 12.0f * log2f(1.0f + fabsf(hz) * 4.0f);
  const float speed = constrain(raw, -60.0f, 60.0f);
  // PHASE: the stripes as wide as the stages are few (4: wide, 20: fine),
  // gliding there and grown / shrunk from the middle of the picture
  // (whole pixels: a fraction of one makes the edge dots shimmer)
  const float per_to = Phasing() ? roundf(24.0f - (Stages() - 4) * 0.875f) : 16.0f;  // 24 .. 10 px
  per_now += (per_to - per_now) * std::min(1.0f, dt * 8.0f);
  if (fabsf(per_to - per_now) < 0.01f) per_now = per_to;
  const float per = per_now;
  pole += speed * dt / per;
  pole -= floorf(pole);
  const float off = pole * per;  // (px)
  // the dither's own count, round its 4 px: wrapped with the stripes
  // (round their spacing) it would jump unless that is a multiple of 4
  slide += speed * dt;
  slide -= 4.0f * floorf(slide / 4.0f);
  slide += (off - floorf(off)) - (slide - floorf(slide));  // (the same fraction as the stripes)
  const float sw = std::max(1.0f, roundf((2 + int(Fdbk()) * 5 / 100) * per / 16.0f));
  // the fill dots: every line of a stripe gets its share (a square dither
  // left whole slanting lines dark at low MIX: they blinked in and out)
  static const uint8_t ord[4] = {0, 2, 1, 3};
  const int fill = 1 + int(Mix()) * 15 / 100;  // 1 .. 16 of 16
  const bool level = !Phasing();
  for (int px = 0; px < w; px++) {
    const bool down = !Phasing() && (Side() == 1 || (Side() == 2 && px >= w / 2));
    for (int py = 0; py < h; py++) {
      // where this pixel is across the stripes, in fractions of a pixel:
      // the edges are lit by how much of them the stripe covers (shaded
      // with the dither), so the stripes glide instead of stepping
      const int cx = px - w / 2, cy = py - h / 2;
      float u = level ? cy + (down ? -off : off)
                      : cx + (down ? -cy : cy) + (down ? -off : off);
      u -= per * floorf((u + 1.0f) / per);  // (-1 .. per - 1: the leading edge too)
      float cover = std::min(1.0f, std::max(0.0f, std::min(u + 1.0f, sw - u)));
      // (thinly filled: hard edges; half-lit edges came and went, pumping)
      if (fill <= 8) cover = cover >= 0.5f ? 1.0f : 0.0f;
      if (cover <= 0.0f) continue;
      // (the dither travels with its stripe: no dots blinking at the edges)
      const int moved = int(floorf(down ? -slide : slide));
      const int a = level ? py + moved : px + (down ? -py : py) + moved;  // which line
      const int t = level ? px : px + moved;  // where along it (the dots ride with it)
      const int d = 4 * ord[(t + a) & 3] + ord[a & 3];
      if (d < int(fill * cover + 0.5f)) gfxPixel(x + px, y + py);
    }
  }
  if (!Phasing() && Side() == 2) gfxLine(x + w / 2, y, x + w / 2, y + h - 1);
}

/////////////////////////////////////////////////////////////////
// RESONATOR: 48-band resonant filter bank
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef RESONATOR_DEF = {
  "REED", "RES", 8, {"HIT", "NEW"},  // (was RESONATOR) A: struck, it rings without any input; B: a new variation
  {
    {"CUT", 0, 100, 70, UNI, 0, 21, 0},
    {"SLOPE", -50, 50, -50, BI, 0, 21, 0},
    {"RESO", 0, 100, 20, UNI, 0, 21, 0},
    {"DCAY", 0, 100, 50, UNI, 0, 2, 0, {}, {"SUST", "CHOKE"}},  // Q 4 .. 1000: from the middle up, the bands ring as pitches
    // modes: variation per hit (PITCH: the notes it rings move by up to
    // +- n semitones, in the scale; TONE: its brightness by up to +- 5 n)
    {"PITCH", -24, 24, 0, BI, 0, 25, 0},
    {"TONE", -50, 50, 0, BI, 0, 11, 0},
    {"SCALE", 0, 3, 0, LIST, 4, 0, 0, {"CHROM", "MAJOR", "MINOR", "PENTA"}},
    {"WET", 0, 100, 100, UNI, 0, 21, 0},
  }
};

class ResonatorDevice : public Device {
public:
  enum { CUT, SLOPE, RESO, DECAY, PITCH, TONE, SCALE, WET };
  bool Varies(int p) const override { return p == CUT || p == SLOPE || p == RESO || p == WET; }
  // the variations' bars: +- steps, out from the middle
  int BarStyle(int p) const override { return p == PITCH || p == TONE ? BAR_CENTRE : Device::BarStyle(p); }
  static constexpr int BANDS = AudioEffectResonator::BANDS;
  ResonatorDevice() : Device(RESONATOR_DEF) {}
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  bool Activate() override { fx.Reset(); active = true; return true; }
  void Action(int a) override { if (a == 0) fx.Strike(); else fx.NewVar(); }
  void Apply(int p) override { Rebuild(); }
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
private:
  AudioEffectResonator fx;
  float CutBand() const { return cur[CUT] * 0.01f * (BANDS - 1); }
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
  float f[BANDS];
  for (int k = 0; k < BANDS; k++) {
    int note = 24 + 2 * k + cur[PITCH];  // MIDI note, C1 = 24
    for (int d = 0; d < 6; d++) {        // nearest note in the scale
      if (mask & (1 << (((note - d) % 12 + 12) % 12))) { note -= d; break; }
      if (mask & (1 << (((note + d) % 12 + 12) % 12))) { note += d; break; }
    }
    const float hz = 440.0f * powf(2.0f, (note - 69) / 12.0f);
    f[k] = hz < 8000.0f ? 2.0f * sinf(float(M_PI) * hz / AUDIO_SAMPLE_RATE_EXACT) : 0.0f;
  }
  // the shape (gains) is worked out in the engine, as every hit may move it
  const AudioEffectResonator::Shape sh = {CutBand(), cur[SLOPE] * 0.02f, float(cur[RESO]), float(cur[TONE]), level};
  fx.SetBank(f, sh, mode[PITCH] * 0.5f, mode[TONE] * 5.0f, 1.0f / q, mode[DECAY] == 1, cur[WET] * 0.01f);
}

FLASHMEM void ResonatorDevice::FormatMode(int p, char* buf, size_t n) const {
  if ((p == PITCH || p == TONE) && mode[p] == 0) snprintf(buf, n, "OFF");
  else if (p == PITCH) snprintf(buf, n, LOOM_PM "%d", mode[p]);
  else if (p == TONE) snprintf(buf, n, LOOM_PM "%d", mode[p] * 5);
  else Device::FormatMode(p, buf, n);
}

// gain of a band (fractional k for the picture) from CUT / SLOPE (12 dB per
// octave sides, 6 bands an octave), before RESO and TONE
FLASHMEM float ResonatorDevice::Base(float k) const {
  return AudioEffectResonator::Base(k, CutBand(), cur[SLOPE] * 0.02f);
}

// the RESO bump around the cutoff, 0..1
FLASHMEM float ResonatorDevice::Bump(float k) const {
  return AudioEffectResonator::Bump(k, CutBand());
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
  // PITCH's variation on: a small arrow at the top over the band the last
  // hit went to
  if (mode[PITCH]) {
    const int hb = constrain(int(roundf(CutBand()) + fx.HitCut()), 0, BANDS - 1);
    const int ax = x + hb * 2;
    gfxLine(ax - 2, y, ax + 2, y);
    gfxLine(ax - 1, y + 1, ax + 1, y + 1);
    gfxPixel(ax, y + 2);
  }
  // output level on the right
  const int mx = x + w - 14;
  gfxFrame(mx, y, 12, h);
  DrawVMeter(mx + 1, y + 1, 10, h - 2, out_m.v);
  // TONE between the bands and PITCH: a box filled to its setting, empty
  // at the darkest, half (checkered) in the middle, solid at the brightest
  static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  const int tx = x + BANDS * 2 + 2, tw = mx - 7 - tx + 1;
  gfxFrame(tx, y, tw, h);
  const int fill = constrain(int((cur[TONE] + fx.HitTone() + 50) * 16 / 100), 0, 16);  // 0..16, with this hit's variation
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
  "FIBRE", "SPR", 8, {"FRZ", "GRN"},  // (was SPRAY)  // B: one grain by hand (RATE 0: only these)
  {
    // left of the middle: P (periodic), right: R (random), middle: no grains;
    // mode: FREE, or CLK: P on the clock's note values (see ClockDiv)
    {"RATE", -100, 100, 55, BI, 0, 2, 0, {}, {"FREE", "CLOCK"}},
    {"SIZE", 0, 100, 50, UNI, 0, 21, 0},   // mode: size spread, 0..100% in 5% steps
    {"PITCH", -24, 24, 0, BI, 0, 25, 0},  // mode: pitch spread, 0..24 semitones
    // 0 = the oldest audio (left on the picture), 100 = now (right); mode:
    // position variation, 0..100% in 5% steps
    {"POS", 0, 100, 50, UNI, 0, 21, 4},
    {"SHAPE", 0, 100, 70, UNI, 0, 21, 0},
    {"FDBK", 0, 100, 0, UNI, 0, 21, 0},
    {"SPRD", 0, 100, 60, UNI, 0, 21, 0},  // stereo spread: each grain's random place left / right
    {"MIX", 0, 100, 50, UNI, 0, 21, 0},  // source <> grains
  },
  {"", ""},
};

class SprayDevice : public Device {
public:
  enum { RATE, SIZE, PITCH, POS, SHAPE, FDBK, SPREAD, LEVEL };
  bool Varies(int p) const override { return p == SHAPE || p == FDBK || p == SPREAD || p == LEVEL; }
  // the variations' bars: all +-, out from the middle
  int BarStyle(int p) const override { return p == PITCH || p == SIZE || p == POS ? BAR_CENTRE : Device::BarStyle(p); }
  static constexpr float BUF_SECS = 4.0f;
  SprayDevice();
  // each grain takes PITCH, SIZE and POS once as it starts: the
  // modulator's step, never half a glide (a grain fired with the step
  // would get the old note)
  bool Sampled(int p) const override { return p == PITCH || p == SIZE || p == POS; }
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
  // P on the clock (RATE's mode CLK): one of the clock's note values (the
  // longest next to the middle)
  bool OnClock() const { return mode[RATE] != SYNC_FREE && cur[RATE] < 0; }
  static int DivOf(int v) { return DivIndex(abs(v), 1, 100); }
  static int DivVal(int i) { return -DivValue(i, 1, 100); }
  float DivBeats() const { return loom::DivBeats(DivOf(cur[RATE])); }
  int Step(int p, int v, int dir) const override;
  void Clocked() override;
  void TempoChanged() override { Apply(RATE); }
  bool clock_was = false;
  int64_t grid_at = -1;  // the last division a grain was fired on
  float grid_div = 0.0f; // ... of which note value (in beats)
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
    case RATE:  // neg = periodic; on the clock while it runs the grains are fired in Idle
      if (OnClock()) fx.setDensity(clock_running ? 0.0f : -tempo_bpm / (60.0f * DivBeats()));
      else fx.setDensity(cur[RATE] < 0 ? -RateHz(cur[RATE]) : RateHz(cur[RATE]));
      break;
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
  else if (p == PITCH) snprintf(buf, n, LOOM_PM "%d", mode[p]);
  else if (p == SIZE || p == POS) FormatVar(buf, n, mode[p] * 5);
  else Device::FormatMode(p, buf, n);
}

// RATE on the clock: P steps through the divisions (one turn each), on to
// OFF and the R side as before
FLASHMEM int SprayDevice::Step(int p, int v, int dir) const {
  if (p != RATE || mode[RATE] == 0 || v + dir > 0 || (v > 0)) return v + dir;
  if (v == 0) return DivVal(0);  // OFF -> the slowest division
  const int idx = DivOf(v) - dir;  // turning left (dir < 0): faster
  if (idx < 0) return 0;
  return DivVal(idx);
}

// RATE on the clock and the clock running: a grain on each division, on
// the same tick a modulator on the same division steps (after it, so a
// grain gets the new step)
FLASHMEM void SprayDevice::Clocked() {
  if (clock_was != clock_running) { clock_was = clock_running; grid_at = -1; Apply(RATE); }
  if (!OnClock() || !clock_running || !active) { grid_at = -1; return; }
  const float db = DivBeats();
  const int64_t g = ClockGrid(db);
  // another note value: counted on its grid from here, the first grain on
  // its next tick (not at once)
  if (db != grid_div) { grid_div = db; if (grid_at >= 0) grid_at = g; }
  if (g != grid_at) {
    // (switched on half way into a division: wait for the next)
    if (grid_at >= 0 || (clock_beat_n == 0 && clock_beat_ph < 0.01f)) fx.triggerGrain();
    grid_at = g;
  }
}

FLASHMEM void SprayDevice::Format(int p, char* buf, size_t n) const {
  if (p == RATE && OnClock()) {
    FormatDiv(DivOf(val[RATE]), buf, n);
  } else if (p == RATE) {
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
  } else {
    fx.triggerGrain();
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
constexpr int MAX_OPTIONS = 4;  // (settings files before version 12 kept 3)

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

FASTRUN __attribute__((optimize("O2"))) void AudioRings::update() {  // (O2: the build is -Os)
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
      o[0]->data[n] = ToSample16(l);
      o[1]->data[n] = ToSample16(r);
    }
    peak = std::max(float(peak), pk);
    transmit(o[0], 0);
    transmit(o[1], 1);
  }
  for (auto b : o) if (b) release(b);
  if (in) release(in);
}

PROGMEM static const DeviceDef MODAL_DEF = {
  "BEATER", "MDL", 8, {"HIT", "DAMP"},  // B: a choke, the ringing stopped  // (was MODAL)
  {
    {"NOTE", 0, 72, 36, UNI},      // C0 .. C6, C3 in the middle
    {"STRUC", 0, 100, 70, UNI, 0, 21, 0},    // the module's STRUCTURE (SYMQ: the chord)
    {"BRITE", 0, 100, 60, UNI, 0, 21, 0},
    {"DAMP", 0, 100, 65, UNI, 0, 21, 0},
    {"POS", 0, 100, 25, UNI, 0, 21, 0},      // where it is struck
    {"SPRD", 0, 100, 70, UNI, 0, 21, 0},     // odd / even apart
    {"MODEL", 0, RingsVoice::MODELS - 1, 0, UNI},
    {"WET", 0, 100, 100, UNI, 0, 21, 0},
  },
  {"", ""},
};

class ModalDevice : public Device {
public:
  enum { NOTE, STRUC, BRITE, DAMP, POS, SPRD, MODEL, WET };
  bool Varies(int p) const override { return p != NOTE && p != MODEL; }
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
    fx.voice.Free();  // (lets the audio go first itself)
    active = false;
  }
  void Apply(int p) override;
  void Action(int a) override {
    if (a == 0) { fx.voice.Strike(); hit_at = millis() | 1; }
    else fx.voice.Damp();
  }
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
// long = smooth). DIFF: Clouds' diffuser (its DENSITY in this mode): it
// smears the windows' seams into a blur, as if behind glass or in the rain.
// TONE: Clouds' filter (left lowpass, right highpass). Every setting has a
// variation (L knob), drawn anew for each window.
/////////////////////////////////////////////////////////////////
class AudioStretch : public AudioStream {
public:
  AudioStretch() : AudioStream(2, queue) {}
  StretchVoice voice;
  // TONE -50..50, DIFF and MIX 0..1, each with its variation (+-)
  volatile float tone = 0.0f, tone_var = 0.0f, diff = 0.0f, diff_var = 0.0f;
  volatile float wet = 1.0f, mix_var = 0.0f;
  // as heard in this window (for the picture)
  volatile float tone_now = 0.0f, diff_now = 0.0f;
  volatile float peak = 0.0f;
  volatile bool rest = false;  // parked: nothing to compute
  volatile bool frz = false;   // frozen: the buffer (and its picture) stands still
  static constexpr float BUF_SECS = 4.0f;
  // level of what goes into the buffer, one column per 1/HIST of it (as on
  // FIBRE), for the picture; hist_pos is the next (oldest) column
  static constexpr int HIST = 120;
  uint8_t hist[HIST] = {};
  volatile int hist_pos = 0;
  // where the record head is, in columns (with the part of the next one
  // written so far, so the picture moves smoothly)
  float HeadPos() const { return head_f; }
  void update() override;
private:
  void Level(const float* x);
  void Diffuse(float* x, float to);
  float hist_peak = 0.0f, hist_blocks = 0.0f;
  volatile float head_f = 0.0f;  // hist_pos + hist_blocks, set in one go
  // Clouds' diffuser: four allpasses a side, its lengths (at 32 kHz) x 1.5
  static constexpr int AP_MEM = 1529 + 1511;
  float ap_mem[AP_MEM] = {};
  int ap_at[2][4] = {};
  float diff_a = 0.0f;  // the amount as heard (ramped per block)
  float mix_g = 1.0f;   // MIX's variation, as a gain on the wet part
  // TONE, DIFF and MIX's variations glide to each window's new value
  // (~30 ms) instead of jumping: jumps in the blend of blurred and clear
  // (they are out of step) or in the filter clicked at every window
  float tone_s = 0.0f, diff_s = 0.0f, mix_s = 1.0f;
  // TONE's make-up gain: the level before and after the filter (smoothed
  // power) and the gain that brings it back, up to +12 dB
  float pw_in = 0.0f, pw_out = 0.0f, tone_g = 1.0f;
  audio_block_t* queue[2];
  float lp_z[2] = {}, hp_z[2] = {};
  bool resting = false;
};

inline void AudioStretch::Level(const float* x) {
  if (frz) return;
  float pk = hist_peak;
  for (int i = 0; i < 2 * AUDIO_BLOCK_SAMPLES; i++) pk = std::max(pk, fabsf(x[i]));
  hist_peak = pk;
  hist_blocks += AUDIO_BLOCK_SAMPLES * HIST / (BUF_SECS * AUDIO_SAMPLE_RATE_EXACT);
  if (hist_blocks >= 1.0f) {
    hist_blocks -= 1.0f;
    hist[hist_pos] = uint8_t(std::min(1.0f, pk) * 255.0f);
    hist_pos = (hist_pos + 1) % HIST;
    hist_peak = 0.0f;
  }
  head_f = hist_pos + hist_blocks;
}

// the windows (interleaved L R) through the diffuser, crossfaded by its
// amount; nothing to do while it is (and stays) at 0
FASTRUN __attribute__((optimize("O2"))) void AudioStretch::Diffuse(float* x, float to) {
  static const int16_t LEN[2][4] = {{189, 270, 404, 666}, {227, 308, 368, 608}};
  const float from = diff_a;
  diff_a = to;
  if (to <= 0.0f && from <= 0.0f) return;
  constexpr float kap = 0.625f;
  const float step = (to - from) / AUDIO_BLOCK_SAMPLES;
  float* mem = ap_mem;
  for (int c = 0; c < 2; c++) {
    float* b[4];
    int at[4];
    for (int k = 0; k < 4; k++) { b[k] = mem; mem += LEN[c][k]; at[k] = ap_at[c][k]; }
    float a = from;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      const float in = x[2 * n + c];
      float v = in;
      for (int k = 0; k < 4; k++) {
        const float d = b[k][at[k]];
        const float w = v + kap * d;
        b[k][at[k]] = w;
        if (++at[k] == LEN[c][k]) at[k] = 0;
        v = d - kap * w;
      }
      a += step;
      x[2 * n + c] = in + a * (v - in);
    }
    for (int k = 0; k < 4; k++) ap_at[c][k] = at[k];
  }
}

FASTRUN __attribute__((optimize("O2"))) void AudioStretch::update() {  // (O2: the build is -Os)
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  if (!voice.ok()) { for (auto b : in) if (b) release(b); return; }
  static float fi[2 * AUDIO_BLOCK_SAMPLES], fo[2 * AUDIO_BLOCK_SAMPLES];
  if (rest) {  // out of the chain: keep recording only
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++)
      for (int c = 0; c < 2; c++) fi[2 * n + c] = in[c] ? in[c]->data[n] * (1.0f / 32768.0f) : 0.0f;
    voice.Record(fi, AUDIO_BLOCK_SAMPLES);
    Level(fi);
    // the picture shows the settings as they are (not the last ones heard)
    tone_now = tone_s = tone;
    diff_now = diff_s = diff;
    resting = true;
    for (auto b : in) if (b) release(b);
    return;
  }
  if (resting) {  // back: fresh windows, no old filter state
    resting = false;
    voice.Restart();
    memset(ap_mem, 0, sizeof(ap_mem));
    for (int c = 0; c < 2; c++) lp_z[c] = hp_z[c] = 0.0f;
  }
  audio_block_t* o[2] = {allocate(), allocate()};
  if (o[0] && o[1]) {
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++)
      for (int c = 0; c < 2; c++) {
        const float x = in[c] ? in[c]->data[n] * (1.0f / 32768.0f) : 0.0f;
        fi[2 * n + c] = x / (1.0f + fabsf(x) * 0.25f);  // softly limited
      }
    voice.Process(fi, fo, AUDIO_BLOCK_SAMPLES);
    Level(fi);
    // this window's TONE, DIFF and MIX: the settings plus their variations,
    // glided to
    constexpr float glide = 0.085f;  // per block: ~30 ms
    tone_s += glide * (constrain(tone + voice.DRand(0) * tone_var, -50.0f, 50.0f) - tone_s);
    diff_s += glide * (constrain(diff + voice.DRand(1) * diff_var, 0.0f, 1.0f) - diff_s);
    const float t = tone_s * 0.02f;
    tone_now = tone_s;
    const float dn = diff_s;
    diff_now = dn;
    Diffuse(fo, dn);
    // TONE, Clouds' texture filter: left closes a lowpass, right opens a highpass
    const float lp_hz = t < 0.0f ? 18000.0f * powf(150.0f / 18000.0f, -t) : 18000.0f;
    // (right: up to 1.5 kHz, gently at first; 4 kHz left only the hiss)
    const float hp_hz = t > 0.0f ? 20.0f * powf(1500.0f / 20.0f, t * t) : 20.0f;
    const float la = 1.0f - expf(-2.0f * float(M_PI) * lp_hz / AUDIO_SAMPLE_RATE_EXACT);
    const float ha = 1.0f - expf(-2.0f * float(M_PI) * hp_hz / AUDIO_SAMPLE_RATE_EXACT);
    // MIX's variation: the slot mixes by MIX, so the wet part here is
    // scaled to what this window's MIX would give (ramped over the block)
    const float w = wet;
    mix_s += glide * ((mix_var > 0.0f ? constrain(w + voice.DRand(2) * mix_var, 0.0f, 1.0f) : w) - mix_s);
    const float g_to = w > 0.01f ? mix_s / w : 1.0f;
    const float g_from = mix_g, g_step = (g_to - g_from) / AUDIO_BLOCK_SAMPLES;
    mix_g = g_to;
    float pk = 0.0f, e_in = 0.0f, e_out = 0.0f;
    const float tg_from = tone_g;
    float tg_to = 1.0f;
    if (fabsf(tone_s) > 0.5f && pw_out > 1e-9f) tg_to = std::min(4.0f, sqrtf(pw_in / pw_out));
    tg_to = std::max(1.0f, tg_to);
    tone_g = tg_to;
    const float tg_step = (tg_to - tg_from) / AUDIO_BLOCK_SAMPLES;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      const float g = g_from + g_step * (n + 1);
      const float tg = tg_from + tg_step * (n + 1);
      for (int c = 0; c < 2; c++) {
        float y = fo[2 * n + c];
        e_in += y * y;
        lp_z[c] += la * (y - lp_z[c]);  // TONE: lowpass ...
        y = lp_z[c];
        hp_z[c] += ha * (y - hp_z[c]);  // ... or highpass
        y -= hp_z[c];
        e_out += y * y;
        y *= tg;                        // ... and back up to the level before
        if (g != 1.0f) y = fi[2 * n + c] + g * (y - fi[2 * n + c]);
        pk = std::max(pk, fabsf(y));
        o[c]->data[n] = ToSample16(y);
      }
    }
    constexpr float lvl = 0.03f;  // per block: ~0.1 s
    pw_in += lvl * (e_in - pw_in);
    pw_out += lvl * (e_out - pw_out);
    peak = std::max(float(peak), pk);
    transmit(o[0], 0);
    transmit(o[1], 1);
  }
  for (auto b : o) if (b) release(b);
  for (auto b : in) if (b) release(b);
}

PROGMEM static const DeviceDef STRETCH_DEF = {
  "DRAFT", "STR", 8, {"FRZ", "GRAB"},  // B: holds a fresh piece, the sound coming in now  // (was STRETCH)
  {
    // page 1 in FIBRE's places (SPEED where FIBRE has RATE); every mode is
    // a random variation per window, 0..100 % (PITCH: 0..24 semitones)
    {"SPEED", -200, 200, 0, BI, 0, 21, 0},  // frozen: through the buffer (%, < 0 backwards); 0 holds it at POS, as Clouds does (and a double click goes there)
    {"SIZE", 0, 100, 50, UNI, 0, 21, 0},  // the window: short = quick and grainy, long = smooth
    {"PITCH", -24, 24, 0, BI, 0, 25, 0},  // semitones, without changing the speed
    {"POS", 0, 100, 50, UNI, 0, 21, 0},   // like FIBRE: 0 = the oldest audio (left), 100 = now (right)
    {"DIFF", 0, 100, 0, UNI, 0, 21, 0},   // the diffuser: clear .. blurred
    {"TONE", -50, 50, 0, BI, 0, 21, 0},   // left lowpass, right highpass
    {"SPRD", 0, 100, 0, UNI, 0, 21, 0},   // stereo spread: the windows wander left / right
    {"MIX", 0, 100, 100, UNI, 0, 21, 0},
  },
  {"", ""},
};

class StretchDevice : public Device {
public:
  enum { SPEED, SIZE, PITCH, POS, DIFF, TONE, SPRD, MIX };
  // the variations' bars: all +-, out from the middle
  int BarStyle(int p) const override { return BAR_CENTRE; }
  static constexpr float BUF_SECS = AudioStretch::BUF_SECS;
  void FormatMode(int p, char* buf, size_t n) const override;
  StretchDevice() : Device(STRETCH_DEF) {}
  void Rest(bool on) override { fx.rest = on; }
  bool FeedsWhileParked() const override { return true; }
  bool Wraps(int p) const override { return p == POS; }
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
    fx.voice.Free();  // (lets the audio go first itself)
    active = false;
  }
  // the correlator's search, from the main loop: each time a whole search
  // (~1 ms; a quarter at a time, as Clouds did, fell behind while the chain
  // view drew, and sounded different). From the audio it hung the module.
  void Idle() override { if (active && !fx.rest) fx.voice.Work(2000); }
  void Apply(int p) override;
  // FRZ: POS changes meaning, its number turned to keep the window where it
  // is (no jump): recording, how far back from the record head; frozen, the
  // place in the held sound (on the picture, 0 left .. 100 right)
  void Action(int a) override {
    if (a == 1) {  // GRAB: let go (if held) and hold again, on what is there now
      if (frozen) Action(0);
      Action(0);
      return;
    }
    const float h = fx.HeadPos() / AudioStretch::HIST;
    if (!frozen) {  // the place on the picture where it plays now
      float f = h - fx.voice.BackOf(1.0f - val[POS] * 0.01f);
      f -= floorf(f);
      val[POS] = int16_t(lroundf(f * 100.0f)) % 101;
    } else {        // how far back that is, in the engine's positions
      float b = h - val[POS] * 0.01f;
      b -= floorf(b);
      val[POS] = int16_t(constrain(lroundf((1.0f - fx.voice.PositionOf(b)) * 100.0f), 0L, 100L));
    }
    frozen = !frozen;
    Refresh(POS);
    Apply(PITCH);
  }
  // the engine's position (0..1, how far back from the record head)
  float Back() const {
    if (!frozen) return 1.0f - cur[POS] * 0.01f;
    float b = fx.HeadPos() / AudioStretch::HIST - cur[POS] * 0.01f;
    b -= floorf(b);
    return constrain(fx.voice.PositionOf(b), 0.0f, 0.999f);
  }
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
  // the window's variations as the picture shows them: every ~0.15 s it
  // takes the playing window's (their whole reach) and glides there, so the
  // box sways instead of flickering with each window (every few ms);
  // following each one smoothly averaged them to a third of their reach
  float v_pos = 0.0f, v_size = 0.0f, v_pitch = 0.0f, v_tone = 0.0f, v_diff = 0.0f;
  float t_pos = 0.0f, t_size = 0.0f, t_pitch = 0.0f, t_tone = 0.0f, t_diff = 0.0f;
  uint32_t v_at = 0, t_at = 0;  // last moved, last taken (us)
  AudioStretch fx;
  Meter out_m;
  bool frozen = false;
};

FLASHMEM void StretchDevice::Apply(int p) {
  switch (p) {
    case MIX:
      wet = cur[MIX] * 0.01f;
      fx.wet = wet;
      fx.mix_var = mode[MIX] * 0.05f;
      return;
    case DIFF:
      fx.diff = cur[DIFF] * 0.01f;
      fx.diff_var = mode[DIFF] * 0.05f;
      return;
    case TONE:
      fx.tone = cur[TONE];
      fx.tone_var = mode[TONE] * 2.5f;  // up to +-50
      return;
    default:
      fx.frz = frozen;
      fx.voice.Set(Back(), cur[SIZE] * 0.01f, cur[PITCH], cur[SPEED] * 0.01f, frozen);
      fx.voice.SetSpread(mode[PITCH], mode[SIZE] * 0.05f, mode[POS] * 0.025f, cur[SPRD] * 0.01f,
                         mode[SPRD] * 0.05f, mode[SPEED] * 0.05f);
  }
}

// the variations: OFF, then +-semitones (PITCH) or percent
FLASHMEM void StretchDevice::FormatMode(int p, char* buf, size_t n) const {
  if (mode[p] == 0) snprintf(buf, n, "OFF");
  else if (p == PITCH) snprintf(buf, n, LOOM_PM "%d", mode[p]);
  else FormatVar(buf, n, mode[p] * 5);
}

FLASHMEM void StretchDevice::Format(int p, char* buf, size_t n) const {
  if (p == SPEED) snprintf(buf, n, "%d%%", val[SPEED]);
  else Device::Format(p, buf, n);
}

// the buffer as it sits in memory, as on FIBRE: the recording stays in
// place and the record head (a line) runs across it, writing over the
// oldest audio, like a tape loop; it stops when frozen. The recording's wave
// is thinner as MIX brings the window in, gone at 100. The window playing
// now is a box POS behind the record head, riding along with it (frozen,
// both stand still), as wide as SIZE, raised or lowered by PITCH, taller
// with TONE (left it grows down, right up) and filled like FIBRE's grains,
// thinning out with DIFF: solid when clear, a scatter of dots when blurred
// (TONE and DIFF as heard, with their variations). Frozen, a dotted line
// shows where SPEED has taken the playback in the held sound. Both pages
// show the same.
FLASHMEM void StretchDevice::DrawViz(int x, int y, int w, int h) {
  static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  const float pk = fx.peak;
  fx.peak = 0.0f;
  out_m.Hit(pk);
  // (by the time passed: the picture is drawn every main loop, a few ms)
  const uint32_t now = micros();
  if (now - t_at >= 150000) {
    t_at = now;
    t_pos = fx.voice.DPos();
    t_size = fx.voice.DSize();
    t_pitch = fx.voice.DPitch();
    t_tone = fx.tone_now;
    t_diff = fx.diff_now;
  }
  const float k = 1.0f - expf(-float(now - v_at) * (1.0f / 50000.0f));
  v_at = now;
  v_pos += k * (t_pos - v_pos);
  v_size += k * (t_size - v_size);
  v_pitch += k * (t_pitch - v_pitch);
  v_tone += k * (t_tone - v_tone);
  v_diff += k * (t_diff - v_diff);
  const int ww = w - 8, cy = y + h / 2, base = y + h - 1;
  constexpr int HIST = AudioStretch::HIST;
  const float head = fx.HeadPos();
  // a place p back from the record head (0 = now, 1 = the oldest), round
  // the loop; recording, the window stops at the ends, as the engine does
  // (an engine position, where it really plays: see BackOf)
  auto pos_x = [&](float p) {
    p = frozen ? p - floorf(p) : constrain(p, 0.0f, 0.98f);
    float f = head / HIST - fx.voice.BackOfWith(p, v_size, v_pitch);  // (smoothed: no jitter)
    f -= floorf(f);
    return x + int(f * ww) % ww;
  };
  // a column of the picture, round the loop (the box and brackets wrap)
  auto wrap = [&](int px) { return x + ((px - x) % ww + ww) % ww; };
  gfxDottedLine(x, cy, x + ww - 1, cy, 2);
  const int wave_fill = 16 - cur[MIX] * 16 / 100;  // 16 .. 0 of 16
  if (wave_fill > 0)
    for (int px = x; px < x + ww; px++) {
      const int a = int(sqrtf(fx.hist[(px - x) * HIST / ww] / 255.0f) * (h / 2 - 1));
      if (!a) continue;
      for (int yy = cy - a; yy <= cy + a; yy++)
        if (bayer[yy & 3][px & 3] < wave_fill) gfxPixel(px, yy);
    }
  const int hx = x + int(head * ww / HIST) % ww;
  if (!frozen) {  // the record head, with a gap either side (none frozen)
    const int cl = std::max(x, hx - 1), cr = std::min(x + ww - 1, hx + 1);
    graphics.clearRect(cl, y, cr - cl + 1, h);
    gfxLine(hx, y + 2, hx, y + h - 3);
  }
  const float pos = Back();  // back from now
  const float semis_px = float(h / 2 - 4) / 24.0f;
  // the variations' reach, like SPRAY's: POS as brackets either side, PITCH
  // as a dotted line up and down through the middle
  const float pv = mode[POS] * 0.025f;
  if (pv > 0.0f) {
    const int wl = pos_x(pos + pv), wr = pos_x(pos - pv);
    for (int k = 0, n = (wr - wl + ww) % ww; k <= n; k++) {
      gfxPixel(wrap(wl + k), y);
      gfxPixel(wrap(wl + k), base);
    }
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
  // the window playing now
  const int px = pos_x(pos + v_pos);
  const float sz = constrain(cur[SIZE] * 0.01f + v_size, 0.0f, 1.0f);
  const int bw = 4 + int(sz * (ww / 4));
  const int tn = int(v_tone);
  const int grow = abs(tn) * 10 / 50, bh = 7 + grow;
  const int py = cy - int((cur[PITCH] + v_pitch) * semis_px);
  const int bx = px - bw / 2;  // (may run over an edge: drawn round the loop)
  const int by = constrain(py - 3 - (tn > 0 ? grow : 0), y, base - bh + 1);
  const int fill = 16 - int(v_diff * 14.0f + 0.5f);  // 2 of 16 left at 100
  for (int i = -1; i <= bw; i++) {
    const int xx = wrap(bx + i);
    const int y0 = std::max(y, by - 1), y1 = std::min(base, by + bh);
    graphics.clearRect(xx, y0, 1, y1 - y0 + 1);  // a gap around it
    if (i < 0 || i == bw) continue;
    // SPRD: a band from the middle out, as wide as the spread (the whole
    // box at 100), where DIFF's dots are drawn the other way round
    const bool band = 2 * abs(2 * i - (bw - 1)) < cur[SPRD] * (bw - 1) / 50;
    for (int yy = by; yy < by + bh; yy++) {
      const bool edge = i == 0 || i == bw - 1 || yy == by || yy == by + bh - 1;
      if (edge || ((bayer[yy & 3][xx & 3] < fill) != band)) gfxPixel(xx, yy);
    }
  }
  // frozen: where SPEED has taken the playback
  if (frozen && cur[SPEED] != 0) {
    float f = pos + fx.voice.Scrub();
    gfxDottedLine(pos_x(f), y, pos_x(f), base, 2);
  }
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
  // as on the page: the window POS behind the record head, riding along
  // with it round the loop; frozen, both stand still
  float f = fx.HeadPos() / AudioStretch::HIST - fx.voice.BackOf(frozen ? Back() : std::min(0.98f, Back()));
  f -= floorf(f);
  const int px = x + int(f * w) % w;
  const int py = cy - cur[PITCH] * (h / 2 - 2) / 24;
  gfxFrame(constrain(px - 2, x, x + w - 5), py - 2, 5, 5);
}

/////////////////////////////////////////////////////////////////
// CHIME (RESONATOR slot): six tuned bells on notes of a scale, each two
// band-passes in a row, so only the notes ring, clean. DIR: how many of
// them ring together (SPRD's mode: which); DIR's mode: each hit moves the
// whole pattern a place round the six (UP, DOWN, RAND; OFF: it stays);
// the bells left ring out at their own pitch.
// ROTA turns all six along 20 places of the scale, C1 .. C8, round
// and round (always in key), MORPH (its mode) how slowly. Page 1 what they
// play, page 2 the sound.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef CHIME_DEF = {
  "TWILL", "CHM", 8, {"HIT", "ROTA"},  // (was CHIME) A: struck, they ring without any input; B: ROTA a place on (ARP's direction)
  {
    // how many bells ring at once (1 .. 6); mode: what a hit (or STEP)
    // does to which (OFF: nothing, they stay)
    {"DIR", 1, 6, 6, UNI, 0, 4, 1, {}, {"OFF", "UP", "DOWN", "RAND"}},
    // places between the six bells (of the 20); mode: how ARP's count of
    // them is spread over the six (as sliders up and down): 1 2 3, 1 3 5 ...
    {"SPRD", 1, 7, 2, UNI, 0, 20, 0},
    // all six turned along the scale, a step a click, round and round (past
    // the top: in again at the bottom); mode: MORPH, how slowly the sound
    // moves to new notes (ROTA, SPRD, SCALE, NOTE)
    {"ROTA", 0, 19, 0, UNI, 0, 13, 0},
    // on each hit, bells move up or down by up to this many notes of the
    // set's family (its notes and the ones that colour it: a chord's 9th,
    // 11th, 13th; a scale, a microtonal set: its own), at random; mode: how
    // many of the six (1 .. 6)
    {"VARI", 0, 15, 0, UNI, 0, 6, 5},
    // the root (key), semitones from C3; mode: KNOB, MIDI (any channel),
    // CH1 .. CH16: the note played is the root, this added to it (the whole
    // set moves with it, as the SMR's pitch input)
    {"KEY", -24, 24, 0, BI, 0, 18, 0},
    // the notes the bells are on (see Apply): chords and scales on the 12
    // notes, then 8 microtonal; mode: how the 12-note ones are tuned (the
    // microtonal ones bring their own)
    {"SCALE", 0, 29, 0, UNI, 0, 4, 0, {}, {"EQUAL", "JUST", "PYTH", "MEANT"}},
    {"DCAY", 0, 100, 66, UNI, 0, 11, 0},  // 20 ms .. 12 s to fade away (low: the wide pass blended in, a filter); mode: variation per hit
    {"WET", 0, 100, 100, UNI, 0, 21, 0},
  },
  {"", "SOUND"},
};

class ChimeDevice : public Device {
public:
  enum { ARP, SPRD, ROTA, VARI, NOTE, SCALE, DECAY, WET };  // (NOTE: KEY)
  // SPRD's name, once its mode was turned: the bells it rings (1 3 5, 1256)
  const char* Name(int p) const override {
    if (p == SPRD && ModeNamed(SPRD)) { BellsText(spread_name, sizeof(spread_name)); return spread_name; }
    return Device::Name(p);
  }
  bool NameIsMode(int p) const override { return p == SPRD; }
  void BellsText(char* buf, size_t n) const {
    const uint8_t b = Bells(constrain(int(cur[ARP]), 1, 6), Way());
    if (b == 0x3F) { snprintf(buf, n, "ALL"); return; }
    const bool gaps = __builtin_popcount(b) <= 3;  // (four, five: no gaps, so they fit)
    char t[12]; int k = 0;
    for (int i = 0; i < 6; i++) if (b >> i & 1) { if (k && gaps) t[k++] = ' '; t[k++] = char('1' + i); }
    t[k] = 0;
    snprintf(buf, n, "%s", t);
  }
  mutable char spread_name[8] = {};
  // ARP's count of bells spread over the six: the ways to pick that many, in
  // order (1 2, 1 3 .. 5 6), the first side by side
  static int Ways(int n) { static const uint8_t w[7] = {1, 6, 15, 20, 15, 6, 1}; return w[constrain(n, 0, 6)]; }
  static uint8_t Bells(int n, int way) {
    way = constrain(way, 0, Ways(n) - 1);
    for (int k = 0, m = 0; m < 64; m++) {  // (masks in the order of their lists of bells)
      const int b = Lex(m);
      if (__builtin_popcount(b) == n && k++ == way) return uint8_t(b);
    }
    return 0x3F;
  }
  static int Lex(int m) {  // the m-th set of bells, sets read as their bells low to high, in that order
    static uint8_t order[64];
    static bool made = false;
    if (!made) {
      for (int i = 0; i < 64; i++) order[i] = uint8_t(i);
      auto key = [](int b) { uint32_t k = 0; for (int i = 0; i < 6; i++) if (b >> i & 1) k = k * 8 + uint32_t(i + 1); int c = __builtin_popcount(b); for (; c < 6; c++) k *= 8; return k; };
      std::sort(order, order + 64, [&](uint8_t a, uint8_t b) { return key(a) < key(b); });
      made = true;
    }
    return order[m];
  }
  int Way() const { return constrain(int(mode[SPRD]), 0, Ways(int(cur[ARP])) - 1); }
  void NudgeMode(int p, int dir) override {
    Device::NudgeMode(p, dir);
    if (p == SPRD && mode[SPRD] >= Ways(int(cur[ARP]))) { mode[SPRD] = uint8_t(Ways(int(cur[ARP])) - 1); Apply(SPRD); }
  }
  int Step(int p, int v, int dir) const override {
    return p == ROTA ? ((v + dir) % rota_w + rota_w) % rota_w : Device::Step(p, v, dir);
  }
  bool Varies(int p) const override { return p == WET; }
  // the variations' bars (NOTE, SPRD's MORPH, ARP's moves are no variations)
  int BarStyle(int p) const override {
    return p == DECAY ? BAR_CENTRE : p == VARI ? BAR_LEFT : Device::BarStyle(p);  // (+-: out from the middle)
  }
  static constexpr int SCALES = 30, SCALES12 = 22;  // the first 22 on the 12 notes (tuned: SCALE's mode)
  ChimeDevice() : Device(CHIME_DEF) {}
  AudioStream* in() override { return &fx; }
  AudioStream* out() override { return &fx; }
  bool Activate() override { fx.Reset(); active = true; ApplyAll(); return true; }
  void Apply(int p) override;
  void Action(int a) override {
    if (a == 0) fx.Strike();
    else {  // STEP: ROTA a place on (ARP DOWN: back; RAND: anywhere), round and round
      const int m = mode[ARP];
      val[ROTA] = int16_t(m == 3 ? random(rota_w) : ((val[ROTA] + (m == 2 ? -1 : 1)) % rota_w + rota_w) % rota_w);
      Refresh(ROTA);
    }
    step_at = millis() | 1;
  }
  void Idle() override;
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
  // A held: a key played sets the note only, or the note and a strike
  bool LongA() override { keys_hit = !keys_hit; keys_at = millis() | 1; return true; }
  void Places(char* buf, size_t n) {  // (USB status)
    AudioEffectChime::Viz v;
    fx.ReadViz(v);
    int k = snprintf(buf, n, "key %d midi %d scale %d mode %d keys %s |", int(cur[NOTE]), int(midi_base), int(cur[SCALE]), int(mode[SCALE]), keys_hit ? "note+hit" : "note");
    for (int p = 0; p < AudioEffectChime::PLACES && k < int(n) - 12; p++)
      k += snprintf(buf + k, n - k, " %d%s", int(v.place_hz[p] + 0.5f), (v.on >> p & 1) ? "*" : "");
  }
private:
  AudioEffectChime fx;
  AudioEffectChime::Viz viz = {};
  float lvl_m[AudioEffectChime::VOICES] = {};
  Meter out_m;
  uint32_t step_at = 0;
  uint32_t note_seen = 0;
  bool keys_hit = true;   // KEY on MIDI: a key strikes them too
  uint32_t keys_at = 0;   // (when switched: shown for a moment)
  static constexpr int rota_w = 20;  // ROTA's places, round and round
  // NOTE on MIDI: the last note played, the key
  int16_t midi_base = 48;
  float Decay(int v) const { return 0.02f * powf(600.0f, v * 0.01f); }  // 20 ms .. 12 s
  static float Blend(int v) { return constrain((40 - v) / 40.0f, 0.0f, 1.0f); }  // below 40: more and more of the wide pass
  static float Morph(int m) { return m ? 0.05f * powf(2.0f, (m - 1) * 0.5f) : 0.003f; }  // 3 ms, 50 ms .. 2.3 s
  static int PitchX(float hz, int x, int w) {  // C1 .. C8 across w
    return x + constrain(int(log2f(std::max(hz, 1.0f) / 32.70f) / 7.0f * (w - 1) + 0.5f), 0, w - 1);
  }
};

// the scale as steps in cents: the first seven pick from the 12 notes,
// tuned by SCALE's mode; the rest are microtonal with their own steps
FLASHMEM void ChimeDevice::Apply(int p) {
  // the sets of notes, as the bells read them: one after the other from
  // below the key up, so a chord spreads over the octaves (ROTA walks
  // through its notes; always in it)
  static const uint16_t masks[SCALES12] = {
    (1 << 0) | (1 << 4) | (1 << 7),                          // MAJ: the major triad
    (1 << 0) | (1 << 3) | (1 << 7),                          // MIN
    (1 << 0) | (1 << 2) | (1 << 7),                          // SUS2
    (1 << 0) | (1 << 5) | (1 << 7),                          // SUS4
    (1 << 0) | (1 << 4) | (1 << 7) | (1 << 11),               // MAJ7
    (1 << 0) | (1 << 3) | (1 << 7) | (1 << 10),               // MIN7
    (1 << 0) | (1 << 4) | (1 << 7) | (1 << 10),               // DOM7
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7),                // ADD9
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 7) | (1 << 10),     // MIN9
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 11),  // MAJ9
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 10),  // MIN11
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 10),  // DOM9
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9),  // 6/9
    (1 << 0) | (1 << 4) | (1 << 6) | (1 << 7) | (1 << 11),  // M7#11: lydian
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9) | (1 << 10),  // DOM13
    (1 << 0) | (1 << 7),                                      // FIFTH
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 11),  // MAJOR scale
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 8) | (1 << 10),  // MINOR
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 10),  // DORIA: dorian
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9),      // PENTA
    (1 << 0) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 10),     // mPENT: minor pentatonic
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10),  // WHOLE tones
  };
  // VARI's family of each chord: its notes and the ones that colour it,
  // which the bells may move to (the scales: their own notes)
  static const uint16_t colours[16] = {
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9) | (1 << 11),  // MAJ: + 9 6 maj7
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 10),  // MIN: + 9 11 7
    (1 << 0) | (1 << 2) | (1 << 5) | (1 << 7) | (1 << 9),  // SUS2: + 4th 6
    (1 << 0) | (1 << 2) | (1 << 5) | (1 << 7) | (1 << 10),  // SUS4: + 9 7
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 7) | (1 << 9) | (1 << 11),  // MAJ7: + 9 #11 13
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 10),  // MIN7: dorian
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9) | (1 << 10),  // DOM7: + 9 13
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9) | (1 << 11),  // ADD9
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 10),  // MIN9
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 7) | (1 << 9) | (1 << 11),  // MAJ9
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 10),  // MIN11
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9) | (1 << 10),  // DOM9
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9) | (1 << 11),  // 6/9
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 7) | (1 << 9) | (1 << 11),  // M7#11
    (1 << 0) | (1 << 2) | (1 << 4) | (1 << 7) | (1 << 9) | (1 << 10),  // DOM13
    (1 << 0) | (1 << 2) | (1 << 5) | (1 << 7) | (1 << 10),  // FIFTH: + 9 4th 7
  };
  // the 12 notes in cents: equal, just (5-limit), Pythagorean, 1/4-comma meantone
  static const float tunings[4][12] = {
    {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100},
    {0, 111.73f, 203.91f, 315.64f, 386.31f, 498.04f, 590.22f, 701.96f, 813.69f, 884.36f, 1017.60f, 1088.27f},
    {0, 90.22f, 203.91f, 294.13f, 407.82f, 498.04f, 611.73f, 701.96f, 792.18f, 905.87f, 996.09f, 1109.78f},
    {0, 76.05f, 193.16f, 310.26f, 386.31f, 503.42f, 579.47f, 696.58f, 772.63f, 889.74f, 1006.84f, 1082.89f},
  };
  // slendro, pelog (approximate: every gamelan is tuned its own way), the
  // 22 shrutis, the harmonic series 8 .. 15
  static const float slendro[5] = {0, 240, 480, 720, 960};
  static const float pelog[7] = {0, 120, 270, 540, 670, 785, 950};
  static const float shruti[22] = {0, 90.2f, 111.7f, 182.4f, 203.9f, 294.1f, 315.6f, 386.3f, 407.8f, 498.0f, 519.6f,
                                   590.2f, 611.7f, 702.0f, 792.2f, 813.7f, 884.4f, 905.9f, 996.1f, 1017.6f, 1088.3f, 1109.8f};
  static const float harmonic[8] = {0, 203.91f, 386.31f, 551.32f, 701.96f, 840.53f, 968.83f, 1088.27f};
  AudioEffectChime::Params a;
  a.root = constrain((mode[NOTE] ? int(midi_base) : 48) + int(cur[NOTE]), 12, 108);  // the key, or the note played (and KEY)
  a.bells = Bells(constrain(int(cur[ARP]), 1, 6), Way());
  const int sc = constrain(int(cur[SCALE]), 0, SCALES - 1);
  a.steps = 0;
  a.period = 1200.0f;
  auto equal = [&](int n, float period) {
    for (int k = 0; k < n; k++) a.step[k] = period * k / n;
    a.steps = n;
    a.period = period;
  };
  auto table = [&](const float* t, int n) {
    for (int k = 0; k < n; k++) a.step[k] = t[k];
    a.steps = n;
  };
  if (sc < SCALES12) {
    const float* tn = tunings[constrain(int(mode[SCALE]), 0, 3)];
    for (int s = 0; s < 12; s++) if (masks[sc] >> s & 1) a.step[a.steps++] = tn[s];
  } else switch (sc) {
    case 22: equal(24, 1200.0f); break;         // QUART: quarter tones
    case 23: equal(17, 1200.0f); break;         // 17ET
    case 24: equal(19, 1200.0f); break;         // 19ET
    case 25: table(slendro, 5); break;          // SLEND
    case 26: table(pelog, 7); break;            // PELOG
    case 27: equal(13, 1901.955f); break;       // BOHLN: Bohlen-Pierce, 13 steps of the twelfth (3:1)
    case 28: table(shruti, 22); break;          // SHRUT
    default: table(harmonic, 8); break;         // HARMO
  }
  a.spread = cur[SPRD];
  a.notes = constrain(int(cur[ARP]), 1, 6);
  a.rotate = uint8_t(constrain(int(mode[ARP]), 0, 3));
  a.rota = cur[ROTA];
  a.step_var = cur[VARI];
  if (sc < SCALES12) {  // VARI's family
    const float* tn = tunings[constrain(int(mode[SCALE]), 0, 3)];
    const uint16_t c = sc < 16 ? colours[sc] : masks[sc];
    a.ncolour = 0;
    for (int s = 0; s < 12; s++) if (c >> s & 1) a.colour[a.ncolour++] = tn[s];
  } else {
    for (int k = 0; k < a.steps; k++) a.colour[k] = a.step[k];
    a.ncolour = a.steps;
  }
  a.var_notes = mode[VARI] + 1;
  a.decay = Decay(cur[DECAY]);
  a.blend = Blend(cur[DECAY]);
  a.decay_var = mode[DECAY] * 0.1f;  // up to x2 / 2
  a.morph = Morph(mode[ROTA]);
  a.wet = cur[WET] * 0.01f;
  fx.Set(a);
}

FLASHMEM void ChimeDevice::Format(int p, char* buf, size_t n) const {
  static const char* const names[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  static const char* const scales[SCALES] = {"MAJ", "MIN", "SUS2", "SUS4", "MAJ7", "MIN7", "DOM7", "ADD9", "MIN9",
                                             "MAJ9", "MIN11", "DOM9", "6/9", "M7#11", "DOM13", "FIFTH",
                                             "MAJOR", "MINOR", "DORIA", "PENTA", "mPENT", "WHOLE",
                                             "QUART", "17ET", "19ET", "SLEND", "PELOG", "BOHLN", "SHRUT", "HARMO"};
  if (p == ARP) { snprintf(buf, n, "%d/6", val[ARP]); return; }  // bells at once
  if (p == ROTA) { snprintf(buf, n, "%d", ((val[ROTA] % rota_w) + rota_w) % rota_w); return; }
  if (p == VARI) { if (val[VARI]) snprintf(buf, n, LOOM_PM "%d", val[VARI]); else snprintf(buf, n, "OFF"); return; }
  if (p == NOTE && mode[NOTE]) {  // KEY on MIDI: the note played, and the offset (E3+2)
    if (val[NOTE]) snprintf(buf, n, "%s%d%+d", names[midi_base % 12], midi_base / 12 - 1, val[NOTE]);
    else snprintf(buf, n, "%s%d", names[midi_base % 12], midi_base / 12 - 1);
  } else if (p == NOTE) {  // the key (D3)
    const int nt = 48 + val[NOTE];
    snprintf(buf, n, "%s%d", names[nt % 12], nt / 12 - 1);
  }
  else if (p == SCALE) snprintf(buf, n, "%s", scales[constrain(int(val[SCALE]), 0, SCALES - 1)]);
  else if (p == DECAY) {
    const float t = Decay(val[DECAY]);
    if (t < 1.0f) snprintf(buf, n, "%dms", int(t * 1000.0f + 0.5f));
    else snprintf(buf, n, "%d.%ds", int(t), int(t * 10.0f) % 10);
  } else Device::Format(p, buf, n);
}

// NOTE on MIDI: a note played (on the channel set) becomes the first note,
// moved by NOTE's offset (the R knob) and by octaves into C1 .. C6
FLASHMEM void ChimeDevice::Idle() {
  if (!mode[NOTE]) return;
  const int ch = mode[NOTE] - 1;  // 0: any
  if (midi_note_seq[ch] == note_seen) return;
  note_seen = midi_note_seq[ch];
  // each key played strikes them (the same one again too), on its note
  if (midi_note[ch] != midi_base) { midi_base = int16_t(midi_note[ch]); Refresh(NOTE); }
  if (keys_hit) { fx.StrikeNew(); step_at = millis() | 1; }
}

FLASHMEM void ChimeDevice::FormatMode(int p, char* buf, size_t n) const {
  if (p == SPRD) { BellsText(buf, n); return; }  // how the bells are spread
  if (p == ROTA) {  // MORPH
    const float t = Morph(mode[ROTA]);
    if (!mode[ROTA]) snprintf(buf, n, "0ms");
    else if (t < 1.0f) snprintf(buf, n, "%dms", int(t * 1000.0f + 0.5f));
    else snprintf(buf, n, "%d.%ds", int(t), int(t * 10.0f) % 10);
  } else if (p == NOTE) {
    if (mode[NOTE] == 0) snprintf(buf, n, "KNOB");
    else if (mode[NOTE] == 1) snprintf(buf, n, "MIDI");
    else snprintf(buf, n, "CH%d", mode[NOTE] - 1);
  } else if (p == DECAY) {
    if (mode[DECAY] == 0) snprintf(buf, n, "OFF");
    else snprintf(buf, n, LOOM_PM "%d%%", int((powf(2.0f, mode[DECAY] * 0.1f) - 1.0f) * 100.0f + 0.5f));
  }
  else if (p == VARI) snprintf(buf, n, "%d/6", mode[VARI] + 1);  // notes moved on each hit
  else Device::FormatMode(p, buf, n);
}

// every resonator as a line at its note (low left, high right, C1 .. C8),
// as tall as it rings; the six notes as small boxes underneath, filled
// where the input goes now. The output level on the right.
FLASHMEM void ChimeDevice::DrawViz(int x, int y, int w, int h) {
  fx.ReadViz(viz);
  out_m.Hit(viz.out);
  const int bw = w - 16, base = y + h - 6;
  for (int i = 0; i < AudioEffectChime::VOICES; i++) {
    lvl_m[i] = std::max(sqrtf(std::min(1.0f, viz.level[i] * 2.0f)), lvl_m[i] * 0.92f);
    if (viz.hz[i] <= 0.0f) continue;
    const int px = PitchX(viz.hz[i], x, bw);
    const int lh = int(lvl_m[i] * (h - 8));
    if (lh > 0) gfxLine(px, base - 1, px, base - 1 - lh);
    if (viz.fed[i] && lh > 0) gfxLine(px + 1, base - 1, px + 1, base - 1 - lh);  // fed: bold
  }
  gfxDottedLine(x, base, x + bw - 1, base, 2);
  for (int p = 0; p < AudioEffectChime::PLACES; p++) {
    if (viz.place_hz[p] <= 0.0f) continue;
    const int px = PitchX(viz.place_hz[p], x, bw);
    if (viz.on >> p & 1) gfxRect(px - 1, base + 2, 3, 3);
    else gfxFrame(px - 1, base + 2, 3, 3);
  }
  if (step_at && millis() - step_at < 150) gfxRect(x + 1, y + 1, 4, 4);  // STEP / HIT: a small mark
  const int mx = x + w - 14;
  gfxFrame(mx, y, 12, h);
  DrawVMeter(mx + 1, y + 1, 10, h - 2, out_m.v);
  if (keys_at && millis() - keys_at < 1500) {  // A held: what a key does now
    const char* t = keys_hit ? "KEYS: NOTE+HIT" : "KEYS: NOTE ONLY";
    const int tw = 6 * int(strlen(t)), tx = x + (w - tw) / 2;
    gfxRect(tx - 3, y + h / 2 - 6, tw + 6, 12);    // (a blank box: filled, then turned black)
    gfxInvert(tx - 3, y + h / 2 - 6, tw + 6, 12);
    gfxFrame(tx - 3, y + h / 2 - 6, tw + 6, 12);
    gfxPrint(tx, y + h / 2 - 3, t);
  }
}

FLASHMEM void ChimeDevice::DrawMini(int x, int y, int w, int h) {
  fx.ReadViz(viz);
  out_m.Hit(viz.out);
  for (int i = 0; i < AudioEffectChime::VOICES; i++) {
    lvl_m[i] = std::max(sqrtf(std::min(1.0f, viz.level[i] * 4.0f)), lvl_m[i] * 0.9f);
    if (viz.hz[i] <= 0.0f) continue;
    const int bh = int(lvl_m[i] * h);
    if (bh > 0) { const int px = PitchX(viz.hz[i], x, w); gfxLine(px, y + h, px, y + h - bh); }
  }
}

/////////////////////////////////////////////////////////////////
// WARP (Source): eight voices playing the other track's sound at the
// pitch of MIDI notes (or of TRIG: the root). SRC picks it: that track's
// SPOOL loop (copied in each time a recording or overdub there ends) or
// what its PLAYER plays (its loop section, read from the card once it
// rests; up to 16 s), all of it, looping; a note at full velocity as
// loud as the sound was recorded. THRU: on track B in THREAD, A's output
// on under the notes (or not). WEFT: a key pressed hard (aftertouch
// from 75 %) wavers, a violinist's vibrato: its speed; mode: its pitch,
// level, tone (a low pass) or place (left / right). Page 2 is the
// notes' envelope.
/////////////////////////////////////////////////////////////////
PROGMEM static const DeviceDef WARP_DEF = {
  "WARP", "WRP", 8, {"TRIG", "STOP"},  // A: the root note, 60 (LOOP on: let go once in); B: every voice let go
  {
    // mode (L): the other track's SPOOL or SHUTTLE, or a THRUM: a waveform
    // from the card (/LOOM/THRUMS; a fresh WARP a random one). Value (R):
    // the thrum, one of the WAVs in its folder (R push: the file list)
    {"SRC", 0, 255, 0, UNI, 0, 3, 2, {}, {"SPL", "SHU", "THR"}},
    {"THRU", 0, 1, 1, LIST, 2, 0, 0, {"OFF", "ON"}},  // THREAD: A's output on under the notes
    {"VEL", 0, 100, 50, UNI, 0, 21, 0},  // %: how much velocity sets a note's loudness; mode: each key's velocity varied, +- 5 n %
    // the vibrato's speed, as one wobble: 0.18 s (5.5 Hz, the fastest) .. 5 s;
    // mode: what wavers (the pitch, the level, the tone or the place)
    {"WEFT", 0, 100, 0, UNI, 0, 4, 0, {}, {"PIT", "LVL", "CUT", "PAN"}},
    // times; mode: the stage's curve
    {"ATK", 0, 100, 55, UNI, 0, 3, 0, {}, {"LOG", "LIN", "EXP"}},  // 1 ms .. 2 s (55: 65 ms)
    {"DEC", 0, 100, 44, UNI, 0, 3, 2, {}, {"LOG", "LIN", "EXP"}},  // 5 ms .. 5 s, to SUS (44: 104 ms)
    {"SUS", 0, 100, 80, UNI},   // %
    {"REL", 0, 100, 59, UNI, 0, 3, 2, {}, {"LOG", "LIN", "EXP"}},  // 5 ms .. 5 s (59: 295 ms)
  },
  {"", ""},
};

class WarpDevice : public Device {
public:
  enum { SRC, THRU, VEL, WEFT, ATK, DECAY, SUS, REL };  // (DEC is taken by Print.h)
  // VEL's variation: each key pressed gets its own velocity, up to +- 5 n %
  // of the full range off what was played
  void FormatMode(int p, char* buf, size_t n) const override {
    if (p != VEL) { Device::FormatMode(p, buf, n); return; }
    if (mode[p]) FormatVar(buf, n, mode[p] * 5); else snprintf(buf, n, "OFF");
  }
  int BarStyle(int p) const override { return p == VEL ? BAR_CENTRE : Device::BarStyle(p); }
  uint32_t held[4] = {};  // the keys down (MIDI notes)
  float lvl_now = 0.7f, lvl_sent = 0.7f;  // the level for them (see Idle)
  uint32_t lvl_ms = 0;
  // a played velocity as the engine's level: VEL in dB (100: 40 dB from
  // the softest to the hardest key, 0: every note full)
  int VelOut(int v) const {
    const float db = -cur[VEL] * 0.4f * (1.0f - v / 127.0f);
    return std::max(1, int(lrintf(127.0f * powf(10.0f, db / 20.0f))));
  }
  enum { SRC_SPOOL, SRC_PLAYER, SRC_THRUM };
  static constexpr const char* THRUM_DIR = "/LOOM/THRUMS";  // (was WAVES on 2026-10-09)
  // THRUM: R push on SRC opens the file list
  int FileParam() const override { return Src() == SRC_THRUM ? SRC : -1; }
  int Src() const { return mode[SRC]; }  // where the sound comes from
  int Step(int p, int v, int dir) const override {  // SRC: the thrums in the folder, no further
    if (p != SRC) return v + dir;
    return tcount ? constrain(v + dir, 0, tcount - 1) : v;
  }
  void OpenBrowser() override;
  void Picked(const char* dir, const char* name) override;
  bool PicksWhileListing() const override { return Src() == SRC_THRUM; }
  bool ListsOnDoubleClick() const override { return true; }
  // the thrum's file (folder/name; "" none yet: a random one is picked);
  // saved with the settings
  char thrum[200] = "";
  // the WAVs in the thrum's folder, sorted (the R knob steps through them)
  static constexpr int TMAX = 64, TLEN = 48;
  char (*tnames)[TLEN] = nullptr;
  int tcount = 0;
  char tdir[160] = "";
  void ThrumList();    // tnames for the thrum's folder, and SRC's value on it
  static constexpr int ROOT = 60;  // the note that plays the sound as recorded (MIDI note 60)
  WarpDevice() : Device(WARP_DEF) {}
  AudioStream* in() override { return &fx; }  // THREAD: track A passes through
  AudioStream* out() override { return &fx; }
  bool Activate() override;
  void Deactivate() override;
  void Apply(int p) override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  bool ActionLit(int a) const override { return a == 0 && fx.Sounding() > 0; }
  void Idle() override;
  bool Busy() const { return loading; }  // reading the card (the output is muted)
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  bool HeaderInfo(char* buf, size_t n) const override;
  void CpuMax(float parts[2]) override {
    parts[0] = fx.processorUsageMax();
    parts[1] = 0.0f;
    fx.processorUsageMaxReset();
  }
  void Reset() override {  // where the sound comes from stays
    const int16_t s = val[SRC];
    const uint8_t m = mode[SRC];
    Defaults();
    val[SRC] = cur[SRC] = s;
    mode[SRC] = m;
    ApplyAll();
  }
  // the other track's SPOOL / PLAYER, when that is its source (the app
  // sets them every loop; nullptr: not)
  const ReelDevice* spool = nullptr;
  const PlayerDevice* player = nullptr;
  char other = 'A';
  bool Thru() const { return cur[THRU]; }  // THREAD: A's output under the notes
  // THRU turned: the app patches A through again (or not)
  bool thru_changed = false;
private:
  bool thru_was = true;
  AudioEffectWarp fx;
  int16_t* bl = nullptr;  // the sound (PSRAM); br null: mono
  int16_t* br = nullptr;
  uint32_t frames = 0;
  float rate = 1.0f;
  int src_was = -1;       // SRC when the sound was taken
  // SPOOL: its loop is what is in memory / being copied in
  bool from_spool = false, copying = false;
  // the loop again (after an overdub): copied beside the one playing, then
  // swapped in with the voices playing on (if memory allows; else after it)
  int16_t* nl = nullptr;
  int16_t* nr = nullptr;
  uint8_t over_new[128] = {};
  uint8_t spool_was = 0;  // its state last time (a recording that ended: copied again)
  uint32_t copy_n = 0;
  // a SPOOL's loop as it was left there: its START..END (copy_from: where
  // that begins), its SPEED for the root note (rev: backwards); from the
  // other track's SPOOL, or a saved one's .loom next to its WAV
  uint32_t copy_from = 0;
  float src_speed = 1.0f;
  bool src_rev = false;
  int16_t spool_sig[3] = {-1, -1, -1};  // START, END, SPEED copied
  uint32_t sig_moved_at = 0;
  bool ReadSpoolMeta(const char* wav, uint32_t& a, uint32_t& b);
  void Reversed();  // the sound turned round (rev), once it is in
  void SpoolBegin();
  void SpoolStep();
  // PLAYER: its file and loop section (in the file's frames), read in once
  // they rest
  char want_path[200] = "", have_path[200] = "";
  uint32_t want_a = 0, want_b = 0, have_a = 0, have_b = 0;
  uint32_t want_at = 0;
  void PlayerIdle();
  void ThrumIdle();
  bool PickRandomThrum();
  // loading, a chunk at a time
  File lf;
  bool loading = false, no_mem = false;
  uint32_t got = 0;
  int bytes = 2, chans = 1;
  bool is_float = false;
  uint8_t over[128] = {};  // the picture: the peak per 1/128 of the sound
  uint32_t midi_seen = 0;
  AudioEffectWarp::Viz viz = {};
  // where notes start and loop: the quiet at either end left out (frames)
  uint32_t trim_a = 0, trim_b = 0;
  void Trim();
  // up to 4096 frames: one cycle of a waveform, played whole and pitched
  // so that MIDI 60 is C4
  bool Cycle() const { return frames && frames <= 4096; }
  void Free();
  void Begin(const char* path, uint32_t from, uint32_t count, bool keep = false);  // opens the file and makes room
  void BeginRead(const char* path, uint32_t from, uint32_t count);
  int16_t* old_l = nullptr;  // Begin keep: the sound still playing
  int16_t* old_r = nullptr;
  void LoadStep();
  static float Attack(int v) { return 0.001f * powf(2000.0f, v * 0.01f); }
  static float VibPeriod(int v) { return (1.0f / 5.5f) * powf(27.5f, v * 0.01f); }  // WEFT: 0.18 s .. 5 s
  static float Time5(int v) { return 0.005f * powf(1000.0f, v * 0.01f); }  // DEC, REL
  static void FormatSecs(float s, char* buf, size_t n) {
    const int t = int(s * 10000.0f + 0.5f);  // in 0.1 ms
    if (!t) snprintf(buf, n, "0ms");
    else if (t < 100) snprintf(buf, n, "%d.%dms", t / 10, t % 10);  // short: every step shows
    else if (s < 1.0f) snprintf(buf, n, "%dms", int(s * 1000.0f + 0.5f));
    else snprintf(buf, n, "%d.%02ds", int(s), int(s * 100.0f) % 100);
  }
};

FLASHMEM bool WarpDevice::Activate() {
  active = true;
  midi_seen = note_ev_n;  // notes from before are not played
  spool_was = 0;          // a SPOOL loop there now is copied in
  have_path[0] = 0;       // PLAYER's sound is read in again
  src_was = -1;
  ApplyAll();
  return true;
}

FLASHMEM void WarpDevice::Deactivate() {
  fx.AllOff();
  Free();
  active = false;
}

FLASHMEM void WarpDevice::Free() {
  if (loading) { lf.close(); loading = false; }
  if (copying) {  // the copy under way
    if (nl) PsramFree(nl);
    if (nr) PsramFree(nr);
    nl = nr = nullptr;
  }
  from_spool = copying = false;
  fx.SetSample(nullptr, nullptr, 0);
  if (old_l) PsramFree(old_l);
  if (old_r) PsramFree(old_r);
  old_l = old_r = nullptr;
  if (bl) PsramFree(bl);
  if (br) PsramFree(br);
  bl = br = nullptr;
  frames = 0;
  have_path[0] = 0;
}

// count frames of the file from frame from (up to 16 s)
FLASHMEM void WarpDevice::Begin(const char* path, uint32_t from, uint32_t count, bool keep) {
  // keep: the sound playing stays in while the new one is read in beside
  // it, then the notes sounding go on in the new one (THRUM: browsing by
  // ear); not when there is no room for both
  const float rate_was = rate;
  if (keep && bl && !loading && !copying && !from_spool) {
    old_l = bl; old_r = br;
    bl = br = nullptr;
  } else {
    Free();
  }
  BeginRead(path, from, count);
  if (!loading && old_l) {  // nothing read in: the old one stays
    if (!bl) { bl = old_l; br = old_r; rate = rate_was; }
    old_l = old_r = nullptr;
  }
}

FLASHMEM void WarpDevice::BeginRead(const char* path, uint32_t from, uint32_t count) {
  no_mem = false;
  memset(over, 0, sizeof(over));
  if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
  lf = SD.open(path, FILE_READ);
  if (!lf) return;
  uint8_t h[12];
  if (lf.read(h, 12) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) { lf.close(); return; }
  int bits = 0;
  uint32_t srate = 48000;
  chans = 0;
  while (lf.available()) {  // the chunks, to fmt and data
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
      int tag = fm[0] | fm[1] << 8;
      if (tag == 0xFFFE && take >= 26) tag = fm[24] | fm[25] << 8;
      is_float = tag == 3;
      if (tag != 1 && tag != 3) bits = 0;
    } else if (!memcmp(c, "data", 4)) {
      const bool ok_bits = is_float ? bits == 32 : (bits == 8 || bits == 16 || bits == 24 || bits == 32);
      if (!ok_bits || chans < 1 || !srate) break;
      bytes = bits / 8;
      rate = srate / AUDIO_SAMPLE_RATE_EXACT;
      const uint32_t total = sz / (bytes * chans);
      from = std::min(from, total);
      const uint32_t n = std::min<uint32_t>({count, total - from, uint32_t(BigSecs() * srate)});
      if (n < 64) break;
      lf.seek(lf.position() + uint64_t(from) * bytes * chans);
      bl = static_cast<int16_t*>(PsramAlloc(n * 2));
      if (chans > 1) br = static_cast<int16_t*>(PsramAlloc(n * 2));
      if (!bl || (chans > 1 && !br)) {
        if (bl) PsramFree(bl);
        if (br) PsramFree(br);
        bl = br = nullptr;
        no_mem = true;
        break;
      }
      frames = n;
      got = 0;
      loading = true;
      return;
    } else {
      lf.seek(lf.position() + sz + (sz & 1));
    }
  }
  lf.close();
}

DMAMEM static uint8_t warp_raw[IO_CHUNK * 8];  // (RAM2: RAM1 is tight)
FLASHMEM void WarpDevice::LoadStep() {
  uint8_t* const raw = warp_raw;
  constexpr size_t RAW = sizeof(warp_raw);
  const size_t frame = size_t(bytes) * chans;
  const uint32_t t0 = millis();
  while (got < frames && millis() - t0 < 3) {
    const size_t k = std::min<size_t>(frames - got, RAW / frame);
    const int rd = lf.read(raw, k * frame);  // -1 on an error: nothing
    if (rd <= 0) { frames = got; break; }
    const size_t m = size_t(rd) / frame;
    for (size_t i = 0; i < m; i++) {
      const uint8_t* fp = raw + i * frame;
      const int16_t l = WavSample(fp, bytes, is_float);
      bl[got + i] = l;
      int16_t pk = abs(l);
      if (br) {
        const int16_t r = WavSample(fp + bytes, bytes, is_float);
        br[got + i] = r;
        pk = std::max<int16_t>(pk, abs(r));
      }
      const int col = int(uint64_t(got + i) * 128 / frames);
      over[col] = std::max<uint8_t>(over[col], uint8_t(std::min(255, pk >> 7)));
    }
    got += m;
  }
  if (got >= frames) {
    lf.close();
    loading = false;
    Reversed();
    Trim();
    if (frames >= 64) fx.SetSample(bl, br, frames, old_l != nullptr);  // (kept: the notes go on in it)
    else if (old_l) fx.SetSample(nullptr, nullptr, 0);
    if (old_l) PsramFree(old_l);
    if (old_r) PsramFree(old_r);
    old_l = old_r = nullptr;
    ApplyAll();  // the rate is known now
  }
}

FLASHMEM void WarpDevice::Idle() {
  if (!active) return;
  // MIDI notes, any channel
  if (note_ev_n - midi_seen > NOTE_EVS) midi_seen = note_ev_n - NOTE_EVS;
  while (midi_seen != note_ev_n) {
    const NoteEv& e = note_ev[midi_seen++ % NOTE_EVS];
    if (e.note < 128) {  // (keys held, for the level)
      if (e.vel) held[e.note >> 5] |= 1u << (e.note & 31);
      else held[e.note >> 5] &= ~(1u << (e.note & 31));
    }
    if (e.vel) {
      int v = e.vel;
      if (mode[VEL]) v = constrain(v + int(random(-1000, 1001) * 0.001f * mode[VEL] * 5 * 1.27f), 1, 127);  // (this key's own)
      // VEL: velocity as the ear hears it, in dB: 100 spans 40 dB (a soft
      // key ~ -34 dB), 50 half that, 0 every note full; the engine then
      // takes the velocity as the level (as a straight line it was barely
      // heard: velocity 20 only -5 dB at VEL 50)
      fx.NoteOn(e.note, VelOut(v));
    } else {
      fx.NoteOff(e.note);
    }
  }
  fx.Pressure(chan_at, poly_at);  // WEFT
  // the level by the keys held: voices add up, so a chord ran into the
  // limiter (grainy from 3 notes on); 0.7 / sqrt(keys): down at once on a
  // new key (~30 ms), back up only as slowly as the notes fade (REL), so
  // tails ringing on don't swell
  {
    int n = 0;
    for (uint32_t w : held) n += __builtin_popcount(w);
    const float to = 0.7f / sqrtf(float(std::max(1, n)));
    const uint32_t now = millis();
    const float dt = std::min(0.1f, (now - lvl_ms) * 0.001f);
    lvl_ms = now;
    if (to != lvl_now) {
      const float k = to < lvl_now ? std::min(1.0f, dt * 30.0f) : std::min(1.0f, dt / std::max(0.3f, Time5(cur[REL])));
      lvl_now += (to - lvl_now) * k;
      if (fabsf(to - lvl_now) < 0.001f) lvl_now = to;
      if (fabsf(lvl_now - lvl_sent) > 0.002f || lvl_now == to) { lvl_sent = lvl_now; Apply(ATK); }
    }
  }
  // SRC turned: the other sound
  if (src_was != Src()) {
    Free();
    src_was = Src();
    spool_was = 0;
    want_at = 0;
  }
  if (Src() == SRC_PLAYER) { PlayerIdle(); return; }
  if (Src() == SRC_THRUM) { ThrumIdle(); return; }
  // the other track's SPOOL loop, once a recording / overdub there has
  // ended (and when there is one already)
  const uint8_t st = spool ? spool->LoopState() : AudioEffectReel::EMPTY;
  const bool rec = st == AudioEffectReel::REC || st == AudioEffectReel::DUB;
  if (!rec && spool && spool->LoopLength() >= 64 && (!from_spool || spool_was != st) &&
      (spool_was == 0 || spool_was == AudioEffectReel::REC || spool_was == AudioEffectReel::DUB))
    SpoolBegin();
  // its START / END / SPEED changed there: copied again once they rest 0.3 s
  if (!rec && spool && from_spool && !copying && spool->LoopLength() >= 64) {
    const bool moved = spool->val[ReelDevice::START] != spool_sig[0] || spool->val[ReelDevice::END] != spool_sig[1] ||
                       spool->val[ReelDevice::SPEED] != spool_sig[2];
    if (!moved) sig_moved_at = 0;
    else if (!sig_moved_at) sig_moved_at = millis() | 1;
    else if (millis() - sig_moved_at > 300) { sig_moved_at = 0; SpoolBegin(); }
  }
  if (copying) SpoolStep();
  spool_was = spool ? st : 0;
}

// the other track's PLAYER: its file's loop section, read in once file and
// section have rested a moment (moving START there doesn't read it over
// and over)
FLASHMEM void WarpDevice::PlayerIdle() {
  if (loading) { LoadStep(); return; }
  if (!player) return;  // that track isn't on PLAYER: what was read stays
  char path[200];
  uint32_t a = 0, b = 0;
  if (!player->Section(path, sizeof(path), a, b)) return;
  if (strcmp(path, want_path) || a != want_a || b != want_b) {
    snprintf(want_path, sizeof(want_path), "%s", path);
    want_a = a; want_b = b;
    want_at = millis() | 1;
  }
  const bool have = !strcmp(want_path, have_path) && want_a == have_a && want_b == have_b;
  if (have || !want_at || millis() - want_at < 500) return;
  want_at = 0;
  src_speed = 1.0f;  // (SHUTTLE's section as it is)
  src_rev = false;
  Begin(want_path, want_a, want_b > want_a ? want_b - want_a : 0);
  snprintf(have_path, sizeof(have_path), "%s", want_path);  // (tried once, worked or not)
  have_a = want_a; have_b = want_b;
}

// THRUM: the wave picked (or, none yet, a random one from /LOOM/WAVES),
// read in whole once
FLASHMEM void WarpDevice::ThrumIdle() {
  if (loading) { LoadStep(); return; }
  if (!strncmp(thrum, "/LOOM/WAVES/", 12)) {  // saved while the folder was WAVES
    char t[sizeof(thrum)];
    snprintf(t, sizeof(t), "%s/%s", THRUM_DIR, thrum + 12);
    snprintf(thrum, sizeof(thrum), "%s", t);
    if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
    if (!SD.exists(THRUM_DIR) && SD.exists("/LOOM/WAVES")) SD.rename("/LOOM/WAVES", THRUM_DIR);
  }
  if (!thrum[0] && !PickRandomThrum()) return;
  ThrumList();
  if (!strcmp(thrum, have_path)) return;
  // a saved SPOOL loop: its section and SPEED, as left there
  uint32_t a = 0, b = 0;
  const bool meta = ReadSpoolMeta(thrum, a, b) && b > a;
  if (meta) Begin(thrum, a, b - a, true);
  else Begin(thrum, 0, 0xFFFFFFFFu, true);
  snprintf(g_warp_report, sizeof(g_warp_report), "WARP file %s: %s %lu..%lu speed %.2f %s, %lu frames\n", thrum,
           meta ? "section" : "whole", (unsigned long)a, (unsigned long)b, double(src_speed), src_rev ? "rev" : "fwd",
           (unsigned long)frames);
  snprintf(have_path, sizeof(have_path), "%s", thrum);  // (tried once, worked or not)
}

// <name>.loom next to a WAV (a SPOOL saved it): EDGES (frames) and SPEED
// (value, 1 forwards / 0 backwards). None: the whole file, as it is
FLASHMEM bool WarpDevice::ReadSpoolMeta(const char* wav, uint32_t& a, uint32_t& b) {
  src_speed = 1.0f;
  src_rev = false;
  char path[200];
  snprintf(path, sizeof(path), "%s", wav);
  if (char* dot = strrchr(path, '.')) *dot = 0;
  strlcat(path, ".loom", sizeof(path));
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  char line[48];
  bool edges = false;
  while (f.available()) {
    int k = 0;
    while (f.available() && k < int(sizeof(line)) - 1) {
      const int c = f.read();
      if (c == '\n') break;
      line[k++] = char(c);
    }
    line[k] = 0;
    unsigned long x = 0, y = 0;
    int v = 0, d = 1;
    if (sscanf(line, "EDGES %lu %lu", &x, &y) == 2) { a = x; b = y; edges = true; }
    else if (sscanf(line, "SPEED %d %d", &v, &d) == 2) { src_speed = std::max(0.05f, v * 0.01f); src_rev = d == 0; }
  }
  f.close();
  return edges;
}

FLASHMEM void WarpDevice::Reversed() {
  if (!src_rev || !bl || frames < 2) return;
  std::reverse(bl, bl + frames);
  if (br) std::reverse(br, br + frames);
  std::reverse(over, over + 128);
}

// a random WAV from a random folder of /LOOM/WAVES (or WAVES itself)
FLASHMEM bool WarpDevice::PickRandomThrum() {
  if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
  if (!SD.exists(THRUM_DIR) && SD.exists("/LOOM/WAVES")) SD.rename("/LOOM/WAVES", THRUM_DIR);
  static uint32_t seed = 0;
  if (!seed) seed = rtc_get() ^ ARM_DWT_CYCCNT ^ micros();
  auto rnd = [](uint32_t n) { seed = seed * 1664525u + 1013904223u; return n ? (seed >> 8) % n : 0u; };
  auto is_wav = [](const char* nm) {
    const size_t l = strlen(nm);
    return nm[0] != '.' && l > 4 && !strcasecmp(nm + l - 4, ".wav");
  };
  // the folders (and the WAVES folder itself, for loose files)
  char dir[160];
  {
    File d = SD.open(THRUM_DIR);
    if (!d) return false;
    int nd = 0;
    for (File e = d.openNextFile(); e; e = d.openNextFile()) { if (e.isDirectory() && e.name()[0] != '.') nd++; e.close(); }
    const int want = int(rnd(nd + 1));  // nd: the WAVES folder itself
    snprintf(dir, sizeof(dir), "%s", THRUM_DIR);
    d.rewindDirectory();
    int i = 0;
    for (File e = d.openNextFile(); e; e = d.openNextFile()) {
      if (e.isDirectory() && e.name()[0] != '.' && i++ == want) snprintf(dir, sizeof(dir), "%s/%s", THRUM_DIR, e.name());
      e.close();
    }
    d.close();
  }
  // a random WAV of that folder; none there: the first WAV of any folder
  File d = SD.open(dir);
  if (d) {
    int nf = 0;
    for (File e = d.openNextFile(); e; e = d.openNextFile()) { if (!e.isDirectory() && is_wav(e.name())) nf++; e.close(); }
    const int want = int(rnd(nf));
    d.rewindDirectory();
    int i = 0;
    for (File e = d.openNextFile(); e && nf; e = d.openNextFile()) {
      if (!e.isDirectory() && is_wav(e.name()) && i++ == want) snprintf(thrum, sizeof(thrum), "%s/%s", dir, e.name());
      e.close();
    }
    d.close();
  }
  if (thrum[0]) return true;
  File w = SD.open(THRUM_DIR);
  if (!w) return false;
  for (File e = w.openNextFile(); e && !thrum[0]; e = w.openNextFile()) {
    if (e.isDirectory() && e.name()[0] != '.') {
      for (File g = e.openNextFile(); g && !thrum[0]; g = e.openNextFile()) {
        if (!g.isDirectory() && is_wav(g.name())) snprintf(thrum, sizeof(thrum), "%s/%s/%s", THRUM_DIR, e.name(), g.name());
        g.close();
      }
    }
    e.close();
  }
  w.close();
  return thrum[0] != 0;
}

// the list opens on the thrum playing (or in WAVES)
FLASHMEM void WarpDevice::OpenBrowser() {
  char dir[200];
  const char* nm = "";
  snprintf(dir, sizeof(dir), "%s", thrum[0] ? thrum : THRUM_DIR);
  if (thrum[0]) {
    char* sl = strrchr(dir, '/');
    if (sl) { *sl = 0; nm = thrum + (sl - dir) + 1; }
  }
  g_browser->Open(this, dir, nm);
}

FLASHMEM void WarpDevice::Picked(const char* dir, const char* name) {
  if (!name || !name[0]) return;
  snprintf(thrum, sizeof(thrum), "%s/%s", dir, name);
  ThrumList();
}

// the WAVs of the thrum's folder, sorted, once per folder; SRC's value
// put on the thrum (so the R knob goes on from it)
FLASHMEM void WarpDevice::ThrumList() {
  const char* sl = strrchr(thrum, '/');
  if (!sl) return;
  char dir[sizeof(tdir)];
  snprintf(dir, std::min<size_t>(sizeof(dir), size_t(sl - thrum) + 1), "%s", thrum);
  if (strcmp(dir, tdir)) {
    snprintf(tdir, sizeof(tdir), "%s", dir);
    tcount = 0;
    if (!tnames) tnames = static_cast<char(*)[TLEN]>(malloc(TMAX * TLEN));
    if (!tnames) return;
    if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
    File d = SD.open(dir);
    for (File e = d ? d.openNextFile() : File(); e && tcount < TMAX; e = d.openNextFile()) {
      const char* nm = e.name();
      const size_t l = strlen(nm);
      if (!e.isDirectory() && nm[0] != '.' && l > 4 && l < TLEN && !strcasecmp(nm + l - 4, ".wav"))
        snprintf(tnames[tcount++], TLEN, "%s", nm);
      e.close();
    }
    if (d) d.close();
    qsort(tnames, tcount, TLEN, [](const void* a, const void* b) { return strcasecmp((const char*)a, (const char*)b); });
  }
  for (int i = 0; i < tcount; i++)
    if (!strcmp(tnames[i], sl + 1)) { val[SRC] = cur[SRC] = int16_t(i); break; }
}

FLASHMEM void WarpDevice::SpoolBegin() {
  // its START..END, at its SPEED (backwards when below 0)
  const uint32_t whole = uint32_t(spool->LoopLength());
  const uint32_t a = uint32_t(uint64_t(whole) * uint32_t(std::max<int>(0, spool->val[ReelDevice::START])) / 10000);
  const uint32_t b = std::max(a + 1, uint32_t(uint64_t(whole) * uint32_t(std::max<int>(0, spool->val[ReelDevice::END])) / 10000));
  const int sp = spool->val[ReelDevice::SPEED];
  src_speed = std::max(0.05f, std::abs(sp) * 0.01f);
  src_rev = sp < 0;
  spool_sig[0] = spool->val[ReelDevice::START];
  spool_sig[1] = spool->val[ReelDevice::END];
  spool_sig[2] = int16_t(sp);
  copy_from = std::min(a, whole);
  snprintf(g_warp_report, sizeof(g_warp_report), "WARP spool: %lu..%lu of %lu speed %.2f %s\n", (unsigned long)a,
           (unsigned long)b, (unsigned long)whole, double(src_speed), src_rev ? "rev" : "fwd");
  const uint32_t n = std::min<uint32_t>(std::min(b, whole) - copy_from, uint32_t(BigSecs() * AUDIO_SAMPLE_RATE_EXACT));
  auto take = [&] {
    nl = static_cast<int16_t*>(PsramAlloc(n * 2));
    nr = nl ? static_cast<int16_t*>(PsramAlloc(n * 2)) : nullptr;
    if (nl && !nr) { PsramFree(nl); nl = nullptr; }
    return nl != nullptr;
  };
  const bool keep = from_spool && !copying && frames;  // playing the loop's last take
  if (!keep) Free();
  if (!take()) {
    if (keep) { Free(); take(); }  // no room for both: the old one goes first
  }
  no_mem = !nl;
  if (no_mem) return;
  memset(over_new, 0, sizeof(over_new));
  copy_n = n;
  got = 0;
  from_spool = copying = true;
}

// a slice at a time (the loop is in PSRAM too); started over if it changes
FLASHMEM void WarpDevice::SpoolStep() {
  const AudioEffectReel::State st = spool ? spool->LoopState() : AudioEffectReel::EMPTY;
  if (!spool || spool->LoopLength() < copy_from + copy_n || st == AudioEffectReel::REC || st == AudioEffectReel::DUB) {
    // gone or being recorded over: dropped (copied again once it is done)
    if (nl) PsramFree(nl);
    if (nr) PsramFree(nr);
    nl = nr = nullptr;
    copying = false;
    from_spool = frames > 0;  // what played before plays on
    spool_was = 0;
    return;
  }
  const int16_t* l = spool->LoopData(0) + copy_from;
  const int16_t* r = spool->LoopData(1) + copy_from;
  const uint32_t t0 = millis();
  while (got < copy_n && millis() - t0 < 3) {
    const uint32_t k = std::min<uint32_t>(copy_n - got, 4096);
    memcpy(nl + got, l + got, k * 2);
    memcpy(nr + got, r + got, k * 2);
    for (uint32_t i = got; i < got + k; i++) {
      const int col = int(uint64_t(i) * 128 / copy_n);
      const int pk = std::max(abs(nl[i]), abs(nr[i]));
      over_new[col] = std::max<uint8_t>(over_new[col], uint8_t(std::min(255, pk >> 7)));
    }
    got += k;
  }
  if (got >= copy_n) {
    const bool keep = frames > 0;  // the last take still in: the voices go on in the new one
    fx.SetSample(nl, nr, copy_n, keep);
    // (trimmed below, once it is in place: Trim reads bl / br)
    if (bl) PsramFree(bl);
    if (br) PsramFree(br);
    bl = nl; br = nr;
    nl = nr = nullptr;
    frames = copy_n;
    rate = 1.0f;
    memcpy(over, over_new, sizeof(over));
    copying = false;
    Reversed();
    Trim();
    ApplyAll();
  }
}

// the quiet at the start and the end of the sound: notes start where it
// gets going (5 ms before), and a held note loops round before the tail
// goes quiet (50 ms after the last of it). Quiet: below 2 % of its peak
FLASHMEM void WarpDevice::Trim() {
  trim_a = 0;
  trim_b = frames;
  if (!bl || frames < 64 || Cycle()) return;
  int pk = 0;
  for (uint32_t i = 0; i < frames; i++) {
    pk = std::max(pk, abs(int(bl[i])));
    if (br) pk = std::max(pk, abs(int(br[i])));
  }
  const int th = std::max(64, pk / 50);
  auto loud = [&](uint32_t i) { return abs(int(bl[i])) > th || (br && abs(int(br[i])) > th); };
  uint32_t a = 0, b = frames;
  while (a < frames && !loud(a)) a++;
  while (b > a && !loud(b - 1)) b--;
  if (b <= a) return;  // all quiet: the whole of it
  const uint32_t pre = uint32_t(0.005f * AUDIO_SAMPLE_RATE_EXACT * rate);
  const uint32_t post = uint32_t(0.05f * AUDIO_SAMPLE_RATE_EXACT * rate);
  trim_a = a > pre ? a - pre : 0;
  trim_b = std::min(frames, b + post);
  if (trim_b < trim_a + 64) { trim_a = 0; trim_b = frames; }
}

FLASHMEM void WarpDevice::Apply(int p) {
  if (Thru() != thru_was) { thru_was = Thru(); thru_changed = true; }
  if (p == SRC && Src() == SRC_THRUM && tnames && tcount) {  // R: another WAV of the folder
    const int i = constrain(int(cur[SRC]), 0, tcount - 1);
    char t[sizeof(thrum)];
    snprintf(t, sizeof(t), "%s/%s", tdir, tnames[i]);
    if (strcmp(t, thrum)) snprintf(thrum, sizeof(thrum), "%s", t);  // (read in by ThrumIdle)
  }
  // the sound without the quiet at either end, looping with a 40 ms
  // crossfade; a single cycle (a waveform) whole, at the pitch of the note
  AudioEffectWarp::Params a;
  if (frames) {
    a.start = float(trim_a) / frames;
    a.end = float(trim_b) / frames;
  }
  a.loop = AudioEffectWarp::LOOP;
  a.cross = true;
  a.xfade = 0.04f;
  a.attack = Attack(cur[ATK]);
  a.decay = Time5(cur[DECAY]);
  a.sustain = cur[SUS] * 0.01f;
  a.release = Time5(cur[REL]);
  a.atk_curve = mode[ATK];
  a.dec_curve = mode[DECAY];
  a.rel_curve = mode[REL];
  a.level = lvl_now;  // (by how many keys are held: see Idle)
  a.vel_amt = 1.0f;  // (VEL is applied to the velocity: see Idle, in dB)
  a.vib_hz = 1.0f / VibPeriod(cur[WEFT]);
  a.weft = mode[WEFT];
  a.root = ROOT;
  a.rate = Cycle() ? frames * 261.6256f / AUDIO_SAMPLE_RATE_EXACT : rate * src_speed;  // MIDI 60 = C4
  fx.Set(a);
}

FLASHMEM void WarpDevice::Format(int p, char* buf, size_t n) const {
  switch (p) {
    case SRC:
      if (Src() != SRC_THRUM) { snprintf(buf, n, "%s %c", Src() == SRC_PLAYER ? "SHU" : "SPL", other); break; }
      {  // the thrum, short: its name without '_' and ".wav", 5 letters (SAW07, TRI03)
        const char* nm = strrchr(thrum, '/');
        nm = nm ? nm + 1 : thrum;
        char s[32];
        int k = 0;
        for (const char* c = nm; *c && *c != '.' && k < 31; c++) if (*c != '_' && *c != ' ') s[k++] = *c;
        s[k] = 0;
        if (!k) { snprintf(buf, n, "THR"); break; }
        if (k > 5) { s[3] = s[k - 2]; s[4] = s[k - 1]; s[5] = 0; }
        snprintf(buf, n, "%s", s);
      }
      break;
    case VEL: snprintf(buf, n, "%d%%", int(val[p])); break;
    case WEFT:
      FormatSecs(VibPeriod(val[WEFT]), buf, n);  // one wobble
      break;
    case ATK: FormatSecs(Attack(val[ATK]), buf, n); break;
    case DECAY: case REL: FormatSecs(Time5(val[p]), buf, n); break;
    case SUS: snprintf(buf, n, "%d%%", int(val[SUS])); break;
    default: Device::Format(p, buf, n);
  }
}

FLASHMEM void WarpDevice::Action(int a) {
  if (a == 0) {  // TRIG: the root, as if a key was struck (LOOP on: let go once in)
    fx.NoteOff(ROOT);
    fx.NoteOn(ROOT, VelOut(100), true);
  } else {
    fx.AllOff();
  }
}

FLASHMEM bool WarpDevice::HeaderInfo(char* buf, size_t n) const {
  const uint32_t of = copying ? copy_n : frames;
  if (loading || copying) snprintf(buf, n, "%d%%", of ? int(uint64_t(got) * 100 / of) : 0);
  else snprintf(buf, n, "%d/8", fx.Sounding());
  return true;
}

// the sound's outline, and a line where each voice is; page 2: the envelope
FLASHMEM void WarpDevice::DrawViz(int x, int y, int w, int h) {
  fx.ReadViz(viz);
  if (view_page == 1) {
    // ATK up, DEC down to SUS, held, REL down; a dot per sounding voice on
    // its height
    const float ta = Attack(cur[ATK]), td = Time5(cur[DECAY]), tr = Time5(cur[REL]);
    const float ws = (ta + td + tr) * 0.4f + 0.05f;  // the held part, drawn this long
    const float total = ta + td + ws + tr;
    const float sus = cur[SUS] * 0.01f;
    const int x1 = x + 1 + int((w - 3) * ta / total), x2 = x1 + int((w - 3) * td / total);
    const int x3 = x2 + int((w - 3) * ws / total), x4 = x + w - 2;
    const int yb = y + h - 2, yt = y + 1;
    const int ys = yb - int((yb - yt) * sus);
    // each stage in its curve (LOG / LIN / EXP), as it sounds
    auto stage = [&](int xa, int xb, int ya, int yz, bool rise, uint8_t c) {
      int py = ya;
      for (int i = xa + 1; i <= xb; i++) {
        const float t = float(i - xa) / std::max(1, xb - xa);
        const float k = rise ? AudioEffectWarp::Rise(c, t) : AudioEffectWarp::Fall(c, t);
        const int yy = ya + int((yz - ya) * k + (yz > ya ? 0.5f : -0.5f));
        gfxLine(i - 1, py, i, yy);
        py = yy;
      }
    };
    stage(x + 1, x1, yb, yt, true, mode[ATK]);
    stage(x1, x2, yt, ys, false, mode[DECAY]);
    gfxLine(x2, ys, x3, ys);
    stage(x3, x4, ys, yb, false, mode[REL]);
    for (int i = 0; i < AudioEffectWarp::VOICES; i++) {
      if (viz.pos[i] < 0.0f) continue;
      const int yy = yb - int((yb - yt) * std::min(1.0f, viz.env[i]));
      gfxRect(x + w - 6 - i * 4, yy - 1, 3, 3);
    }
    return;
  }
  if (!frames && !loading && !copying) {
    const char* t = no_mem ? "NO MEMORY"
        : Src() == SRC_THRUM ? (thrum[0] ? "NOT A WAV" : "NO WAVES ON CARD")
        : Src() == SRC_PLAYER ? (player ? "NOT A WAV" : "SHUTTLE IS OFF")
        : spool ? "RECORD IN SPOOL" : "SPOOL IS OFF";
    gfxPrint(x + 2, y + h / 2 - 4, t);
    return;
  }
  const int cy = y + h / 2, half = h / 2 - 1;
  int top = 1;  // the loudest part fills the height (a quiet sound shows too)
  for (uint8_t o : over) top = std::max<int>(top, o);
  for (int i = 0; i < w; i++) {
    const int a = over[i * 128 / w] * half / top;
    if (!a) continue;
    const uint32_t at = uint32_t(uint64_t(i) * frames / w);  // the quiet ends, left out: dotted
    if (at >= trim_a && at < trim_b) gfxLine(x + i, cy - a, x + i, cy + a);
    else if (i % 2 == 0) { gfxPixel(x + i, cy - a); gfxPixel(x + i, cy + a); }
  }
  for (int i = 0; i < AudioEffectWarp::VOICES; i++) {
    if (viz.pos[i] < 0.0f) continue;
    const int px = x + constrain(int(viz.pos[i] * (w - 1)), 0, w - 1);
    gfxInvert(px, y + int((1.0f - viz.env[i]) * half), 1, std::max(1, int(viz.env[i] * 2 * half)));
  }
  if (Src() == SRC_THRUM && thrum[0] && !loading) {  // the thrum's name, top left
    const char* nm = strrchr(thrum, '/');
    char t[24];
    snprintf(t, sizeof(t), "%s", nm ? nm + 1 : thrum);
    if (char* dot = strrchr(t, '.')) *dot = 0;
    graphics.clearRect(x, y, 6 * int(strlen(t)) + 2, 9);
    gfxPrint(x + 1, y + 1, t);
  }
  {  // WEFT: the pressure coming in (the most of any key), a bar at the right
    int at = chan_at;
    for (int i = 0; i < AudioEffectWarp::VOICES; i++) if (viz.pos[i] >= 0.0f) at = std::max<int>(at, poly_at[viz.note[i] & 127]);
    const int bh = at * (h - 2) / 127;
    if (bh > 0) { graphics.clearRect(x + w - 4, y, 4, h); gfxRect(x + w - 3, y + h - 1 - bh, 2, bh); }
  }
  if (loading) {
    char t[16];
    snprintf(t, sizeof(t), "LOAD %d%%", frames ? int(uint64_t(got) * 100 / frames) : 0);
    graphics.clearRect(x, y, 6 * int(strlen(t)) + 3, 10);
    gfxPrint(x + 1, y + 1, t);
  }
}

// a bar per voice, as loud as its envelope
FLASHMEM void WarpDevice::DrawMini(int x, int y, int w, int h) {
  fx.ReadViz(viz);
  const int bw = std::max(1, (w - 7) / 8);
  for (int i = 0; i < AudioEffectWarp::VOICES; i++) {
    const int bh = viz.pos[i] < 0.0f ? 0 : std::max(1, int(viz.env[i] * h));
    const int bx = x + i * (bw + 1);
    if (bh) gfxRect(bx, y + h - bh, bw, bh);
    else gfxPixel(bx, y + h - 1);
  }
}

struct SlotDef {
  char name[10];
  char tag[5];       // chain box label, 3 letters (4 drawn tighter)
  uint8_t noptions;  // BYPASS + devices that exist now (SOURCE: no bypass)
  uint8_t planned;   // devices this slot will offer (dots on the chain view)
};

PROGMEM static const SlotDef SLOT_DEFS[SLOT_COUNT] = {
  {"SOURCE",   "SRC", 4, 4},  // INPUT, SPOOL, PLAYER, WARP (owned by the track, not made here)
  {"FIBRE",    "FIB", 3, 1},  // BYPASS, FIBRE (grains), DRAFT (pitch / time)
  {"REED",     "RD",  4, 1},  // BYPASS, REED (filter bank), BEATER (struck object), TWILL (tuned resonators)
  {"TEXTURE",  "TEX", 3, 1},  // BYPASS, TEXTURE, HEDDLE (shifter / phaser)
  {"REPEAT",   "RPT", 2, 1},  // BYPASS, REPEAT (delay, reverb)
};

// a device's short name on screen. The saved tag stays what it was (files
// and presets use it); the names moved on to weaving words on 2026-10-10.
static const char* ShownTag(const char* tag) {
  static const char map[][2][5] = {
    {"SPR", "FIB"}, {"STR", "DFT"}, {"RES", "REED"}, {"MDL", "BTR"},
    {"CHM", "TWL"}, {"ECO", "RPT"}, {"MIX", "CLTH"}, {"REL", "SPL"}};
  for (auto& m : map) if (!strcmp(tag, m[0])) return m[1];
  return tag;
}

// nullptr = BYPASS. SOURCE/INPUT is owned by the track, not made here.
FLASHMEM static Device* MakeDevice(int slot, int option) {
  if (slot == SPRAY && option == 1) return new SprayDevice;
  if (slot == SPRAY && option == 2) return new StretchDevice;
  if (slot == RESONATOR && option == 1) return new ResonatorDevice;
  if (slot == RESONATOR && option == 2) return new ModalDevice;
  if (slot == RESONATOR && option == 3) return new ChimeDevice;
  if (slot == TEXTURE && option == 1) return new DirtDevice;
  if (slot == TEXTURE && option == 2) return new HeddleDevice;
  if (slot == ECHO && option == 1) return new EchoDevice;
  return nullptr;
}

} // namespace loom
