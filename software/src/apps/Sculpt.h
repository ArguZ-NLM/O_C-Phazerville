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

// Sculpt: a full-screen effect chain app.
//
// Two stereo tracks, each a fixed chain SOURCE > SPRAY > RESONATOR > TEXTURE
// > ECHO, mixed to the stereo output. Every effect slot can be bypassed.
//
// Everywhere: R enc moves the cursor, L enc changes what is under it (device
// type, a setting's mode, modulator type), R push opens / grabs, Y goes back.
//
// Chain view (home):
//   R enc  move between the 12 boxes      L enc  change the slot's device
//                                                (MIX: track level)
//   DOWN   bypass on/off (MIX: its band stage)   R push open the device
//   UP     the device's first action (e.g. ECHO freeze)
//   X      the track's modulators (MOD A-D) and the clock
//   UP + DOWN  link / unlink the tracks' controls. Linking copies the track
//              you are on to the other one; while linked every edit goes to
//              both tracks (except input source and pan). Both keep running.
// Device view:
//   R enc  move between the 8 settings    L enc  the setting's mode
//   R push grab / let go; grabbed, R enc changes the value
//   L push modulation for the setting (L push again to leave). It has two
//          sources that add up, MOD1 and MOD2: R push switches between
//          them. L enc picks the source, each one unipolar (U) then bipolar
//          (B): off, MdA U, MdA B, MdB U ... (MOD1 then CV in, CV out, MIDI
//          maps, noise; MOD2 only MOD A-H). R enc the amount.
//          MOD1: X held = MIDI learn (move a knob)
//   X      the MOD view, to set up a modulator; Y / X there come back here
//   L held action triggers: L enc source for UP's action, R enc for DOWN's
//          (gate in, CV in as gate, MIDI map), L push / Y to leave
//   Y      back                           UP / DOWN  actions
// MOD view (4 modulators per track, plus the clock):
//   R enc  choose A-D or CLK              L enc  type: WAVE/RANDOM/ADSR/FOLLOW
//   R push open its settings              Y / X  back to the chain
//   CLK is the app's tempo: internal (UP = tap), MIDI clock or a trigger
//   input. ECHO TIME and the WAVE / RANDOM rates can sync to it (L enc on the
//   setting: FREE / STR / TRI / DOT).
//
// Settings are saved to SCULPT.BIN on the internal flash when Sculpt is left
// (e.g. app menu) and with the global save, and loaded at power-up.
//
// While Sculpt is the active app it claims the audio outputs from AudioIO.
// The Quadrants audio applet chain is left intact and comes back as soon as
// Sculpt is suspended (e.g. when the app menu opens).

#include "../HSApplication.h"
#include "../AudioIO.h"
#include "../OC_gpio.h"
#include "../HSMIDI.h"
#include "../PhzConfig.h"
#include "SculptDevices.h"
#include "SculptMods.h"

namespace sculpt {
  constexpr int NUM_TRACKS = 2;
  constexpr int COLS = SLOT_COUNT + 1; // + MIX
  constexpr int MIX_COL = SLOT_COUNT;

  struct Track {
    InputDevice* input = nullptr;
    ReelDevice* reel = nullptr;    // SOURCE option 1: a looper after the input
    PlayerDevice* player = nullptr;  // SOURCE option 2: a WAV from the SD card
    WavWriter writer;              // saving REEL's loop to the SD card
    MixDevice* mix = nullptr;
    uint8_t choice[SLOT_COUNT] = {};
    uint8_t last_choice[SLOT_COUNT] = {}; // for the bypass toggle
    uint8_t bypass = 0;  // bit per slot (+ MIX_COL: its band stage): passed around
    uint8_t parked = 0;  // bit per slot: bypassed and unpatched (no CPU)
    uint32_t park_at[SLOT_COUNT] = {};  // when a bypassed device gets parked
    Device* inst[SLOT_COUNT][MAX_OPTIONS] = {};
    SculptFade* fade[SLOT_COUNT] = {};  // after each effect slot: bypass fade
    SculptGate* gate = nullptr;         // before ECHO: its tail rings out
    Modulator* mods[NUM_MODS][MOD_TYPES] = {};
    uint8_t mod_type[NUM_MODS] = {};
    // MOD A-D, then a copy of the shared E-H; -1 = no modulator there
    float mod_out[MOD_BUSES] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

    Modulator* mod(int i) const { return mods[i][mod_type[i]]; }

    Device* current(int s) const {
      if (s == SOURCE) {
        if (choice[SOURCE] == 1 && reel) return reel;
        if (choice[SOURCE] == 2 && player) return player;
        return input;
      }
      return inst[s][choice[s]];
    }
    bool bypassed(int s) const { return s < SLOT_COUNT && (bypass >> s & 1); }
    bool shown_bypassed(int c) const { return bypass >> c & 1; } // incl. OUT
    Device* at(int col) const {
      return col == MIX_COL ? mix : current(col);
    }
  };

  // patch cords live on the heap: no RAM1 until Sculpt is first opened
  struct Patch {
    AudioConnection in[NUM_TRACKS][2];
    // per slot: in -> (gate ->) device, in -> fade dry, device -> fade wet
    AudioConnection link[NUM_TRACKS][SLOT_COUNT][8];
    AudioConnection to_mix[NUM_TRACKS][2];
    AudioConnection mix_out[NUM_TRACKS][2];
  };

  Track tracks[NUM_TRACKS];
  SculptMixer* mixer = nullptr;
  ClockDevice* tempo_clock = nullptr;
  Patch* patch = nullptr;
  // MOD E-H: shared by both tracks (in the MOD view's second page)
  Modulator* shared_mods[NUM_MODS][MOD_TYPES] = {};
  uint8_t shared_type[NUM_MODS] = {};
  char load_note[160] = "";  // how the last settings load went (USB stats)
  // MOD view cursor: 0-3 the track's A-D, 4-7 the shared E-H, then CLOCK,
  // then the 8 CV outputs (page 3)
  constexpr int CLK_CURSOR = 2 * NUM_MODS;
  constexpr int NUM_CV_OUTS = 8;
  constexpr int OUT_CURSOR = CLK_CURSOR + 1;
  constexpr int LAST_CURSOR = OUT_CURSOR + NUM_CV_OUTS - 1;
  // what each CV output A-H sends (0-10 V): off, track L's MOD A-D, track
  // R's MOD A-D, the shared E-H, or the clock as a gate
  enum { OUT_OFF, OUT_L = 1, OUT_R = 5, OUT_SHARED = 9, OUT_CLOCK = 13, OUT_SOURCES };
  // by default A-D = track L's MOD A-D, E-H = track R's
  uint8_t cv_out_src[NUM_CV_OUTS] = {OUT_L, OUT_L + 1, OUT_L + 2, OUT_L + 3, OUT_R, OUT_R + 1, OUT_R + 2, OUT_R + 3};
  FLASHMEM void FormatOutSource(int s, char* buf, size_t n, bool long_name) {
    if (s >= OUT_CLOCK) snprintf(buf, n, long_name ? "clock (beats)" : "CK");
    else if (s >= OUT_SHARED) snprintf(buf, n, long_name ? "shared MOD %c" : "%c", 'E' + s - OUT_SHARED);
    else if (s >= OUT_R) snprintf(buf, n, long_name ? "track R MOD %c" : "R%c", 'A' + s - OUT_R);
    else if (s >= OUT_L) snprintf(buf, n, long_name ? "track L MOD %c" : "L%c", 'A' + s - OUT_L);
    else snprintf(buf, n, long_name ? "off" : "--");
  }

  bool audio_hw() { return I2S2_Audio_ADC && I2S2_Audio_DAC; }
}

OC_APP_CLASS(AppSculpt, TWOCCS("SP"), "Sculpt", "Sculpt"),
  public HSApplication {
public:
  OC_APP_INTERFACE_DECLARE(AppSculpt, 0);

  void Start();
  void Resume();
  void Suspend();
  // latch rising edges of the gate inputs and CV inputs (as gates) for the
  // modulators, which only look about once per millisecond
  void Controller() {
    uint16_t e = 0;
    for (int i = 0; i < 12; i++) e |= HS::frame.clocked[i] << i;
    sculpt::trig_latch |= e;
  }
  void MainLoop();
  void ModulateAll();
  void ReadMIDI();
  void MidiLearn(sculpt::Device* d);
  void View() const;

  void OnButton(const UI::Event &event);
  void OnEncoder(const UI::Event &event);

private:
  enum ViewMode : uint8_t { CHAIN, DEVICE, MODS };
  ViewMode view = CHAIN;
  bool focus_mod = false;   // the device view shows a modulator
  int8_t mod_cursor = 0;
  elapsedMicros tick_timer;
  int8_t track = 0, col = sculpt::ECHO, cursor = 0;
  bool l_held = false, l_used = false;
  bool loop_save_held = false;  // REEL: A is down (held 2 s saves the loop) ...
  uint32_t loop_save_hold = 0;  // ... since then
  bool save_held = false;   // chain view: Y is down (held 2 s saves) ...
  uint32_t save_hold = 0, saved_at = 0;  // ... since then; when the save happened
  bool clear_held = false;  // REEL: B is down on the REEL page ...
  uint32_t clear_hold = 0;  // ... since then
  bool editing = false;   // device view: setting grabbed, R enc changes it
  bool edit_mode = false; // the mode was just changed: show its name
  mutable elapsedMillis mode_shown;
  bool mod_mode = false;  // device view: L enc amount, R enc source
  uint8_t mod_slot = 0;   // ... of the first or the second source
  // the MOD view opened from a device page (X): where Y / X go back to
  struct { bool on; bool focus_mod; int8_t mod_cursor, cursor; } back = {};
  // L enc in modulation: each source comes as U then B (off has neither)
  void StepModOption(sculpt::Device* d, int p, int dir) {
    const int bit = 1 << p;
    uint8_t& bi = mod_slot ? d->mod_bi2 : d->mod_bi;
    auto on = [&]() { return mod_slot ? d->mod_bus2[p] != 0 : d->ModSourceOn(p); };
    auto source = [&](int s) { if (mod_slot) d->ChangeModSource2(p, s); else d->ChangeModSource(p, s); };
    for (int n = 0; n < abs(dir); n++) {
      if (dir > 0) {
        if (on() && !(bi & bit)) bi |= bit;
        else {
          const auto was = std::make_pair(d->mod_in[p].Pack(), mod_slot ? d->mod_bus2[p] : d->mod_bus_sel[p]);
          source(1);
          if (was == std::make_pair(d->mod_in[p].Pack(), mod_slot ? d->mod_bus2[p] : d->mod_bus_sel[p])) break;  // the end of the list
          bi &= ~bit;
        }
      } else {
        if (!on()) break;
        if (bi & bit) bi &= ~bit;
        else { source(-1); if (on()) bi |= bit; else bi &= ~bit; }
      }
    }
  }
  void BackToDevice() {
    view = DEVICE;
    focus_mod = back.focus_mod;
    mod_cursor = back.mod_cursor;
    cursor = back.cursor;
    back.on = false;
  }
  bool trig_mode = false; // device view: L / R enc trigger source per action
  elapsedMicros mod_timer;
  uint8_t chord_releases = 0; // UP/DOWN releases to swallow after UP+DOWN
  bool active = false;
  bool linked = false;
  mutable bool dirty = false; // something changed since the last save
  float applied_tempo = 120.0f;
  int cpu = 0;
  elapsedMillis stats_timer;

  sculpt::Device* FocusOn(int t) const;
  void SaveFile() const;
  void LoadFile();
  template <typename F> void ForEachDevice(F f) const;
  sculpt::Device* Focused() const { return FocusOn(track); }
  // modulator i (0-7) as seen from track t: its row of types and its type
  static sculpt::Modulator** ModRow(int t, int i) {
    return i < sculpt::NUM_MODS ? sculpt::tracks[t].mods[i] : sculpt::shared_mods[i - sculpt::NUM_MODS];
  }
  static uint8_t& ModType(int t, int i) {
    return i < sculpt::NUM_MODS ? sculpt::tracks[t].mod_type[i] : sculpt::shared_type[i - sculpt::NUM_MODS];
  }
  static sculpt::Modulator* ModAt(int t, int i) {
    if (i > sculpt::CLK_CURSOR) return nullptr;  // the CV outputs
    if (i == sculpt::CLK_CURSOR) return sculpt::tempo_clock;
    return ModRow(t, i)[ModType(t, i)];
  }
  bool SharedFocus() const { return focus_mod && mod_cursor >= sculpt::NUM_MODS && mod_cursor < sculpt::CLK_CURSOR; }
  void SetModType(int t, int i, int type);
  void DrawMods() const;
  void DrawOutputs() const;
  float OutLevel(int o) const;
  void WriteOutputs();
  void SetChoice(int t, int s, int option);
  void SetChoiceLinked(int s, int option);
  void ToggleBypass(int s);
  sculpt::Device* Twin(sculpt::Device* d) const;
  void Mirror(sculpt::Device* d, int p) const;
  void Repatch(int t);
  void ApplyBypass(int t);
  void ParkBypassed(int t);
  void ConnectTrack(int t);
  void DisconnectTrack(int t);
  void ToggleLink();
  void UpdateMixLevels() const;
  void DrawChain() const;
  bool Muted() const;
  bool BypassKey() const;
  void DrawDevice() const;
};

FLASHMEM sculpt::Device* AppSculpt::FocusOn(int t) const {
  return focus_mod ? ModAt(t, mod_cursor) : sculpt::tracks[t].at(col);
}

FLASHMEM void AppSculpt::Start() {
  using namespace sculpt;
  // Create every device up front in chain order, so the audio update order
  // follows the signal flow (no extra block of latency per hop). Devices only
  // hold their big buffers while they are in use.
  for (int t = 0; t < NUM_TRACKS; t++) {
    Track& tr = tracks[t];
    tr.input = new InputDevice;
    tr.input->mod_bus = tr.mod_out;
    tr.input->Defaults();
    tr.input->val[0] = t ? InputDevice::IN2 : InputDevice::IN1;
    tr.input->ApplyAll();
    tr.reel = new ReelDevice(tr.input);
    tr.reel->mod_bus = tr.mod_out;
    tr.reel->Defaults();
    tr.reel->val[ReelDevice::IN] = tr.input->val[0];
    tr.reel->ApplyAll();
    tr.player = new PlayerDevice;
    tr.player->mod_bus = tr.mod_out;
    tr.player->Defaults();
    for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
      if (s == ECHO) tr.gate = new SculptGate;
      for (int o = 0; o < SLOT_DEFS[s].noptions; o++) {
        Device* d = MakeDevice(s, o);
        if (d) {
          d->mod_bus = tr.mod_out;
          d->Defaults();
          d->ApplyAll();
        }
        tr.inst[s][o] = d;
      }
      tr.fade[s] = new SculptFade;
      tr.last_choice[s] = SLOT_DEFS[s].noptions > 1 ? 1 : 0;
    }
  }
  // the MIX band stages before the mixer, so the update order follows the
  // signal
  for (int t = 0; t < NUM_TRACKS; t++) tracks[t].mix = new MixDevice(t);
  mixer = new SculptMixer;
  for (int t = 0; t < NUM_TRACKS; t++) {
    Track& tr = tracks[t];
    tr.mix->mixer = mixer;
    tr.mix->mod_bus = tr.mod_out;
    tr.mix->Defaults();
    tr.mix->val[1] = t ? 25 : -25; // pan the tracks apart a little
    tr.mix->ApplyAll();
    // start with ECHO on both tracks, a bit different on track 2
    tr.choice[ECHO] = 1;
  }
  Device* v2 = tracks[1].inst[ECHO][1];
  v2->val[EchoDevice::TIME] = 680;
  v2->val[EchoDevice::TONE] = 15;
  v2->mode[EchoDevice::SPREAD] = EchoDevice::DIFFUSE;

  tempo_clock = new ClockDevice;
  tempo_clock->mod_bus = tracks[0].mod_out;
  tempo_clock->Defaults();

  LoadFile();
}

FLASHMEM void AppSculpt::Repatch(int t) {
  using namespace sculpt;
  if (!active) return;
  Track& tr = tracks[t];
  auto& link = patch->link[t];
  AudioNoInterrupts();
  for (auto& slot : link) for (auto& c : slot) c.disconnect();
  for (auto& c : patch->to_mix[t]) c.disconnect();
  for (auto& c : patch->mix_out[t]) c.disconnect();
  AudioStream* prev = tr.input;
  if (tr.choice[SOURCE] == 1 && tr.reel) {  // REEL records and plays after the input
    for (int c = 0; c < 2; c++) link[SOURCE][c].connect(*tr.input, c, *tr.reel->in(), c);
    prev = tr.reel->out();
  } else if (tr.choice[SOURCE] == 2 && tr.player) {  // PLAYER instead of the input
    prev = tr.player->out();
  }
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
    Device* d = tr.current(s);
    if (!d || !d->in()) continue;
    // bypassed slots stay patched while their fade (or tail) runs; parked
    // ones only pass the dry signal through the fade
    AudioConnection* l = link[s];
    if (tr.parked >> s & 1) {
      for (int c = 0; c < 2; c++) (l++)->connect(*prev, c, *tr.fade[s], c);
      prev = tr.fade[s];
      continue;
    }
    AudioStream* feed = prev;
    if (s == ECHO && tr.gate) {
      for (int c = 0; c < 2; c++) (l++)->connect(*prev, c, *tr.gate, c);
      feed = tr.gate;
    }
    for (int c = 0; c < 2; c++) (l++)->connect(*feed, c, *d->in(), c);
    for (int c = 0; c < 2; c++) (l++)->connect(*prev, c, *tr.fade[s], c);
    for (int c = 0; c < 2; c++) (l++)->connect(*d->out(), c, *tr.fade[s], 2 + c);
    prev = tr.fade[s];
  }
  // through the track's MIX band stage into the mixer
  for (int c = 0; c < 2; c++) patch->to_mix[t][c].connect(*prev, c, *tr.mix->in(), c);
  for (int c = 0; c < 2; c++) patch->mix_out[t][c].connect(*tr.mix->out(), c, *mixer, t * 2 + c);
  AudioInterrupts();
  ApplyBypass(t);
}

// bypass targets for the track's fades: dry only, or for ECHO dry plus
// the device's output with its input closed, so the tail rings out
FLASHMEM void AppSculpt::ApplyBypass(int t) {
  using namespace sculpt;
  Track& tr = tracks[t];
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
    if (!tr.fade[s]) continue;
    const bool bp = tr.bypassed(s);
    if (s == ECHO) {
      tr.fade[s]->Set(bp ? 1.0f : 0.0f, 1.0f);
      tr.gate->Set(bp ? 0.0f : 1.0f);
    } else {
      // a device may mix itself with the dry signal (SPRAY: LEVEL)
      const Device* d = tr.current(s);
      const float w = d ? constrain(d->wet, 0.0f, 1.0f) : 1.0f;
      if (bp) tr.fade[s]->Set(1.0f, 0.0f);
      else if (w >= 1.0f) tr.fade[s]->Set(0.0f, 1.0f);
      else tr.fade[s]->Set(cosf(w * float(M_PI_2)), sinf(w * float(M_PI_2)));
    }
  }
  tr.mix->SetBypass(tr.bypass >> MIX_COL & 1);
}

FLASHMEM void AppSculpt::SetChoice(int t, int s, int option) {
  using namespace sculpt;
  Track& tr = tracks[t];
  option = constrain(option, 0, SLOT_DEFS[s].noptions - 1);
  if (option == tr.choice[s]) return;
  Device* old = tr.current(s);
  tr.choice[s] = option;
  if (option) tr.last_choice[s] = option;
  tr.parked &= ~(1 << s);  // a new device starts patched; parks again if bypassed
  tr.park_at[s] = millis() + 40;
  if (!active) return;
  Device* now = tr.current(s);
  if (now) now->Activate();
  Repatch(t);
  if (old) old->Deactivate();
}

FLASHMEM void AppSculpt::ConnectTrack(int t) {
  using namespace sculpt;
  AudioNoInterrupts();
  for (int c = 0; c < 2; c++)
    patch->in[t][c].connect(OC::AudioIO::InputStream(), c, *tracks[t].input, c);
  AudioInterrupts();
  for (int s = SOURCE; s < SLOT_COUNT; s++) {
    if (Device* d = tracks[t].current(s)) d->Activate();
  }
  Repatch(t);
}

FLASHMEM void AppSculpt::DisconnectTrack(int t) {
  using namespace sculpt;
  AudioNoInterrupts();
  for (auto& c : patch->in[t]) c.disconnect();
  for (auto& slot : patch->link[t]) for (auto& c : slot) c.disconnect();
  for (auto& c : patch->to_mix[t]) c.disconnect();
  for (auto& c : patch->mix_out[t]) c.disconnect();
  AudioInterrupts();
  // disconnected devices stop updating; now they can drop their buffers
  for (int s = SOURCE; s < SLOT_COUNT; s++) {
    if (Device* d = tracks[t].current(s)) d->Deactivate();
  }
}

// the same device on the other track (only meaningful while linked)
FLASHMEM sculpt::Device* AppSculpt::Twin(sculpt::Device* d) const {
  using namespace sculpt;
  if (!linked || !d) return nullptr;
  Device* tw = FocusOn(1 - track);
  return (tw && tw != d && &tw->def == &d->def) ? tw : nullptr;  // shared: no twin
}

// copy one parameter to the twin; input source and pan stay per track
FLASHMEM void AppSculpt::Mirror(sculpt::Device* d, int p) const {
  using namespace sculpt;
  Device* tw = Twin(d);
  if (!tw) return;
  if (!focus_mod && col == SOURCE && p == 0) return;
  if (!focus_mod && col == MIX_COL && p == 1) return;
  tw->val[p] = d->val[p];
  tw->mode[p] = d->mode[p];
  tw->mod_amt[p] = d->mod_amt[p];
  tw->mod_in[p] = d->mod_in[p];
  tw->mod_bus_sel[p] = d->mod_bus_sel[p];
  tw->mod_bi = (tw->mod_bi & ~(1 << p)) | (d->mod_bi & (1 << p));
  tw->mod_amt2[p] = d->mod_amt2[p];
  tw->mod_bus2[p] = d->mod_bus2[p];
  tw->mod_bi2 = (tw->mod_bi2 & ~(1 << p)) | (d->mod_bi2 & (1 << p));
  tw->Refresh(p);
}

FLASHMEM void AppSculpt::SetModType(int t, int i, int type) {
  using namespace sculpt;
  Modulator** row = ModRow(t, i);
  type = constrain(type, 0, MOD_TYPES - 1);
  if (type && !row[type]) {
    Modulator* m = MakeModulator(type);
    // a shared one's own settings take MOD A-D from track L
    m->mod_bus = tracks[i < NUM_MODS ? t : 0].mod_out;
    m->Defaults();
    row[type] = m;
  }
  ModType(t, i) = type;
  if (!type) {
    if (i < NUM_MODS) tracks[t].mod_out[i] = -1.0f;  // a setting listening to it stays put
    else for (auto& tr : tracks) tr.mod_out[i] = -1.0f;
  }
}

FLASHMEM void AppSculpt::SetChoiceLinked(int s, int option) {
  SetChoice(track, s, option);
  if (linked) SetChoice(1 - track, s, sculpt::tracks[track].choice[s]);
}

// one-click bypass of a slot: the device stays (and keeps its settings and
// page) but the track's audio goes around it
// unpatch devices that have been bypassed long enough (fade done, tail rung
// out), so they stop using CPU
FLASHMEM void AppSculpt::ParkBypassed(int t) {
  using namespace sculpt;
  Track& tr = tracks[t];
  uint8_t fresh = 0;
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
    const uint8_t bit = 1 << s;
    if (!(tr.bypass & bit) || (tr.parked & bit) || !tr.current(s)) continue;
    if (int32_t(millis() - tr.park_at[s]) < 0) continue;
    fresh |= bit;
  }
  if (!fresh) return;
  tr.parked |= fresh;
  Repatch(t);
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++)
    if (fresh >> s & 1) tr.current(s)->Parked();
}

FLASHMEM void AppSculpt::ToggleBypass(int s) {
  using namespace sculpt;
  const uint8_t bit = 1 << s;
  const bool on = !(tracks[track].bypass & bit);
  for (int t = 0; t < NUM_TRACKS; t++) {
    if (t != track && !linked) continue;
    Track& tr = tracks[t];
    if (on) {
      tr.bypass |= bit;
      const Device* d = s < SLOT_COUNT ? tr.current(s) : nullptr;
      tr.park_at[s < SLOT_COUNT ? s : 0] = millis() + (d ? d->TailMs() : 0);
    } else {
      tr.bypass &= ~bit;
      if (tr.parked & bit) {  // back in the chain before it fades in
        tr.parked &= ~bit;
        Repatch(t);
      }
    }
    if (active) ApplyBypass(t);
  }
}

FLASHMEM void AppSculpt::ToggleLink() {
  using namespace sculpt;
  linked = !linked;
  if (!linked) return;
  const int master = track;
  // the track under the cursor is the master: copy it to the other one
  Track& a = tracks[master];
  Track& b = tracks[1 - master];
  SetChoice(1 - master, SOURCE, a.choice[SOURCE]);
  if (a.reel && b.reel) {
    const int16_t keep_in = b.reel->val[ReelDevice::IN];  // the input stays per track
    b.reel->CopyFrom(*a.reel);
    b.reel->val[ReelDevice::IN] = keep_in;
    b.reel->ApplyAll();
  }
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
    SetChoice(1 - master, s, a.choice[s]);
    b.last_choice[s] = a.last_choice[s];
    for (int o = 0; o < MAX_OPTIONS; o++) {
      Device* from = a.inst[s][o];
      Device* to = b.inst[s][o];
      if (!from || !to) continue;
      to->CopyFrom(*from);
    }
  }
  b.bypass = a.bypass;
  b.parked = 0;
  for (auto& at : b.park_at) at = millis() + 40;
  Repatch(1 - master);
  if (active) ApplyBypass(1 - master);
  for (int i = 0; i < NUM_MODS; i++) {
    SetModType(1 - master, i, a.mod_type[i]);
    for (int type = 1; type < MOD_TYPES; type++) {
      if (!a.mods[i][type]) continue;
      if (!b.mods[i][type]) {
        SetModType(1 - master, i, type);
        SetModType(1 - master, i, a.mod_type[i]);
      }
      b.mods[i][type]->CopyFrom(*a.mods[i][type]);
    }
  }
  for (int p = 1; p < b.input->num_params(); p++) b.input->val[p] = a.input->val[p];
  b.input->ApplyAll();
  b.mix->val[0] = a.mix->val[0]; // level
  b.mix->val[2] = a.mix->val[2]; // mute
  b.mix->ApplyAll();
  track = 0; // the cursor lives on row 1 while linked
}

FLASHMEM void AppSculpt::Resume() {
  // SPRAY: at most 8 overlapping grains while Sculpt runs (CPU peaks)
  AudioEffectClouds::grain_limit = 8;
  using namespace sculpt;
  if (!audio_hw() || active) return;
  if (!patch) patch = new Patch;
  active = true;
  for (int t = 0; t < NUM_TRACKS; t++) ConnectTrack(t);
  OC::AudioIO::ClaimOutput(*mixer);
  OC::AudioIO::ResyncInput();  // L and R in the same order on every start
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  stats_timer = 0;
}

FLASHMEM void AppSculpt::Suspend() {
  // the Quadrants applets get all their grains back
  AudioEffectClouds::grain_limit = AudioEffectClouds::MAX_GRAINS;
  using namespace sculpt;
  if (!active) return;
  OC::AudioIO::ReleaseOutput();
  for (int t = 0; t < NUM_TRACKS; t++) DisconnectTrack(t);
  active = false;
  l_held = false;
  if (dirty) SaveFile();
}

// MIDI is normally read by Quadrants; while Sculpt runs it feeds the same
// shared MIDI maps itself (no thru, no SysEx, no program changes)
template <typename T>
FLASHMEM static void SculptReadMIDI(T& device) {
  for (int n = 0; n < 32 && device.read(); n++) {
    const HS::MIDIMessage msg = {
      device.getChannel(), device.getType(), device.getData1(), device.getData2()
    };
    if (msg.message == midi::SystemExclusive || msg.message == midi::ProgramChange) continue;
    if (msg.message == midi::Clock) sculpt::midi_clocks++;
    if (sculpt::tempo_clock && sculpt::tempo_clock->midi()) {
      if (msg.message == midi::Start) sculpt::ClockStart();
      else if (msg.message == midi::Continue) sculpt::clock_running = true;
      else if (msg.message == midi::Stop) sculpt::ClockStop();
    }
    const bool realtime = msg.message == midi::Clock || msg.message == midi::Start
      || msg.message == midi::Stop;
    if (realtime || (msg.message >> 4) != 0xF) HS::frame.MIDIState.ProcessMIDIMsg(msg);
  }
}

FLASHMEM void AppSculpt::ReadMIDI() {
  SculptReadMIDI(usbMIDI);
  SculptReadMIDI(usbHostMIDI[0]);
  SculptReadMIDI(usbHostMIDI[1]);
  SculptReadMIDI(MIDI1);
}

// point the setting at a free MIDI map (or its current one) and let the next
// CC or note teach it
FLASHMEM void AppSculpt::MidiLearn(sculpt::Device* d) {
  HS::CVInputMap& in = d->mod_in[cursor];
  auto& maps = HS::frame.MIDIState.mapping;
  if (in.source_type() != HS::CVInputMap::TYPE_MIDI) {
    int free_map = -1;
    for (int i = 0; i < MIDIMAP_MAX && free_map < 0; i++) {
      if (!maps[i].enabled() && !maps[i].learning()) free_map = i;
    }
    if (free_map < 0) return;
    in.SetMidiMap(free_map);
  }
  in.AutoLearn();
  if (!d->mod_amt[cursor]) d->mod_amt[cursor] = 50;
  d->Refresh(cursor);
  Mirror(d, cursor);
}

FLASHMEM void AppSculpt::ModulateAll() {
  using namespace sculpt;
  ModCtx ctx;
  ctx.dt = std::min(tick_timer * 1e-6f, 0.05f);
  tick_timer = 0;
  __disable_irq();
  ctx.edges = trig_latch;
  trig_latch = 0;
  __enable_irq();
  ctx.in_level = ctx.out_level = 0.0f;
  tempo_clock->Run(ctx);
  if (fabsf(tempo_bpm - applied_tempo) > applied_tempo * 0.002f) {
    applied_tempo = tempo_bpm;
    ForEachDevice([](Device* d) { d->TempoChanged(); });
  }
  // the shared ones: FOLLOW hears the louder track
  ctx.in_level = std::max(tracks[0].input->level, tracks[1].input->level);
  ctx.out_level = std::max(mixer->level[0], mixer->level[1]);
  for (int i = 0; i < NUM_MODS; i++) {
    Modulator* m = shared_mods[i][shared_type[i]];
    if (!m) continue;
    m->Run(ctx);
    for (auto& tr : tracks) tr.mod_out[NUM_MODS + i] = m->out;
  }
  for (int t = 0; t < NUM_TRACKS; t++) {
    Track& tr = tracks[t];
    ctx.in_level = tr.input->level;
    ctx.out_level = mixer->level[t];
    for (int i = 0; i < NUM_MODS; i++) {
      Modulator* m = tr.mod(i);
      if (!m) continue;
      m->Run(ctx);
      tr.mod_out[i] = m->out;
    }
  }
  for (auto& tr : tracks) {
    for (int s = SOURCE; s < SLOT_COUNT; s++) {
      if (Device* d = tr.current(s)) {
        d->Modulate();
        d->RunTriggers(ctx.edges);
      }
    }
    tr.mix->Modulate();
    tr.mix->RunTriggers(ctx.edges);
    for (int i = 0; i < NUM_MODS; i++) {
      if (Modulator* m = tr.mod(i)) m->RunTriggers(ctx.edges);
    }
  }
  for (int i = 0; i < NUM_MODS; i++) {
    if (Modulator* m = shared_mods[i][shared_type[i]]) m->RunTriggers(ctx.edges);
  }
  WriteOutputs();
}

// the CV outputs: 0..1 -> 0..10 V. They go out through the shared IO frame,
// which the core writes to the jacks every tick (writing the DAC directly
// was overwritten by it)
FLASHMEM float AppSculpt::OutLevel(int o) const {
  using namespace sculpt;
  const int s = cv_out_src[o];
  float v = -1.0f;
  if (s >= OUT_CLOCK) v = clock_running && tempo_clock->out < 0.5f ? 1.0f : 0.0f;
  else if (s >= OUT_SHARED) v = tracks[0].mod_out[NUM_MODS + s - OUT_SHARED];
  else if (s >= OUT_R) v = tracks[1].mod_out[s - OUT_R];
  else if (s >= OUT_L) v = tracks[0].mod_out[s - OUT_L];
  return constrain(v, 0.0f, 1.0f);  // empty slot (-1) or off: 0 V
}

FLASHMEM void AppSculpt::WriteOutputs() {
  for (int o = 0; o < sculpt::NUM_CV_OUTS && o < DAC_CHANNEL_COUNT; o++)
    HS::frame.Out(DAC_CHANNEL(o), int(OutLevel(o) * 8.5f * ONE_OCTAVE));  // measured: 8.5 "volts" give 10 V at the jack
}

FLASHMEM void AppSculpt::MainLoop() {
  if (active) ReadMIDI();
  {
    using namespace sculpt;
    // REEL: A held 2 s starts writing the loop; the files go in steps
    const uint32_t ms = loop_save_held ? millis() - loop_save_hold : 0;
    for (int t = 0; t < NUM_TRACKS; t++) {
      Track& tr = tracks[t];
      if (!tr.reel) continue;
      const bool mine = t == track || linked;
      tr.reel->save_hold = mine && loop_save_held && ms > 250 ? std::min(1.0f, ms / 2000.0f) : 0.0f;
      if (mine && loop_save_held && ms >= 2000 && !tr.writer.busy) {
        tr.reel->saved_name[0] = 0;
        if (tr.writer.Begin(tr.reel->LoopData(0), tr.reel->LoopData(1), tr.reel->LoopLength()))
          tr.reel->saving = 0.0f;
        else
          tr.reel->saved_at = millis() | 1;  // shows SD ERROR
      }
      if (tr.writer.busy) {
        const bool done = tr.writer.Step();
        tr.reel->saving = tr.writer.Progress();
        if (done) {
          tr.reel->saving = -1.0f;
          snprintf(tr.reel->saved_name, sizeof(tr.reel->saved_name), "%s", tr.writer.name);
          tr.reel->saved_at = millis() | 1;
          if (tr.player) tr.player->Select(tr.writer.name);  // ready in PLAYER
        }
      }
      if (tr.player && tr.choice[SOURCE] == 2) tr.player->Idle();
    }
    if (loop_save_held && ms >= 2000) loop_save_held = false;
  }
  if (save_held && millis() - save_hold >= 2000) {
    save_held = false;
    SaveFile();  // a short audio stop while the flash is written
    saved_at = millis() | 1;
  }
  {
    using namespace sculpt;
    const bool held = clear_held;
    const uint32_t ms = held ? millis() - clear_hold : 0;
    // only after a moment, so a short press (stop) shows nothing
    const float hold = held && ms > 250 ? std::min(1.0f, ms / 3000.0f) : 0.0f;
    for (int t = 0; t < NUM_TRACKS; t++) {
      if ((t != track && !linked) || !tracks[t].reel) continue;
      tracks[t].reel->hold = hold;
      if (held && ms >= 3000) tracks[t].reel->ClearLoop();
    }
    if (held && ms >= 3000) clear_held = false;
  }
  if (active && mod_timer >= 1000) { // follow the sources about once per ms
    mod_timer = 0;
    ModulateAll();
    for (int t = 0; t < sculpt::NUM_TRACKS; t++) {
      ApplyBypass(t);  // follows SPRAY LEVEL
      ParkBypassed(t);
    }
  }
  if (!active || stats_timer < 1000) return;
  stats_timer = 0;
  const float usage = AudioProcessorUsageMax();
  cpu = int(usage + 0.5f);
  Serial.printf("Sculpt: AudioProcessorUsageMax %d.%d%%  mem %u/%d blocks\n",
    int(usage), int(usage * 10) % 10, AudioMemoryUsageMax(), OC::AudioIO::AUDIO_MEMORY);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  Serial.printf("  load:%s\n", sculpt::load_note);
  // modulated settings: value, result, both sources (amount, MOD, u/b)
  for (int t = 0; t < sculpt::NUM_TRACKS; t++) {
    const sculpt::Track& tr = sculpt::tracks[t];
    Serial.printf("  %c mods", t ? 'R' : 'L');
    for (int i = 0; i < sculpt::MOD_BUSES; i++) {
      const sculpt::Modulator* m = ModAt(t, i);
      Serial.printf(" %c:%s=%d.%02d", 'A' + i, m ? m->def.tag : "-", int(tr.mod_out[i]), int(tr.mod_out[i] * 100) % 100);
    }
    Serial.println();
    ForEachDevice([&](sculpt::Device* d) {
      if (d->mod_bus != tr.mod_out) return;
      for (int p = 0; p < d->num_params(); p++) {
        if (!d->Modded(p)) continue;
        Serial.printf("  %c %s %s val %d cur %d | 1: %+d%% bus %d src %04x %c | 2: %+d%% bus %d %c\n", t ? 'R' : 'L',
          d->def.tag, d->param(p).name, d->val[p], d->cur[p], d->mod_amt[p], d->mod_bus_sel[p], d->mod_in[p].Pack(),
          d->Bipolar(p) ? 'b' : 'u', d->mod_amt2[p], d->mod_bus2[p], d->Bipolar2(p) ? 'b' : 'u');
      }
    });
  }

  // per device: own stream + owned streams (e.g. ECHO core + delay lines)
  using namespace sculpt;
  for (int t = 0; t < NUM_TRACKS; t++) {
    Serial.printf("  %c", t ? 'R' : 'L');
    for (int s = SOURCE; s < SLOT_COUNT; s++) {
      Device* d = tracks[t].current(s);
      if (!d) continue;
      float parts[2];
      d->CpuMax(parts);
      Serial.printf("  %s %d.%d+%d.%d", d->def.tag,
        int(parts[0]), int(parts[0] * 10) % 10, int(parts[1]), int(parts[1] * 10) % 10);
    }
    Serial.println();
  }
  const float mx = mixer->processorUsageMax();
  mixer->processorUsageMaxReset();
  Serial.printf("  MIX %d.%d\n", int(mx), int(mx * 10) % 10);
}

/////////////////////////////////////////////////////////////////
// Controls
/////////////////////////////////////////////////////////////////
FLASHMEM void AppSculpt::OnButton(const UI::Event &event) {
  using namespace sculpt;
  dirty = true;
  if (event.control == OC::CONTROL_BUTTON_Z) {
    // anywhere in the app: clock start / stop
    if (event.type == UI::EVENT_BUTTON_PRESS) clock_running ? ClockStop() : ClockStart();
    return;
  }
  if (event.control == OC::CONTROL_BUTTON_L) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      l_held = true;
      l_used = false;
    } else if (event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      l_held = false;
      // L held (and not used with R enc for a mode): the action triggers page
      if (!l_used && view == DEVICE && !editing) {
        trig_mode = !trig_mode;
        mod_mode = false;
      }
    } else if (event.type == UI::EVENT_BUTTON_PRESS) {
      l_held = false;
      if (l_used) return;
      // L push: modulation on / off (starting with MOD1)
      if (view == DEVICE) {
        if (trig_mode) trig_mode = false;
        else if (!mod_mode) { mod_mode = true; mod_slot = 0; }
        else mod_mode = false;
        editing = false;
      }
    }
    return;
  }
  // REEL: B held for 3 s clears the loop (see MainLoop); followed by the
  // button's own down / release events
  // REEL: A held for 2 s writes the loop to the SD card (see MainLoop)
  if (event.control == OC::CONTROL_BUTTON_UP) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      loop_save_held = view == DEVICE && !focus_mod && col == SOURCE && tracks[track].choice[SOURCE] == 1;
      loop_save_hold = millis();
    } else if (event.type == UI::EVENT_BUTTON_PRESS || event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      loop_save_held = false;
    }
  }
  // chain view: Y held for 2 s saves the settings (see MainLoop)
  if (event.control == OC::CONTROL_BUTTON_Y) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      save_held = view == CHAIN;
      save_hold = millis();
    } else if (event.type == UI::EVENT_BUTTON_PRESS || event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      save_held = false;
    }
  }
  if (event.control == OC::CONTROL_BUTTON_DOWN) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      clear_held = view == DEVICE && !focus_mod && col == SOURCE && tracks[track].choice[SOURCE] == 1;
      clear_hold = millis();
    } else if (event.type == UI::EVENT_BUTTON_PRESS || event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      clear_held = false;
    }
  }
  if (event.control == OC::CONTROL_BUTTON_UP || event.control == OC::CONTROL_BUTTON_DOWN) {
    constexpr uint16_t both = OC::CONTROL_BUTTON_UP | OC::CONTROL_BUTTON_DOWN;
    if (event.type == UI::EVENT_BUTTON_DOWN && (event.mask & both) == both) {
      ToggleLink();
      chord_releases = 2; // neither release should fire an action
      return;
    }
    if (event.type == UI::EVENT_BUTTON_PRESS || event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      if (chord_releases) {
        chord_releases--;
        return;
      }
    }
  }
  // X held in MOD1: MIDI learn (the release that follows does nothing)
  if (event.control == OC::CONTROL_BUTTON_X && view == DEVICE && mod_mode) {
    if (event.type == UI::EVENT_BUTTON_LONG_PRESS && !mod_slot && Focused()) MidiLearn(Focused());
    if (event.type != UI::EVENT_BUTTON_PRESS) return;
  }
  if (event.type != UI::EVENT_BUTTON_PRESS) return;

  if (view == MODS) focus_mod = true;
  else if (view == CHAIN) focus_mod = false;
  Device* d = Focused();
  // PLAYER: R push on FILE opens the browser; in it R enters / picks, Y
  // goes up (closes at the top)
  if (view == DEVICE && !focus_mod && col == SOURCE && d && d == tracks[track].player) {
    PlayerDevice* pl = tracks[track].player;
    if (pl->browsing) {
      if (event.control == OC::CONTROL_BUTTON_R) pl->BrowseEnter();
      else if (event.control == OC::CONTROL_BUTTON_Y) pl->BrowseBack();
      return;
    }
    if (event.control == OC::CONTROL_BUTTON_R && cursor == PlayerDevice::FILE_ && !mod_mode) {
      pl->OpenBrowser();
      return;
    }
  }
  switch (event.control) {
    case OC::CONTROL_BUTTON_R:
      if (view == MODS) {
        focus_mod = true;
        if (Focused()) {
          view = DEVICE;
          cursor = 0;
          editing = mod_mode = trig_mode = false;
        }
      } else if (view == CHAIN) {
        focus_mod = false;
        if (Focused()) {
          view = DEVICE;
          cursor = 0;
          editing = mod_mode = trig_mode = false;
        }
      } else if (d && mod_mode) {
        mod_slot ^= 1;  // MOD1 <-> MOD2
      } else {
        editing = !editing;
        edit_mode = false;
      }
      break;
    case OC::CONTROL_BUTTON_Y:
      if (view == DEVICE && trig_mode) trig_mode = false;
      else if (view == MODS && back.on) BackToDevice();
      else view = (view == DEVICE && focus_mod) ? MODS : CHAIN;
      editing = mod_mode = false;
      break;
    case OC::CONTROL_BUTTON_X:
      // device page: the MOD view, coming back to this page afterwards
      if (view == DEVICE) {
        back = {true, focus_mod, mod_cursor, cursor};
        view = MODS;
        editing = mod_mode = trig_mode = false;
      }
      else if (view == CHAIN) { view = MODS; back.on = false; }
      else if (view == MODS && back.on) BackToDevice();
      else if (view == MODS) view = CHAIN;
      break;
    case OC::CONTROL_BUTTON_UP:
    case OC::CONTROL_BUTTON_DOWN:
      if (view == CHAIN && event.control == OC::CONTROL_BUTTON_DOWN) {
        // bypass toggle of the slot under the cursor (MIX: its band stage);
        // a slot set to no device gets its last one back
        Track& tr = tracks[track];
        if (col == MIX_COL || (col > SOURCE && tr.choice[col])) ToggleBypass(col);
        else if (col > SOURCE) SetChoiceLinked(col, tr.last_choice[col]);
      } else if (d && event.control == OC::CONTROL_BUTTON_DOWN && BypassKey()) {
        ToggleBypass(col);
      } else if (d) {
        const int a = event.control == OC::CONTROL_BUTTON_UP ? 0 : 1;
        const bool was = d->ActionLit(a);
        d->Action(a);
        // linked: toggles end up in the same state, one-shots fire on both
        if (Device* tw = Twin(d)) {
          const bool now = d->ActionLit(a);
          if (was == now || tw->ActionLit(a) != now) tw->Action(a);
        }
      }
      break;
    default: break;
  }
}

FLASHMEM void AppSculpt::OnEncoder(const UI::Event &event) {
  using namespace sculpt;
  dirty = true;
  const int dir = event.value;
  const bool left = event.control == OC::CONTROL_ENCODER_L;
  if (view == DEVICE && !focus_mod && col == SOURCE && tracks[track].player &&
      tracks[track].player->browsing && Focused() == tracks[track].player) {
    tracks[track].player->BrowseMove(dir);  // either knob scrolls the browser
    return;
  }
  if (view == MODS) {
    if (!left) {
      mod_cursor = constrain(mod_cursor + dir, 0, LAST_CURSOR);
    } else if (mod_cursor >= OUT_CURSOR) {
      uint8_t& s = cv_out_src[mod_cursor - OUT_CURSOR];
      s = constrain(s + dir, 0, OUT_SOURCES - 1);
    } else if (mod_cursor < CLK_CURSOR) {
      SetModType(track, mod_cursor, ModType(track, mod_cursor) + dir);
      if (linked && mod_cursor < NUM_MODS) SetModType(1 - track, mod_cursor, ModType(track, mod_cursor));
    }
    return;
  }
  if (view == CHAIN) {
    if (!left) {
      // linked: both rows are the same, so the cursor stays on row 1
      const int rows = linked ? 1 : NUM_TRACKS;
      const int idx = constrain(track * COLS + col + dir, 0, rows * COLS - 1);
      track = idx / COLS;
      col = idx % COLS;
    } else if (col == MIX_COL) {
      tracks[track].mix->Nudge(0, dir); // level
      Mirror(tracks[track].mix, 0);
    } else {
      SetChoiceLinked(col, tracks[track].choice[col] + dir);  // SOURCE: INPUT / REEL
    }
    return;
  }

  Device* d = Focused();
  if (!d) { view = CHAIN; return; }
  if (trig_mode) {
    uint8_t& src = d->act_trig[left ? 0 : 1];
    src = constrain(src + dir, 0, TRIG_SOURCES - 1);
    if (Device* tw = Twin(d)) tw->act_trig[left ? 0 : 1] = src;
    return;
  }
  if (mod_mode) {
    int8_t& amt = mod_slot ? d->mod_amt2[cursor] : d->mod_amt[cursor];
    if (!left) amt = constrain(amt + dir, -100, 100);
    else StepModOption(d, cursor, dir);
    d->Refresh(cursor);
    Mirror(d, cursor);
  } else if (left) {
    // the left encoder picks the mode of the setting under the cursor
    if (d->param(cursor).nmodes) {
      d->NudgeMode(cursor, dir);
      Mirror(d, cursor);
      edit_mode = true; // show the mode name for a moment
      mode_shown = 0;
    }
  } else if (!editing) {
    cursor = constrain(cursor + dir, 0, d->num_params() - 1);
  } else if (l_held && d->param(cursor).nmodes) {
    d->NudgeMode(cursor, dir);
    Mirror(d, cursor);
    l_used = true;
  } else {
    d->Nudge(cursor, dir);
    Mirror(d, cursor);
    edit_mode = false;
  }
}

/////////////////////////////////////////////////////////////////
// Screens
/////////////////////////////////////////////////////////////////
FLASHMEM void AppSculpt::UpdateMixLevels() const {
  using namespace sculpt;
  if (!mixer) return;
  float pk[2][2];
  mixer->ReadPeaks(pk);
  for (int t = 0; t < NUM_TRACKS; t++) {
    tracks[t].mix->level[0] = pk[t][0];
    tracks[t].mix->level[1] = pk[t][1];
  }
}

// the track on screen is muted (while linked: either of them)
FLASHMEM bool AppSculpt::Muted() const {
  using namespace sculpt;
  for (int t = 0; t < NUM_TRACKS; t++)
    if ((t == track || linked) && tracks[t].mix && tracks[t].mix->Muted()) return true;
  return false;
}

// on an effect slot's page B (DOWN) bypasses the slot (ECHO's CLR stays
// reachable through a trigger); on OUTPUT it bypasses the band stage
FLASHMEM bool AppSculpt::BypassKey() const {
  using namespace sculpt;
  if (view != DEVICE || focus_mod || col <= SOURCE || col > MIX_COL) return false;
  return Focused() != nullptr;
}

FLASHMEM void AppSculpt::DrawChain() const {
  using namespace sculpt;
  // boxes are drawn one pixel wider than the spacing so neighbours share a
  // border; 19 px inside each. Glyphs are 6 px with a blank first column.
  // Inside: the slot's name and a small live picture of its device.
  constexpr int bw = 20, bh = 26, x0 = 7;
  for (int t = 0; t < NUM_TRACKS; t++) {
    const int y = t * (bh + 1);
    // L / R drawn by hand at the left edge (text can't start left of x 0)
    static const uint8_t glyph[2][7] = {
      {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},  // L
      {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},  // R
    };
    for (int r = 0; r < 7; r++)
      for (int c = 0; c < 5; c++)
        if (glyph[t][r] >> (4 - c) & 1) gfxPixel(c, y + 9 + r);
    for (int c = 0; c < COLS; c++) {
      const int x = x0 + c * bw;
      const Track& tr = tracks[t];
      Device* d = tr.at(c);
      if (tr.shown_bypassed(c)) d = nullptr;  // drawn like an empty slot
      if (d) gfxFrame(x, y, bw + 1, bh);
      else gfxFrame(x, y, bw + 1, bh, true);

      // the slot's name at the top, always; underneath its device's little
      // picture, or BP when bypassed
      // SOURCE is named after what it does: IN (input) or REC (REEL)
      const char* tag = c == MIX_COL ? PSTR("OUT")
                      : c == SOURCE ? (tr.choice[SOURCE] == 1 ? PSTR("REC")
                                       : tr.choice[SOURCE] == 2 ? PSTR("PLY") : PSTR("IN"))
                      : SLOT_DEFS[c].tag;
      gfxPrint(x + 1 + (bw - 1 - 6 * int(strlen(tag))) / 2, y + 3, tag);
      if (!d) gfxPrint(x + 1 + (bw - 1 - 12) / 2, y + 14, PSTR("BP"));
      else if (d->ok()) d->DrawMini(x + 2, y + 13, bw - 3, 10);
      else gfxPrint(x + 5, y + 14, PSTR("!!"));
      if (t == track && c == col) gfxInvert(x + 1, y + 1, bw - 1, bh - 2);
    }
  }

  if (linked) {
    // chain link between the track numbers
    gfxFrame(0, 21, 5, 6);
    gfxFrame(0, 25, 5, 6);
  }

  // status line: what is under the cursor, CPU load
  const Track& tr = tracks[track];
  Device* d = tr.at(col);
  if (!linked) gfxPrint(0, 56, track ? PSTR("R") : PSTR("L"));
  // the slot name; SOURCE names its source (INPUT, REEL or PLAYER)
  gfxPrint(8, 56, col == MIX_COL ? PSTR("OUTPUT")
                  : col == SOURCE && d ? d->def.name : SLOT_DEFS[col].name);
  // bypass (when on) at the right end; the CPU load always left of its place
  const bool bp = !d || tr.shown_bypassed(col);
  const int right = 128 - 6 * 6 - 6;
  if (bp) {
    gfxPrint(128 - 6 * 6, 56, PSTR("bypass"));
  }
  if (!audio_hw()) {
    gfxPrint(right - 24, 56, PSTR("noIO"));
  } else {
    char buf[6];
    snprintf(buf, sizeof(buf), "%d%%", cpu);
    gfxPrint(right - 6 * int(strlen(buf)), 56, buf);
  }
}

// text centred in a 32 px parameter column
FLASHMEM static void PrintCentred(int x, int y, const char* str) {
  const int w = 6 * int(strlen(str));
  gfxPrint(x + std::max(0, (32 - w) / 2), y, str);
}

FLASHMEM void AppSculpt::DrawDevice() const {
  using namespace sculpt;
  Device* d = Focused();
  if (!d) return;
  const int n = d->num_params();

  // header: track, device, action labels, page; text one pixel down so the
  // inverted boxes have a white row above it. A long device name gives way
  // to its tag when MOD is shown next to it.
  // a shared modulator (MOD E-H) belongs to both tracks: L+R
  const bool both = linked || SharedFocus();
  const int name_x = both ? 24 : 12;
  const char* name = mod_mode && name_x + 6 * int(strlen(d->def.name)) > 62 ? d->def.tag : d->def.name;
  if (both) {
    gfxPrint(1, 1, PSTR("L+R"));
    gfxInvert(0, 0, 21, 9);
  } else {
    gfxPrint(1, 1, track ? PSTR("R") : PSTR("L"));
    gfxInvert(0, 0, 9, 9);
  }
  if (focus_mod && mod_cursor < CLK_CURSOR) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%c %s", 'A' + mod_cursor, d->def.tag); // short: actions follow
    gfxPrint(name_x, 1, buf);
  } else {
    gfxPrint(name_x, 1, name);
    // e.g. ECHO DELAY / ECHO REVRB; left out where it would run into a
    // BYP or MUTE label that is showing
    const char* sub = d->def.page_title[cursor / 4];
    const int sx = name_x + 6 * int(strlen(name)) + 6;
    const bool busy = (BypassKey() && (tracks[track].bypass >> col & 1)) || Muted();
    if (!mod_mode && sub[0] && (sx + 6 * int(strlen(sub)) <= 81 || !busy)) gfxPrint(sx, 1, sub);
  }
  if (mod_mode) {
    // MOD1 / MOD2: the first or the second source (L push steps through)
    gfxPrint(64, 1, mod_slot ? PSTR("MOD2") : PSTR("MOD1"));
    gfxInvert(63, 0, 26, 9);
  }
  if (trig_mode) {
    // which outside signal presses A / B for you: a gate (TR1-4), a CV
    // input used as a gate, or a MIDI mapping; the knobs pick them
    gfxFrame(0, 10, 128, 32);
    for (int a = 0; a < 2; a++) {
      char src[8], buf[24];
      FormatTrig(d->act_trig[a], src, sizeof(src));
      const char* act = d->def.actions[a][0] ? d->def.actions[a] : "-";
      snprintf(buf, sizeof(buf), "%s %-5s <- %s", a ? "B" : "A", act, src);
      gfxPrint(4, 13 + a * 10, buf);
    }
    gfxPrint(4, 33, PSTR("L knob: A  R knob: B"));
  }  // BYP shows only while the slot is bypassed
  const bool byp_on = BypassKey() && (tracks[track].bypass >> col & 1);
  const bool mute_turn = !mod_mode && Muted()
      && (!((focus_mod && d->def.actions[1][0]) || byp_on) || (millis() / 600) % 2);
  for (int a = 0; a < 2 && !mod_mode; a++) {
    const bool byp = a == 1 && byp_on;
    // slot devices keep their actions (ECHO: FRZ / CLR) off the header;
    // the modulators show theirs (RST, STP, TRG)
    const char* label = byp ? PSTR("BYP") : focus_mod ? d->def.actions[a] : "";
    if (!label[0] || (a == 1 && mute_turn)) continue;
    const int x = 56 + a * 26; // clear of the page number at 110
    gfxPrint(x, 1, label);
    // lit actions (e.g. a modulator's STP) are inverted
    const int lw = 6 * int(strlen(label)) + 2;
    if (!byp && d->ActionLit(a)) gfxInvert(x - 1, 0, lw, 9);
  }
  // a muted track shows MUTE blinking on every page, in the second action
  // place; where a device has its own action there, the two take turns
  if (mute_turn && (millis() / 300) % 2) {
    gfxPrint(82, 1, PSTR("MUTE"));
    gfxInvert(81, 0, 26, 9);
  }
  if (n > 4) {
    gfxPrint(110, 1, cursor / 4 + 1);
    gfxPrint(116, 1, PSTR("/2"));
  }
  gfxLine(0, 9, 127, 9);

  // PLAYER's file browser takes the whole page
  if (!focus_mod && col == SOURCE && d == tracks[track].player && tracks[track].player->browsing) {
    tracks[track].player->DrawBrowser(0, 11, 128, 53);
    return;
  }

  // the device's picture
  d->view_page = cursor / 4;
  d->view_cursor = cursor;
  if (!trig_mode) d->DrawViz(0, 11, 128, 30);

  // parameter grid, one page of 4
  const int page = cursor / 4;
  for (int i = 0; i < 4; i++) {
    const int p = page * 4 + i;
    if (p >= n) break;
    const ParamDef& pd = d->param(p);
    const int x = i * 32;
    const bool sel = p == cursor;
    char buf[8];

    // name row (in modulation mode: the source for the selected setting,
    // e.g. "1 MCb" = source 1 is MOD C, bipolar; "2 off")
    if (sel && mod_mode) {
      const HS::CVInputMap& in = d->mod_in[p];
      const bool learn = !mod_slot && !d->mod_bus_sel[p] && in.source_type() == HS::CVInputMap::TYPE_MIDI
        && HS::frame.MIDIState.mapping[in.index()].learning();
      const uint8_t bus = mod_slot ? d->mod_bus2[p] : d->mod_bus_sel[p];
      // small u / b, so the wave mark fits above it
      const char ub = (mod_slot ? d->Bipolar2(p) : d->Bipolar(p)) ? 'b' : 'u';
      char src[8] = {};
      if (bus) snprintf(src, sizeof(src), "M%c%c", 'A' + bus - 1, ub);
      else if (!mod_slot && in.enabled()) {
        // CV / MIDI: its short name without the gaps (C 1 -> C1)
        int k = 0;
        for (const char* c = in.InputName(); *c && k < 4; c++) if (*c != ' ') src[k++] = *c;
        src[k++] = ub;
        src[k] = 0;
      } else snprintf(src, sizeof(src), "off");
      if (learn) snprintf(buf, sizeof(buf), "learn");
      else snprintf(buf, sizeof(buf), strlen(src) > 3 ? "%d%s" : "%d %s", mod_slot + 1, src);
      PrintCentred(x, 43, buf);
    } else {
      PrintCentred(x, 43, pd.name);
      // has modes: a small square (a 5-letter name: just outside its column)
      if (pd.nmodes) {
        if (strlen(pd.name) >= 5) gfxRect(x + 33, 43, 2, 2);  // in the gap before the next name
        else gfxRect(x + 29, 43, 2, 2);
      }
    }
    if (d->ModsLive(p) && !(sel && mod_mode)) { // modulated: small wave mark
      gfxPixel(x + 27, 49); gfxPixel(x + 28, 48); gfxPixel(x + 29, 49); gfxPixel(x + 30, 48);
    }

    // value row
    const bool show_mode = sel && pd.nmodes
      && ((editing && l_held) || (edit_mode && mode_shown < 1500));
    if (sel && mod_mode) snprintf(buf, sizeof(buf), "%+d%%", mod_slot ? d->mod_amt2[p] : d->mod_amt[p]);
    else if (show_mode) d->FormatMode(p, buf, sizeof(buf));
    else d->Format(p, buf, sizeof(buf));
    PrintCentred(x, 52, buf);
    if (sel && (editing || mod_mode)) gfxInvert(x, 51, 32, 9);

    // value bar; modulated, the reach of the modulation instead. In
    // modulation: the two sources instead, 1 above 2, from the middle by
    // their amount (- to the left); bipolar ones spread both ways
    const int span = std::max(1, pd.max - pd.min);
    if (mod_mode) {
      const int c = x + 15;
      gfxLine(c, 61, c, 62);
      for (int k = 0; k < 2; k++) {
        const int amt = k ? d->mod_amt2[p] : d->mod_amt[p];
        const bool on = k ? d->mod_bus2[p] && d->mod_bus : d->ModSourceOn(p);
        if (!amt || !on) continue;
        // an empty modulator: the line dotted (it does nothing)
        const int bus = k ? d->mod_bus2[p] : d->mod_bus_sel[p];
        const bool empty = bus && d->mod_bus && d->mod_bus[bus - 1] < 0.0f;
        const int len = std::max(1, abs(amt) * 14 / 100);
        const bool bi = k ? d->Bipolar2(p) : d->Bipolar(p);
        const int ry = 60 + 2 * k;
        const int x0 = bi || amt < 0 ? c - len : c, x1 = bi || amt > 0 ? c + len : c;
        for (int px = x0; px <= x1; px++) if (!empty || !(px & 1)) gfxPixel(px, ry);
      }
    } else if (pd.kind != LIST && d->ModsLive(p)) {
      // modulated: the stretch both sources together can reach (a line with
      // end ticks) and the playhead where they put it right now
      auto px = [&](int v) {
        return pd.kind == BI ? x + 15 + v * 15 / std::max(1, (int)pd.max)
                             : x + 1 + (v - pd.min) * 29 / span;
      };
      int lo, hi;
      d->ModRange(p, lo, hi);
      const int x0 = px(lo), x1 = px(hi);
      gfxLine(x0, 62, x1, 62);
      gfxPixel(x0, 61);
      gfxPixel(x1, 61);
      gfxLine(px(d->cur[p]), 59, px(d->cur[p]), 63);
    } else if (pd.kind == BI) {
      const int c = x + 15, len = d->val[p] * 15 / std::max(1, (int)pd.max);
      gfxPixel(c, 61);
      if (len > 0) gfxRect(c, 61, len, 2);
      else if (len < 0) gfxRect(c + len, 61, -len, 2);
    } else if (pd.kind == LIST) {
      for (int k = 0; k < pd.nlabels; k++) {
        const int bx = x + 1 + k * 7;
        if (k == d->cur[p]) gfxRect(bx, 61, 5, 2);
        else gfxPixel(bx + 2, 62);
      }
    } else {
      gfxRect(x + 1, 61, (d->val[p] - pd.min) * 29 / span, 2);
    }
    if (sel) gfxInvert(x, 42, 32, 9);
  }
}

// two pages of four plus CLOCK: the track's A-D, then the shared E-H
FLASHMEM void AppSculpt::DrawMods() const {
  using namespace sculpt;
  if (mod_cursor >= OUT_CURSOR) { DrawOutputs(); return; }
  const int page = mod_cursor < NUM_MODS ? 0 : 1;
  if (linked || page) gfxPrint(1, 0, PSTR("L+R"));
  else gfxPrint(1, 0, track ? PSTR("R") : PSTR("L"));
  gfxPrint(linked || page ? 24 : 12, 0, page ? PSTR("SHARED") : PSTR("MODULATORS"));
  if (Muted() && (millis() / 300) % 2) {
    gfxPrint(104, 0, PSTR("MUTE"));
    gfxInvert(103, 0, 25, 8);
  } else {
    gfxPrint(110, 0, page + 1);
    gfxPrint(116, 0, PSTR("/3"));
  }
  gfxLine(0, 9, 127, 9);
  constexpr int bw = 25;
  for (int b = 0; b <= NUM_MODS; b++) {
    const int i = b < NUM_MODS ? page * NUM_MODS + b : CLK_CURSOR;
    const int x = b * bw + 1;
    Modulator* m = ModAt(track, i);
    char buf[8];
    // fixed labels; the type is named in the bottom line
    if (i < CLK_CURSOR) snprintf(buf, sizeof(buf), "-%c-", 'A' + i);
    else snprintf(buf, sizeof(buf), "CLK");
    gfxPrint(x + (bw - 1 - 6 * int(strlen(buf))) / 2, 12, buf);
    if (m) {
      gfxFrame(x, 21, bw - 1, 30);
      m->DrawMini(x + 1, 22, bw - 3, 28);
    } else {
      gfxFrame(x, 21, bw - 1, 30, true);
    }
    if (i == mod_cursor) gfxInvert(x, 11, bw - 1, 9);
  }
  Modulator* m = ModAt(track, mod_cursor);
  gfxPrint(0, 56, m ? m->def.name : PSTR("off"));
  gfxPrint(80, 56, PSTR("Y:back"));
}

// page 3: the 8 CV outputs, each with its source and its voltage as a bar
FLASHMEM void AppSculpt::DrawOutputs() const {
  using namespace sculpt;
  gfxPrint(1, 0, PSTR("CV OUTPUTS"));
  gfxPrint(110, 0, PSTR("3/3"));
  gfxLine(0, 9, 127, 9);
  constexpr int bw = 32, bh = 21;
  for (int o = 0; o < NUM_CV_OUTS; o++) {
    const int x = (o % 4) * bw, y = 11 + (o / 4) * (bh + 1);
    char buf[8];
    snprintf(buf, sizeof(buf), "%c", 'A' + o);
    gfxPrint(x + 2, y + 2, buf);
    FormatOutSource(cv_out_src[o], buf, sizeof(buf), false);
    gfxPrint(x + bw - 2 - 6 * int(strlen(buf)), y + 2, buf);
    // 0..10 V across the box
    gfxFrame(x + 2, y + 12, bw - 5, 6);
    const int fill = int(OutLevel(o) * (bw - 7) + 0.5f);
    if (fill > 0) gfxRect(x + 3, y + 13, fill, 4);
    if (OUT_CURSOR + o == mod_cursor) gfxInvert(x + 1, y + 1, 9, 9);
  }
  char buf[24];
  FormatOutSource(cv_out_src[mod_cursor - OUT_CURSOR], buf, sizeof(buf), true);
  gfxPrint(0, 56, buf);
}

/////////////////////////////////////////////////////////////////
// Saving: SCULPT.BIN on the internal flash
/////////////////////////////////////////////////////////////////
namespace sculpt {
  constexpr char SAVE_FILE[] = "SCULPT.BIN";
  constexpr char SAVE_TMP[] = "SCULPT.TMP";
  // 2: modulator types sorted by name; 3: second mod source, MOD E-H
  constexpr uint8_t SAVE_VERSION = 3;
  static uint8_t load_version = SAVE_VERSION;  // of the file being read

  // per device: tag, count, then per setting value/mode/mod amount/
  // mod source/bus select, then bipolar bits and action triggers
  FLASHMEM static void WriteDevice(File& f, const Device* d) {
    uint8_t present = d ? 1 : 0;
    f.write(&present, 1);
    if (!d) return;
    f.write(d->def.tag, 4);
    const uint8_t n = d->num_params();
    f.write(&n, 1);
    for (int p = 0; p < n; p++) {
      const uint16_t src = d->mod_in[p].Pack();
      f.write(&d->val[p], 2);
      f.write(&d->mode[p], 1);
      f.write(&d->mod_amt[p], 1);
      f.write(&src, 2);
      f.write(&d->mod_bus_sel[p], 1);
      f.write(&d->mod_amt2[p], 1);
      f.write(&d->mod_bus2[p], 1);
    }
    f.write(&d->mod_bi, 1);
    f.write(&d->mod_bi2, 1);
    f.write(d->act_trig, 2);
  }

  // whether the next record holds a device (its first byte), without
  // using it up: peek() doesn't work on the internal flash's files
  FLASHMEM static bool NextPresent(File& f) {
    const uint64_t at = f.position();
    uint8_t present = 0;
    const bool got = f.read(&present, 1) == 1;
    f.seek(at);
    return got && present == 1;
  }

  // reads one record; applies it if d is there and of the same kind
  FLASHMEM static bool ReadDevice(File& f, Device* d) {
    uint8_t present = 0;
    if (f.read(&present, 1) != 1) return false;
    if (!present) return true;
    char tag[4];
    uint8_t n = 0;
    if (f.read(tag, 4) != 4 || f.read(&n, 1) != 1 || n > MAX_PARAMS) return false;
    // a device may have gained settings since: read the ones that are there
    // files saved before the devices were renamed carry their old tags;
    // the next save writes the new ones (this table can go after that)
    static const char renamed[6][2][4] = {
      {"RNG", "RES"}, {"DEF", "TEX"}, {"DRT", "TEX"}, {"VST", "ECO"}, {"ECH", "ECO"},
      {"MOS", "SPR"}};
    for (auto& r : renamed) if (!memcmp(tag, r[0], 4)) memcpy(tag, r[1], 4);
    const bool use = d && !memcmp(tag, d->def.tag, 4) && n <= d->num_params();
    for (int p = 0; p < n; p++) {
      int16_t v; uint8_t md; int8_t amt; uint16_t src; uint8_t bus;
      if (f.read(&v, 2) != 2 || f.read(&md, 1) != 1 || f.read(&amt, 1) != 1
          || f.read(&src, 2) != 2 || f.read(&bus, 1) != 1) return false;
      int8_t amt2 = 0; uint8_t bus2 = 0;
      if (load_version >= 3 && (f.read(&amt2, 1) != 1 || f.read(&bus2, 1) != 1)) return false;
      if (!use) continue;
      d->mod_amt2[p] = constrain(amt2, -100, 100);
      d->mod_bus2[p] = bus2 <= MOD_BUSES ? bus2 : 0;
      const ParamDef& pd = d->param(p);
      d->val[p] = constrain(v, pd.min, pd.max);
      d->mode[p] = pd.nmodes ? constrain(md, 0, pd.nmodes - 1) : 0;
      d->mod_amt[p] = constrain(amt, -100, 100);
      d->mod_in[p].Unpack(src);
      d->mod_bus_sel[p] = bus <= MOD_BUSES ? bus : 0;
    }
    uint8_t bi, bi2 = 0, trig[2];
    if (f.read(&bi, 1) != 1 || (load_version >= 3 && f.read(&bi2, 1) != 1)
        || f.read(trig, 2) != 2) return false;
    if (use) {
      d->mod_bi = bi;
      d->mod_bi2 = bi2;
      for (int a = 0; a < 2; a++) d->act_trig[a] = trig[a] < TRIG_SOURCES ? trig[a] : 0;
    }
    return true;
  }
}

template <typename F>
FLASHMEM void AppSculpt::ForEachDevice(F f) const {
  using namespace sculpt;
  if (tempo_clock) f(tempo_clock);
  for (auto& tr : tracks) {
    f(tr.input);
    if (tr.reel) f(tr.reel);
    if (tr.player) f(tr.player);
    f(tr.mix);
    for (int s = SOURCE + 1; s < SLOT_COUNT; s++)
      for (auto d : tr.inst[s]) if (d) f(d);
    for (auto& slot : tr.mods)
      for (auto m : slot) if (m) f(m);
  }
  for (auto& slot : shared_mods)
    for (auto m : slot) if (m) f(m);
}

FLASHMEM void AppSculpt::SaveFile() const {
  using namespace sculpt;
  auto& fs = PhzConfig::myfs;
  File f = fs.open(SAVE_TMP, FILE_WRITE_BEGIN);
  if (!f) return;
  f.write("SCU", 3);
  f.write(&SAVE_VERSION, 1);
  const uint8_t lk = linked;
  f.write(&lk, 1);
  WriteDevice(f, tempo_clock);
  for (auto& tr : tracks) {
    f.write(tr.choice, SLOT_COUNT);
    f.write(tr.last_choice, SLOT_COUNT);
    WriteDevice(f, tr.input);
    WriteDevice(f, tr.mix);
    for (int s = SOURCE + 1; s < SLOT_COUNT; s++)
      for (int o = 0; o < MAX_OPTIONS; o++) WriteDevice(f, tr.inst[s][o]);
    f.write(tr.mod_type, NUM_MODS);
    for (int i = 0; i < NUM_MODS; i++)
      for (int type = 1; type < MOD_TYPES; type++) WriteDevice(f, tr.mods[i][type]);
  }
  // where the user was, so Sculpt opens there again (older files end before)
  const uint8_t ui[6] = {uint8_t(view), uint8_t(track), uint8_t(col),
                         uint8_t(cursor), uint8_t(focus_mod), uint8_t(mod_cursor)};
  f.write(ui, sizeof(ui));
  // bypassed slots (older files end before)
  for (auto& tr : tracks) f.write(&tr.bypass, 1);
  // REEL's settings (older files end before); the recording isn't kept
  for (auto& tr : tracks) WriteDevice(f, tr.reel);
  for (auto& tr : tracks) WriteDevice(f, tr.player);
  for (auto& tr : tracks) {  // PLAYER's file by folder and name
    const char* fd = tr.player ? tr.player->Folder() : "";
    const char* fn = tr.player ? tr.player->FileName() : "";
    const uint8_t a = strlen(fd), b = strlen(fn);
    f.write(&a, 1); f.write(fd, a);
    f.write(&b, 1); f.write(fn, b);
  }
  // ... and START / END to the sample
  f.write("X", 1);
  for (auto& tr : tracks) {
    uint32_t e[2] = {0, 0};
    if (tr.player) { e[0] = tr.player->EdgeAt(PlayerDevice::START); e[1] = tr.player->EdgeAt(PlayerDevice::END); }
    f.write(reinterpret_cast<uint8_t*>(e), 8);
  }
  // ... and the shared MOD E-H
  f.write("S", 1);
  f.write(shared_type, NUM_MODS);
  for (int i = 0; i < NUM_MODS; i++)
    for (int type = 1; type < MOD_TYPES; type++) WriteDevice(f, shared_mods[i][type]);
  // ... and what the CV outputs send
  f.write("C", 1);
  f.write(cv_out_src, NUM_CV_OUTS);
  f.close();
  fs.remove(SAVE_FILE);
  fs.rename(SAVE_TMP, SAVE_FILE);
  dirty = false;
  Serial.println("Sculpt: settings saved");
}

FLASHMEM void AppSculpt::LoadFile() {
  using namespace sculpt;
  File f = PhzConfig::myfs.open(SAVE_FILE, FILE_READ);
  if (!f) return;
  char magic[3];
  uint8_t version = 0, lk = 0;
  if (f.read(magic, 3) != 3 || memcmp(magic, "SCU", 3) || f.read(&version, 1) != 1
      || version < 1 || version > SAVE_VERSION || f.read(&lk, 1) != 1) {
    f.close();
    return;
  }
  load_version = version;
  bool ok = ReadDevice(f, tempo_clock);
  int stage = 0;
  for (int t = 0; t < NUM_TRACKS && ok; t++) {
    Track& tr = tracks[t];
    uint8_t choice[SLOT_COUNT], last[SLOT_COUNT], types[NUM_MODS];
    ok = f.read(choice, SLOT_COUNT) == SLOT_COUNT && f.read(last, SLOT_COUNT) == SLOT_COUNT;
    if (!ok) break;
    tr.choice[SOURCE] = choice[SOURCE] < SLOT_DEFS[SOURCE].noptions ? choice[SOURCE] : 0;
    for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
      tr.choice[s] = choice[s] < SLOT_DEFS[s].noptions ? choice[s] : 0;
      tr.last_choice[s] = last[s] < SLOT_DEFS[s].noptions ? last[s] : 0;
    }
    ok = ReadDevice(f, tr.input) && ReadDevice(f, tr.mix);
    for (int s = SOURCE + 1; s < SLOT_COUNT && ok; s++)
      for (int o = 0; o < MAX_OPTIONS && ok; o++) ok = ReadDevice(f, tr.inst[s][o]);
    stage = 10 * t + 1;
    if (!ok || f.read(types, NUM_MODS) != NUM_MODS) { ok = false; break; }
    stage = 10 * t + 2;
    for (int i = 0; i < NUM_MODS && ok; i++) {
      for (int slot = 1; slot < MOD_TYPES && ok; slot++) {
        // version 1 kept them in the old order
        const int type = version == 1 ? OLD_MOD_TYPE[slot] : slot;
        // create the modulator before reading into it
        if (NextPresent(f) && !tr.mods[i][type]) SetModType(t, i, type);
        ok = ReadDevice(f, tr.mods[i][type]);
      }
      const int ty = types[i] < MOD_TYPES ? types[i] : 0;
      tr.mod_type[i] = version == 1 ? OLD_MOD_TYPE[ty] : ty;
      if (tr.mod_type[i] && !tr.mod(i)) tr.mod_type[i] = 0;
    }
    stage = 10 * t + 3;
    snprintf(load_note + strlen(load_note), sizeof(load_note) - strlen(load_note), " t%d types %d%d%d%d->%d%d%d%d",
             t, types[0], types[1], types[2], types[3], tr.mod_type[0], tr.mod_type[1], tr.mod_type[2], tr.mod_type[3]);
  }
  uint8_t ui[6];
  if (ok && f.read(ui, sizeof(ui)) == sizeof(ui)) {
    view = ui[0] <= MODS ? ViewMode(ui[0]) : CHAIN;
    track = ui[1] < NUM_TRACKS ? ui[1] : 0;
    col = ui[2] < COLS ? ui[2] : ECHO;
    cursor = ui[3] < MAX_PARAMS ? ui[3] : 0;
    focus_mod = ui[4];
    mod_cursor = ui[5] <= LAST_CURSOR ? ui[5] : 0;
    if (version < 3 && mod_cursor == NUM_MODS) mod_cursor = CLK_CURSOR;  // CLOCK was 5th
    for (auto& tr : tracks) {
      uint8_t b = 0;
      if (f.read(&b, 1) == 1) tr.bypass = b & ~1;  // SOURCE can't be bypassed
    }
    bool more = true;
    for (auto& tr : tracks) if (more && !ReadDevice(f, tr.reel)) more = false;
    for (auto& tr : tracks) if (more && !ReadDevice(f, tr.player)) more = false;
    for (auto& tr : tracks) {
      uint8_t a = 0, b = 0;
      char fd[128] = {}, fn[96] = {};
      if (!more || f.read(&a, 1) != 1 || a >= sizeof(fd) || f.read(fd, a) != a ||
          f.read(&b, 1) != 1 || b >= sizeof(fn) || f.read(fn, b) != b) { more = false; break; }
      if (tr.player && a) tr.player->SetFile(fd, fn);
    }
    char mark = 0;
    if (more && f.read(&mark, 1) == 1 && mark == 'X') {
      for (auto& tr : tracks) {
        uint32_t e[2];
        if (f.read(reinterpret_cast<uint8_t*>(e), 8) != 8) break;
        if (tr.player && e[1] > e[0]) tr.player->RestoreEdges(e[0], e[1]);
      }
      uint8_t types[NUM_MODS];
      if (f.read(&mark, 1) == 1 && mark == 'S' && f.read(types, NUM_MODS) == NUM_MODS) {
        bool okm = true;
        for (int i = 0; i < NUM_MODS && okm; i++) {
          for (int type = 1; type < MOD_TYPES && okm; type++) {
            if (NextPresent(f) && !shared_mods[i][type]) SetModType(0, NUM_MODS + i, type);
            okm = ReadDevice(f, shared_mods[i][type]);
          }
          shared_type[i] = types[i] < MOD_TYPES && shared_mods[i][types[i]] ? types[i] : 0;
        }
        uint8_t outs[NUM_CV_OUTS];
        if (okm && f.read(&mark, 1) == 1 && mark == 'C' && f.read(outs, NUM_CV_OUTS) == NUM_CV_OUTS)
          for (int o = 0; o < NUM_CV_OUTS; o++) cv_out_src[o] = outs[o] < OUT_SOURCES ? outs[o] : 0;
      }
    }
  }
  snprintf(load_note + strlen(load_note), sizeof(load_note) - strlen(load_note), " v%d ok %d stage %d pos %d/%d",
           version, ok, stage, int(f.position()), int(f.size()));
  f.close();
  linked = lk;
  if (linked) track = 0;
  if (view == DEVICE && (!Focused() || cursor >= Focused()->num_params())) {
    view = CHAIN;
    cursor = 0;
  }
  ForEachDevice([](Device* d) { d->ApplyAll(); });
  Serial.printf("Sculpt: settings loaded%s\n", ok ? "" : " (partly)");

}

FLASHMEM void AppSculpt::View() const {
  UpdateMixLevels();
  if (view == DEVICE && Focused()) DrawDevice();
  else if (view == MODS) DrawMods();
  else DrawChain();
  // saving: a bar fills while Y is held, then SAVED
  const uint32_t since = millis() - saved_at;
  if (saved_at && since < 1200) {
    graphics.clearRect(36, 22, 56, 15);
    gfxFrame(36, 22, 56, 15);
    gfxPrint(49, 26, PSTR("SAVED"));
    gfxInvert(37, 23, 54, 13);
  } else if (save_held && view == CHAIN && millis() - save_hold > 250) {
    const int fill = std::min<int>(52, (millis() - save_hold) * 52 / 2000);
    graphics.clearRect(36, 22, 56, 15);
    gfxFrame(36, 22, 56, 15);
    gfxPrint(46, 26, PSTR("SAVE..."));
    gfxInvert(38, 24, fill, 11);
  }
}

/////////////////////////////////////////////////////////////////
// App glue
/////////////////////////////////////////////////////////////////
FLASHMEM void AppSculpt::Init() {
  BaseStart();
}

// the global save (app menu, USB 'g') also writes Sculpt's own file
FLASHMEM size_t AppSculpt::SaveAppData(util::StreamBufferWriter &) const {
  if (dirty) SaveFile();
  return 0;
}
size_t AppSculpt::RestoreAppData(util::StreamBufferReader &) { return 0; }

void AppSculpt::Process(OC::IOFrame *ioframe) {
  BaseController(ioframe);
}

FLASHMEM void AppSculpt::HandleAppEvent(OC::AppEvent event) {
  if (event == OC::APP_EVENT_RESUME) Resume();
  if (event == OC::APP_EVENT_SUSPEND) Suspend();
}

FLASHMEM void AppSculpt::Loop() {
  MainLoop();
}

FLASHMEM void AppSculpt::GetIOConfig(OC::IOConfig &) const { }
void AppSculpt::DrawDebugInfo() const { }

FLASHMEM void AppSculpt::DrawMenu() const { BaseView(); }

FLASHMEM void AppSculpt::DrawScreensaver() const { BaseView(); }

FLASHMEM void AppSculpt::HandleButtonEvent(const UI::Event &event) {
  OnButton(event);
}

FLASHMEM void AppSculpt::HandleEncoderEvent(const UI::Event &event) {
  OnEncoder(event);
}
