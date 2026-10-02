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

// Sculpt: a full-screen take on the Torso S4 device chain.
//
// Two stereo tracks, each a fixed chain MATERIAL > GRANULAR > FILTER > COLOR
// > SPACE, mixed to the stereo output. Every effect slot can be bypassed.
//
// Chain view (home):
//   L enc  move between the 12 boxes      R enc  change the slot's device
//   L push bypass on/off                  R push open the device
//   UP / DOWN  the device's two actions (e.g. VAST freeze / clear)
//   X      the track's modulators (MOD A-D)
//   UP + DOWN  link / unlink the tracks' controls. Linking copies the track
//              you are on to the other one; while linked every edit goes to
//              both tracks (except input source and pan). Both keep running.
// Device view:
//   R enc  scroll through the 8 settings  R push  grab / let go of a setting
//   (grabbed) R enc  change the value     hold L + R enc  the setting's mode
//   L push modulation for the setting:    L enc amount, R enc source (CV in,
//          CV out, MIDI map, noise),      R push unipolar / bipolar,
//          X  MIDI learn (move a knob),   L push again to leave
//   Y      back to the chain              UP / DOWN  actions
//   X      action triggers: L enc source for UP's action, R enc for DOWN's
//          (gate in, CV in as gate, MIDI map), X / Y to leave
// MOD view (4 modulators per track, like the S4's):
//   L enc  choose A-D or CLK              R enc  type: WAVE/RANDOM/ADSR/FOLLOW
//   R push open its settings              Y / X  back to the chain
//   CLK is the app's tempo: internal (UP = tap), MIDI clock or a trigger
//   input. VAST TIME and the WAVE / RANDOM rates can sync to it (modes STR,
//   TRI, DOT: hold L + turn R on the setting).
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
    MixDevice* mix = nullptr;
    uint8_t choice[SLOT_COUNT] = {};
    uint8_t last_choice[SLOT_COUNT] = {}; // for the bypass toggle
    Device* inst[SLOT_COUNT][MAX_OPTIONS] = {};
    Modulator* mods[NUM_MODS][MOD_TYPES] = {};
    uint8_t mod_type[NUM_MODS] = {};
    float mod_out[NUM_MODS] = {};

    Modulator* mod(int i) const { return mods[i][mod_type[i]]; }

    Device* current(int s) const {
      return s == MATERIAL ? static_cast<Device*>(input) : inst[s][choice[s]];
    }
    Device* at(int col) const {
      return col == MIX_COL ? mix : current(col);
    }
  };

  // patch cords live on the heap: no RAM1 until Sculpt is first opened
  struct Patch {
    AudioConnection in[NUM_TRACKS][2];
    AudioConnection link[NUM_TRACKS][SLOT_COUNT][2];
  };

  Track tracks[NUM_TRACKS];
  SculptMixer* mixer = nullptr;
  ClockDevice* tempo_clock = nullptr;
  Patch* patch = nullptr;

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
  int8_t track = 0, col = sculpt::SPACE, cursor = 0;
  bool l_held = false, l_used = false;
  bool editing = false;   // device view: setting grabbed, R enc changes it
  bool mod_mode = false;  // device view: L enc amount, R enc source
  bool trig_mode = false; // device view: L / R enc trigger source per action
  elapsedMicros mod_timer;
  uint8_t chord_releases = 0; // UP/DOWN releases to swallow after UP+DOWN
  bool active = false;
  bool linked = false;
  mutable bool dirty = false; // something changed since the last save
  float applied_tempo = 120.0f;
  int cpu = 0;
  elapsedMillis stats_timer;

  sculpt::Device* FocusOn(int t) const {
    if (focus_mod && mod_cursor == sculpt::NUM_MODS) return sculpt::tempo_clock;
    return focus_mod ? sculpt::tracks[t].mod(mod_cursor) : sculpt::tracks[t].at(col);
  }
  void SaveFile() const;
  void LoadFile();
  template <typename F> void ForEachDevice(F f) const;
  sculpt::Device* Focused() const { return FocusOn(track); }
  void SetModType(int t, int i, int type);
  void DrawMods() const;
  void SetChoice(int t, int s, int option);
  void SetChoiceLinked(int s, int option);
  sculpt::Device* Twin(sculpt::Device* d) const;
  void Mirror(sculpt::Device* d, int p) const;
  void Repatch(int t);
  void ConnectTrack(int t);
  void DisconnectTrack(int t);
  void ToggleLink();
  void UpdateMixLevels() const;
  void DrawChain() const;
  void DrawDevice() const;
};

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
    for (int s = MATERIAL + 1; s < SLOT_COUNT; s++) {
      for (int o = 0; o < SLOT_DEFS[s].noptions; o++) {
        Device* d = MakeDevice(s, o);
        if (d) {
          d->mod_bus = tr.mod_out;
          d->Defaults();
          d->ApplyAll();
        }
        tr.inst[s][o] = d;
      }
      tr.last_choice[s] = SLOT_DEFS[s].noptions > 1 ? 1 : 0;
    }
  }
  mixer = new SculptMixer;
  for (int t = 0; t < NUM_TRACKS; t++) {
    Track& tr = tracks[t];
    tr.mix = new MixDevice(*mixer, t);
    tr.mix->mod_bus = tr.mod_out;
    tr.mix->Defaults();
    tr.mix->val[1] = t ? 25 : -25; // pan the tracks apart a little
    tr.mix->ApplyAll();
    // start with VAST on both tracks, a bit different on track 2
    tr.choice[SPACE] = 1;
  }
  Device* v2 = tracks[1].inst[SPACE][1];
  v2->val[VastDevice::TIME] = 680;
  v2->val[VastDevice::TONE] = 15;
  v2->mode[VastDevice::SPREAD] = VastDevice::DIFFUSE;

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
  for (auto& hop : link) for (auto& c : hop) c.disconnect();
  AudioStream* prev = tr.input;
  int hop = 0;
  for (int s = MATERIAL + 1; s < SLOT_COUNT; s++) {
    Device* d = tr.current(s);
    if (!d || !d->in()) continue;
    for (int c = 0; c < 2; c++) link[hop][c].connect(*prev, c, *d->in(), c);
    prev = d->out();
    hop++;
  }
  for (int c = 0; c < 2; c++) link[hop][c].connect(*prev, c, *mixer, t * 2 + c);
  AudioInterrupts();
}

FLASHMEM void AppSculpt::SetChoice(int t, int s, int option) {
  using namespace sculpt;
  Track& tr = tracks[t];
  option = constrain(option, 0, SLOT_DEFS[s].noptions - 1);
  if (option == tr.choice[s]) return;
  Device* old = tr.current(s);
  tr.choice[s] = option;
  if (option) tr.last_choice[s] = option;
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
  for (int s = MATERIAL + 1; s < SLOT_COUNT; s++) {
    if (Device* d = tracks[t].current(s)) d->Activate();
  }
  Repatch(t);
}

FLASHMEM void AppSculpt::DisconnectTrack(int t) {
  using namespace sculpt;
  AudioNoInterrupts();
  for (auto& c : patch->in[t]) c.disconnect();
  for (auto& hop : patch->link[t]) for (auto& c : hop) c.disconnect();
  AudioInterrupts();
  // disconnected devices stop updating; now they can drop their buffers
  for (int s = MATERIAL + 1; s < SLOT_COUNT; s++) {
    if (Device* d = tracks[t].current(s)) d->Deactivate();
  }
}

// the same device on the other track (only meaningful while linked)
FLASHMEM sculpt::Device* AppSculpt::Twin(sculpt::Device* d) const {
  using namespace sculpt;
  if (!linked || !d) return nullptr;
  Device* tw = FocusOn(1 - track);
  return (tw && &tw->def == &d->def) ? tw : nullptr;
}

// copy one parameter to the twin; input source and pan stay per track
FLASHMEM void AppSculpt::Mirror(sculpt::Device* d, int p) const {
  using namespace sculpt;
  Device* tw = Twin(d);
  if (!tw) return;
  if (!focus_mod && col == MATERIAL && p == 0) return;
  if (!focus_mod && col == MIX_COL && p == 1) return;
  tw->val[p] = d->val[p];
  tw->mode[p] = d->mode[p];
  tw->mod_amt[p] = d->mod_amt[p];
  tw->mod_in[p] = d->mod_in[p];
  tw->mod_bus_sel[p] = d->mod_bus_sel[p];
  tw->mod_bi = (tw->mod_bi & ~(1 << p)) | (d->mod_bi & (1 << p));
  tw->Refresh(p);
}

FLASHMEM void AppSculpt::SetModType(int t, int i, int type) {
  using namespace sculpt;
  Track& tr = tracks[t];
  type = constrain(type, 0, MOD_TYPES - 1);
  if (type && !tr.mods[i][type]) {
    Modulator* m = MakeModulator(type);
    m->mod_bus = tr.mod_out;
    m->Defaults();
    tr.mods[i][type] = m;
  }
  tr.mod_type[i] = type;
  if (!type) tr.mod_out[i] = 0.0f;
}

FLASHMEM void AppSculpt::SetChoiceLinked(int s, int option) {
  SetChoice(track, s, option);
  if (linked) SetChoice(1 - track, s, sculpt::tracks[track].choice[s]);
}

FLASHMEM void AppSculpt::ToggleLink() {
  using namespace sculpt;
  linked = !linked;
  if (!linked) return;
  // the track under the cursor is the master: copy it to the other one
  Track& a = tracks[track];
  Track& b = tracks[1 - track];
  for (int s = MATERIAL + 1; s < SLOT_COUNT; s++) {
    SetChoice(1 - track, s, a.choice[s]);
    b.last_choice[s] = a.last_choice[s];
    for (int o = 0; o < MAX_OPTIONS; o++) {
      Device* from = a.inst[s][o];
      Device* to = b.inst[s][o];
      if (!from || !to) continue;
      to->CopyFrom(*from);
    }
  }
  for (int i = 0; i < NUM_MODS; i++) {
    SetModType(1 - track, i, a.mod_type[i]);
    for (int type = 1; type < MOD_TYPES; type++) {
      if (!a.mods[i][type]) continue;
      if (!b.mods[i][type]) {
        SetModType(1 - track, i, type);
        SetModType(1 - track, i, a.mod_type[i]);
      }
      b.mods[i][type]->CopyFrom(*a.mods[i][type]);
    }
  }
  for (int p = 1; p < b.input->num_params(); p++) b.input->val[p] = a.input->val[p];
  b.input->ApplyAll();
  b.mix->val[0] = a.mix->val[0]; // level
  b.mix->val[2] = a.mix->val[2]; // mute
  b.mix->ApplyAll();
}

FLASHMEM void AppSculpt::Resume() {
  using namespace sculpt;
  if (!audio_hw() || active) return;
  if (!patch) patch = new Patch;
  active = true;
  for (int t = 0; t < NUM_TRACKS; t++) ConnectTrack(t);
  OC::AudioIO::ClaimOutput(*mixer);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  stats_timer = 0;
}

FLASHMEM void AppSculpt::Suspend() {
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
    for (int s = MATERIAL; s < SLOT_COUNT; s++) {
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
}

FLASHMEM void AppSculpt::MainLoop() {
  if (active) ReadMIDI();
  if (active && mod_timer >= 1000) { // follow the sources about once per ms
    mod_timer = 0;
    ModulateAll();
  }
  if (!active || stats_timer < 1000) return;
  stats_timer = 0;
  const float usage = AudioProcessorUsageMax();
  cpu = int(usage + 0.5f);
  Serial.printf("Sculpt: AudioProcessorUsageMax %d.%d%%  mem %u/%d blocks\n",
    int(usage), int(usage * 10) % 10, AudioMemoryUsageMax(), OC::AudioIO::AUDIO_MEMORY);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();

  // per device: own stream + owned streams (e.g. VAST core + delay lines)
  using namespace sculpt;
  for (int t = 0; t < NUM_TRACKS; t++) {
    Serial.printf("  T%d", t + 1);
    for (int s = MATERIAL; s < SLOT_COUNT; s++) {
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
  if (event.control == OC::CONTROL_BUTTON_L) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      l_held = true;
      l_used = false;
    } else if (event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      l_held = false;
    } else if (event.type == UI::EVENT_BUTTON_PRESS) {
      l_held = false;
      if (l_used) return;
      if (view == DEVICE) {
        mod_mode = !mod_mode;
        editing = trig_mode = false;
      } else if (col > MATERIAL && col < MIX_COL) {
        // bypass toggle
        Track& tr = tracks[track];
        SetChoiceLinked(col, tr.choice[col] ? 0 : tr.last_choice[col]);
      }
    }
    return;
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
  if (event.type != UI::EVENT_BUTTON_PRESS) return;

  if (view == MODS) focus_mod = true;
  else if (view == CHAIN) focus_mod = false;
  Device* d = Focused();
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
        // unipolar / bipolar, like tapping the encoder in S4 mapping mode
        d->mod_bi ^= 1 << cursor;
        d->Refresh(cursor);
        Mirror(d, cursor);
      } else {
        editing = !editing;
      }
      break;
    case OC::CONTROL_BUTTON_Y:
      if (view == DEVICE && trig_mode) trig_mode = false;
      else view = (view == DEVICE && focus_mod) ? MODS : CHAIN;
      editing = mod_mode = false;
      break;
    case OC::CONTROL_BUTTON_X:
      if (view == DEVICE && mod_mode && d) MidiLearn(d);
      else if (view == DEVICE) { trig_mode = !trig_mode; editing = false; }
      else if (view == CHAIN) view = MODS;
      else if (view == MODS) view = CHAIN;
      break;
    case OC::CONTROL_BUTTON_UP:
    case OC::CONTROL_BUTTON_DOWN:
      if (d) {
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
  if (view == MODS) {
    if (left) {
      mod_cursor = constrain(mod_cursor + dir, 0, NUM_MODS);
    } else if (mod_cursor < NUM_MODS) {
      const int type = tracks[track].mod_type[mod_cursor] + dir;
      SetModType(track, mod_cursor, type);
      if (linked) SetModType(1 - track, mod_cursor, tracks[track].mod_type[mod_cursor]);
    }
    return;
  }
  if (view == CHAIN) {
    if (left) {
      const int idx = constrain(track * COLS + col + dir, 0, NUM_TRACKS * COLS - 1);
      track = idx / COLS;
      col = idx % COLS;
    } else if (col == MIX_COL) {
      tracks[track].mix->Nudge(0, dir); // level
      Mirror(tracks[track].mix, 0);
    } else if (col > MATERIAL) {
      SetChoiceLinked(col, tracks[track].choice[col] + dir);
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
    if (left) d->mod_amt[cursor] = constrain(d->mod_amt[cursor] + dir, -100, 100);
    else d->ChangeModSource(cursor, dir);
    d->Refresh(cursor);
    Mirror(d, cursor);
  } else if (left) {
    // left encoder only works in modulation mode
  } else if (!editing) {
    cursor = constrain(cursor + dir, 0, d->num_params() - 1);
  } else if (l_held && d->param(cursor).nmodes) {
    d->NudgeMode(cursor, dir);
    Mirror(d, cursor);
    l_used = true;
  } else {
    d->Nudge(cursor, dir);
    Mirror(d, cursor);
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

FLASHMEM void AppSculpt::DrawChain() const {
  using namespace sculpt;
  constexpr int bw = 20, bh = 20, x0 = 8;
  for (int c = 0; c < COLS; c++) {
    gfxPrint(x0 + c * bw + 1, 0, c == MIX_COL ? PSTR("MIX") : SLOT_DEFS[c].tag);
  }
  for (int t = 0; t < NUM_TRACKS; t++) {
    const int y = 9 + t * (bh + 2);
    gfxPrint(1, y + 6, t + 1);
    for (int c = 0; c < COLS; c++) {
      const int x = x0 + c * bw;
      Device* d = tracks[t].at(c);
      if (d) {
        gfxFrame(x, y, bw, bh);
        gfxPrint(x + 2, y + 2, d->def.tag);
        if (d->ok()) d->DrawMini(x + 2, y + 12, bw - 4, 5);
        else gfxPrint(x + 2, y + 11, PSTR("!!"));
      } else {
        gfxFrame(x, y, bw, bh, true);
        gfxLine(x + 6, y + bh / 2, x + bw - 7, y + bh / 2); // bypass: wire
      }
      if (t == track && c == col) gfxInvert(x + 1, y + 1, bw - 2, bh - 2);
    }
  }

  if (linked) {
    // chain link between the track numbers
    gfxFrame(1, 24, 5, 6);
    gfxFrame(1, 28, 5, 6);
  }

  // status line: what is under the cursor, CPU load
  const Track& tr = tracks[track];
  Device* d = tr.at(col);
  gfxPrint(0, 56, track + 1);
  gfxPrint(8, 56, col == MIX_COL ? PSTR("MIX") : SLOT_DEFS[col].name);
  gfxPrint(62, 56, d ? d->def.name : PSTR("bypass"));
  if (!audio_hw()) {
    gfxPrint(104, 56, PSTR("noIO"));
  } else {
    char buf[6];
    snprintf(buf, sizeof(buf), "%d%%", cpu);
    gfxPrint(128 - 6 * int(strlen(buf)), 56, buf);
  }
}

FLASHMEM void AppSculpt::DrawDevice() const {
  using namespace sculpt;
  Device* d = Focused();
  if (!d) return;
  const int n = d->num_params();

  // header: track, device, action labels, page
  if (linked) {
    gfxPrint(1, 0, PSTR("1+2"));
    gfxInvert(0, 0, 19, 8);
    gfxPrint(22, 0, d->def.name);
  } else {
    gfxPrint(1, 0, track + 1);
    gfxInvert(0, 0, 7, 8);
    if (focus_mod) {
      char buf[8];
      snprintf(buf, sizeof(buf), "%c %s", 'A' + mod_cursor, d->def.name);
      gfxPrint(10, 0, buf);
    } else {
      gfxPrint(10, 0, d->def.name);
    }
  }
  if (mod_mode) {
    gfxPrint(64, 0, PSTR("MOD"));
    gfxInvert(63, 0, 20, 8);
  }
  if (trig_mode) {
    // which trigger fires each action
    gfxFrame(0, 10, 128, 32);
    for (int a = 0; a < 2; a++) {
      char src[6], buf[16];
      FormatTrig(d->act_trig[a], src, sizeof(src));
      const char* act = d->def.actions[a][0] ? d->def.actions[a] : "-";
      snprintf(buf, sizeof(buf), "%s %s:%s", a ? "DN" : "UP", act, src);
      gfxPrint(4, 13 + a * 10, buf);
    }
    gfxPrint(4, 33, PSTR("L/R enc: trigger"));
  }
  for (int a = 0; a < 2 && !mod_mode; a++) {
    if (!d->def.actions[a][0]) continue;
    const int x = 64 + a * 22;
    gfxPrint(x, 0, d->def.actions[a]);
    if (d->ActionLit(a)) gfxInvert(x - 1, 0, 20, 8);
  }
  if (n > 4) {
    gfxPrint(110, 0, cursor / 4 + 1);
    gfxPrint(116, 0, PSTR("/2"));
  }
  gfxLine(0, 9, 127, 9);

  // the device's picture
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

    // name row (in modulation mode: the source for the selected setting)
    if (sel && mod_mode) {
      const HS::CVInputMap& in = d->mod_in[p];
      const bool learn = !d->mod_bus_sel[p] && in.source_type() == HS::CVInputMap::TYPE_MIDI
        && HS::frame.MIDIState.mapping[in.index()].learning();
      if (learn) snprintf(buf, sizeof(buf), "learn");
      else {
        char src[6];
        d->ModSourceName(p, src, sizeof(src));
        snprintf(buf, sizeof(buf), "%s %c", src, d->Bipolar(p) ? 'B' : 'U');
      }
      gfxPrint(x + 1, 43, buf);
    } else {
      gfxPrint(x + 1, 43, pd.name);
      if (pd.nmodes) gfxRect(x + 29, 43, 2, 2); // has modes (hold L + turn R)
    }
    if (d->mod_amt[p]) { // modulated: small wave mark
      gfxPixel(x + 27, 49); gfxPixel(x + 28, 48); gfxPixel(x + 29, 49); gfxPixel(x + 30, 48);
    }

    // value row
    const bool show_mode = sel && editing && l_held && pd.nmodes;
    if (sel && mod_mode) snprintf(buf, sizeof(buf), "%+d%%", d->mod_amt[p]);
    else if (show_mode) snprintf(buf, sizeof(buf), "%s", pd.modes[d->mode[p]]);
    else d->Format(p, buf, sizeof(buf));
    gfxPrint(x + 1, 52, buf);
    if (sel && (editing || mod_mode)) gfxInvert(x, 51, 32, 9);

    // value bar, plus a tick where modulation puts it right now
    const int span = std::max(1, pd.max - pd.min);
    if (pd.kind == BI) {
      const int c = x + 15, len = d->val[p] * 15 / std::max(1, (int)pd.max);
      gfxPixel(c, 60);
      if (len > 0) gfxRect(c, 61, len, 2);
      else if (len < 0) gfxRect(c + len, 61, -len, 2);
      if (d->mod_amt[p]) gfxLine(c + d->cur[p] * 15 / std::max(1, (int)pd.max), 58, c + d->cur[p] * 15 / std::max(1, (int)pd.max), 60);
    } else if (pd.kind == LIST) {
      for (int k = 0; k < pd.nlabels; k++) {
        const int bx = x + 1 + k * 7;
        if (k == d->cur[p]) gfxRect(bx, 61, 5, 2);
        else gfxPixel(bx + 2, 62);
      }
    } else {
      gfxRect(x + 1, 61, (d->val[p] - pd.min) * 29 / span, 2);
      if (d->mod_amt[p]) {
        const int mx = x + 1 + (d->cur[p] - pd.min) * 29 / span;
        gfxLine(mx, 58, mx, 60);
      }
    }
    if (sel) gfxInvert(x, 42, 32, 9);
  }
}

FLASHMEM void AppSculpt::DrawMods() const {
  using namespace sculpt;
  const Track& tr = tracks[track];
  if (linked) gfxPrint(1, 0, PSTR("1+2"));
  else gfxPrint(1, 0, track + 1);
  gfxPrint(linked ? 24 : 12, 0, PSTR("MODULATORS"));
  gfxLine(0, 9, 127, 9);
  constexpr int bw = 25;
  for (int i = 0; i <= NUM_MODS; i++) {
    const int x = i * bw + 1;
    Modulator* m = i < NUM_MODS ? tr.mod(i) : tempo_clock;
    char buf[8];
    if (i < NUM_MODS) snprintf(buf, sizeof(buf), "%c%s", 'A' + i, MOD_TYPE_TAGS[tr.mod_type[i]]);
    else snprintf(buf, sizeof(buf), "CLK");
    gfxPrint(x + 1, 12, buf);
    if (m) {
      gfxFrame(x, 21, bw - 1, 30);
      m->DrawMini(x + 1, 22, bw - 3, 28);
    } else {
      gfxFrame(x, 21, bw - 1, 30, true);
    }
    if (i == mod_cursor) gfxInvert(x, 11, bw - 1, 9);
  }
  Modulator* m = mod_cursor < NUM_MODS ? tr.mod(mod_cursor) : tempo_clock;
  gfxPrint(0, 56, m ? m->def.name : PSTR("off"));
  gfxPrint(80, 56, PSTR("Y:back"));
}

/////////////////////////////////////////////////////////////////
// Saving: SCULPT.BIN on the internal flash
/////////////////////////////////////////////////////////////////
namespace sculpt {
  constexpr char SAVE_FILE[] = "SCULPT.BIN";
  constexpr char SAVE_TMP[] = "SCULPT.TMP";
  constexpr uint8_t SAVE_VERSION = 1;

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
    }
    f.write(&d->mod_bi, 1);
    f.write(d->act_trig, 2);
  }

  // reads one record; applies it if d is there and of the same kind
  FLASHMEM static bool ReadDevice(File& f, Device* d) {
    uint8_t present = 0;
    if (f.read(&present, 1) != 1) return false;
    if (!present) return true;
    char tag[4];
    uint8_t n = 0;
    if (f.read(tag, 4) != 4 || f.read(&n, 1) != 1 || n > MAX_PARAMS) return false;
    const bool use = d && !memcmp(tag, d->def.tag, 4) && n == d->num_params();
    for (int p = 0; p < n; p++) {
      int16_t v; uint8_t md; int8_t amt; uint16_t src; uint8_t bus;
      if (f.read(&v, 2) != 2 || f.read(&md, 1) != 1 || f.read(&amt, 1) != 1
          || f.read(&src, 2) != 2 || f.read(&bus, 1) != 1) return false;
      if (!use) continue;
      const ParamDef& pd = d->param(p);
      d->val[p] = constrain(v, pd.min, pd.max);
      d->mode[p] = pd.nmodes ? constrain(md, 0, pd.nmodes - 1) : 0;
      d->mod_amt[p] = constrain(amt, -100, 100);
      d->mod_in[p].Unpack(src);
      d->mod_bus_sel[p] = bus <= 4 ? bus : 0;
    }
    uint8_t bi, trig[2];
    if (f.read(&bi, 1) != 1 || f.read(trig, 2) != 2) return false;
    if (use) {
      d->mod_bi = bi;
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
    f(tr.mix);
    for (int s = MATERIAL + 1; s < SLOT_COUNT; s++)
      for (auto d : tr.inst[s]) if (d) f(d);
    for (auto& slot : tr.mods)
      for (auto m : slot) if (m) f(m);
  }
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
    for (int s = MATERIAL + 1; s < SLOT_COUNT; s++)
      for (int o = 0; o < MAX_OPTIONS; o++) WriteDevice(f, tr.inst[s][o]);
    f.write(tr.mod_type, NUM_MODS);
    for (int i = 0; i < NUM_MODS; i++)
      for (int type = 1; type < MOD_TYPES; type++) WriteDevice(f, tr.mods[i][type]);
  }
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
      || version != SAVE_VERSION || f.read(&lk, 1) != 1) {
    f.close();
    return;
  }
  bool ok = ReadDevice(f, tempo_clock);
  for (int t = 0; t < NUM_TRACKS && ok; t++) {
    Track& tr = tracks[t];
    uint8_t choice[SLOT_COUNT], last[SLOT_COUNT], types[NUM_MODS];
    ok = f.read(choice, SLOT_COUNT) == SLOT_COUNT && f.read(last, SLOT_COUNT) == SLOT_COUNT;
    if (!ok) break;
    for (int s = MATERIAL + 1; s < SLOT_COUNT; s++) {
      tr.choice[s] = choice[s] < SLOT_DEFS[s].noptions ? choice[s] : 0;
      tr.last_choice[s] = last[s] < SLOT_DEFS[s].noptions ? last[s] : 0;
    }
    ok = ReadDevice(f, tr.input) && ReadDevice(f, tr.mix);
    for (int s = MATERIAL + 1; s < SLOT_COUNT && ok; s++)
      for (int o = 0; o < MAX_OPTIONS && ok; o++) ok = ReadDevice(f, tr.inst[s][o]);
    if (!ok || f.read(types, NUM_MODS) != NUM_MODS) { ok = false; break; }
    for (int i = 0; i < NUM_MODS && ok; i++) {
      for (int type = 1; type < MOD_TYPES && ok; type++) {
        // create the modulator before reading into it
        if (f.peek() == 1 && !tr.mods[i][type]) SetModType(t, i, type);
        ok = ReadDevice(f, tr.mods[i][type]);
      }
      tr.mod_type[i] = types[i] < MOD_TYPES ? types[i] : 0;
      if (tr.mod_type[i] && !tr.mod(i)) tr.mod_type[i] = 0;
    }
  }
  f.close();
  linked = lk;
  ForEachDevice([](Device* d) { d->ApplyAll(); });
  Serial.printf("Sculpt: settings loaded%s\n", ok ? "" : " (partly)");
}

FLASHMEM void AppSculpt::View() const {
  UpdateMixLevels();
  if (view == DEVICE && Focused()) DrawDevice();
  else if (view == MODS) DrawMods();
  else DrawChain();
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
