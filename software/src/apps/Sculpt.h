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
//   UP + DOWN  link / unlink the tracks' controls. Linking copies the track
//              you are on to the other one; while linked every edit goes to
//              both tracks (except input source and pan). Both keep running.
// Device view:
//   L enc  choose parameter (2 pages of 4) R enc  change value
//   hold L + R enc  parameter mode        R push next page
//   L push back to chain                  UP / DOWN  actions
//
// While Sculpt is the active app it claims the audio outputs from AudioIO.
// The Quadrants audio applet chain is left intact and comes back as soon as
// Sculpt is suspended (e.g. when the app menu opens).

#include "../HSApplication.h"
#include "../AudioIO.h"
#include "../OC_gpio.h"
#include "SculptDevices.h"

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
  void Controller() {}
  void MainLoop();
  void View() const;

  void OnButton(const UI::Event &event);
  void OnEncoder(const UI::Event &event);

private:
  enum ViewMode : uint8_t { CHAIN, DEVICE };
  ViewMode view = CHAIN;
  int8_t track = 0, col = sculpt::SPACE, cursor = 0;
  bool l_held = false, l_used = false;
  uint8_t chord_releases = 0; // UP/DOWN releases to swallow after UP+DOWN
  bool active = false;
  bool linked = false;
  int cpu = 0;
  elapsedMillis stats_timer;

  sculpt::Device* Focused() const { return sculpt::tracks[track].at(col); }
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
    tr.input->Defaults();
    tr.input->val[0] = t ? InputDevice::IN2 : InputDevice::IN1;
    tr.input->ApplyAll();
    for (int s = MATERIAL + 1; s < SLOT_COUNT; s++) {
      for (int o = 0; o < SLOT_DEFS[s].noptions; o++) {
        Device* d = MakeDevice(s, o);
        if (d) {
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
  const Track& other = tracks[1 - track];
  Device* tw = other.at(col);
  return (tw && &tw->def == &d->def) ? tw : nullptr;
}

// copy one parameter to the twin; input source and pan stay per track
FLASHMEM void AppSculpt::Mirror(sculpt::Device* d, int p) const {
  using namespace sculpt;
  Device* tw = Twin(d);
  if (!tw) return;
  if (col == MATERIAL && p == 0) return;
  if (col == MIX_COL && p == 1) return;
  tw->val[p] = d->val[p];
  tw->mode[p] = d->mode[p];
  tw->Apply(p);
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
      memcpy(to->val, from->val, sizeof(to->val));
      memcpy(to->mode, from->mode, sizeof(to->mode));
      to->ApplyAll();
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
}

FLASHMEM void AppSculpt::MainLoop() {
  if (!active || stats_timer < 1000) return;
  stats_timer = 0;
  const float usage = AudioProcessorUsageMax();
  cpu = int(usage + 0.5f);
  Serial.printf("Sculpt: AudioProcessorUsageMax %d.%d%%  mem %u/%d blocks\n",
    int(usage), int(usage * 10) % 10, AudioMemoryUsageMax(), OC::AudioIO::AUDIO_MEMORY);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
}

/////////////////////////////////////////////////////////////////
// Controls
/////////////////////////////////////////////////////////////////
FLASHMEM void AppSculpt::OnButton(const UI::Event &event) {
  using namespace sculpt;
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
        view = CHAIN;
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

  Device* d = Focused();
  switch (event.control) {
    case OC::CONTROL_BUTTON_R:
      if (view == CHAIN) {
        if (d) {
          view = DEVICE;
          cursor = 0;
        }
      } else if (d && d->num_params() > 4) {
        // next page, like tapping a DEVICE button again on the S4
        cursor = (cursor + 4) % 8;
        if (cursor >= d->num_params()) cursor = d->num_params() - 1;
      }
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
  const int dir = event.value;
  const bool left = event.control == OC::CONTROL_ENCODER_L;
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
  if (left) {
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
    gfxPrint(10, 0, d->def.name);
  }
  for (int a = 0; a < 2; a++) {
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
  d->DrawViz(0, 11, 128, 30);

  // parameter grid, one page of 4
  const int page = cursor / 4;
  for (int i = 0; i < 4; i++) {
    const int p = page * 4 + i;
    if (p >= n) break;
    const ParamDef& pd = d->param(p);
    const int x = i * 32;
    gfxPrint(x + 1, 43, pd.name);
    if (pd.nmodes) gfxRect(x + 29, 43, 2, 2); // has modes (hold L + turn R)

    char buf[8];
    const bool show_mode = p == cursor && l_held && pd.nmodes;
    if (show_mode) snprintf(buf, sizeof(buf), "%s", pd.modes[d->mode[p]]);
    else d->Format(p, buf, sizeof(buf));
    gfxPrint(x + 1, 52, buf);
    if (show_mode) gfxInvert(x, 51, 32, 9);

    // value bar
    const int span = pd.max - pd.min;
    if (pd.kind == BI) {
      const int c = x + 15, len = d->val[p] * 15 / std::max(1, (int)pd.max);
      gfxPixel(c, 60);
      if (len > 0) gfxRect(c, 61, len, 2);
      else if (len < 0) gfxRect(c + len, 61, -len, 2);
    } else if (pd.kind == LIST) {
      for (int k = 0; k < pd.nlabels; k++) {
        const int bx = x + 1 + k * 7;
        if (k == d->val[p]) gfxRect(bx, 61, 5, 2);
        else gfxPixel(bx + 2, 62);
      }
    } else {
      gfxRect(x + 1, 61, (d->val[p] - pd.min) * 29 / std::max(1, span), 2);
    }
    if (p == cursor) gfxInvert(x, 42, 32, 9);
  }
}

FLASHMEM void AppSculpt::View() const {
  UpdateMixLevels();
  if (view == DEVICE && Focused()) DrawDevice();
  else DrawChain();
}

/////////////////////////////////////////////////////////////////
// App glue
/////////////////////////////////////////////////////////////////
FLASHMEM void AppSculpt::Init() {
  BaseStart();
}

size_t AppSculpt::SaveAppData(util::StreamBufferWriter &) const { return 0; }
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
