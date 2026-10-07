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

// Loom: a full-screen effect chain app.
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
// Settings are saved to LOOM.BIN on the internal flash when Loom is left
// (e.g. app menu) and with the global save, and loaded at power-up.
//
// While Loom is the active app it claims the audio outputs from AudioIO.
// The Quadrants audio applet chain is left intact and comes back as soon as
// Loom is suspended (e.g. when the app menu opens).

#include "../HSApplication.h"
#include "../AudioIO.h"
#include "../OC_gpio.h"
#include "../HSMIDI.h"
#include "../PhzConfig.h"
#include <new>
#include "../PresetEngine.h"
#include "LoomDevices.h"
#include "LoomMods.h"

namespace loom {
  constexpr int NUM_TRACKS = 2;
  constexpr int COLS = SLOT_COUNT + 1; // + MIX
  constexpr int MIX_COL = SLOT_COUNT;

  struct Track {
    InputDevice* input = nullptr;
    ReelDevice* reel = nullptr;    // SOURCE option 1: a looper after the input
    PlayerDevice* player = nullptr;  // SOURCE option 2: a WAV from the SD card
    WavWriter writer;              // saving SPOOL's loop to the SD card
    MixDevice* mix = nullptr;
    uint8_t choice[SLOT_COUNT] = {};
    uint8_t last_choice[SLOT_COUNT] = {}; // for the bypass toggle
    uint8_t bypass = 0;  // bit per slot (+ MIX_COL: its band stage): passed around
    uint8_t parked = 0;  // bit per slot: bypassed and unpatched (no CPU)
    uint32_t park_at[SLOT_COUNT] = {};  // when a bypassed device gets parked
    uint32_t dry_at[SLOT_COUNT] = {};   // when a device resting at WET 0 gets parked (0: not dry)
    // a device chosen on the panel: the slot fades to dry first, then it
    // changes (at choice_at) and the new one fades in; -1: none waiting
    static_assert(SLOT_COUNT == 5, "next_choice: one -1 per slot");
    int8_t next_choice[SLOT_COUNT] = {-1, -1, -1, -1, -1};
    uint32_t choice_at[SLOT_COUNT] = {};
    Device* inst[SLOT_COUNT][MAX_OPTIONS] = {};
    LoomFade* fade[SLOT_COUNT] = {};  // after each effect slot: bypass fade
    LoomGate* gate = nullptr;         // before ECHO: its tail rings out
    LoomFade* out_fade = nullptr;     // OUTPUT MIX: the source against the chain
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
    // the device chosen, also while the slot still fades out the old one
    int shown_choice(int s) const { return next_choice[s] >= 0 ? next_choice[s] : choice[s]; }
    bool bypassed(int s) const { return s < SLOT_COUNT && (bypass >> s & 1); }
    bool shown_bypassed(int c) const { return bypass >> c & 1; } // incl. OUT
    Device* at(int col) const {
      return col == MIX_COL ? mix : current(col);
    }
    // the effects' order in the chain (they can be moved; SOURCE first and
    // OUT last stay): order[k] = the slot at position k + 1
    uint8_t order[SLOT_COUNT - 1] = {SPRAY, RESONATOR, TEXTURE, ECHO};
    int SlotAt(int pos) const {  // column -> slot
      return pos == SOURCE || pos >= MIX_COL ? pos : order[pos - 1];
    }
    int PosOf(int slot) const {  // slot -> column
      if (slot == SOURCE || slot >= MIX_COL) return slot;
      for (int k = 0; k < SLOT_COUNT - 1; k++) if (order[k] == slot) return k + 1;
      return slot;
    }
  };

  // a patch cord that remembers where Repatch wants it: Want() for each
  // cord in use (Drop() for the rest), then Release() on all takes down only
  // the ones that change and Raise() puts up the new ones. A cord left up
  // keeps the block waiting in it (taking it down would drop that: a click).
  struct LoomCord : AudioConnection {
    AudioStream* want_src = nullptr;
    AudioStream* want_dst = nullptr;
    uint8_t want_si = 0, want_di = 0;
    void Want(AudioStream& s, int si, AudioStream& d, int di) {
      want_src = &s; want_si = si; want_dst = &d; want_di = di;
    }
    void Drop() { want_src = nullptr; }
    bool Stays() const {
      return isConnected && want_src && src == want_src && dst == want_dst &&
             src_index == want_si && dest_index == want_di;
    }
    void Release() { if (!Stays()) disconnect(); }
    void Raise() { if (want_src && !isConnected) connect(*want_src, want_si, *want_dst, want_di); }
  };

  // patch cords live on the heap: no RAM1 until Loom is first opened
  struct Patch {
    AudioConnection in[NUM_TRACKS][2];
    // per slot: in -> fade dry, in -> (gate ->) device, device -> fade wet
    LoomCord link[NUM_TRACKS][SLOT_COUNT][8];
    LoomCord to_mix[NUM_TRACKS][2];
    LoomCord out_mix[NUM_TRACKS][4];  // source (dry) and chain (wet) into the OUTPUT fade
    LoomCord mix_out[NUM_TRACKS][2];
    // every track's output into both tracks' INPUT (OUT L / OUT R)
    AudioConnection chain_ab[2];  // track A's output into track B's INPUT (OUT A)
  };

  Track tracks[NUM_TRACKS];
  LoomMixer* mixer = nullptr;
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
  // then the 32 MIDI maps M1-M32 (page 4)
  constexpr int MIDI_CURSOR = OUT_CURSOR + NUM_CV_OUTS;
  constexpr int LAST_CURSOR = MIDI_CURSOR + MIDIMAP_MAX - 1;
  constexpr bool IsOutCursor(int c) { return c >= OUT_CURSOR && c < MIDI_CURSOR; }
  // what each CV output A-H sends (0-10 V): track L's MOD A-D, track R's
  // MOD A-D, the shared (L+R) E-H or the clock as a gate. Three groups
  // (L / R / LR, the L knob) of up to five (the R knob); off (0) is only
  // left from older files
  enum { OUT_OFF, OUT_L = 1, OUT_R = 5, OUT_SHARED = 9, OUT_CLOCK = 13, OUT_SOURCES };
  // by default A-D = track L's MOD A-D, E-H = track R's
  uint8_t cv_out_src[NUM_CV_OUTS] = {OUT_L, OUT_L + 1, OUT_L + 2, OUT_L + 3, OUT_R, OUT_R + 1, OUT_R + 2, OUT_R + 3};
  uint8_t cv_out_bypass = 0;  // bit per output: 0 V, source kept
  constexpr uint8_t OUT_GROUP_START[3] = {OUT_L, OUT_R, OUT_SHARED};
  constexpr uint8_t OUT_GROUP_SIZE[3] = {4, 4, 5};
  inline int OutGroup(int s) { return s >= OUT_SHARED ? 2 : s >= OUT_R ? 1 : 0; }
  inline int OutIndex(int s) { return s >= OUT_L ? s - OUT_GROUP_START[OutGroup(s)] : 0; }
  FLASHMEM void FormatOutSource(int s, char* buf, size_t n, bool long_name) {
    if (s >= OUT_CLOCK) snprintf(buf, n, long_name ? "clock (beats)" : "CK");
    else if (s >= OUT_SHARED) snprintf(buf, n, long_name ? "A+B MOD %c" : "AB%c", 'E' + s - OUT_SHARED);
    else if (s >= OUT_R) snprintf(buf, n, long_name ? "track B MOD %c" : "B%c", 'A' + s - OUT_R);
    else if (s >= OUT_L) snprintf(buf, n, long_name ? "track A MOD %c" : "A%c", 'A' + s - OUT_L);
    else snprintf(buf, n, long_name ? "off" : "--");
  }

  bool audio_hw() { return I2S2_Audio_ADC && I2S2_Audio_DAC; }
}

// the USB remote's '~' (screen captures): link mode straight back to FREE
void (*g_loom_unlink)() = nullptr;

OC_APP_CLASS(AppLoom, TWOCCS("LM"), "Loom", "Loom"),
  public HSApplication {
public:
  OC_APP_INTERFACE_DECLARE(AppLoom, 0);

  void Start();
  void Resume();
  void Suspend();
  // latch rising edges of the gate inputs and CV inputs (as gates) for the
  // modulators, which only look about once per millisecond
  void Controller() {
    uint16_t e = 0;
    for (int i = 0; i < 12; i++) e |= HS::frame.clocked[i] << i;
    loom::trig_latch |= e;
  }
  void MainLoop();
  void ModulateAll();
  void ReadMIDI();
  void MidiLearn(loom::Device* d);
  void View() const;

  void OnButton(const UI::Event &event);
  void OnEncoder(const UI::Event &event);

private:
  enum ViewMode : uint8_t { CHAIN, DEVICE, MODS };
  ViewMode view = CHAIN;
  bool focus_mod = false;   // the device view shows a modulator
  int8_t mod_cursor = 0;
  elapsedMicros tick_timer;
  int8_t track = 0, col = loom::ECHO, cursor = 0;
  bool l_held = false, l_used = false;
  bool r_held = false, r_moved = false;  // chain view: R held + turned moves an effect
  bool loop_save_held = false;  // SPOOL: A is down (held 2 s saves the loop) ...
  uint32_t loop_save_hold = 0;  // ... since then
  bool save_held = false;   // chain view: Y is down (held 2 s saves) ...
  uint32_t save_hold = 0, saved_at = 0;  // ... since then; when the save happened
  bool clear_held = false;  // SPOOL: B is down on the SPOOL page ...
  uint32_t clear_hold = 0;  // ... since then
  bool editing = false;   // device view: setting grabbed, R enc changes it
  bool out_editing = false;  // CV outputs page: output grabbed, R enc picks its modulator
  // MIDI maps page: the map's editor open (-1: the grid), its field under
  // the cursor (CHAN TYPE VOICE LOW HIGH), the field grabbed
  int8_t map_field = -1;
  bool map_grab = false;
  bool maps_dirty = false;  // changed here: saved with the global settings on leaving the editor
  bool edit_mode = false; // the mode was just changed: show its name
  mutable elapsedMillis mode_shown;
  bool mod_mode = false;  // device view: L enc amount, R enc source
  uint8_t mod_slot = 0;   // ... of the first or the second source
  // the MOD view opened from a device page (X): where Y / X go back to
  struct { bool on; bool focus_mod; int8_t mod_cursor, cursor; } back = {};
  // the clock page opened by holding Z: where Y goes back to
  struct { bool on; ViewMode view; bool focus_mod; int8_t mod_cursor, cursor; } clock_back = {};
  // L enc in modulation: each source comes as U then B (off has neither)
  void StepModOption(loom::Device* d, int p, int dir) {
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
  bool reload_on_resume = false;  // a preset recall replaced LOOM.BIN
  bool linked = false;
  bool link_stereo = false;  // linked as SPLIT
  bool link_rel = false;     // linked as LOOSE: B keeps its values, A's changes add to them (else TIGHT)
  bool threaded = false;     // THREAD: B's source is A's output (not linked)
  int16_t thread_in = -1, thread_spool_in = 0;  // B's own source from before THREAD
  // the link squares: TIGHT both filled, LOOSE both hollow
  bool LinkSquareFilled(bool top) const { (void)top; return !link_rel; }
  uint32_t link_note_at = 0;   // when the link mode changed (shown briefly)
  int16_t unlinked_in[2] = {-1, -1}, unlinked_spool_in[2] = {0, 0};  // input choices from before the link
  // the other track's PLAYER from before the link (its own file comes back)
  struct {
    bool valid = false, ex_valid = false;
    int track = 1;
    char dir[128], name[96];
    int16_t val[loom::MAX_PARAMS];
    uint8_t mode[loom::MAX_PARAMS];
    uint32_t ex[2], sec[2];
  } player_stash;
  mutable bool dirty = false; // something changed since the last save
  float applied_tempo = 120.0f;
  int cpu = 0;
  elapsedMillis stats_timer;

  loom::Device* FocusOn(int t) const;
  void SaveFile() const;
  void LoadFile();
  template <typename F> void ForEachDevice(F f) const;
  loom::Device* Focused() const { return FocusOn(track); }
  // modulator i (0-7) as seen from track t: its row of types and its type
  static loom::Modulator** ModRow(int t, int i) {
    return i < loom::NUM_MODS ? loom::tracks[t].mods[i] : loom::shared_mods[i - loom::NUM_MODS];
  }
  static uint8_t& ModType(int t, int i) {
    return i < loom::NUM_MODS ? loom::tracks[t].mod_type[i] : loom::shared_type[i - loom::NUM_MODS];
  }
  static loom::Modulator* ModAt(int t, int i) {
    if (i > loom::CLK_CURSOR) return nullptr;  // the CV outputs
    if (i == loom::CLK_CURSOR) return loom::tempo_clock;
    return ModRow(t, i)[ModType(t, i)];
  }
  bool SharedFocus() const { return focus_mod && mod_cursor >= loom::NUM_MODS && mod_cursor < loom::CLK_CURSOR; }
  void SetModType(int t, int i, int type);
  void DrawMods() const;
  void DrawOutputs() const;
  static void DrawClockMark(int x, int y);
  void DrawLinkLabel(int x, int y) const;  // L, two squares (filled: SPLIT), R
  float OutLevel(int o) const;
  void WriteOutputs();
  void SetChoice(int t, int s, int option);
  void SetChoiceLinked(int s, int option);
  void ChoiceFaded(int t, int s, int option);
  void SwitchChoices(int t);
  void ToggleBypass(int s);
  loom::Device* Twin(loom::Device* d) const;
  void Mirror(loom::Device* d, int p) const;
  void Repatch(int t);
  void ApplyBypass(int t);
  void ParkBypassed(int t);
  void ConnectTrack(int t);
  void DisconnectTrack(int t);
  void ToggleLink();
public:
  void UnlinkNow();  // straight to FREE, no TIGHT copy on the way (USB remote, for screen captures)
private:
  void LinkCopy();   // the track under the cursor copied to the other one
  // TIGHT copies A onto B; B's own settings are kept here meanwhile and come
  // back when TIGHT is left, so going round the modes loses nothing
  struct DevState {
    bool has = false;
    int16_t val[loom::MAX_PARAMS];
    uint8_t mode[loom::MAX_PARAMS];
    int8_t mod_amt[loom::MAX_PARAMS], mod_amt2[loom::MAX_PARAMS];
    HS::CVInputMap mod_in[loom::MAX_PARAMS];
    uint8_t mod_bus_sel[loom::MAX_PARAMS], mod_bus2[loom::MAX_PARAMS];
    uint8_t mod_bi = 0, mod_bi2 = 0, act_trig[2] = {};
    void Take(const loom::Device* d);
    void Give(loom::Device* d) const;
  };
  struct TrackSnap {
    uint8_t choice[loom::SLOT_COUNT], last_choice[loom::SLOT_COUNT], order[loom::SLOT_COUNT - 1];
    uint8_t bypass, mod_type[loom::NUM_MODS];
    DevState input, reel, player, mix;
    DevState inst[loom::SLOT_COUNT][loom::MAX_OPTIONS];
    DevState mods[loom::NUM_MODS][loom::MOD_TYPES];
  };
  TrackSnap* b_own = nullptr;  // track B before TIGHT
  void KeepB();
  void BringBackB();
  void StereoOn(int master);  // linked as a stereo pair (master: the track whose PLAYER file both play)
  void StereoOff();  // ... and back to each track's own input / file
  void RouteStereo(bool on);
  void SetPans(int l, int r);
  void MoveEffect(int dir);
  int16_t unlinked_pan[2] = {INT16_MIN, 0};  // the tracks' own PAN from before the link
  void UpdateMixLevels() const;
  void DrawChain() const;
  bool Muted() const;
  bool BypassKey() const;
  void DrawDevice() const;
  void DrawClockPage() const;
  void DrawMidiMaps() const;
};

FLASHMEM loom::Device* AppLoom::FocusOn(int t) const {
  return focus_mod ? ModAt(t, mod_cursor) : loom::tracks[t].at(col);
}

FLASHMEM void AppLoom::Start() {
  static AppLoom* self;
  self = this;
  g_loom_unlink = [] { self->UnlinkNow(); };
  using namespace loom;
  // Create every device up front in chain order, so the audio update order
  // follows the signal flow (no extra block of latency per hop). Devices only
  // hold their big buffers while they are in use.
  for (int t = 0; t < NUM_TRACKS; t++) {
    Track& tr = tracks[t];
    tr.input = new InputDevice;
    tr.input->chained = t == 1;  // track B may take track A's output
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
      if (s == ECHO) tr.gate = new LoomGate;
      for (int o = 0; o < SLOT_DEFS[s].noptions; o++) {
        Device* d = MakeDevice(s, o);
        if (d) {
          d->mod_bus = tr.mod_out;
          d->Defaults();
          d->ApplyAll();
        }
        tr.inst[s][o] = d;
      }
      tr.fade[s] = new LoomFade;
      tr.last_choice[s] = SLOT_DEFS[s].noptions > 1 ? 1 : 0;
    }
  }
  // the MIX band stages before the mixer, so the update order follows the
  // signal
  for (int t = 0; t < NUM_TRACKS; t++) {
    tracks[t].out_fade = new LoomFade;
    tracks[t].mix = new MixDevice(t);
    tracks[t].mix->out_fade = tracks[t].out_fade;
  }
  mixer = new LoomMixer;
  for (int t = 0; t < NUM_TRACKS; t++) {
    Track& tr = tracks[t];
    tr.mix->mixer = mixer;
    tr.mix->mod_bus = tr.mod_out;
    tr.mix->Defaults();
    tr.mix->val[MixDevice::PAN] = t ? 25 : -25; // pan the tracks apart a little
    tr.mix->ApplyAll();
    // start with ECHO on both tracks, a bit different on track 2
    tr.choice[ECHO] = 1;
  }
  Device* v2 = tracks[1].inst[ECHO][1];
  v2->val[EchoDevice::TIME] = 680;
  v2->val[EchoDevice::WALL] = 15;
  v2->mode[EchoDevice::SPREAD] = EchoDevice::DIFFUSE;

  tempo_clock = new ClockDevice;
  tempo_clock->mod_bus = tracks[0].mod_out;
  tempo_clock->Defaults();

  LoadFile();
}

FLASHMEM void AppLoom::Repatch(int t) {
  using namespace loom;
  if (!active) return;
  Track& tr = tracks[t];
  auto& link = patch->link[t];
  AudioNoInterrupts();
  for (auto& slot : link) for (auto& c : slot) c.Drop();
  AudioStream* prev = tr.input;
  if (tr.choice[SOURCE] == 1 && tr.reel) {  // SPOOL records and plays after the input
    for (int c = 0; c < 2; c++) link[SOURCE][c].Want(*tr.input, c, *tr.reel->in(), c);
    prev = tr.reel->out();
  } else if (tr.choice[SOURCE] == 2 && tr.player) {  // PLAYER instead of the input
    prev = tr.player->out();
  }
  AudioStream* const source = prev;  // what came in, for OUTPUT's MIX
  for (int k = 0; k < SLOT_COUNT - 1; k++) {  // the effects in their order
    const int s = tr.order[k];
    Device* d = tr.current(s);
    LoomCord* l = link[s];
    // the dry signal always on the slot's first two cords, so it is never
    // taken down when the device comes or goes
    for (int c = 0; c < 2; c++) (l++)->Want(*prev, c, *tr.fade[s], c);
    AudioStream* const before = prev;
    prev = tr.fade[s];
    if (!d || !d->in()) continue;  // empty slot: only the dry signal
    d->Rest(tr.parked >> s & 1);
    // bypassed slots stay patched while their fade (or tail) runs; parked
    // ones only pass the dry signal through the fade (and a device with a
    // recording keeps its input)
    if (tr.parked >> s & 1) {
      if (d->FeedsWhileParked())
        for (int c = 0; c < 2; c++) (l++)->Want(*before, c, *d->in(), c);
      continue;
    }
    AudioStream* feed = before;
    if (s == ECHO && tr.gate) {
      for (int c = 0; c < 2; c++) (l++)->Want(*before, c, *tr.gate, c);
      feed = tr.gate;
    }
    for (int c = 0; c < 2; c++) (l++)->Want(*feed, c, *d->in(), c);
    for (int c = 0; c < 2; c++) (l++)->Want(*d->out(), c, *tr.fade[s], 2 + c);
  }
  // OUTPUT's MIX: the source (dry) against the chain (wet), then through the
  // track's band stage into the mixer
  for (int c = 0; c < 2; c++) {
    patch->out_mix[t][c].Want(*source, c, *tr.out_fade, c);
    patch->out_mix[t][2 + c].Want(*prev, c, *tr.out_fade, 2 + c);
    patch->to_mix[t][c].Want(*tr.out_fade, c, *tr.mix->in(), c);
  }
  for (int c = 0; c < 2; c++) patch->mix_out[t][c].Want(*tr.mix->out(), c, *mixer, t * 2 + c);
  // only the cords that change are taken down (that drops the block waiting
  // in them: a click), all of them first, then the new ones go up
  auto each = [&](auto f) {
    for (auto& slot : link) for (auto& c : slot) f(c);
    for (auto& c : patch->to_mix[t]) f(c);
    for (auto& c : patch->mix_out[t]) f(c);
    for (auto& c : patch->out_mix[t]) f(c);
  };
  each([](LoomCord& c) { c.Release(); });
  each([](LoomCord& c) { c.Raise(); });
  AudioInterrupts();
  ApplyBypass(t);
}

// bypass targets for the track's fades: dry only, or for ECHO dry plus
// the device's output with its input closed, so the tail rings out
FLASHMEM void AppLoom::ApplyBypass(int t) {
  using namespace loom;
  Track& tr = tracks[t];
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
    if (!tr.fade[s]) continue;
    // an empty slot, or one about to change its device: dry only
    if (!tr.current(s) || tr.next_choice[s] >= 0) {
      tr.fade[s]->Set(1.0f, 0.0f);
      if (s == ECHO) tr.gate->Set(0.0f);
      continue;
    }
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

FLASHMEM void AppLoom::SetChoice(int t, int s, int option) {
  using namespace loom;
  Track& tr = tracks[t];
  option = constrain(option, 0, SLOT_DEFS[s].noptions - 1);
  tr.next_choice[s] = -1;
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

FLASHMEM void AppLoom::ConnectTrack(int t) {
  using namespace loom;
  AudioNoInterrupts();
  for (int c = 0; c < 2; c++)
    patch->in[t][c].connect(OC::AudioIO::InputStream(), c, *tracks[t].input, c);
  AudioInterrupts();
  for (int s = SOURCE; s < SLOT_COUNT; s++) {
    if (Device* d = tracks[t].current(s)) d->Activate();
  }
  Repatch(t);
}

FLASHMEM void AppLoom::DisconnectTrack(int t) {
  using namespace loom;
  AudioNoInterrupts();
  for (auto& c : patch->in[t]) c.disconnect();
  for (auto& slot : patch->link[t]) for (auto& c : slot) c.disconnect();
  for (auto& c : patch->to_mix[t]) c.disconnect();
  for (auto& c : patch->mix_out[t]) c.disconnect();
  for (auto& c : patch->out_mix[t]) c.disconnect();
  AudioInterrupts();
  // disconnected devices stop updating; now they can drop their buffers
  for (int s = SOURCE; s < SLOT_COUNT; s++) {
    if (Device* d = tracks[t].current(s)) d->Deactivate();
  }
}

// the same device on the other track (only meaningful while linked)
FLASHMEM loom::Device* AppLoom::Twin(loom::Device* d) const {
  using namespace loom;
  if (!linked || !d) return nullptr;
  Device* tw = FocusOn(1 - track);
  return (tw && tw != d && &tw->def == &d->def) ? tw : nullptr;  // shared: no twin
}

// copy one parameter to the twin; input source and pan stay per track
FLASHMEM void AppLoom::Mirror(loom::Device* d, int p) const {
  using namespace loom;
  Device* tw = Twin(d);
  if (!tw) return;
  // PLAYER's FILE goes along only when linked as SPLIT (one file for both)
  if (!focus_mod && col == SOURCE && p == 0 && !(d == tracks[track].player && link_stereo)) return;
  if (!focus_mod && col == MIX_COL && p == MixDevice::PAN) return;
  if (link_rel) {
    // LINKED: B keeps its own value and moves by as much as A just did
    if (d->last_delta[p]) {
      const ParamDef& pd = tw->param(p);
      tw->val[p] = constrain(tw->val[p] + d->last_delta[p], pd.min, pd.max);
      d->last_delta[p] = 0;
      if (tw == tracks[1 - track].player) tracks[1 - track].player->ex_valid = false;  // its own edges, roughly
    }
  } else {
    tw->val[p] = d->val[p];
  }
  tw->mode[p] = d->mode[p];
  tw->mod_amt[p] = d->mod_amt[p];
  tw->mod_in[p] = d->mod_in[p];
  tw->mod_bus_sel[p] = d->mod_bus_sel[p];
  tw->mod_bi = (tw->mod_bi & ~(1 << p)) | (d->mod_bi & (1 << p));
  tw->mod_amt2[p] = d->mod_amt2[p];
  tw->mod_bus2[p] = d->mod_bus2[p];
  tw->mod_bi2 = (tw->mod_bi2 & ~(1 << p)) | (d->mod_bi2 & (1 << p));
  if (d == tracks[track].player && link_stereo) {  // and the exact edges (both, one may have pushed the other)
    PlayerDevice* a = tracks[track].player;
    PlayerDevice* b = tracks[1 - track].player;
    b->ex_pos[0] = a->ex_pos[0]; b->ex_pos[1] = a->ex_pos[1];
    b->ex_valid = a->ex_valid;
    for (int q : {int(PlayerDevice::START), int(PlayerDevice::END)}) b->val[q] = a->val[q];
    b->sec_off = a->sec_off; b->sec_len = a->sec_len;
    b->Refresh(PlayerDevice::START);
    b->Refresh(PlayerDevice::END);
  }
  tw->Refresh(p);
}

FLASHMEM void AppLoom::SetModType(int t, int i, int type) {
  using namespace loom;
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

FLASHMEM void AppLoom::SetChoiceLinked(int s, int option) {
  using namespace loom;
  if (s == SOURCE) {
    SetChoice(track, s, option);
    if (linked) SetChoice(1 - track, s, tracks[track].choice[s]);
    return;
  }
  option = constrain(option, 0, SLOT_DEFS[s].noptions - 1);
  for (int t = 0; t < NUM_TRACKS; t++)
    if (t == track || linked) ChoiceFaded(t, s, option);
}

// a device change on the panel without a click: while the old device is
// heard, the slot first fades to dry (ApplyBypass) and SwitchChoices()
// changes it after the fade; the new one then fades in
FLASHMEM void AppLoom::ChoiceFaded(int t, int s, int option) {
  using namespace loom;
  Track& tr = tracks[t];
  const bool heard = active && tr.current(s) && !(tr.parked >> s & 1);
  if (!heard) {
    tr.next_choice[s] = -1;
    SetChoice(t, s, option);
    return;
  }
  if (option == tr.choice[s]) {  // back to the one still there
    tr.next_choice[s] = -1;
    ApplyBypass(t);
    return;
  }
  if (tr.next_choice[s] < 0) tr.choice_at[s] = millis() + 20;  // 15 ms fade + a block
  tr.next_choice[s] = option;
  ApplyBypass(t);
}

FLASHMEM void AppLoom::SwitchChoices(int t) {
  using namespace loom;
  Track& tr = tracks[t];
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
    if (tr.next_choice[s] < 0 || int32_t(millis() - tr.choice_at[s]) < 0) continue;
    const int option = tr.next_choice[s];
    tr.next_choice[s] = -1;
    SetChoice(t, s, option);
    ApplyBypass(t);
  }
}

// one-click bypass of a slot: the device stays (and keeps its settings and
// page) but the track's audio goes around it
// unpatch devices that have been bypassed long enough (fade done, tail rung
// out), so they stop using CPU
FLASHMEM void AppLoom::ParkBypassed(int t) {
  using namespace loom;
  Track& tr = tracks[t];
  uint8_t fresh = 0;
  bool woke = false;
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++) {
    const uint8_t bit = 1 << s;
    const Device* d = tr.current(s);
    // a device that rests at WET 0 (nothing of it heard): parked once its
    // fade is done, back in the chain as soon as WET opens again
    const bool dry = d && d->RestsWhenDry() && d->wet <= 0.0f && !(tr.bypass & bit);
    if (!dry) tr.dry_at[s] = 0;
    else if (!tr.dry_at[s]) tr.dry_at[s] = millis() + 300;
    if (!dry && !(tr.bypass & bit) && (tr.parked & bit)) {
      tr.parked &= ~bit;
      woke = true;
      continue;
    }
    if ((tr.parked & bit) || !d) continue;
    if (dry) {
      if (int32_t(millis() - tr.dry_at[s]) < 0) continue;
    } else if (!(tr.bypass & bit) || int32_t(millis() - tr.park_at[s]) < 0) {
      continue;
    }
    fresh |= bit;
  }
  if (woke && !fresh) Repatch(t);
  if (!fresh) return;
  tr.parked |= fresh;
  Repatch(t);
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++)
    if (fresh >> s & 1) tr.current(s)->Parked();
}

FLASHMEM void AppLoom::ToggleBypass(int s) {
  using namespace loom;
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

// UP+DOWN cycles: off -> LINKED -> MIRROR -> SPLIT -> off. Linked, the controls
// of both tracks are the same; SPLIT also makes them one stereo pair (track
// L the left input / left side of the PLAYER's file, track R the right),
// MIRROR copies A to B; LINKED keeps B's values and moves them by A's changes; both keep each track's own input and file.
// A+B cycles FREE -> THREAD -> LOOSE -> TIGHT -> FREE. FREE: two separate
// stereo tracks. THREAD: B's source is A's output, the two chains one after
// the other (controls separate, A still heard). LOOSE: B keeps its own
// values, A's changes move them by as much. TIGHT: B becomes a copy of A and
// follows it. (SPLIT, one stereo pair over both tracks, is still here but
// not in the cycle.)
FLASHMEM void AppLoom::ToggleLink() {
  using namespace loom;
  if (!linked && !threaded) {
    // THREAD: B listens to A (its own source comes back after)
    Track& b = tracks[1];
    thread_in = b.input->val[0];
    thread_spool_in = b.reel ? b.reel->val[ReelDevice::IN] : 0;
    b.input->val[0] = InputDevice::OUT_A;
    b.input->ApplyAll();
    if (b.reel) { b.reel->val[ReelDevice::IN] = InputDevice::OUT_A; b.reel->ApplyAll(); }
    threaded = true;
  } else if (threaded) {
    // B's own source again, then LOOSE: B keeps its own values; A's changes
    // move them by as much
    Track& b = tracks[1];
    b.input->val[0] = thread_in >= 0 ? thread_in : 0;
    b.input->ApplyAll();
    if (b.reel) { b.reel->val[ReelDevice::IN] = thread_in >= 0 ? thread_spool_in : 0; b.reel->ApplyAll(); }
    thread_in = -1;
    threaded = false;
    unlinked_pan[0] = tracks[0].mix->val[MixDevice::PAN];
    unlinked_pan[1] = tracks[1].mix->val[MixDevice::PAN];
    linked = true;
    link_rel = true;
    link_stereo = false;
    track = 0;  // the cursor lives on row A while linked
  } else if (link_rel) {
    link_rel = false;  // TIGHT
    KeepB();
    LinkCopy();
    SetPans(0, 0);
  } else {
    if (link_stereo) StereoOff();
    link_stereo = false;
    linked = false;    // FREE
    BringBackB();      // B as it was before TIGHT
    if (unlinked_pan[0] != INT16_MIN) SetPans(unlinked_pan[0], unlinked_pan[1]);  // their own again
    unlinked_pan[0] = INT16_MIN;
  }
  link_note_at = millis() | 1;  // the mode, shown for a moment
}

FLASHMEM void AppLoom::UnlinkNow() {
  using namespace loom;
  if (threaded) {
    Track& b = tracks[1];
    b.input->val[0] = thread_in >= 0 ? thread_in : 0;
    b.input->ApplyAll();
    if (b.reel) { b.reel->val[ReelDevice::IN] = thread_in >= 0 ? thread_spool_in : 0; b.reel->ApplyAll(); }
    thread_in = -1;
    threaded = false;
  }
  if (linked) {
    if (link_stereo) StereoOff();
    link_stereo = link_rel = linked = false;
    BringBackB();
    if (unlinked_pan[0] != INT16_MIN) SetPans(unlinked_pan[0], unlinked_pan[1]);
    unlinked_pan[0] = INT16_MIN;
  }
  link_note_at = millis() | 1;
}

FLASHMEM void AppLoom::StereoOff() {
  using namespace loom;
  RouteStereo(false);
  // each track's own input choice back (as before the link)
  for (int t = 0; t < NUM_TRACKS && unlinked_in[t] >= 0; t++) {
    Track& tr = tracks[t];
    tr.input->val[0] = unlinked_in[t];
    tr.input->ApplyAll();
    if (tr.reel) { tr.reel->val[ReelDevice::IN] = unlinked_spool_in[t]; tr.reel->ApplyAll(); }
  }
  unlinked_in[0] = unlinked_in[1] = -1;
  // the PLAYERs: stereo again, the other track's own file back
  for (auto& tr : tracks) if (tr.player) tr.player->SetPick(0);
  if (player_stash.valid && tracks[player_stash.track].player) {
    PlayerDevice* p = tracks[player_stash.track].player;
    memcpy(p->val, player_stash.val, sizeof(p->val));
    memcpy(p->mode, player_stash.mode, sizeof(p->mode));
    p->ex_pos[0] = player_stash.ex[0]; p->ex_pos[1] = player_stash.ex[1];
    p->ex_valid = p->keep_exact = player_stash.ex_valid;
    p->sec_off = player_stash.sec[0]; p->sec_len = player_stash.sec[1];
    p->ApplyAll();
    p->SetFile(player_stash.dir, player_stash.name);
  }
  player_stash.valid = false;
}

FLASHMEM void AppLoom::DevState::Take(const loom::Device* d) {
  has = d != nullptr;
  if (!d) return;
  memcpy(val, d->val, sizeof(val));
  memcpy(mode, d->mode, sizeof(mode));
  memcpy(mod_amt, d->mod_amt, sizeof(mod_amt));
  memcpy(mod_amt2, d->mod_amt2, sizeof(mod_amt2));
  for (int p = 0; p < loom::MAX_PARAMS; p++) mod_in[p] = d->mod_in[p];
  memcpy(mod_bus_sel, d->mod_bus_sel, sizeof(mod_bus_sel));
  memcpy(mod_bus2, d->mod_bus2, sizeof(mod_bus2));
  mod_bi = d->mod_bi; mod_bi2 = d->mod_bi2;
  memcpy(act_trig, d->act_trig, sizeof(act_trig));
}

FLASHMEM void AppLoom::DevState::Give(loom::Device* d) const {
  if (!has || !d) return;
  memcpy(d->val, val, sizeof(val));
  memcpy(d->mode, mode, sizeof(mode));
  memcpy(d->mod_amt, mod_amt, sizeof(mod_amt));
  memcpy(d->mod_amt2, mod_amt2, sizeof(mod_amt2));
  for (int p = 0; p < loom::MAX_PARAMS; p++) d->mod_in[p] = mod_in[p];
  memcpy(d->mod_bus_sel, mod_bus_sel, sizeof(mod_bus_sel));
  memcpy(d->mod_bus2, mod_bus2, sizeof(mod_bus2));
  d->mod_bi = mod_bi; d->mod_bi2 = mod_bi2;
  memcpy(d->act_trig, act_trig, sizeof(act_trig));
  d->ApplyAll();
}

FLASHMEM void AppLoom::KeepB() {
  using namespace loom;
  if (!b_own) b_own = new (std::nothrow) TrackSnap;
  if (!b_own) return;  // no memory: TIGHT just copies, as before
  const Track& b = tracks[1];
  TrackSnap& k = *b_own;
  memcpy(k.choice, b.choice, sizeof(k.choice));
  memcpy(k.last_choice, b.last_choice, sizeof(k.last_choice));
  memcpy(k.order, b.order, sizeof(k.order));
  k.bypass = b.bypass;
  memcpy(k.mod_type, b.mod_type, sizeof(k.mod_type));
  k.input.Take(b.input);
  k.reel.Take(b.reel);
  k.player.Take(b.player);
  k.mix.Take(b.mix);
  for (int s = 0; s < SLOT_COUNT; s++)
    for (int o = 0; o < MAX_OPTIONS; o++) k.inst[s][o].Take(s > SOURCE ? b.inst[s][o] : nullptr);
  for (int i = 0; i < NUM_MODS; i++)
    for (int t = 0; t < MOD_TYPES; t++) k.mods[i][t].Take(b.mods[i][t]);
}

FLASHMEM void AppLoom::BringBackB() {
  using namespace loom;
  if (!b_own) return;
  Track& b = tracks[1];
  const TrackSnap& k = *b_own;
  for (int s = SOURCE; s < SLOT_COUNT; s++) SetChoice(1, s, k.choice[s]);
  memcpy(b.last_choice, k.last_choice, sizeof(b.last_choice));
  memcpy(b.order, k.order, sizeof(b.order));
  b.bypass = k.bypass;
  b.parked = 0;
  for (auto& at : b.park_at) at = millis() + 40;
  k.input.Give(b.input);
  k.reel.Give(b.reel);
  k.player.Give(b.player);
  k.mix.Give(b.mix);
  for (int s = SOURCE + 1; s < SLOT_COUNT; s++)
    for (int o = 0; o < MAX_OPTIONS; o++) k.inst[s][o].Give(b.inst[s][o]);
  Repatch(1);
  if (active) ApplyBypass(1);
  for (int i = 0; i < NUM_MODS; i++) {
    for (int t = 1; t < MOD_TYPES; t++) k.mods[i][t].Give(b.mods[i][t]);
    SetModType(1, i, k.mod_type[i]);
  }
  delete b_own;
  b_own = nullptr;
}

FLASHMEM void AppLoom::LinkCopy() {
  using namespace loom;
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
  memcpy(b.order, a.order, sizeof(b.order));  // the same effect order
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
  b.mix->val[MixDevice::LEVEL] = a.mix->val[MixDevice::LEVEL];
  b.mix->val[MixDevice::MUTE] = a.mix->val[MixDevice::MUTE];
  b.mix->val[MixDevice::DRYWET] = a.mix->val[MixDevice::DRYWET];
  b.mix->ApplyAll();
  track = 0; // the cursor lives on row 1 while linked
}

// SPLIT: track L all to the left output, track R all to the right
// the effect under the cursor one place left / right along the chain
// (both tracks when linked)
FLASHMEM void AppLoom::MoveEffect(int dir) {
  using namespace loom;
  for (int t = 0; t < NUM_TRACKS; t++) {
    if (t != track && !linked) continue;
    Track& tr = tracks[t];
    const int from = tr.PosOf(col) - 1;
    const int to = constrain(from + (dir > 0 ? 1 : -1), 0, SLOT_COUNT - 2);
    if (to == from) continue;
    std::swap(tr.order[from], tr.order[to]);
    Repatch(t);
  }
  dirty = true;
}

FLASHMEM void AppLoom::SetPans(int l, int r) {
  using namespace loom;
  tracks[0].mix->val[MixDevice::PAN] = l;
  tracks[1].mix->val[MixDevice::PAN] = r;
  for (auto& tr : tracks) tr.mix->ApplyAll();
}

FLASHMEM void AppLoom::RouteStereo(bool on) {
  using namespace loom;
  for (int t = 0; t < NUM_TRACKS; t++) {
    mixer->route[t] = on ? uint8_t(t + 1) : 0;
    tracks[t].mix->ApplyAll();
  }
}

FLASHMEM void AppLoom::StereoOn(int master) {
  using namespace loom;
  SetPans(-50, 50);  // SPLIT: track L hard left, track R hard right
  RouteStereo(true);
  Track& a = tracks[master];
  Track& b = tracks[1 - master];
  // linked = one stereo pair: track L hears the left input, track R the
  // right one (their own choices come back with MIRROR / unlinking)
  for (int t = 0; t < NUM_TRACKS; t++) {
    Track& tr = tracks[t];
    unlinked_in[t] = tr.input->val[0];
    unlinked_spool_in[t] = tr.reel ? tr.reel->val[ReelDevice::IN] : 0;
    tr.input->val[0] = t ? InputDevice::IN2 : InputDevice::IN1;
    tr.input->ApplyAll();
    if (tr.reel) { tr.reel->val[ReelDevice::IN] = t ? InputDevice::IN2 : InputDevice::IN1; tr.reel->ApplyAll(); }
  }
  // the PLAYERs: track R plays track L's file, each its own side of it
  if (a.player && b.player) {
    PlayerDevice* p = b.player;
    player_stash.track = 1 - master;
    snprintf(player_stash.dir, sizeof(player_stash.dir), "%s", p->Folder());
    snprintf(player_stash.name, sizeof(player_stash.name), "%s", p->FileName());
    memcpy(player_stash.val, p->val, sizeof(player_stash.val));
    memcpy(player_stash.mode, p->mode, sizeof(player_stash.mode));
    player_stash.ex[0] = p->ex_pos[0]; player_stash.ex[1] = p->ex_pos[1];
    player_stash.ex_valid = p->ex_valid;
    player_stash.sec[0] = p->sec_off; player_stash.sec[1] = p->sec_len;
    player_stash.valid = true;
    p->LinkFrom(*a.player);
    tracks[0].player->SetPick(1);
    tracks[1].player->SetPick(2);
  }
}

FLASHMEM void AppLoom::Resume() {
  // SPRAY: at most 8 overlapping grains while Loom runs (CPU peaks)
  AudioEffectClouds::grain_limit = 8;
  using namespace loom;
  if (!audio_hw() || active) return;
  if (reload_on_resume) {  // a preset was recalled: its settings
    reload_on_resume = false;
    LoadFile();
    dirty = false;
  }
  if (!patch) patch = new Patch;
  active = true;
  for (int t = 0; t < NUM_TRACKS; t++) ConnectTrack(t);
  AudioNoInterrupts();
  for (int c = 0; c < 2; c++) patch->chain_ab[c].connect(*tracks[0].mix->out(), c, *tracks[1].input, 2 + c);
  AudioInterrupts();
  OC::AudioIO::ClaimOutput(*mixer);
  OC::AudioIO::ResyncInput();  // L and R in the same order on every start
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  stats_timer = 0;
}

FLASHMEM void AppLoom::Suspend() {
  // the Quadrants applets get all their grains back
  AudioEffectClouds::grain_limit = AudioEffectClouds::MAX_GRAINS;
  using namespace loom;
  if (!active) return;
  OC::AudioIO::ReleaseOutput();
  AudioNoInterrupts();
  for (auto& c : patch->chain_ab) c.disconnect();
  AudioInterrupts();
  for (int t = 0; t < NUM_TRACKS; t++) DisconnectTrack(t);
  active = false;
  l_held = false;
  // a preset recall that brings Loom's settings: no point saving the ones
  // about to be replaced; they are read in again on resume
  if (OC::PresetEngine::RecallReplacing() & OC::PresetEngine::REPLACES_LOOM) {
    reload_on_resume = true;
    return;
  }
  if (dirty) SaveFile();
}

// MIDI is normally read by Quadrants; while Loom runs it feeds the same
// shared MIDI maps itself (no thru, no SysEx, no program changes)
template <typename T>
FLASHMEM static void LoomReadMIDI(T& device) {
  for (int n = 0; n < 32 && device.read(); n++) {
    const HS::MIDIMessage msg = {
      device.getChannel(), device.getType(), device.getData1(), device.getData2()
    };
    if (msg.message == midi::SystemExclusive || msg.message == midi::ProgramChange) continue;
    if (msg.message == midi::Clock) loom::midi_clocks++;
    if (loom::tempo_clock && loom::tempo_clock->midi()) {
      if (msg.message == midi::Start) loom::ClockStart();
      else if (msg.message == midi::Continue) loom::clock_running = true;
      else if (msg.message == midi::Stop) loom::ClockStop();
    }
    const bool realtime = msg.message == midi::Clock || msg.message == midi::Start
      || msg.message == midi::Stop;
    if (realtime || (msg.message >> 4) != 0xF) HS::frame.MIDIState.ProcessMIDIMsg(msg);
  }
}

FLASHMEM void AppLoom::ReadMIDI() {
  LoomReadMIDI(usbMIDI);
  LoomReadMIDI(usbHostMIDI[0]);
  LoomReadMIDI(usbHostMIDI[1]);
  LoomReadMIDI(MIDI1);
}

// point the setting at a free MIDI map (or its current one) and let the next
// CC or note teach it
FLASHMEM void AppLoom::MidiLearn(loom::Device* d) {
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

FLASHMEM void AppLoom::ModulateAll() {
  using namespace loom;
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
FLASHMEM float AppLoom::OutLevel(int o) const {
  using namespace loom;
  const int s = cv_out_src[o];
  float v = -1.0f;
  if (cv_out_bypass & (1 << o)) return 0.0f;
  if (s >= OUT_CLOCK) v = clock_running && tempo_clock->out < 0.5f ? 1.0f : 0.0f;
  else if (s >= OUT_SHARED) v = tracks[0].mod_out[NUM_MODS + s - OUT_SHARED];
  else if (s >= OUT_R) v = tracks[1].mod_out[s - OUT_R];
  else if (s >= OUT_L) v = tracks[0].mod_out[s - OUT_L];
  return constrain(v, 0.0f, 1.0f);  // empty slot (-1) or off: 0 V
}

FLASHMEM void AppLoom::WriteOutputs() {
  for (int o = 0; o < loom::NUM_CV_OUTS && o < DAC_CHANNEL_COUNT; o++)
    HS::frame.Out(DAC_CHANNEL(o), int(OutLevel(o) * 8.5f * ONE_OCTAVE));  // measured: 8.5 "volts" give 10 V at the jack
}

FLASHMEM void AppLoom::MainLoop() {
  if (active) ReadMIDI();
  {
    using namespace loom;
    // SPOOL: A held 2 s starts writing the loop; the files go in steps
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
      if (tr.reel && tr.choice[SOURCE] == 1) tr.reel->Idle();
      for (int s = SOURCE + 1; s < SLOT_COUNT; s++)  // devices with main-loop work (STRETCH)
        if (Device* d = tr.current(s)) d->Idle();
    }
    // the output is muted while a PLAYER reads a file in (the card reading
    // made a high whine on the output)
    bool busy = false;
    for (auto& tr : tracks) busy |= tr.player && tr.choice[SOURCE] == 2 && tr.player->Busy();
    mixer->mute = busy;
    if (loop_save_held && ms >= 2000) loop_save_held = false;
  }
  if (save_held && millis() - save_hold >= 2000) {
    save_held = false;
    SaveFile();  // a short audio stop while the flash is written
    saved_at = millis() | 1;
  }
  {
    using namespace loom;
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
    for (int t = 0; t < loom::NUM_TRACKS; t++) {
      SwitchChoices(t);
      ApplyBypass(t);  // follows SPRAY LEVEL
      ParkBypassed(t);
    }
  }
  if (!active || stats_timer < 1000) return;
  stats_timer = 0;
  const float usage = AudioProcessorUsageMax();
  cpu = int(usage + 0.5f);
  Serial.printf("Loom: AudioProcessorUsageMax %d.%d%%  mem %u/%d blocks  PSRAM %dMB\n",
    int(usage), int(usage * 10) % 10, AudioMemoryUsageMax(), OC::AudioIO::AUDIO_MEMORY, external_psram_size);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  Serial.printf("  load:%s\n", loom::load_note);
  for (int t = 0; t < loom::NUM_TRACKS; t++)  // SPOOL's memory: the loop, or the small buffer
    if (loom::tracks[t].reel && loom::tracks[t].choice[loom::SOURCE] == 1)
      Serial.printf("  SPOOL %c: %u frames held, loop %u\n", 'A' + t,
        unsigned(loom::tracks[t].reel->HeldFrames()), unsigned(loom::tracks[t].reel->LoopLength()));
  for (int t = 0; t < loom::NUM_TRACKS; t++) {
    if (!loom::tracks[t].player || loom::tracks[t].choice[loom::SOURCE] != 2) continue;
    char dbg[200];
    loom::tracks[t].player->Debug(dbg, sizeof(dbg));
    Serial.printf("  PLY%c %s\n", t ? 'R' : 'L', dbg);
  }
  // modulated settings: value, result, both sources (amount, MOD, u/b)
  for (int t = 0; t < loom::NUM_TRACKS; t++) {
    const loom::Track& tr = loom::tracks[t];
    Serial.printf("  %c mods", t ? 'R' : 'L');
    for (int i = 0; i < loom::MOD_BUSES; i++) {
      const loom::Modulator* m = ModAt(t, i);
      Serial.printf(" %c:%s=%d.%02d", 'A' + i, m ? m->def.tag : "-", int(tr.mod_out[i]), int(tr.mod_out[i] * 100) % 100);
    }
    Serial.println();
    ForEachDevice([&](loom::Device* d) {
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
  using namespace loom;
  for (int t = 0; t < NUM_TRACKS; t++) {
    Serial.printf("  %c [bypass %02x parked %02x]", t ? 'R' : 'L', tracks[t].bypass, tracks[t].parked);
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
FLASHMEM void AppLoom::OnButton(const UI::Event &event) {
  using namespace loom;
  dirty = true;
  if (event.control == OC::CONTROL_BUTTON_Z) {
    // anywhere in the app: clock start / stop; held: the clock page, on
    // letting go (Z held + an encoder push is a shortcut, which swallows
    // the release)
    if (event.type == UI::EVENT_BUTTON_PRESS) {
      clock_running ? ClockStop() : ClockStart();
    } else if (event.type == UI::EVENT_BUTTON_LONG_RELEASE && !(view == DEVICE && focus_mod && mod_cursor == CLK_CURSOR)) {
      clock_back = {true, view, focus_mod, mod_cursor, cursor};
      view = DEVICE;
      focus_mod = true;
      mod_cursor = CLK_CURSOR;
      cursor = 0;
      editing = mod_mode = trig_mode = out_editing = false;
    }
    return;
  }
  if (event.control == OC::CONTROL_BUTTON_R) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      r_held = view == CHAIN;
      r_moved = false;
      return;
    }
    if (event.type == UI::EVENT_BUTTON_PRESS || event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      r_held = false;
      if (r_moved) { r_moved = false; return; }  // it was a move, not a push
      if (event.type == UI::EVENT_BUTTON_LONG_RELEASE) return;
    }
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
      // CV outputs page: L push bypasses the output (0 V)
      if (view == MODS && IsOutCursor(mod_cursor)) cv_out_bypass ^= 1 << (mod_cursor - OUT_CURSOR);
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
  // SPOOL: B held for 3 s clears the loop (see MainLoop); followed by the
  // button's own down / release events
  // SPOOL: A held for 2 s writes the loop to the SD card (see MainLoop)
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
  // MIDI maps page: R push opens the map (then grabs a field), A learns
  // (the next CC or note), B clears it, Y steps back
  if (view == MODS && mod_cursor >= MIDI_CURSOR) {
    MIDIMapping& map = HS::frame.MIDIState.mapping[mod_cursor - MIDI_CURSOR];
    switch (event.control) {
      case OC::CONTROL_BUTTON_R:
        if (map_field < 0) { map_field = 0; map_grab = false; }
        else map_grab = !map_grab;
        break;
      case OC::CONTROL_BUTTON_UP:
        map.AutoLearn();
        maps_dirty = true;
        break;
      case OC::CONTROL_BUTTON_DOWN:
        map.Init();
        HS::frame.MIDIState.UpdateMidiChannelFilter();
        HS::frame.MIDIState.UpdateMaxPolyphony();
        maps_dirty = true;
        break;
      case OC::CONTROL_BUTTON_Y:
        if (map_grab) map_grab = false;
        else if (map_field >= 0) {
          map_field = -1;
          if (maps_dirty) { maps_dirty = false; OC::SaveAppData(); }
        } else {
          view = CHAIN;
        }
        break;
      case OC::CONTROL_BUTTON_X:
        map_field = -1; map_grab = false;
        if (maps_dirty) { maps_dirty = false; OC::SaveAppData(); }
        if (back.on) BackToDevice(); else view = CHAIN;
        break;
      default: break;
    }
    return;
  }
  switch (event.control) {
    case OC::CONTROL_BUTTON_R:
      if (view == MODS && IsOutCursor(mod_cursor)) {
        out_editing = !out_editing;
      } else if (view == MODS) {
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
      if (view == MODS && out_editing) out_editing = false;
      else if (view == DEVICE && trig_mode) trig_mode = false;
      else if (view == DEVICE && clock_back.on && focus_mod && mod_cursor == CLK_CURSOR) {
        view = clock_back.view;
        focus_mod = clock_back.focus_mod;
        mod_cursor = clock_back.mod_cursor;
        cursor = clock_back.cursor;
        clock_back.on = false;
      }
      else if (view == MODS && back.on) BackToDevice();
      else view = (view == DEVICE && focus_mod) ? MODS : CHAIN;
      editing = mod_mode = false;
      break;
    case OC::CONTROL_BUTTON_X:
      out_editing = clock_back.on = false;
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

FLASHMEM void AppLoom::OnEncoder(const UI::Event &event) {
  using namespace loom;
  dirty = true;
  const int dir = event.value;
  const bool left = event.control == OC::CONTROL_ENCODER_L;
  if (view == DEVICE && !focus_mod && col == SOURCE && tracks[track].player &&
      tracks[track].player->browsing && Focused() == tracks[track].player) {
    tracks[track].player->BrowseMove(dir);  // either knob scrolls the browser
    return;
  }
  if (view == MODS && mod_cursor >= MIDI_CURSOR && map_field >= 0) {
    // the map's editor: R moves through the fields; grabbed, it changes one
    if (left) return;
    if (!map_grab) { map_field = constrain(map_field + dir, 0, 4); return; }
    MIDIMapping& map = HS::frame.MIDIState.mapping[mod_cursor - MIDI_CURSOR];
    switch (map_field) {
      case 0: map.AdjustChannel(dir); break;
      case 1: map.AdjustFunction(dir); break;
      case 2: map.AdjustVoice(dir); break;
      case 3: map.AdjustRangeLow(dir); break;
      case 4: map.AdjustRangeHigh(dir); break;
    }
    HS::frame.MIDIState.UpdateMidiChannelFilter();
    HS::frame.MIDIState.UpdateMaxPolyphony();
    maps_dirty = true;
    return;
  }
  if (view == MODS) {
    if (!left) {
      if (out_editing) {
        // the modulator within the output's group
        uint8_t& s = cv_out_src[mod_cursor - OUT_CURSOR];
        const int g = OutGroup(s);
        s = OUT_GROUP_START[g] + constrain(OutIndex(s) + dir, 0, OUT_GROUP_SIZE[g] - 1);
      } else {
        mod_cursor = constrain(mod_cursor + dir, 0, LAST_CURSOR);
      }
    } else if (IsOutCursor(mod_cursor)) {
      // L / R / LR, keeping the letter's place (A-D <-> E-H)
      uint8_t& s = cv_out_src[mod_cursor - OUT_CURSOR];
      const int g = constrain(OutGroup(s) + dir, 0, 2);
      s = OUT_GROUP_START[g] + std::min<int>(OutIndex(s), OUT_GROUP_SIZE[g] - 1);
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
      if (r_held && col > SOURCE && col < MIX_COL) {
        // R held: the effect under the cursor moves along the chain
        MoveEffect(dir);
        r_moved = true;
        return;
      }
      const int idx = constrain(track * COLS + tracks[track].PosOf(col) + dir, 0, rows * COLS - 1);
      track = idx / COLS;
      col = tracks[track].SlotAt(idx % COLS);
    } else if (col == MIX_COL) {
      tracks[track].mix->Nudge(MixDevice::LEVEL, dir);
      Mirror(tracks[track].mix, MixDevice::LEVEL);
    } else {
      SetChoiceLinked(col, tracks[track].shown_choice(col) + dir);  // SOURCE: INPUT / SPOOL
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
FLASHMEM void AppLoom::UpdateMixLevels() const {
  using namespace loom;
  if (!mixer) return;
  float pk[2][2];
  mixer->ReadPeaks(pk);
  for (int t = 0; t < NUM_TRACKS; t++) {
    tracks[t].mix->level[0] = pk[t][0];
    tracks[t].mix->level[1] = pk[t][1];
  }
}

// the track on screen is muted (while linked: either of them)
FLASHMEM bool AppLoom::Muted() const {
  using namespace loom;
  for (int t = 0; t < NUM_TRACKS; t++)
    if ((t == track || linked) && tracks[t].mix && tracks[t].mix->Muted()) return true;
  return false;
}

// on an effect slot's page B (DOWN) bypasses the slot (ECHO's CLR stays
// reachable through a trigger); on OUTPUT it bypasses the band stage
FLASHMEM bool AppLoom::BypassKey() const {
  using namespace loom;
  if (view != DEVICE || focus_mod || col <= SOURCE || col > MIX_COL) return false;
  return Focused() != nullptr;
}

FLASHMEM void AppLoom::DrawChain() const {
  using namespace loom;
  // boxes are drawn one pixel wider than the spacing so neighbours share a
  // border; 19 px inside each. Glyphs are 6 px with a blank first column.
  // Inside: the slot's name and a small live picture of its device.
  constexpr int bw = 20, bh = 26, x0 = 7;
  for (int t = 0; t < NUM_TRACKS; t++) {
    const int y = t * (bh + 1);
    // L / R drawn by hand at the left edge (text can't start left of x 0)
    static const uint8_t glyph[2][7] = {
      {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},  // A
      {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},  // B
    };
    for (int r = 0; r < 7; r++)
      for (int c = 0; c < 5; c++)
        if (glyph[t][r] >> (4 - c) & 1) gfxPixel(c, y + 9 + r);
    for (int pos = 0; pos < COLS; pos++) {
      const int x = x0 + pos * bw;
      const Track& tr = tracks[t];
      const int c = tr.SlotAt(pos);  // the effects in their order
      Device* d = tr.at(c);
      if (tr.shown_bypassed(c)) d = nullptr;  // drawn like an empty slot
      if (d) gfxFrame(x, y, bw + 1, bh);
      else gfxFrame(x, y, bw + 1, bh, true);

      // the slot's name at the top, always; underneath its device's little
      // picture, or BP when bypassed
      // SOURCE is named after what it does: IN (input), SPL (SPOOL) or PLY
      const char* tag = c == MIX_COL ? PSTR("OUT")
                      : c == SOURCE ? (tr.choice[SOURCE] == 1 ? PSTR("SPL")
                                       : tr.choice[SOURCE] == 2 ? PSTR("PLY") : PSTR("IN"))
                      : c == RESONATOR && tr.choice[c] == 2 ? PSTR("MOD")  // MODAL
                      : c == SPRAY && tr.choice[c] == 2 ? PSTR("STR")      // STRETCH
                      : SLOT_DEFS[c].tag;
      if (strlen(tag) <= 3) {
        gfxPrint(x + 1 + (bw - 1 - 6 * int(strlen(tag))) / 2, y + 3, tag);
      } else {  // 4 letters (REED, SLUB): 5 px apart so they fit the box
        char ch[2] = {0, 0};
        for (int i = 0; i < 4; i++) { ch[0] = tag[i]; gfxPrint(x + 5 * i, y + 3, ch); }
      }
      if (!d) gfxPrint(x + 1 + (bw - 1 - 12) / 2, y + 14, PSTR("BP"));
      else if (d->ok()) d->DrawMini(x + 2, y + 13, bw - 3, 10);
      else gfxPrint(x + 5, y + 14, PSTR("!!"));
      if (t == track && c == col) {
        gfxInvert(x + 1, y + 1, bw - 1, bh - 2);
        // being moved (R held): arrows either side
        if (r_held && c > SOURCE && c < MIX_COL) {
          gfxPrint(x + 1, y + 14, PSTR("<"));
          gfxPrint(x + bw - 6, y + 14, PSTR(">"));
        }
      }
    }
  }

  if (threaded) {
    // THREAD: an arrow from A down into B
    gfxLine(2, 19, 2, 31);
    gfxLine(0, 29, 2, 31);
    gfxLine(4, 29, 2, 31);
  }
  if (linked) {
    // two squares between L and R: filled = SPLIT, hollow = MIRROR, top filled = LINKED
    for (const int sy : {21, 27}) {
      if (LinkSquareFilled(sy == 21)) gfxRect(0, sy, 5, 5);
      else gfxFrame(0, sy, 5, 5);
    }
  }

  // status line: what is under the cursor, CPU load
  const Track& tr = tracks[track];
  Device* d = tr.at(col);
  if (!linked) gfxPrint(0, 56, track ? PSTR("B") : PSTR("A"));
  // the slot name; SOURCE names its source (INPUT, SPOOL or PLAYER)
  gfxPrint(8, 56, col == MIX_COL ? PSTR("OUTPUT")
                  : (col == SOURCE || col == RESONATOR || col == SPRAY) && d ? d->def.name : SLOT_DEFS[col].name);
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

FLASHMEM void AppLoom::DrawDevice() const {
  using namespace loom;
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
    // linked: L, squares, R; the shared mods L+R
    if (linked && !SharedFocus()) DrawLinkLabel(1, 1);
    else gfxPrint(1, 1, PSTR("A+B"));
    gfxInvert(0, 0, 21, 9);
  } else {
    gfxPrint(1, 1, track ? PSTR("B") : PSTR("A"));
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
  } else {
    DrawClockMark(103, 2);
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

  // the clock has its own roomy page (the grid only for modulation)
  if (d == tempo_clock && !mod_mode && !trig_mode) {
    DrawClockPage();
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
      // the upper row is the mode's (the left knob): its name for a moment
      // after it changed, else the setting's name; the number stays below
      const bool show_mode = sel && pd.nmodes
        && ((editing && l_held) || (edit_mode && mode_shown < 1500));
      char mb[8];
      if (show_mode) d->FormatMode(p, mb, sizeof(mb));
      const char* nm = show_mode ? mb : d->Name(p);
      PrintCentred(x, 43, nm);
    }
    // has modes (the left knob): a small square at the right end of the
    // value lane, out of the way of the names
    // has modes (the left knob): two dots under each end of the name, in the
    // free row between name and number
    if (pd.nmodes && !(sel && mod_mode)) {
      const int w = 6 * int(strlen(d->Name(p)));
      const int x0 = x + std::max(0, (32 - w) / 2) + 1;
      // just the ends: two dots at each side of the name
      for (int px : {x0, x0 + 2, x0 + w - 4, x0 + w - 2}) gfxPixel(px, 51);
    }

    // value row: always the number
    if (sel && mod_mode) snprintf(buf, sizeof(buf), "%+d%%", mod_slot ? d->mod_amt2[p] : d->mod_amt[p]);
    else d->Format(p, buf, sizeof(buf));
    PrintCentred(x, 52, buf);
    if (sel && (editing || mod_mode)) gfxInvert(x, 51, 32, 9);

    // no value bar (the number says it); a modulated setting shows the
    // stretch its sources can reach (a line with end ticks) and the
    // playhead where they put it right now
    const int span = std::max(1, pd.max - pd.min);
    if (pd.kind != LIST && d->ModsLive(p)) {
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
    }
    if (sel) gfxInvert(x, 42, 32, 9);
  }
}

// two pages of four plus CLOCK: the track's A-D, then the shared E-H
FLASHMEM void AppLoom::DrawMods() const {
  using namespace loom;
  if (mod_cursor >= MIDI_CURSOR) { DrawMidiMaps(); return; }
  if (mod_cursor >= OUT_CURSOR) { DrawOutputs(); return; }
  const int page = mod_cursor < NUM_MODS ? 0 : 1;
  if (linked && !page) DrawLinkLabel(1, 0);
  else if (linked || page) gfxPrint(1, 0, PSTR("A+B"));
  else gfxPrint(1, 0, track ? PSTR("B") : PSTR("A"));
  gfxPrint(linked || page ? 24 : 12, 0, page ? PSTR("SHARED") : PSTR("MODULATORS"));
  DrawClockMark(96, 1);
  if (Muted() && (millis() / 300) % 2) {
    gfxPrint(104, 0, PSTR("MUTE"));
    gfxInvert(103, 0, 25, 8);
  } else {
    gfxPrint(110, 0, page + 1);
    gfxPrint(116, 0, PSTR("/4"));
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

// the clock page: one setting per line with room for full names, then the
// four beats and whether it runs
FLASHMEM void AppLoom::DrawClockPage() const {
  using namespace loom;
  const ClockDevice* c = tempo_clock;
  static const char* const names[3] = {"SOURCE", "BPM", "PPQN"};
  for (int p = 0; p < 3; p++) {
    const int y = 13 + p * 11;
    gfxPrint(2, y, names[p]);
    char buf[20];
    if (p == ClockDevice::P_SOURCE && c->val[p] == 0) snprintf(buf, sizeof(buf), "internal");
    else if (p == ClockDevice::P_SOURCE && c->val[p] == 1) snprintf(buf, sizeof(buf), "MIDI clock");
    else if (p == ClockDevice::P_PPQN && !c->external()) snprintf(buf, sizeof(buf), "-");
    else c->Format(p, buf, sizeof(buf));
    gfxPrint(48, y, buf);
    if (p == cursor) {
      gfxInvert(1, y - 1, 41, 9);
      if (editing) gfxInvert(47, y - 1, 6 * int(strlen(buf)) + 1, 9);
    }
  }
  // four beats, the current one filled while running
  for (int b = 0; b < 4; b++) {
    gfxFrame(2 + b * 11, 50, 9, 9);
    if (clock_running && b == c->Beat()) gfxRect(4 + b * 11, 52, 5, 5);
  }
  gfxPrint(50, 51, !clock_running ? PSTR("stopped") : c->Waiting() ? PSTR("waiting...") : PSTR("running"));
}

// linked tracks in the top line: L and R with the two squares of the chain
// view between them, filled for SPLIT, hollow for MIRROR, top filled for LINKED
FLASHMEM void AppLoom::DrawLinkLabel(int x, int y) const {
  gfxPrint(x, y, PSTR("A"));
  // stacked like in the chain view, between A and B
  for (const int sy : {y, y + 4}) {
    if (LinkSquareFilled(sy == y)) gfxRect(x + 8, sy, 3, 3);
    else gfxFrame(x + 8, sy, 3, 3);
  }
  gfxPrint(x + 12, y, PSTR("B"));
}

// the clock in the top line: a play triangle while it runs, a square when
// stopped (Z), 5 x 5 px from x, y
FLASHMEM void AppLoom::DrawClockMark(int x, int y) {
  if (loom::clock_running) {
    for (int i = 0; i < 3; i++) gfxLine(x + i, y + i, x + i, y + 4 - i);
  } else {
    gfxRect(x, y, 5, 5);
  }
}

// page 3: the 8 CV outputs, each with its source and its voltage as a bar
// page 4: the MIDI maps M1-M32 (the same ones Phazerville's Setup edits),
// in use filled; R push opens one: channel, type, voice, range
FLASHMEM void AppLoom::DrawMidiMaps() const {
  using namespace loom;
  const int m = mod_cursor - MIDI_CURSOR;
  const MIDIMapping& map = HS::frame.MIDIState.mapping[m];
  char buf[24];
  if (map_field < 0) gfxPrint(1, 0, PSTR("MIDI MAPS"));
  else { snprintf(buf, sizeof(buf), "MIDI MAP M%d", m + 1); gfxPrint(1, 0, buf); }
  DrawClockMark(96, 1);
  gfxPrint(110, 0, PSTR("4/4"));
  gfxLine(0, 9, 127, 9);
  auto type_name = [&](char* b, size_t n) {
    if (map.learning()) snprintf(b, n, "LEARN...");
    else if (map.get_type() == MIDIMapSettings::CCONTROL) snprintf(b, n, "CC %d", map.get_subtype());
    else snprintf(b, n, "%s", map.get_label());
  };
  auto chan_name = [&](char* b, size_t n) {
    if (map.get_channel() >= 16) snprintf(b, n, "OMNI"); else snprintf(b, n, "%d", map.get_channel() + 1);
  };
  if (map_field < 0) {
    // the grid: 8 x 4, numbered; in use filled, learning blinking
    constexpr int cw = 16, ch = 11;
    for (int i = 0; i < MIDIMAP_MAX; i++) {
      const int x = (i % 8) * cw, y = 11 + (i / 8) * ch;
      const MIDIMapping& mi = HS::frame.MIDIState.mapping[i];
      snprintf(buf, sizeof(buf), "%d", i + 1);
      gfxPrint(x + (cw - 6 * int(strlen(buf))) / 2, y + 2, buf);
      if (mi.enabled() || (mi.learning() && (millis() / 300) % 2)) gfxInvert(x + 1, y + 1, cw - 2, ch - 1);
      if (i == m) gfxFrame(x, y, cw, ch + 1);
    }
    // the map under the cursor
    char t[12], c[8];
    if (!map.enabled() && !map.learning()) snprintf(buf, sizeof(buf), "M%d  off", m + 1);
    else { type_name(t, sizeof(t)); chan_name(c, sizeof(c)); snprintf(buf, sizeof(buf), "M%d ch%s %s", m + 1, c, t); }
    gfxPrint(0, 56, buf);
    return;
  }
  // the editor: five fields, R moves, R push grabs
  static const char* const names[5] = {"CHAN", "TYPE", "VOICE", "LOW", "HIGH"};
  for (int f = 0; f < 5; f++) {
    const int y = 12 + f * 9;
    gfxPrint(2, y, names[f]);
    switch (f) {
      case 0: chan_name(buf, sizeof(buf)); break;
      case 1: type_name(buf, sizeof(buf)); break;
      case 2: snprintf(buf, sizeof(buf), "%d", map.get_voice() + 1); break;
      case 3: snprintf(buf, sizeof(buf), "%d", map.get_low()); break;
      case 4: snprintf(buf, sizeof(buf), "%d", map.get_high()); break;
    }
    gfxPrint(44, y, buf);
    if (f == map_field) {
      gfxInvert(0, y - 1, 40, 9);
      if (map_grab) gfxInvert(42, y - 1, 6 * int(strlen(buf)) + 3, 9);
    }
  }
  gfxPrint(0, 56, PSTR("A:learn B:clear"));
}

FLASHMEM void AppLoom::DrawOutputs() const {
  using namespace loom;
  gfxPrint(1, 0, PSTR("CV OUTPUTS"));
  DrawClockMark(96, 1);
  gfxPrint(110, 0, PSTR("3/4"));
  gfxLine(0, 9, 127, 9);
  constexpr int bw = 32, bh = 21;
  for (int o = 0; o < NUM_CV_OUTS; o++) {
    const int x = (o % 4) * bw, y = 11 + (o / 4) * (bh + 1);
    char buf[8];
    snprintf(buf, sizeof(buf), "%c", 'A' + o);
    gfxPrint(x + 2, y + 2, buf);
    const bool byp = cv_out_bypass & (1 << o);
    if (byp) snprintf(buf, sizeof(buf), "BP");
    else FormatOutSource(cv_out_src[o], buf, sizeof(buf), false);
    const int w = 6 * int(strlen(buf));
    gfxPrint(x + bw - 2 - w, y + 2, buf);
    // 0..10 V across the box (dotted while bypassed)
    gfxFrame(x + 2, y + 12, bw - 5, 6, byp);
    const int fill = int(OutLevel(o) * (bw - 7) + 0.5f);
    if (fill > 0) gfxRect(x + 3, y + 13, fill, 4);
    if (OUT_CURSOR + o == mod_cursor) {
      gfxInvert(x + 1, y + 1, 9, 9);
      if (out_editing) gfxInvert(x + bw - 3 - w, y + 1, w + 1, 9);  // grabbed
    }
  }
  const int o = mod_cursor - OUT_CURSOR;
  char buf[24];
  FormatOutSource(cv_out_src[o], buf, sizeof(buf), true);
  if (cv_out_bypass & (1 << o)) {
    char b2[24];
    snprintf(b2, sizeof(b2), "BYP %s", buf);
    gfxPrint(0, 56, b2);
  } else {
    gfxPrint(0, 56, buf);
  }
}

/////////////////////////////////////////////////////////////////
// Saving: LOOM.BIN on the internal flash
/////////////////////////////////////////////////////////////////
namespace loom {
  constexpr char SAVE_FILE[] = "LOOM.BIN";
  constexpr char SAVE_TMP[] = "LOOM.TMP";
  // 2: modulator types sorted by name; 3: second mod source, MOD E-H
  // 4: three choices per effect slot (RESONATOR: MODAL)
  // 5: OUTPUT page 2 LOW MID HIGH OTT (was BANDZ LOW MID HIGH)
  // 6: ECHO's reverb LEVEL PRE ROOM WALL (was LEVEL SIZE DECAY TONE)
  // 7: STRETCH POS turned round (0 = oldest, 100 = now, like SPRAY)
  constexpr uint8_t SAVE_VERSION = 7;
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
    const bool remap_spr = use && !memcmp(tag, "SPR", 4) && n == 7;
    const bool remap_mix = use && !memcmp(tag, "MIX", 4) && n == 7;  // MIX came in first
    const bool reorder_out = use && !memcmp(tag, "MIX", 4) && load_version < 5;
    const bool remap_mdl = use && !memcmp(tag, "MDL", 4) && n == 7;  // MODEL came in before WET
    const bool remap_str = use && !memcmp(tag, "STR", 4) && n == 7;  // SPRD came in before MIX
    // before v7 STRETCH's POS ran the other way (0 = now)
    const bool flip_pos = use && !memcmp(tag, "STR", 4) && load_version < 7;
    // before v6 the reverb's 6th was SIZE (now PRE: left at its default);
    // DECAY's value becomes ROOM, TONE's WALL (same places)
    const bool skip_size = use && !memcmp(tag, "ECO", 4) && load_version < 6;
    for (int p = 0; p < n; p++) {
      int16_t v; uint8_t md; int8_t amt; uint16_t src; uint8_t bus;
      if (f.read(&v, 2) != 2 || f.read(&md, 1) != 1 || f.read(&amt, 1) != 1
          || f.read(&src, 2) != 2 || f.read(&bus, 1) != 1) return false;
      int8_t amt2 = 0; uint8_t bus2 = 0;
      if (load_version >= 3 && (f.read(&amt2, 1) != 1 || f.read(&bus2, 1) != 1)) return false;
      if (!use) continue;
      // SPRAY gained SPRD before MIX: an older file's MIX (its 7th) goes to 8th
      const int q = p;
      if (remap_spr && p == 6) p = 7;
      if (remap_mix) p = q + 1;
      // before v5 OUTPUT's page 2 was BANDZ LOW MID HIGH: now LOW MID HIGH OTT
      if (reorder_out && p >= 4) p = p == 4 ? 7 : p - 1;
      if ((remap_mdl || remap_str) && p == 6) p = 7;
      if (skip_size && p == 5) { p = q; continue; }
      d->mod_amt2[p] = constrain(amt2, -100, 100);
      d->mod_bus2[p] = bus2 <= MOD_BUSES ? bus2 : 0;
      const ParamDef& pd = d->param(p);
      d->val[p] = constrain(v, pd.min, pd.max);
      d->mode[p] = pd.nmodes ? constrain(md, 0, pd.nmodes - 1) : 0;
      d->mod_amt[p] = constrain(amt, -100, 100);
      d->mod_in[p].Unpack(src);
      d->mod_bus_sel[p] = bus <= MOD_BUSES ? bus : 0;
      if (flip_pos && p == 1) {
        d->val[p] = 100 - d->val[p];
        d->mod_amt[p] = -d->mod_amt[p];
        d->mod_amt2[p] = -d->mod_amt2[p];
      }
      p = q;
    }
    uint8_t bi, bi2 = 0, trig[2];
    if (f.read(&bi, 1) != 1 || (load_version >= 3 && f.read(&bi2, 1) != 1)
        || f.read(trig, 2) != 2) return false;
    if (remap_spr) {  // the bipolar bits move with MIX too
      auto mv = [](uint8_t b) { return uint8_t((b & 0x3F) | ((b & 0x40) << 1)); };
      bi = mv(bi); bi2 = mv(bi2);
    }
    if (remap_mix) { bi = uint8_t(bi << 1); bi2 = uint8_t(bi2 << 1); }
    if (remap_mdl || remap_str) {
      auto mv = [](uint8_t b) { return uint8_t((b & 0x3F) | ((b & 0x40) << 1)); };
      bi = mv(bi); bi2 = mv(bi2);
    }
    if (reorder_out) {  // bit 4 (BANDZ) to 7, bits 5-7 down one
      auto ro = [](uint8_t b) { return uint8_t((b & 0x0F) | ((b & 0xE0) >> 1) | ((b & 0x10) << 3)); };
      bi = ro(bi); bi2 = ro(bi2);
    }
    if (use) {
      d->mod_bi = bi;
      d->mod_bi2 = bi2;
      for (int a = 0; a < 2; a++) d->act_trig[a] = trig[a] < TRIG_SOURCES ? trig[a] : 0;
    }
    return true;
  }
}

template <typename F>
FLASHMEM void AppLoom::ForEachDevice(F f) const {
  using namespace loom;
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

FLASHMEM void AppLoom::SaveFile() const {
  using namespace loom;
  auto& fs = PhzConfig::myfs;
  File f = fs.open(SAVE_TMP, FILE_WRITE_BEGIN);
  if (!f) return;
  f.write("SCU", 3);  // the file's mark since it was Sculpt; kept so all files load
  f.write(&SAVE_VERSION, 1);
  const uint8_t lk = threaded ? 4 : linked ? (link_stereo ? 2 : link_rel ? 3 : 1) : 0;  // 1 TIGHT, 2 SPLIT, 3 LOOSE, 4 THREAD
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
  // where the user was, so Loom opens there again (older files end before)
  const uint8_t ui[6] = {uint8_t(view), uint8_t(track), uint8_t(col),
                         uint8_t(cursor), uint8_t(focus_mod), uint8_t(mod_cursor)};
  f.write(ui, sizeof(ui));
  // bypassed slots (older files end before)
  for (auto& tr : tracks) f.write(&tr.bypass, 1);
  // SPOOL's settings (older files end before); the recording isn't kept
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
  f.write("D", 1);
  f.write(cv_out_src, NUM_CV_OUTS);
  f.write(&cv_out_bypass, 1);
  // ... and the effects' order
  f.write("R", 1);
  for (auto& tr : tracks) f.write(tr.order, SLOT_COUNT - 1);
  // ... and PLAYER's MEM section of a long file
  f.write("M", 1);
  for (auto& tr : tracks) {
    uint32_t sec[2] = {0, 0};
    if (tr.player) { sec[0] = tr.player->sec_off; sec[1] = tr.player->sec_len; }
    f.write(reinterpret_cast<uint8_t*>(sec), 8);
  }
  f.close();
  fs.remove(SAVE_FILE);
  fs.rename(SAVE_TMP, SAVE_FILE);
  dirty = false;
  Serial.println("Loom: settings saved");
}

FLASHMEM void AppLoom::LoadFile() {
  delete b_own;  // a loaded state has nothing to go back to
  b_own = nullptr;
  using namespace loom;
  File f = PhzConfig::myfs.open(SAVE_FILE, FILE_READ);
  if (!f) f = PhzConfig::myfs.open("SCULPT.BIN", FILE_READ);  // from before the rename
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
    const int opts = version >= 4 ? MAX_OPTIONS : 2;  // older files: two per slot
    for (int s = SOURCE + 1; s < SLOT_COUNT && ok; s++)
      for (int o = 0; o < opts && ok; o++) ok = ReadDevice(f, tr.inst[s][o]);
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
      // the card folder was /SCULPT before the rename (moved to /LOOM)
      if (!strncmp(fd, "/SCULPT", 7)) { memmove(fd + 5, fd + 7, strlen(fd + 7) + 1); memcpy(fd, "/LOOM", 5); }
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
        // 'C': sources; 'D': sources + bypass bits
        if (okm && f.read(&mark, 1) == 1 && (mark == 'C' || mark == 'D') && f.read(outs, NUM_CV_OUTS) == NUM_CV_OUTS) {
          for (int o = 0; o < NUM_CV_OUTS; o++) cv_out_src[o] = outs[o] < OUT_SOURCES ? outs[o] : 0;
          if (mark == 'D') f.read(&cv_out_bypass, 1);
          bool got_mark = f.read(&mark, 1) == 1;
          if (got_mark && mark == 'R') {  // the effects' order (a permutation, else left as is)
            for (auto& tr : tracks) {
              uint8_t o[SLOT_COUNT - 1];
              if (f.read(o, sizeof(o)) != sizeof(o)) break;
              uint8_t seen = 0;
              for (uint8_t v : o) if (v > SOURCE && v < SLOT_COUNT) seen |= 1 << v;
              if (seen == ((1 << SLOT_COUNT) - 2)) memcpy(tr.order, o, sizeof(o));
            }
            got_mark = f.read(&mark, 1) == 1;
          }
          if (got_mark && mark == 'M') {
            for (auto& tr : tracks) {
              uint32_t sec[2];
              if (f.read(reinterpret_cast<uint8_t*>(sec), 8) != 8) break;
              if (tr.player) { tr.player->sec_off = sec[0]; tr.player->sec_len = sec[1]; }
            }
          }
        }
      }
    }
  }
  snprintf(load_note + strlen(load_note), sizeof(load_note) - strlen(load_note), " v%d ok %d stage %d pos %d/%d",
           version, ok, stage, int(f.position()), int(f.size()));
  f.close();
  // OUT A is only for track B (older files had more track outputs)
  for (int t = 0; t < NUM_TRACKS; t++) {
    const int top = t == 1 ? InputDevice::OUT_A : InputDevice::OFF;
    if (tracks[t].input->val[0] > top) tracks[t].input->val[0] = 0;
    if (tracks[t].reel && tracks[t].reel->val[ReelDevice::IN] > top) tracks[t].reel->val[ReelDevice::IN] = 0;
  }
  threaded = lk == 4;   // B's source is saved as OUT A already
  linked = lk != 0 && !threaded;
  link_stereo = false;  // SPLIT (2) is out of the cycle: such files come back TIGHT
  link_rel = lk == 3;
  if (linked) track = 0;
  // SPLIT: each PLAYER its own side (the rest is in the saved settings)
  if (link_stereo && tracks[0].player && tracks[1].player) {
    tracks[0].player->SetPick(1);
    tracks[1].player->SetPick(2);
  }
  if (mixer) RouteStereo(link_stereo);
  if (view == DEVICE && (!Focused() || cursor >= Focused()->num_params())) {
    view = CHAIN;
    cursor = 0;
  }
  ForEachDevice([](Device* d) { d->ApplyAll(); });
  Serial.printf("Loom: settings loaded%s\n", ok ? "" : " (partly)");

}

FLASHMEM void AppLoom::View() const {
  UpdateMixLevels();
  if (view == DEVICE && Focused()) DrawDevice();
  else if (view == MODS) DrawMods();
  else DrawChain();
  // the link mode, for a moment after it changed: in a box with a double
  // outline (like the boot picture's LOOM, smaller)
  if (link_note_at && millis() - link_note_at < 1200) {
    const char* txt = threaded ? PSTR("THREAD") : !linked ? PSTR("FREE") : link_stereo ? PSTR("SPLIT") : link_rel ? PSTR("LOOSE") : PSTR("TIGHT");
    const int tw = 6 * int(strlen(txt)), bw = tw + 14, bx = (128 - bw) / 2;
    graphics.clearRect(bx, 21, bw, 19);
    gfxFrame(bx, 21, bw, 19);
    gfxFrame(bx + 2, 23, bw - 4, 15);
    gfxPrint(bx + 7, 27, txt);
  }
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
FLASHMEM void AppLoom::Init() {
  BaseStart();
}

// the global save (app menu, USB 'g') also writes Loom's own file
FLASHMEM size_t AppLoom::SaveAppData(util::StreamBufferWriter &) const {
  if (dirty) SaveFile();
  return 0;
}
size_t AppLoom::RestoreAppData(util::StreamBufferReader &) { return 0; }

void AppLoom::Process(OC::IOFrame *ioframe) {
  BaseController(ioframe);
}

FLASHMEM void AppLoom::HandleAppEvent(OC::AppEvent event) {
  if (event == OC::APP_EVENT_RESUME) Resume();
  if (event == OC::APP_EVENT_SUSPEND) Suspend();
}

FLASHMEM void AppLoom::Loop() {
  MainLoop();
}

FLASHMEM void AppLoom::GetIOConfig(OC::IOConfig &) const { }
void AppLoom::DrawDebugInfo() const { }

FLASHMEM void AppLoom::DrawMenu() const { BaseView(); }

// screensaver: the NLM wave as a 3D wire mesh: a sheet with two troughs at
// the front, flattening towards the back, seen like the logo. The camera
// floats inside a cube (left / right, up / down, forward / back, three slow
// random glides) and looks at a point that wanders the same way inside a
// smaller cube. The wave slowly travels.
namespace loom {
// glides from one random value in -1..1 to the next, a new one every 3-7 s
struct Drift {
  float from = 0.0f, to = 0.0f;
  uint32_t start = 0, len = 1;
  float At(uint32_t now) {
    if (now - start >= len) {
      from = to;
      to = random(-1000, 1001) * 0.001f;
      start = now;
      len = 3000 + random(4000);
    }
    const float x = float(now - start) / len;
    return from + (to - from) * x * x * (3.0f - 2.0f * x);  // smooth start and stop
  }
};
}

FLASHMEM void AppLoom::DrawScreensaver() const { loom::DrawWave(); }

// a big LOOM in the middle, on a black box (boot picture)
FLASHMEM void loom::DrawTitle() {
  constexpr int W = 12, H = 17, T = 3, GAP = 4;
  constexpr int X0 = (128 - (4 * W + 3 * GAP)) / 2, Y0 = (64 - H) / 2;
  graphics.clearRect(X0 - 4, Y0 - 4, 4 * W + 3 * GAP + 8, H + 8);
  gfxFrame(X0 - 4, Y0 - 4, 4 * W + 3 * GAP + 8, H + 8);
  auto rect = [](int x, int y, int w, int h) { gfxRect(x, y, w, h); };
  int x = X0;
  // L
  rect(x, Y0, T, H);
  rect(x, Y0 + H - T, W, T);
  x += W + GAP;
  // O O (corners cut a little)
  for (int k = 0; k < 2; k++, x += W + GAP) {
    rect(x + 2, Y0, W - 4, T);
    rect(x + 2, Y0 + H - T, W - 4, T);
    rect(x, Y0 + 2, T, H - 4);
    rect(x + W - T, Y0 + 2, T, H - 4);
    rect(x + 1, Y0 + 1, T - 1, T - 1);
    rect(x + W - T, Y0 + 1, T - 1, T - 1);
    rect(x + 1, Y0 + H - T, T - 1, T - 1);
    rect(x + W - T, Y0 + H - T, T - 1, T - 1);
  }
  // M: two legs and the V between them, T pixels thick
  rect(x, Y0, T, H);
  rect(x + W - T, Y0, T, H);
  for (int d = 0; d < T; d++) {
    gfxLine(x + d, Y0, x + W / 2 - 1 + d / 2, Y0 + H / 2 + 1);
    gfxLine(x + W - 1 - d, Y0, x + W / 2 - d / 2, Y0 + H / 2 + 1);
  }
}

FLASHMEM void loom::DrawWave() {
  using namespace loom;
  constexpr float L = 2.5f, AMP = 0.7f, F = 130.0f;
  // how far back each of the 13 long lines goes (x L), left to right: a
  // ragged back edge, the right side reaching further, like the logo
  static const float LEN[13] = {0.72f, 0.77f, 0.74f, 0.80f, 0.79f, 0.83f, 0.80f,
                                0.86f, 0.89f, 0.94f, 1.09f, 1.17f, 1.23f};
  auto len_at = [](float u) {  // between the lines
    const float f = (u + 1.0f) * 6.0f;
    const int i = std::min(11, int(f));
    return L * (LEN[i] + (LEN[i + 1] - LEN[i]) * (f - i));
  };
  constexpr float CAM_CUBE = 0.8f, AIM_CUBE = 0.25f;  // half sizes
  static Drift cam_drift[3], aim_drift[3];
  const uint32_t now = millis();
  const float t = now * 0.001f;
  const float phase = t * 0.5f;

  // the logo view: looking at (0, -0.3, 0.7) from 2.4 units away, down 45
  // degrees and turned 45; both points drift around these
  float aim[3] = {0.0f, -0.3f, 0.7f}, cam[3] = {-1.2f, 1.4f, -0.5f};
  for (int a = 0; a < 3; a++) {
    aim[a] += AIM_CUBE * aim_drift[a].At(now);
    cam[a] += CAM_CUBE * cam_drift[a].At(now);
  }
  // turn and tilt so the camera looks at the aim point
  const float dx = aim[0] - cam[0], dy = aim[1] - cam[1], dz = aim[2] - cam[2];
  const float yaw = atan2f(dx, dz), pitch = atan2f(-dy, sqrtf(dx * dx + dz * dz));
  const float cyw = cosf(yaw), syw = sinf(yaw), cpt = cosf(pitch), spt = sinf(pitch);
  static const float crl = cosf(-0.2618f), srl = sinf(-0.2618f);  // roll -15

  // a point of the sheet (across u -1..1, depth z 0..L) on the screen;
  // false when it is behind the camera
  auto at = [&](float u, float z, float& sx, float& sy) {
    const float down = 0.5f - 0.5f * cosf(2.0f * float(M_PI) * (u + 1.0f) + 0.5f * float(M_PI) + phase);
    const float k = std::max(0.0f, 1.0f - z / L);
    const float x = u - cam[0], y = -AMP * powf(k, 1.6f) * down - cam[1], zz = z - cam[2];
    const float x1 = x * cyw - zz * syw, z1 = x * syw + zz * cyw;
    const float y1 = y * cpt + z1 * spt, Z = -y * spt + z1 * cpt;
    if (Z < 0.1f) return false;
    const float X = x1 * crl - y1 * srl, Y = x1 * srl + y1 * crl;
    sx = 64.0f - F * X / Z;
    sy = 32.0f - F * Y / Z;
    return true;
  };
  auto line = [](float x0, float y0, float x1, float y1) {
    if (fabsf(x1 - x0) > 400.0f || fabsf(y1 - y0) > 400.0f) return;
    const int n = int(std::max(fabsf(x1 - x0), fabsf(y1 - y0))) + 1;
    for (int i = 0; i <= n; i++) {
      const int x = int(roundf(x0 + (x1 - x0) * i / n)), y = int(roundf(y0 + (y1 - y0) * i / n));
      if (x >= 0 && x < 128 && y >= 0 && y < 64) gfxPixel(x, y);
    }
  };
  constexpr int S = 40;  // segments per line
  for (int j = 0; j < 10; j++) {  // across: close together at the front, sparse at the back, only where the long lines reach
    const float z = powf(j / 9.0f, 2.3f) * 1.15f * L;
    float px = 0, py = 0;
    bool prev = at(-1.0f, z, px, py);
    for (int k = 1; k <= S; k++) {
      float x, y;
      const float u = -1.0f + 2.0f * k / S;
      const bool ok = at(u, z, x, y);
      if (ok && prev && z <= len_at(u - 1.0f / S)) line(px, py, x, y);
      px = x; py = y; prev = ok;
    }
  }
  for (int i = 0; i < 13; i++) {  // front to back
    const float u = -1.0f + 2.0f * i / 12.0f;
    float px = 0, py = 0;
    bool prev = at(u, 0.0f, px, py);
    for (int k = 1; k <= S; k++) {
      float x, y;
      const bool ok = at(u, LEN[i] * L * k / S, x, y);
      if (ok && prev) line(px, py, x, y);
      px = x; py = y; prev = ok;
    }
  }
}

FLASHMEM void AppLoom::HandleButtonEvent(const UI::Event &event) {
  OnButton(event);
}

FLASHMEM void AppLoom::HandleEncoderEvent(const UI::Event &event) {
  OnEncoder(event);
}
