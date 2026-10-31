// DREAM's device around one engine (LoomDream.h reads this twice).

// (this file is read twice, by LoomDream.h: once for track A's engine, once
// for track B's, in namespaces of their own)
namespace loom {
namespace DREAM_NS {

extern "C" {
#include "dream/amb.h"
void dream_zero(void);
}

// the four layers, as the engine's parameters and their voices
static constexpr int DREAM_LAYER_P[4] = {P_CHORD, P_SEQ, P_LEAD, P_SEQ2};
static constexpr int DREAM_VOICE_P[4] = {P_CVOICE, P_SVOICE, P_LVOICE, P_S2VOICE};
static constexpr uint8_t DREAM_VOICES[4] = {EN_NCV, EN_NSV, EN_NBV, EN_NS2V};
// each layer's octave, away from the piece's own: the engine's (the Drone's
// its own, Loom's), their ranges
static constexpr int DREAM_OCT_P[4] = {-1, P_SEQOCT, P_LOCT, P_SEQ2OCT};
static constexpr int8_t DREAM_OCT_LO[4] = {-2, -2, -2, -2}, DREAM_OCT_HI[4] = {2, 2, 1, 1};
// a layer's mode (L knob): its voice, and its octave as kept: voice + voices *
// k, k 0..4 the octave 0, +1, +2, -2, -1 (a voice saved before: octave 0); the
// knob turns through them in order, octave by octave, voice by voice
static constexpr int8_t DREAM_OCT_OF[5] = {0, 1, 2, -2, -1};
static const char* const DREAM_VOICE_SHORT[4][7] = {
  {"ORG", "STR", "HRN", "SAW", "GLS", "CHO"},
  {"260", "MOG", "SYN", "PLS", "GLS", "PLK"},
  {"ODY", "MIN", "FLT", "THR", "SYN"},
  {"260", "MOG", "SYN", "PLS", "GLS", "SUB", "PLK"},
};
// the pieces, short (STYLE's value: five letters); the modes, short
static const char* const DREAM_STYLE_SHORT[EN_NPIECES] = {
  "TMWND", "WAHNF", "PHAED", "RUBYC", "STRAT", "SUNRN", "MIRAG", "CYBRG",
  "OXIDE", "MOOND", "DEEPF", "LYDIA", "PENTA", "MIXTR", "NOCTN", "VELOC",
};
static const char* const DREAM_MODE_SHORT[EN_NMODES] = {"MAJ", "DOR", "PHR", "LYD", "MIX", "MIN", "PNTMJ", "PNTMN"};
static const char* const DREAM_SEQ_VOICE[EN_NSV] = {"2600", "Moog", "Synthi", "Pulse", "Glass", "Pluck"};

PROGMEM static const DeviceDef DREAM_DEF = {
  "DREAM", "DRM", 16, {"PLAY", "NEXT"},
  {
    {"STYLE", 0, EN_NPIECES + DREAM_USER_N - 1, 0, UNI},   // the piece (then U1..U8, the user's)
    {"TEMPO", 60, 160, 96, UNI, 0, 2, 0, {}, {"OFF", "ON"}},   // mode: SYNC to Loom's clock
    {"ROOT", 0, 11, 2, UNI, 0, EN_NMODES, 5},                 // mode: the MODE (scale)
    {"LEVEL", 0, 100, 100, UNI},
    // each layer's level; the mode (L knob) its voice (as piece 1 has them)
    // (and its octave: five each)
    {"DRONE", 0, 100, 60, UNI, 0, 5 * EN_NCV, 0},
    {"SEQ", 0, 100, 70, UNI, 0, 5 * EN_NSV, 0},
    {"LEAD", 0, 100, 40, UNI, 0, 5 * EN_NBV, 0},
    {"SEQ2", 0, 100, 60, UNI, 0, 5 * EN_NS2V, 1},
    // the modes (L knob): LENGTH the sequences' notes held, a share of their
    // step (0: plucked, the style's); REPEAT the Sequence's echoed notes, a
    // share of the style's (10: its own)
    {"MOTN", 0, 100, 50, UNI, 0, 11, 0},
    {"ENRGY", -50, 50, 0, BI, 0, 11, 10},
    {"SPACE", -50, 50, 0, BI},
    // ONLY: the CV, no sound (and next to no CPU); the mode (B's): LINK, A's
    // DREAM followed (B's own default: CV ONLY, LINK); KEYS (both): CV in 1
    // plays the Lead like the white keys, CV in 2 holds the black keys'
    // effects (see CvKeys)
#if DREAM_TRACK_B
    {"CV", 0, 2, 2, LIST, 3, 4, 1, {"OFF", "ON", "ONLY"}, {"FREE", "LINK", "KEYS", "L+KEY"}},
#else
    {"CV", 0, 2, 1, LIST, 3, 2, 0, {"OFF", "ON", "ONLY"}, {"OFF", "KEYS"}},
#endif
    // page 4, FX: the engine's own echo and plate, from the style's own (0);
    // the modes: the echo's TIME, the plate's SIZE (0: the style's)
    {"ECHO", -50, 50, 0, BI, 0, 7, 0},
    {"FDBK", -50, 50, 0, BI},
    {"PLATE", -50, 50, 0, BI, 0, 11, 0},
    {"TONE", -50, 50, 0, BI},
  },
  {"", "", "", "FX"},
};

class DreamDevice;
inline DreamDevice* g_dream_owner = nullptr;  // the one playing the engine

// the engine's notes (its MIDI out: Drone 1, Sequence 2, Lead 3, Sequence 2
// 4), caught for the CV outputs (written in the audio update)
struct DreamNotes {
  volatile uint8_t note[4] = {};      // each layer's last note on
  volatile uint32_t ons[4] = {};      // note ons so far
  volatile bool held[4] = {};         // its last note still held (Lead)
  volatile uint32_t drone[4] = {};    // the Drone's notes held, a bit each (0-127)
};
inline DreamNotes g_dream_notes;

class DreamDevice final : public DreamBase {
public:
  DreamDevice() : DreamBase(DREAM_DEF) {}
  AudioStream* in() override { return nullptr; }
  AudioStream* out() override { return this; }
  bool Activate() override;
  void Deactivate() override;
  bool ok() const override { return Owns(); }
  bool Owns() const { return g_dream_owner == this; }
  void Apply(int p) override;
  void Format(int p, char* buf, size_t n) const override;
  void FormatMode(int p, char* buf, size_t n) const override;
  // TEMPO's SYNC, ROOT's MODE, the layers' voices: second values (L knob)
  const char* AltName(int p) const override {
    return p == TEMPO ? "SYNC" : p == ROOT ? "MODE" : p >= DRONE && p <= SEQ2 ? "VOICE" : p == CV ? (DREAM_TRACK_B ? "LINK" : "KEYS") : p == MOTION ? "LENGTH" : p == ENERGY ? "REPEAT" : p == ECHO ? "TIME" : p == PLATE ? "SIZE" : nullptr;
  }
  void Action(int a) override;
  bool ActionLit(int a) const override { return a == 0 && Owns() && (en_playing || joining); }
  void Stop() override { if (Owns()) { armed = joining = false; en_play(0); } }  // A held (Loom.h)
  bool Playing() const override { return Owns() && (en_playing || joining); }
  // B held (Loom.h): a fresh piece; on a user slot, what plays kept there
  void Regrow() override {
    if (!Owns()) return;
    if (cur[PIECE] >= EN_NPIECES) { SaveUser(); return; }
    en_seed(micros());
    en_regrow();
    regrown_at = millis() | 1;
  }
  // SPACE out: REPEAT is on this track, SPACE moves its levels (Loom.h)
  // instead of the engine's own space
  bool space_out = false;
  void SpaceOut(bool on) override { if (on != space_out) { space_out = on; Refresh(SPACE); } }
  bool HeaderInfo(char* buf, size_t n) const override;
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void Idle() override;
  void CpuMax(float parts[2]) override {
    parts[0] = processorUsageMax();
    parts[1] = 0.0f;
    processorUsageMaxReset();
  }
  void update() override;
  // R held: back to the piece's own settings (the piece kept)
  void Reset() override {
    const int16_t pc = val[PIECE];
    Defaults();
    val[PIECE] = cur[PIECE] = pc;
    if (Owns()) LoadPiece(false);
    ApplyAll();
  }
  // the CV outputs
  bool CvOn() const override { return cur[CV] != 0 && Owns(); }
  void CvTick() override;
  int CvOut(int o, float& v) const override;
  void CvName(int o, char* buf, size_t n, bool long_name) const override;

private:
  int loaded_piece = -1;      // the recipe in the engine (-1: none yet)
  int16_t sent[P_NPARAMS];    // what the engine was last told (each en_set only when it changes)
  bool sent_ok = false;
  int8_t sent_mac[EN_NMAC] = {};
  uint32_t regrown_at = 0;    // a fresh piece (shown for a moment)
  // the user's pieces: the recipe under what plays; one kept (SAVED shown)
  int loaded_base = 0;
  uint32_t saved_at = 0;
  bool save_ok = false;
  static_assert(sizeof(DreamUserPiece::seq) == EN_SEQ_N, "a user piece keeps both sequences");
  const DreamUserPiece* UserSlot(int sel) const {
    return sel >= EN_NPIECES && sel < EN_NPIECES + DREAM_USER_N ? &g_dream_user.slot[sel - EN_NPIECES] : nullptr;
  }
  bool EmptySlot(int sel) const { const DreamUserPiece* u = UserSlot(sel); return u && !u->used; }
  void SaveUser();
  uint32_t notes_seen = 0;    // the MIDI notes in taken so far (note_ev_n)
  static constexpr int8_t NO_KEY = INT8_MIN;
  int8_t key_deg[128];        // a MIDI note held: the Lead's degree it plays (NO_KEY: none)
  // the keyboard's octave doesn't matter: its notes are moved by whole
  // octaves (key_shift) into a 3-octave window from the Lead's root up; the
  // shift stays until a key lands outside, so a 2-octave keyboard (or the
  // FM-1's D4 .. E6) plays its keys in order wherever it sits
  static constexpr int KEY_WINDOW = 36;
  int key_shift = 0;
  int KeyDeg(int note, int ns, uint32_t pool);
  // KEYS (the CV setting's L knob: A's KEYS, B's KEYS / L+KEY): the FM-1's
  // keyboard on two CV inputs, 1 V an octave, 0 V C0
  bool KeysOn() const { return DREAM_TRACK_B ? mode[CV] >= 2 : mode[CV] == 1; }
  void CvKeys(int ns, uint32_t pool);
  struct CvKey {
    int note = -1;          // the note it stands on (-1: not read yet)
    int heard = -1;         // ... and the one acted on
    uint32_t since = 0;     // when it got there
  };
  static bool CvKeyRead(CvKey& k, int ch, uint32_t now);
  CvKey cv_key, cv_fx;
  int8_t cv_deg = NO_KEY;   // CV 1's Lead note sounding (its degree)
  uint32_t cv_deg_at = 0;   // ... since
  int fx_shift = 0;         // CV 2's octaves moved (as key_shift)
  int8_t cv_fx_on = -1;     // the effect CV 2 holds (0..10, -1: none)
  // the MIDI keys' black keys: the same eleven effects, several at once
  // (key_deg holds -1 - the effect for a black key held)
  int midi_fx_shift = 0;
  uint16_t midi_fx = 0;     // the effects held by MIDI keys, a bit each
  uint16_t fx_sent = 0;     // MIDI's and CV 2's together, as sent
  static int BlackFx(int note, int& shift);
  volatile float gain = 1.0f;
  // PIECE turned while playing: the sound fades out, the piece changes
  // (Idle), it fades back in (update ramps fade toward fade_to)
  volatile float fade = 1.0f, fade_to = 1.0f;
  bool piece_due = false;
  // a layer's VOICE turned while playing: it changes on the next bar line
  // (Idle; voice_bar: the bar it was turned in)
  uint8_t voice_due = 0;
  int base_rev = 35, base_echo = 55;  // the piece's own reverb and echo (SPACE 0)
  int base_tapa = 100, base_tapb = 0; // ... its repeats (REPEAT 10)
  int base_etime = 3, base_size = 55, base_fb = 60, base_tone = 50;  // ... its echo and plate (FX)
  // a style's value moved by a setting: -50 none .. 0 its own .. +50 all
  static int Around(int base, int s) { return s >= 0 ? base + (100 - base) * s / 50 : base + base * s / 50; }
  float echo_meter = 0.0f;  // (the screen's)
  int8_t sent_gate = -1;              // LENGTH, as sent
  uint32_t voice_bar = 0;
  int Voice(int l) const { return mode[DRONE + l] % DREAM_VOICES[l]; }
  int Oct(int l) const { return DREAM_OCT_OF[std::min(4, mode[DRONE + l] / DREAM_VOICES[l])]; }
  int8_t base_oct[4] = {};   // the piece's own octaves
  int8_t sent_doct = 0;      // the Drone's, as sent
  int OctNow(int l) const { return constrain(base_oct[l] + Oct(l), int(DREAM_OCT_LO[l]), int(DREAM_OCT_HI[l])); }
  bool VoiceSent(int l) const {
    return sent_ok && sent[DREAM_VOICE_P[l]] == Voice(l) && (l ? sent[DREAM_OCT_P[l]] == OctNow(l) : sent_doct == OctNow(l));
  }
  void SendVoice(int l) {
    Send(DREAM_VOICE_P[l], Voice(l));
    if (l) Send(DREAM_OCT_P[l], OctNow(l));
    else if (Owns() && sent_doct != OctNow(l)) { sent_doct = int8_t(OctNow(l)); en_drone_oct(sent_doct); }
  }
  void SetVoice(int l, int v, int o) {
    int k = 0;
    while (k < 4 && DREAM_OCT_OF[k] != o) k++;
    mode[DRONE + l] = uint8_t(v + DREAM_VOICES[l] * k);
    mode_named |= uint16_t(1u << (DRONE + l));
    Apply(DRONE + l);
  }
  // a layer's VOICE grabbed: R the instrument (its octave kept), L the octave
  void NudgeMode(int p, int dir) override {
    if (p < DRONE || p > SEQ2) { Device::NudgeMode(p, dir); return; }
    const int l = p - DRONE;
    SetVoice(l, constrain(Voice(l) + dir, 0, DREAM_VOICES[l] - 1), Oct(l));
  }
  bool TurnAltLeft(int p, int dir) override {
    if (p < DRONE || p > SEQ2) return false;
    const int l = p - DRONE;
    SetVoice(l, Voice(l), constrain(Oct(l) + dir, -2, 2));
    return true;
  }
  // STYLE grabbed: turning it only shows the style, R pressed again plays it
  bool Browsing() const { return g_edit_dev == this && g_edit_p == PIECE; }
  bool browsed = false;
  static constexpr float FADE_OUT_S = 1.5f, FADE_IN_S = 1.0f;
  // the recipe into the engine; keep: these settings over it, else from it
  // (keep_tempo: but the tempo kept)
  void LoadPiece(bool keep, bool keep_tempo = false);
  void Send(int p, int v);
  uint32_t Pool() const { return EN_MODE_POOL[constrain(ModeNow(), 0, EN_NMODES - 1)]; }
  // LINK (B's): A's tempo, key and mode instead of these settings' (Idle
  // links and unlinks; update follows A's clock)
  volatile bool linked = false;
  bool armed = false;            // its A pressed: it plays when A does
  volatile bool joining = false; // ... from A's next bar
  volatile bool resync = false;  // (update: A's clock taken from where it is now)
  uint32_t seen_pulses = 0, seen_starts = 0, seen_stops = 0;
  int8_t sent_deg = -1;
  int TempoNow() const { return linked ? g_dream_link.tempo : cur[TEMPO]; }
  int RootNow() const { return linked ? g_dream_link.root : cur[ROOT]; }
  int ModeNow() const { return linked ? g_dream_link.mode : mode[ROOT]; }
  void Link();     // (Idle)
  void Follow();   // (update)
  // the CV state (main loop)
  uint32_t ons_seen[4] = {};
  uint32_t trig_until[2] = {};  // the sequences' triggers
  bool lead_gate = false;
  float semis[4] = {36.0f, 36.0f, 36.0f, 24.0f};
  float arc = 0.0f;
  bool gate[3] = {};
};

// the engine's MIDI out, from its render (the audio update): kept for the CV
extern "C" FASTRUN void en_midi_out(uint32_t st, uint32_t d1, uint32_t d2) {
  if (st >= 0xF8) {  // its clock (A's: B's LINK follows it)
    DreamLink& L = g_dream_link;
    if (DREAM_TRACK_B) return;
    if (st == 0xF8) L.pulses = L.pulses + 1;
    else if (st == 0xFA || st == 0xFB) { L.start_pulse = L.pulses; L.starts = L.starts + 1; }
    else if (st == 0xFC) L.stops = L.stops + 1;
    return;
  }
  const uint32_t ch = st & 15, kind = st & 0xF0;
  if (ch > 3 || d1 > 127) return;
  DreamNotes& n = g_dream_notes;
  if (kind == 0x90 && d2) {
    n.note[ch] = uint8_t(d1);
    n.ons[ch] = n.ons[ch] + 1;
    n.held[ch] = true;
    if (ch == 0) n.drone[d1 >> 5] = n.drone[d1 >> 5] | (1u << (d1 & 31));
  } else if (kind == 0x80 || (kind == 0x90 && !d2)) {
    if (d1 == n.note[ch]) n.held[ch] = false;
    if (ch == 0) n.drone[d1 >> 5] = n.drone[d1 >> 5] & ~(1u << (d1 & 31));
  } else if (kind == 0xB0 && d1 == 123) {  // all notes off
    n.held[ch] = false;
    if (ch == 0) for (auto& w : n.drone) w = 0;
  }
}

FLASHMEM void DreamDevice::Send(int p, int v) {
  if (!Owns() || (sent_ok && sent[p] == v)) return;
  sent[p] = int16_t(v);
  en_set(p, v);
}

// the piece's recipe into the engine. keep: this device's settings over it
// (its saved state, a piece played before); else they take the recipe's
FLASHMEM void DreamDevice::LoadPiece(bool keep, bool keep_tempo) {
  int16_t par[P_NPARAMS];
  uint16_t pool = 0;
  g_dream_user.Load();
  const int sel = constrain(int(cur[PIECE]), 0, EN_NPIECES + DREAM_USER_N - 1);
  const DreamUserPiece* u = UserSlot(sel);
  if (u && !u->used) {  // an empty slot: what plays goes on (nothing yet: the first piece)
    if (loaded_piece >= 0) { loaded_piece = sel; return; }
    u = nullptr;
  }
  const int pc = u ? std::min<int>(u->base, EN_NPIECES - 1) : std::min(sel, EN_NPIECES - 1);
  en_piece(pc, par, &pool);
  par[P_MIDI] = 1;  // (its notes come out: the CV outputs)
  if (!keep && u) {  // a user's piece: its settings over its recipe
    for (int p = TEMPO; p < num_params(); p++) {
      if (p == CV || (p == TEMPO && keep_tempo)) continue;
      val[p] = u->val[p];
      mode[p] = u->mode[p];
    }
  } else if (!keep) {
    if (!keep_tempo) val[TEMPO] = par[P_TEMPO];
    val[ROOT] = par[P_ROOT];
    mode[ROOT] = uint8_t(par[P_MODE]);
    val[ECHO] = val[FDBK] = val[PLATE] = val[TONE] = 0;  // (FX: the style's own)
    mode[ECHO] = mode[PLATE] = 0;
    for (int l = 0; l < 4; l++) {
      val[DRONE + l] = par[DREAM_LAYER_P[l]];
      mode[DRONE + l] = uint8_t(par[DREAM_VOICE_P[l]] + DREAM_VOICES[l] * (mode[DRONE + l] / DREAM_VOICES[l]));  // (its octave kept)
    }
    val[MOTION] = par[P_MOTION];
    val[ENERGY] = val[SPACE] = 0;
  }
  for (int p = 0; p < P_NPARAMS; p++) { sent[p] = par[p]; en_set(p, par[p]); }
  for (int l = 1; l < 4; l++) base_oct[l] = int8_t(par[DREAM_OCT_P[l]]);
  base_rev = par[P_REVERB];  // (SPACE moves these from the piece's own)
  base_tapa = par[P_TAPA];
  base_etime = par[P_ETIME];
  base_size = par[P_SIZE];
  base_fb = par[P_FEEDBACK];
  base_tone = par[P_ETONE];
  base_tapb = par[P_TAPB];
  base_echo = par[P_ECHO];
  sent_ok = true;
  en_pool(pool);
  en_seq(u ? u->seq : nullptr);  // (its sequences: a user's as kept, else grown afresh)
  loaded_piece = sel;
  loaded_base = pc;
  voice_due = 0;  // (the voices sent with the recipe)
  sent_deg = -1;  // (LINK: A's chord again, over the recipe's)
  for (int p = TEMPO; p < num_params(); p++) Refresh(p);  // (these settings over the recipe)
}

// B held on a user slot: what plays kept there (its recipe, these settings,
// both sequences as they are), DREAMU.BIN written
FLASHMEM void DreamDevice::SaveUser() {
  DreamUserPiece& u = g_dream_user.slot[cur[PIECE] - EN_NPIECES];
  u.used = 1;
  u.base = uint8_t(loaded_base);
  for (int p = 0; p < num_params(); p++) { u.val[p] = val[p]; u.mode[p] = mode[p]; }
  en_seq_get(u.seq);
  save_ok = g_dream_user.Save();
  saved_at = millis() | 1;
  loaded_piece = cur[PIECE];
}

FLASHMEM bool DreamDevice::Activate() {
  if (Device::active) return ok();
  Device::active = true;
  if (g_dream_owner && g_dream_owner != this) return false;  // the other track has it
  g_dream_owner = this;
  static bool inited = false;
  if (!inited) {  // (its state in RAM2 / PSRAM: cleared first, once)
    dream_zero();
    en_init();
    inited = true;
  }
  en_panic();
  fade = fade_to = 1.0f;
  piece_due = false;
  voice_due = 0;
  sent_ok = false;
  loaded_piece = -1;
  linked = joining = false;  // (Idle links it again)
  sent_gate = -1;
  for (auto& k : key_deg) k = NO_KEY;
  midi_fx = 0;
  fx_sent = 0x8000;  // (sent again: none held)
  notes_seen = note_ev_n;
  LoadPiece(true);
  ApplyAll();
  return true;
}

FLASHMEM void DreamDevice::Deactivate() {
  if (!Device::active) return;
  if (Owns()) {
    en_play(0);
    en_panic();
    if (linked) { en_follow(0); linked = joining = false; }
    g_dream_owner = nullptr;
  }
  Device::active = false;
}

FLASHMEM void DreamDevice::Apply(int p) {
  switch (p) {
    case PIECE:
      g_dream_user.Load();
      if (Browsing()) { browsed = true; break; }  // (Idle: played once let go)
      if (!Owns() || loaded_piece < 0 || cur[PIECE] == loaded_piece) piece_due = false;
      else if (EmptySlot(cur[PIECE])) { piece_due = false; fade_to = 1.0f; loaded_piece = cur[PIECE]; }  // (what plays goes on: B held keeps it there)
      else if (en_playing) { piece_due = true; fade_to = 0.0f; }  // (Idle loads it once faded out)
      else LoadPiece(false, true);
      break;
    case TEMPO:
      if (linked || !mode[TEMPO]) Send(P_TEMPO, TempoNow());  // (SYNC: Idle follows the clock)
      break;
    case ROOT:
      Send(P_ROOT, RootNow());
      if (Owns() && (!sent_ok || sent[P_MODE] != ModeNow())) {
        Send(P_MODE, ModeNow());
        en_pool(Pool());
        sent_deg = -1;  // (LINK: a new scale walks its own chord: A's again)
      }
      break;
    case LEVEL: gain = cur[LEVEL] * 0.01f; break;
    case DRONE: case SEQ: case LEAD: case SEQ2:
      Send(DREAM_LAYER_P[p - DRONE], cur[p]);
      if (!Owns() || !en_playing) SendVoice(p - DRONE);
      else if (!VoiceSent(p - DRONE)) {
        if (!voice_due) voice_bar = en_bars / 16;
        voice_due |= uint8_t(1u << (p - DRONE));
      }
      break;
    case MOTION:
      Send(P_MOTION, cur[MOTION]);
      if (Owns() && sent_gate != mode[MOTION]) { sent_gate = int8_t(mode[MOTION]); en_gate(10 * mode[MOTION]); }
      break;
    case CV: if (Owns()) en_silent = cur[CV] == CV_ONLY; break;  // ONLY: the piece goes on, unheard
    case ENERGY:
      if (sent_ok) {  // REPEAT: the style's repeats, scaled
        Send(P_TAPA, base_tapa * std::min<int>(mode[ENERGY], 10) / 10);
        Send(P_TAPB, base_tapb * std::min<int>(mode[ENERGY], 10) / 10);
      }
      if (Owns() && sent_mac[EN_MAC_ENERGY] != cur[p]) { sent_mac[EN_MAC_ENERGY] = int8_t(cur[p]); en_macro(EN_MAC_ENERGY, cur[p]); }
      break;
    case SPACE: case ECHO: case PLATE:  // the echo's and the plate's levels: their page's, SPACE over both
      if (Owns() && sent_ok) {
        const int s = space_out ? 0 : cur[SPACE];  // (REPEAT's instead: space_out)
        Send(P_ECHO, Around(Around(base_echo, cur[ECHO]), s));
        Send(P_REVERB, Around(Around(base_rev, cur[PLATE]), s));
        Send(P_ETIME, mode[ECHO] ? std::min<int>(mode[ECHO], 6) - 1 : base_etime);
        Send(P_SIZE, mode[PLATE] ? 10 * std::min<int>(mode[PLATE], 10) : base_size);
      }
      break;
    case FDBK: if (sent_ok) Send(P_FEEDBACK, Around(base_fb, cur[FDBK])); break;
    case TONE: if (sent_ok) Send(P_ETONE, Around(base_tone, cur[TONE])); break;
  }
}

FLASHMEM void DreamDevice::Format(int p, char* buf, size_t n) const {
  if (p == PIECE && EmptySlot(cur[PIECE])) snprintf(buf, n, "U%d -", cur[PIECE] - EN_NPIECES + 1);
  else if (p == PIECE && cur[PIECE] >= EN_NPIECES)  // a user's: its source's name, its slot ("RUBY3")
    snprintf(buf, n, "%.4s%d", DREAM_STYLE_SHORT[std::min<int>(UserSlot(cur[PIECE])->base, EN_NPIECES - 1)], cur[PIECE] - EN_NPIECES + 1);
  else if (p == PIECE) snprintf(buf, n, "%s", DREAM_STYLE_SHORT[constrain(int(cur[PIECE]), 0, EN_NPIECES - 1)]);
  else if (p == ROOT) snprintf(buf, n, "%s", EN_NOTE_NAME[constrain(RootNow(), 0, 11)]);
  else if (p == TEMPO && linked) snprintf(buf, n, "%d", TempoNow());
  else if (p == TEMPO && mode[TEMPO]) snprintf(buf, n, "%d", int(constrain(tempo_bpm, 60.0f, 160.0f) + 0.5f));
  else Device::Format(p, buf, n);
}

// SYNC OFF / ON, the mode's name, the layers' voices
FLASHMEM void DreamDevice::FormatMode(int p, char* buf, size_t n) const {
  const int m = mode[p];
  if (p == TEMPO) snprintf(buf, n, "%s", linked ? "LINK" : m ? "ON" : "OFF");
  else if (p == ROOT) {
    const int m = ModeNow();
    snprintf(buf, n, "%s", DREAM_MODE_SHORT[constrain(m, 0, EN_NMODES - 1)]);
  } else if (p == MOTION || p == ENERGY) snprintf(buf, n, "%d%%", 10 * m);
  else if ((p == ECHO || p == PLATE) && !m) snprintf(buf, n, "STYLE");
  else if (p == ECHO) en_param_text(P_ETIME, std::min(m, 6) - 1, buf);
  else if (p == PLATE) snprintf(buf, n, "%d%%", 10 * m);
  else if (p >= DRONE && p <= SEQ2) {  // the voice; moved an octave or two: "MOG-1"
    const int l = p - DRONE, v = Voice(l), o = Oct(l);
    static const char* const* const LONG[4] = {EN_CV_NAME, DREAM_SEQ_VOICE, EN_BV_NAME, EN_SV_NAME};
    if (o) snprintf(buf, n, "%s%+d", DREAM_VOICE_SHORT[l][v], o);
    else snprintf(buf, n, "%.5s", LONG[l][v]);
  }
  else Device::FormatMode(p, buf, n);
}

// A: play (a click while playing does nothing: A held stops, Loom.h);
// B: the next chord (on the next bar)
FLASHMEM void DreamDevice::Action(int a) {
  if (!Owns()) return;
  if (a == 0) {
    armed = true;
    if (linked) {  // (LINK: it plays with A, joining on A's next bar)
      if (!en_playing && g_dream_link.playing) joining = true;
    } else if (!en_playing) { en_seed(micros()); en_play(1); }
  } else if (!linked) {  // (LINK: A's chords)
    en_next();
  }
}

// main loop: the tempo after the clock (SYNC), MIDI notes to the Lead (the
// black keys: the effects, as the FM-1's)
FLASHMEM void DreamDevice::Idle() {
  if (!Owns()) return;
  if (browsed && !Browsing()) { browsed = false; Apply(PIECE); }  // STYLE let go: the one shown plays
  if (piece_due && (fade <= 0.0f || !en_playing)) {  // faded out: the new piece, fading in
    piece_due = false;
    LoadPiece(false, true);
    fade_to = 1.0f;
  } else if (!piece_due) fade_to = 1.0f;
  if (voice_due && (!en_playing || en_bars / 16 != voice_bar)) {  // the bar line: the voices turned
    for (int l = 0; l < 4; l++) if (voice_due & (1u << l)) SendVoice(l);
    voice_due = 0;
  }
  Link();
  if (!linked && mode[TEMPO] && clock_running && tempo_bpm > 0.0f)
    Send(P_TEMPO, constrain(int(tempo_bpm + 0.5f), 60, 160));
  // MIDI notes: the Lead plays them in the piece's scale, from its root up
  // (any octave: see key_shift; a note between two of the scale's takes the
  // nearer)
  int steps[12];
  const uint32_t pool = Pool();
  const int ns = std::max(1, en_scale(pool, steps));
  while (notes_seen != note_ev_n) {
    if (note_ev_n - notes_seen > NOTE_EVS) notes_seen = note_ev_n - NOTE_EVS;  // (fell behind: the latest)
    const NoteEv e = note_ev[notes_seen % NOTE_EVS];
    notes_seen++;
    int8_t& kd = key_deg[e.note & 127];
    if (e.vel) g_dream_link.key_note = e.note;
    static const uint16_t BLACK_PC = 1 << 1 | 1 << 3 | 1 << 6 | 1 << 8 | 1 << 10;
    if (e.vel && (BLACK_PC >> (e.note % 12) & 1)) {  // a black key: its effect, held
      const int fx = BlackFx(e.note, midi_fx_shift);
      if (fx >= 0) { kd = int8_t(-1 - fx); midi_fx |= 1u << fx; }
      g_dream_link.key_took = int16_t(-1 - fx);
    } else if (e.vel) {
      const int deg = KeyDeg(e.note, ns, pool);
      kd = int8_t(deg);
      en_lead_key(deg, 1);
      g_dream_link.key_took = int16_t(deg);
    } else if (kd != NO_KEY && kd < 0) {
      midi_fx &= ~(1u << (-1 - kd));
      kd = NO_KEY;
    } else if (kd != NO_KEY) { en_lead_key(kd, 0); kd = NO_KEY; }
  }
  CvKeys(ns, pool);
}

// a key (MIDI note) as the Lead's scale degree: moved into the window by
// whole octaves, as few as it takes (key_shift kept for the next)
FLASHMEM int DreamDevice::KeyDeg(int note, int ns, uint32_t pool) {
  int rel = note - 60 - RootNow() + key_shift;
  while (rel < 0) { rel += 12; key_shift += 12; }
  while (rel >= KEY_WINDOW) { rel -= 12; key_shift -= 12; }
  return (rel / 12) * ns + en_deg_of(pool, rel % 12);
}

// a CV input as a note: true when there is a new one to act on
FLASHMEM bool DreamDevice::CvKeyRead(CvKey& k, int ch, uint32_t now) {
  const float semis = float(HS::frame.In(ch)) / 128.0f;  // (128 a semitone)
  const int n = int(floorf(semis + 0.5f)) + 12;
  if (k.note < 0) { k.note = k.heard = n; return false; }
  if (n != k.note && fabsf(semis + 12.0f - float(k.note)) > 0.83f) { k.note = n; k.since = now; }
  if (k.note != k.heard && now - k.since >= 4) { k.heard = k.note; return true; }
  return false;
}

// KEYS: CV in 1 the white keys, CV in 2 the black keys, as the FM-1's.
// Each is read as a note (1 V an octave, 0 V C0), taken once it has stood
// still 4 ms (a sequencer's step between two notes isn't played), with a
// third of a semitone either way before it moves (a still voltage doesn't
// flicker). The level it stands on at the start plays nothing (an empty
// jack: 0 V).
//  CV 1: each new note plays the Lead (the octave as the MIDI keys': any),
//   held until the next, let go after a beat standing still (so the Lead
//   hears a pause, takes the notes as its theme and answers)
//  CV 2: the eleven black keys F#3 .. F#5 (any octave: moved as CV 1's,
//   into F3 .. G5), left to right Take-off, Supernova, Twitter, Repeat,
//   Shimmer, Freeze, Octave, Return, Rise, Close, Transpose, each held
//   while the CV stands on it; a white note: none
FLASHMEM void DreamDevice::CvKeys(int ns, uint32_t pool) {
  const uint32_t now = millis();
  const bool on = KeysOn();
  // CV 1: the Lead
  if (on && CvKeyRead(cv_key, 0, now)) {
    cv_deg = int8_t(KeyDeg(cv_key.note, ns, pool));
    cv_deg_at = now;
    en_lead_key(cv_deg, 1);
  }
  const float beat_ms = 60000.0f / float(std::max(30, TempoNow()));
  if (cv_deg != NO_KEY && (!on || now - cv_deg_at >= uint32_t(beat_ms))) {
    en_lead_key(cv_deg, 0);
    cv_deg = NO_KEY;
  }
  if (!on) cv_key = CvKey{};
  // CV 2: the effects
  int fx = cv_fx_on;
  if (on && CvKeyRead(cv_fx, 1, now)) fx = BlackFx(cv_fx.note, fx_shift);
  if (!on) { cv_fx = CvKey{}; fx = -1; }
  cv_fx_on = int8_t(fx);
  // the effects held, MIDI's and CV 2's, to the engine
  const uint16_t held = midi_fx | (fx >= 0 ? 1u << fx : 0u);
  if (held != fx_sent) {
    static const uint8_t FX[8] = {EN_FX_TAKEOFF, EN_FX_SUPERNOVA, EN_FX_TWITTER, EN_FX_REPEAT,
                                  EN_FX_SHIMMER, EN_FX_FREEZE, EN_FX_OCTAVE, EN_FX_RETURN};
    static const uint8_t MV[3] = {EN_MV_RISE, EN_MV_CLOSE, EN_MV_TRANSPOSE};
    uint32_t fxm = 0, mvm = 0;
    for (int i = 0; i < 11; i++)
      if (held >> i & 1) { if (i < 8) fxm |= 1u << FX[i]; else mvm |= MV[i - 8]; }
    en_fx(fxm);
    en_move(mvm);
    fx_sent = held;
    g_dream_link.fx_held = held;
  }
}

// a note as one of the FM-1's eleven black keys, F#3 .. F#5 left to right
// (0..10; any octave: moved into F3 .. G5 as the keys are, `shift` kept);
// a white note: -1
FLASHMEM int DreamDevice::BlackFx(int note, int& shift) {
  static const uint8_t BLACK[12] = {0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0};
  int rel = note - 53 + shift;  // (F3: 0)
  while (rel < 0) { rel += 12; shift += 12; }
  while (rel > 26) { rel -= 12; shift -= 12; }  // (.. G5)
  if (!BLACK[(rel + 5) % 12]) return -1;
  int fx = 0;
  for (int i = 0; i < rel; i++) fx += BLACK[(i + 5) % 12];
  return fx;
}

// LINK, the main loop's half. A's DREAM: what it plays, for B's. B's: linked
// (its LINK on, A's DREAM there), A's tempo, key, mode and chord sent
FLASHMEM void DreamDevice::Link() {
  DreamLink& L = g_dream_link;
  if (!DREAM_TRACK_B) {
    L.playing = en_playing;
    L.deg = en_deg;
    L.tempo = sent_ok ? sent[P_TEMPO] : cur[TEMPO];
    L.root = cur[ROOT];
    L.mode = mode[ROOT];
    L.seen = millis() | 1;
    return;
  }
  const bool l = (mode[CV] & 1) && L.Alive();
  if (l != linked) {
    if (l) {  // (playing on its own: it starts over on A's next bar)
      armed = armed || en_playing;
      joining = en_playing;
      resync = true;
    } else joining = false;
    linked = l;
    L.b_linked = l;
    L.b_flips = L.b_flips + 1;
    en_follow(l);
    sent_deg = -1;
    Refresh(TEMPO);
    Refresh(ROOT);
  }
  if (!linked) return;
  if (sent[P_TEMPO] != L.tempo) Refresh(TEMPO);
  if (sent[P_ROOT] != L.root || sent[P_MODE] != L.mode) Refresh(ROOT);
  if (sent_deg != L.deg) { sent_deg = L.deg; en_follow_deg(L.deg); L.b_degs = L.b_degs + 1; }
  L.b_deg = en_deg;
}

// LINK, the audio's half (B's, before it renders): A's clock pulses, its
// starts and stops, as A rendered them (a block late at most)
FLASHMEM __attribute__((noinline)) void DreamDevice::Follow() {  // (in flash: once a block, RAM1 is short)
  DreamLink& L = g_dream_link;
  if (resync) {
    resync = false;
    seen_pulses = L.pulses;
    seen_starts = L.starts;
    seen_stops = L.stops;
  }
  if (seen_starts != L.starts) {
    seen_starts = L.starts;
    seen_pulses = L.start_pulse;
    joining = false;
    if (armed) en_clock_now(EN_CLK_START);
  }
  if (L.pulses - seen_pulses > 24) seen_pulses = L.pulses - 24;  // (fell behind: a beat's at most)
  while (seen_pulses != L.pulses) {
    if (joining && (seen_pulses - L.start_pulse) % 96 == 0) {  // A's bar line
      joining = false;
      if (armed) en_clock_now(EN_CLK_START);
    }
    en_clock_now(EN_CLK_TICK);
    seen_pulses++;
  }
  if (seen_stops != L.stops) {
    seen_stops = L.stops;
    joining = false;
    en_clock_now(EN_CLK_STOP);
  }
}

FLASHMEM bool DreamDevice::HeaderInfo(char* buf, size_t n) const {
  if (!Owns()) return false;
  const uint32_t bar = en_bars / 16 + 1;
  snprintf(buf, n, "%lu", (unsigned long)bar);
  return true;
}

FLASHMEM void DreamDevice::DrawViz(int x, int y, int w, int h) {
  const int cy = y + h / 2;
  if (!Owns()) {
    char t[24];
    snprintf(t, sizeof(t), "DREAM is on %c", g_dream_owner && g_dream_owner->track ? 'B' : 'A');
    gfxPrint(x + (w - 6 * int(strlen(t))) / 2, cy - 4, t);
    return;
  }
  // the piece, its section and the chord; then its key and tempo
  char t[24], ch[12];
  int pcs[4];
  const int nt = std::max(1, std::min<int>(en_chord_n, 4));
  for (int i = 0; i < nt; i++) pcs[i] = en_chord[i] % 12;
  en_chord_name(pcs, nt, ch);
  const char* sec = EN_SEC_NAME[constrain(int(en_section), 0, EN_NSEC - 1)];
  if (saved_at && millis() - saved_at < 1500) snprintf(t, sizeof(t), save_ok ? "SAVED U%d" : "SAVE FAILED", cur[PIECE] - EN_NPIECES + 1);
  else if (regrown_at && millis() - regrown_at < 1500) snprintf(t, sizeof(t), "NEW VARIATION");
  else {
    // the style (browsing: the one shown, "> "; a user's: its source's name, its slot)
    const int sel = cur[PIECE];
    const char* pre = Browsing() && sel != loaded_piece ? "> " : "";
    const int slot = sel - EN_NPIECES + 1;
    if (g_dream_hold > 0.0f && g_dream_hold_track == track && sel >= EN_NPIECES)
      snprintf(t, sizeof(t), "SAVE TO U%d", slot);
    else if (g_dream_hold > 0.0f && g_dream_hold_track == track) snprintf(t, sizeof(t), "NEW VARIATION");
    else if (EmptySlot(sel)) snprintf(t, sizeof(t), "%sU%d EMPTY:HOLD B", pre, slot);
    else if (sel >= EN_NPIECES) snprintf(t, sizeof(t), "%s%s %d", pre, en_piece_name(UserSlot(sel)->base), slot);
    else snprintf(t, sizeof(t), "%s%s", pre, en_piece_name(sel));
  }
  gfxPrint(x + 1, y, t);
  gfxPrint(x + w - 6 * int(strlen(ch)), y, ch);
  if (g_dream_hold > 0.0f && g_dream_hold_track == track)  // B held: how far, filling the row
    gfxInvert(x, y - 1, std::max(1, int(g_dream_hold * w)), 9);
  snprintf(t, sizeof(t), "%s %s %s", EN_NOTE_NAME[constrain(RootNow(), 0, 11)], DREAM_MODE_SHORT[constrain(ModeNow(), 0, EN_NMODES - 1)], sec);
  gfxPrint(x + 1, y + 9, t);
  // the drift to the next chord, a bar along the right of the second row
  const int dw = 30;
  gfxFrame(x + w - dw, y + 10, dw, 5);
  const int dl = int(en_drift * (dw - 2));
  if (dl > 0) gfxRect(x + w - dw + 1, y + 11, dl, 3);
  // four boxes, one under each setting of the page on screen, each showing
  // what that setting moves as it plays
  const int by = y + 19, bh = h - 19, cw = w / 4;
  {
    const float pk = en_echo_pk;  // (the echo's peak since the screen last looked)
    en_echo_pk = 0.0f;
    echo_meter = std::max(std::min(1.0f, 2.0f * pk), echo_meter * 0.85f);
  }
  const float act = std::max(std::max(float(en_layer_act[0]), float(en_layer_act[1])),
                             std::max(float(en_layer_act[2]), float(en_layer_act[3])));
  for (int c = 0; c < 4; c++) {
    const int bx = x + c * cw + 2, bw = cw - 4;
    const int p = 4 * int(view_page) + c;
    gfxFrame(bx, by, bw, bh);
    if (!en_playing && p != PLATE && p != TONE && p != FDBK) continue;
    float f = 0.0f;
    int cells = 0, lit = -1, mark = -1;  // (cells: the box in parts, one lit; mark: a tick across)
    switch (p) {
      case PIECE: cells = EN_NSEC; lit = en_section; break;               // the arc's sections, the one playing
      case TEMPO: cells = 4; lit = int(en_bars / 4 % 4); break;           // the beat in the bar
      case ROOT: f = en_drift; break;                                     // on to the next chord
      case LEVEL: f = act * cur[LEVEL] * 0.01f; break;                    // all of it
      case DRONE: case SEQ: case LEAD: case SEQ2:                         // each layer, as busy as it is
        f = en_layer_act[p - DRONE];
        if (p == SEQ || p == SEQ2) {                                      // (its step, along its length)
          const int l = p == SEQ2, len = std::max<int>(1, en_seq_len[l]);
          mark = (bw - 4) * std::min<int>(en_loop_pos[l], len - 1) / std::max(1, len - 1);
        }
        break;
      case MOTION: f = constrain((en_fo[0] + 2.0f) * 0.25f, 0.0f, 1.0f); break;   // the filter's arc
      case ENERGY: f = 0.5f * (en_layer_act[1] + en_layer_act[3]); break;          // the sequences' drive
      case SPACE: case ECHO: f = echo_meter; break;                                // the echo sounding
      case CV: cells = 3; break;                                                   // the gates E F G
      case FDBK: f = sent_ok ? sent[P_FEEDBACK] * 0.01f : 0.0f; break;
      case PLATE: f = sent_ok ? sent[P_REVERB] * 0.01f : 0.0f; break;
      case TONE: f = sent_ok ? sent[P_ETONE] * 0.01f : 0.0f; break;
    }
    if (cells) {
      const int pw = (bw - 2) / cells;
      for (int k = 0; k < cells; k++) {
        const bool on = p == CV ? gate[k] : k == lit;
        if (on) gfxRect(bx + 1 + k * pw, by + 1, pw - 1, bh - 2);
        else gfxPixel(bx + 1 + k * pw + pw / 2, by + bh / 2);
      }
    } else {
      const int len = int(constrain(f, 0.0f, 1.0f) * (bw - 2));
      if (len > 0) gfxRect(bx + 1, by + 1, len, bh - 2);
    }
    if (mark >= 0) gfxInvert(bx + 1 + mark, by + 1, 2, bh - 2);
  }
}

FLASHMEM void DreamDevice::DrawMini(int x, int y, int w, int h) {
  const int bw = std::max(1, (w - 3) / 4);
  for (int l = 0; l < 4; l++) {
    const int len = en_playing ? int(constrain(float(en_layer_act[l]), 0.0f, 1.0f) * h) : 0;
    if (len > 0) gfxRect(x + l * (bw + 1), y + h - len, bw, len);
    else gfxLine(x + l * (bw + 1), y + h - 1, x + l * (bw + 1) + bw - 1, y + h - 1);
  }
}

// the CV outputs, once a tick: the sequences' notes as triggers (half a
// sixteenth), the Lead's gate (down for a tick when a note starts while
// one was held), the pitches (held between notes), the arc
FLASHMEM void DreamDevice::CvTick() {
  const DreamNotes& n = g_dream_notes;
  const uint32_t now = millis();
  const float bpm = sent_ok ? float(sent[P_TEMPO]) : 96.0f;
  const uint32_t trig = uint32_t(std::max(5.0f, 60000.0f / bpm / 8.0f));
  static constexpr int CH[3] = {1, 3, 2};  // Sequence, Sequence 2, the Lead
  for (int k = 0; k < 3; k++) {
    const int c = CH[k];
    const uint32_t ons = n.ons[c];
    const bool fresh = ons != ons_seen[c];
    ons_seen[c] = ons;
    if (fresh) semis[k] = float(n.note[c]) - 12.0f;  // (MIDI 12 is C0)
    if (k < 2) {
      if (fresh) {
        const bool again = gate[k];  // (a trigger still high: down for this tick)
        trig_until[k] = now + trig;
        gate[k] = !again;
      } else {
        gate[k] = int32_t(trig_until[k] - now) > 0;
      }
    } else {
      gate[2] = n.held[c] && !(fresh && lead_gate);
      lead_gate = n.held[c];
    }
  }
  // the Drone: its lowest note held (else the last)
  for (int wd = 0; wd < 4; wd++)
    if (n.drone[wd]) { semis[3] = float(wd * 32 + __builtin_ctz(n.drone[wd])) - 12.0f; break; }
  // the arc: the Drone's filter as the sections move it, -2 .. +2 octaves
  arc = constrain((en_fo[0] + 2.0f) * 0.25f, 0.0f, 1.0f);
}

FLASHMEM int DreamDevice::CvOut(int o, float& v) const {
  switch (o) {
    case 0: case 1: case 2: v = semis[o]; return CV_PITCH;   // A B C: Sequence, Sequence 2, Lead
    case 3: v = semis[3]; return CV_PITCH;                    // D: the Drone
    case 4: case 5: case 6: v = gate[o - 4] ? 1.0f : 0.0f; return CV_LEVEL;
    case 7: v = arc; return CV_LEVEL;                         // H: the arc
  }
  return CV_NONE;
}

FLASHMEM void DreamDevice::CvName(int o, char* buf, size_t n, bool long_name) const {
  static const char* const SHORT[8] = {"SQP", "S2P", "LDP", "DRP", "SQT", "S2T", "LDG", "ARC"};
  static const char* const LONG[8] = {"DREAM seq pitch", "DREAM seq2 pitch", "DREAM lead pitch", "DREAM drone pitch",
                                      "DREAM seq trigger", "DREAM seq2 trigger", "DREAM lead gate", "DREAM arc"};
  snprintf(buf, n, "%s", (long_name ? LONG : SHORT)[o & 7]);
}

// the engine renders a block (24-bit stereo in int32) at LEVEL; it sheds
// what is least heard when the audio is near its limit (en_load, %)
FASTRUN void DreamDevice::update() {
  audio_block_t* o[2] = {allocate(), allocate()};
  if (o[0] && o[1]) {
    if (Owns() && Device::active) {
      DMAMEM static int32_t lr[2 * AUDIO_BLOCK_SAMPLES];  // (RAM2: RAM1 is short)
      en_load = uint8_t(std::min(255.0f, AudioProcessorUsage()));
      if (linked) Follow();
      en_render(lr, AUDIO_BLOCK_SAMPLES, uint32_t(gain * 4096.0f));
      // the fade, ramped a sample at a time
      float f = fade;
      const float to = fade_to;
      const float step = 1.0f / (AUDIO_SAMPLE_RATE_EXACT * (to < f ? FADE_OUT_S : FADE_IN_S));
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        f = to < f ? std::max(to, f - step) : std::min(to, f + step);
        const float k = f * f * (1.0f / 256.0f);  // (squared: an even fall to the ear)
        o[0]->data[i] = int16_t(constrain(int32_t(float(lr[2 * i]) * k), -32768, 32767));
        o[1]->data[i] = int16_t(constrain(int32_t(float(lr[2 * i + 1]) * k), -32768, 32767));
      }
      fade = f;
    } else {
      memset(o[0]->data, 0, sizeof(o[0]->data));
      memset(o[1]->data, 0, sizeof(o[1]->data));
    }
    transmit(o[0], 0);
    transmit(o[1], 1);
  }
  for (auto b : o) if (b) release(b);
}

}  // namespace DREAM_NS
}  // namespace loom
