// SID: Commodore 64 tunes (.sid files, PSID / RSID) as a Loom SOURCE.
//
// The C64 and the SID chip are SiDJ's (Thomas Jansson, github.com/thomasj/
// fm1-sidj): a cycle-counted 6510 with both CIAs and the VIC's raster, and
// a fixed-point port of reSID 0.16 (Dag Lem). They are in src/extern/sidj,
// under the GPL (see the README there); this file is the Loom device around
// them.
//
// FILE steps through the .sid files in the folder (R push: the file list,
// /SID to start with), SONG through the file's subsongs, CHIP picks the
// chip (AUTO: the one the tune was written for), LEVEL the output. A: play
// (held for 2 s: stop, as SHUTTLE), B: the song from the top (a tune picked
// in the list plays at once).
//
// Page 2: V1 V2 V3 each voice's level (0: off), the L knob turns over to
// its PAN (the voices come out apart, SiDJ's stems: they add up to the
// chip's own mix, filter and all); FILT the filter on / off, the L knob
// turns over to CV: the tune also plays the CV outputs, over what they were
// set to, while SID is the track's source: A B C the three voices' pitch
// (the module's octave, 0 V = C0), E F G their gates (Loom.h, WriteOutputs);
// with more chips each voice's from the chip its V1-V3 have picked.
// A tune for two or three SIDs: the first chip left, the second right, a
// third in the middle, as the tune means them. Each chip's voices have a
// level of their own: on V1 V2 V3 the L knob picks the chip (S1V1, S2V1,
// S3V1), the R knob sets that one; there is no PAN then (a one-SID tune
// gets its own back).

#pragma once

extern "C" {
#include "sidj/c64.h"
#include "sidj/sid.h"
}

namespace loom {

PROGMEM static const DeviceDef SID_DEF = {
  "SID", "SID", 8, {"PLAY", "TOP"},
  {
    {"FILE", 0, BROWSE_MAX - 1, 0, UNI},
    {"SONG", 1, 256, 1, UNI},
    {"CHIP", 0, 2, 0, LIST, 3, 0, 0, {"AUTO", "6581", "8580"}},
    {"LEVEL", 0, 100, 100, UNI},
    // level; the mode (L knob) the voice's PAN, L100 .. C .. R100 in tens
    {"V1", 0, 100, 100, UNI, 0, 21, 10},
    {"V2", 0, 100, 100, UNI, 0, 21, 10},
    {"V3", 0, 100, 100, UNI, 0, 21, 10},
    // the filter; the mode (L knob) CV: the voices to the CV outputs
    {"FILT", 0, 1, 1, LIST, 2, 2, 1, {"OFF", "ON"}, {"OFF", "ON"}},
  },
  {"", ""},
};

class SidDevice : public Device, public AudioStream {
public:
  enum { FILE_, SONG, CHIP, LEVEL, V1, V2, V3, FILT };
  // V1-V3: level, PAN their second value; FILT: on / off, CV its second
  const char* AltName(int p) const override { return p >= V1 && p <= V3 && !multi ? "PAN" : p == FILT ? "CV" : nullptr; }
  void FormatMode(int p, char* buf, size_t n) const override;
  // two or three chips: V1-V3's L knob picks the chip (their mode), no PAN
  bool Multi() const { return multi; }
  void NudgeMode(int p, int dir) override;
  bool NameStop(int p) const override { return multi && p >= V1 && p <= V3 ? false : Device::NameStop(p); }
  int ModeCount(int p) const override { return multi && p >= V1 && p <= V3 ? nsid : Device::ModeCount(p); }
  const char* Name(int p) const override;
  // each chip's voice levels and the one-SID pans kept meanwhile (saved:
  // Loom.h, the 'K' record)
  uint8_t chip_lv[C64_NSID][3] = {{100, 100, 100}, {100, 100, 100}, {100, 100, 100}};
  uint8_t pan_keep[3] = {10, 10, 10};
  // the saved settings' 'K' record: more chips or not, the kept pans, the
  // chips' levels; then made to fit the tune that is in
  void RestoreChips(const uint8_t* k) {
    multi = k[0] != 0;
    for (int v = 0; v < 3; v++) pan_keep[v] = std::min<uint8_t>(k[1 + v], 20);
    for (int i = 0; i < 9; i++) chip_lv[i / 3][i % 3] = std::min<uint8_t>(k[4 + i], 100);
    if (multi)
      for (int v = 0; v < 3; v++) {
        mode[V1 + v] = uint8_t(std::min<int>(mode[V1 + v], C64_NSID - 1));
        val[V1 + v] = chip_lv[mode[V1 + v]][v];
      }
    if (loaded >= 0) SetMulti(nsid > 1);
    for (int v = 0; v < 3; v++) Refresh(V1 + v);
  }
  // chip i's voice v level, 0-100 (the one picked: as modulated)
  int Level(int i, int v) const { return !multi || i == mode[V1 + v] ? cur[V1 + v] : chip_lv[i][v]; }
  static constexpr uint32_t MAX_SID = 0x7c + 2 + 65536;  // header, load address, 64 KB

  SidDevice() : Device(SID_DEF), AudioStream(0, nullptr) {}
  AudioStream* in() override { return nullptr; }
  AudioStream* out() override { return this; }
  bool Activate() override;
  void Deactivate() override;
  bool ok() const override { return mem != nullptr; }
  void Apply(int p) override;
  int Step(int p, int v, int dir) const override;
  void Format(int p, char* buf, size_t n) const override;
  void Action(int a) override;
  bool ActionLit(int a) const override { return a == 0 && playing; }
  void Stop() { playing = false; }  // A held (Loom.h)
  // the CV outputs: once a tick (main loop) the voices' pitch in semitones
  // above C0 and their gates; a note started again within the tick takes
  // its gate down for that tick, so an envelope fires again
  bool CvOn() const { return mode[FILT] != 0 && ok(); }
  void CvTick();
  float cv_semis[3] = {};
  bool cv_gate[3] = {};
  bool HeaderInfo(char* buf, size_t n) const override;
  // a tune for two or three chips: 2SID / 3SID over the page
  const char* Title() const override { return loaded >= 0 && nsid == 2 ? "2SID" : loaded >= 0 && nsid == 3 ? "3SID" : "SID"; }
  void DrawViz(int x, int y, int w, int h) override;
  void DrawMini(int x, int y, int w, int h) override;
  void CpuMax(float parts[2]) override {
    parts[0] = processorUsageMax();
    parts[1] = 0.0f;
    processorUsageMaxReset();
  }
  void update() override;
  // R held: back to the defaults, the file and its song kept
  void Reset() override {
    const int16_t f = val[FILE_], s = val[SONG];
    Defaults();
    val[FILE_] = cur[FILE_] = f;
    val[SONG] = cur[SONG] = s;
    for (auto& c : chip_lv) for (auto& l : c) l = 100;
    for (auto& k : pan_keep) k = 10;
    if (multi) for (int v = 0; v < 3; v++) mode[V1 + v] = 0;  // (chip 1 picked)
    ApplyAll();
  }
  // the file list (R push on FILE)
  int FileParam() const override { return FILE_; }
  void OpenBrowser() override { g_browser->Open(this, dir, FileName(), nullptr, ".SID"); }
  void Picked(const char* folder, const char* name) override;
  // the folder and file, by name (the saved settings): SONG stays as it was
  void SetFile(const char* folder, const char* name);
  const char* Folder() const { return dir; }
  const char* FileName() const { return nfiles ? names[std::min<int>(cur[FILE_], nfiles - 1)] : ""; }
  // linked: the same file, song and settings as the other track's
  void LinkFrom(const SidDevice& a) {
    CopyFrom(a);
    memcpy(chip_lv, a.chip_lv, sizeof(chip_lv));
    memcpy(pan_keep, a.pan_keep, sizeof(pan_keep));
    multi = a.multi;
    SetFile(a.Folder(), a.FileName());
  }

private:
  char dir[96] = "/SID";
  // the folder's tunes (as many as the file list shows), in PSRAM once scanned
  char (*names)[NAME_LEN] = nullptr;
  int nfiles = 0;
  // the file whose song SONG holds (another file starts on its own start song)
  char song_for[NAME_LEN] = "";
  // what is loaded: the file, its song and chip (-1: nothing yet)
  int loaded = -1, loaded_song = -1, loaded_chip = -1;
  uint32_t loaded_at = 0;  // the tune came in (its name crawls from the start)
  void Scan();
  void Load();
  // the tune: the file's bytes (PSRAM, kept for a change of song), the
  // C64's 64 KB (RAM2: the 6510 reads it all the time) and up to three SIDs
  uint8_t* file = nullptr;
  uint32_t file_len = 0;
  uint8_t* mem = nullptr;
  sidfile_t sf = {};
  c64_t c64 = {};
  sid_t sids[C64_NSID] = {};
  int nsid = 1;
  int model = SID_MOS6581;
  const char* fault = nullptr;  // why nothing plays (shown in the picture)
  // the audio side (the C64 runs in the audio update, in chunks)
  volatile bool ready = false;    // c64 / sids hold a tune
  volatile bool playing = false;
  volatile float gain = 1.0f;
  // each chip's voices to L / R, Q15 (the chip's place, then the voice's pan)
  volatile int32_t vg[C64_NSID][3][2] = {};
  void VoiceGains();
  void Mutes();
  bool multi = false;  // V1-V3 hold a chip's level and pick it (else level and PAN)
  void SetMulti(bool m);
  uint32_t chunk_len = 0, pos = 0;
  int log_i = 0, nlog = 0;
  volatile uint32_t played = 0;   // samples since the song started
  volatile int16_t peak = 0;
  volatile uint32_t gate_seq[C64_NSID][3] = {};  // each chip's voices' gates turned on (audio side)
  uint32_t gate_seen[3] = {};
  int cv_chip[3] = {};  // the chip each voice's CV came from last tick
  static uint8_t ReadSid(void* ctx, int chip, uint8_t reg) {
    return sid_read(&static_cast<SidDevice*>(ctx)->sids[chip], reg);
  }
  void Render(int16_t* const bufs[C64_NSID], int n);
  // voice v's envelope (0-255) for the bars: on two or three chips the
  // louder of that voice on each
  int Envelope(int v) const {
    int e = 0;
    for (int i = 0; i < nsid; i++) e = std::max<int>(e, int(sids[i].voice[v].envelope.envelope_counter));
    return e;
  }
};

// the .sid files in the folder, sorted by name
FLASHMEM void SidDevice::Scan() {
  nfiles = 0;
  if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
  if (!SDcard_Ready) return;
  if (!names) names = static_cast<char(*)[NAME_LEN]>(PsramAlloc(BROWSE_MAX * NAME_LEN));
  if (!names) return;
  File d = SD.open(dir);
  if (!d) { snprintf(dir, sizeof(dir), "/"); d = SD.open(dir); }
  if (!d) return;
  while (nfiles < BROWSE_MAX) {
    File e = d.openNextFile();
    if (!e) break;
    const char* nm = e.name();
    const size_t l = strlen(nm);
    if (!e.isDirectory() && nm[0] != '.' && l > 4 && l < NAME_LEN - 1 && !strcasecmp(nm + l - 4, ".SID"))
      snprintf(names[nfiles++], NAME_LEN, "%s", nm);
    e.close();
  }
  d.close();
  for (int i = 1; i < nfiles; i++)
    for (int j = i; j > 0 && ListOrder(names[j - 1], names[j]) > 0; j--) {
      char t[NAME_LEN];
      memcpy(t, names[j], NAME_LEN); memcpy(names[j], names[j - 1], NAME_LEN); memcpy(names[j - 1], t, NAME_LEN);
    }
  Serial.printf("SID: %d tunes in %s\n", nfiles, dir);
}

FLASHMEM void SidDevice::SetFile(const char* folder, const char* name) {
  snprintf(dir, sizeof(dir), "%s", folder && folder[0] ? folder : "/SID");
  Scan();
  for (int i = 0; i < nfiles; i++)
    if (!strcmp(names[i], name)) { val[FILE_] = cur[FILE_] = i; break; }
  snprintf(song_for, sizeof(song_for), "%s", name ? name : "");  // SONG is this file's
  loaded = -1;
  if (Device::active) Load();
}

// a tune picked in the list: it plays at once
FLASHMEM void SidDevice::Picked(const char* folder, const char* name) {
  playing = true;
  if (!strcmp(dir, folder) && !strcmp(FileName(), name)) return;
  snprintf(dir, sizeof(dir), "%s", folder);
  Scan();
  for (int i = 0; i < nfiles; i++)
    if (!strcmp(names[i], name)) { val[FILE_] = cur[FILE_] = i; break; }
  loaded = -1;
  if (Device::active) Load();
}

// called while the stream is not patched in
FLASHMEM bool SidDevice::Activate() {
  if (Device::active) return ok();
  if (!mem) mem = static_cast<uint8_t*>(malloc(65536));
  if (!mem) mem = static_cast<uint8_t*>(PsramAlloc(65536));
  Device::active = true;
  Scan();
  loaded = -1;
  ApplyAll();  // (loads the tune)
  return ok();
}

FLASHMEM void SidDevice::Deactivate() {
  if (!Device::active) return;
  ready = false;
  if (mem) { if (IsPsram(mem)) PsramFree(mem); else free(mem); mem = nullptr; }
  if (file) { PsramFree(file); file = nullptr; }
  file_len = 0;
  loaded = -1;
  Device::active = false;
}

// reads the file (if it is another one) and starts the song on the chip
FLASHMEM void SidDevice::Load() {
  ready = false;  // (the audio update leaves the C64 alone from here)
  fault = nullptr;
  if (!mem) { fault = "out of memory"; return; }
  if (!nfiles) return;
  const int fi = std::min<int>(cur[FILE_], nfiles - 1);
  if (fi != loaded || !file) {
    if (file) { PsramFree(file); file = nullptr; }
    file_len = 0;
    loaded = -1;
    char path[200];
    snprintf(path, sizeof(path), "%s/%s", strcmp(dir, "/") ? dir : "", names[fi]);
    File f = SD.open(path, FILE_READ);
    if (!f) { fault = "can't open"; return; }
    const uint32_t len = std::min<uint32_t>(f.size(), MAX_SID);
    file = static_cast<uint8_t*>(PsramAlloc(len));
    if (!file) { f.close(); fault = "out of memory"; return; }
    file_len = f.read(file, len);
    f.close();
    if (sidfile_parse(&sf, file, file_len) != 0) { fault = "not a SID file"; return; }
    if (sf.rsid && sf.basic) { fault = "BASIC tune"; return; }
    loaded = fi;
    loaded_at = millis();
    // another file: its own start song
    if (strcmp(song_for, names[fi])) {
      val[SONG] = cur[SONG] = sf.start_song;
      snprintf(song_for, sizeof(song_for), "%s", names[fi]);
    }
    Serial.printf("SID: %s  %s / %s / %s  %s v%d, %d songs, %d SID\n", path, sf.name, sf.author, sf.released,
                  sf.rsid ? "RSID" : "PSID", sf.version, sf.songs, sidfile_nsids(&sf));
  }
  if (loaded < 0) return;
  const int song = constrain(int(cur[SONG]), 1, int(sf.songs));
  const bool ntsc = sidfile_clock(&sf) == SID_CLOCK_NTSC;
  model = cur[CHIP] == 1 ? SID_MOS6581 : cur[CHIP] == 2 ? SID_MOS8580
        : sidfile_model(&sf, 0) == SID_MODEL_8580 ? SID_MOS8580 : SID_MOS6581;
  c64_init(&c64, mem);
  c64.sid_read = ReadSid;
  c64.sid_ctx = this;
  for (int i = 0; i < C64_NSID; i++) {
    sid_t& s = sids[i];
    sid_init(&s);
    // AUTO: each chip the model the file says (a 2SID tune may have one of each)
    const int m = cur[CHIP] ? model : sidfile_model(&sf, i) == SID_MODEL_8580 ? SID_MOS8580
                : sidfile_model(&sf, i) == SID_MODEL_6581 ? SID_MOS6581 : model;
    sid_set_model(&s, sid_model_t(m));
    sid_set_sampling(&s, ntsc ? C64_NTSC_HZ : C64_PAL_HZ, uint32_t(AUDIO_SAMPLE_RATE_EXACT + 0.5f));
    sid_set_averaging(&s, 1);
    sid_set_floating_dac(&s, 1);
    sid_enable_filter(&s, cur[FILT] != 0);
    sid_set_stems(&s, 1);  // each voice on its own, for its level and pan
  }
  c64_load(&c64, &sf, song, ntsc);
  nsid = std::max(1, std::min<int>(c64.nsid, C64_NSID));
  SetMulti(nsid > 1);
  Mutes();
  VoiceGains();  // (the chips' places: as many as the tune has)
  chunk_len = pos = 0;
  log_i = nlog = 0;
  played = 0;
  loaded_song = song;
  loaded_chip = cur[CHIP];
  ready = true;
}

FLASHMEM void SidDevice::Apply(int p) {
  switch (p) {
    case FILE_:
      if (Device::active && nfiles && std::min<int>(cur[FILE_], nfiles - 1) != loaded) Load();
      break;
    case SONG:
    case CHIP:
      if (Device::active && loaded >= 0 && (constrain(int(cur[SONG]), 1, int(sf.songs)) != loaded_song || cur[CHIP] != loaded_chip))
        Load();
      break;
    case LEVEL: gain = cur[LEVEL] * 0.01f; break;
    case V1: case V2: case V3:
      if (multi) chip_lv[constrain(int(mode[p]), 0, C64_NSID - 1)][p - V1] = uint8_t(val[p]);
      Mutes();  // (0: off, no CPU for it in the mix)
      VoiceGains();
      break;
    case FILT:
      for (auto& s : sids) sid_enable_filter(&s, cur[FILT] != 0);
      break;
  }
}

FLASHMEM int SidDevice::Step(int p, int v, int dir) const {
  if (p == FILE_) return constrain(v + dir, 0, std::max(0, nfiles - 1));
  if (p == SONG) return constrain(v + dir, 1, loaded >= 0 ? int(sf.songs) : 256);
  return v + dir;
}

FLASHMEM void SidDevice::Format(int p, char* buf, size_t n) const {
  if (p == FILE_) {
    if (!nfiles) snprintf(buf, n, "none");
    else snprintf(buf, n, "%d", std::min<int>(cur[FILE_], nfiles - 1) + 1);
  } else if (p == SONG) {
    if (loaded >= 0) snprintf(buf, n, "%d/%d", constrain(int(val[SONG]), 1, int(sf.songs)), sf.songs);
    else snprintf(buf, n, "%d", val[SONG]);
  } else if (p == CHIP && val[CHIP] == 0 && loaded >= 0) {  // AUTO: the chip it chose
    snprintf(buf, n, "%s", model == SID_MOS8580 ? "A8580" : "A6581");
  } else {
    Device::Format(p, buf, n);
  }
}

// A: play (a click while playing does nothing: A held stops, see Loom.h);
// B: the song from the top (and playing)
FLASHMEM void SidDevice::Action(int a) {
  if (a == 0) { playing = true; return; }
  if (Device::active && loaded >= 0) Load();
  playing = true;
}

FLASHMEM bool SidDevice::HeaderInfo(char* buf, size_t n) const {
  if (!Device::active || loaded < 0) return false;
  const uint32_t s = uint32_t(played / AUDIO_SAMPLE_RATE_EXACT);
  snprintf(buf, n, "%lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
  return true;
}

FLASHMEM void SidDevice::DrawViz(int x, int y, int w, int h) {
  const int cy = y + h / 2;
  if (!mem) { gfxPrint(x + 22, cy - 4, PSTR("out of memory")); return; }
  if (!SDcard_Ready) { gfxPrint(x + 34, cy - 4, PSTR("no SD card")); return; }
  if (!nfiles) { gfxPrint(x + 10, cy - 4, PSTR("no .sid in folder")); return; }
  if (fault) { gfxPrint(x + (w - 6 * int(strlen(fault))) / 2, cy - 4, fault); return; }
  if (loaded < 0) return;
  // the name and the composer (crawling when long), then a bar per voice:
  // its envelope
  Crawl(x + 1, y, (w - 1) / 6, sf.name, loaded_at);
  Crawl(x + 1, y + 9, (w - 1) / 6, sf.author, loaded_at);
  // page 1: each voice as heard (the louder of its chips); page 2 (the
  // voices' own page): each voice's bar split side by side, a part per
  // chip (chip 1 at the left), each filling on its own
  const int by = y + 19, bh = h - 19, bw = (w - 2 - 4 * 3) / 3;
  const int parts = view_page == 1 ? nsid : 1;
  for (int v = 0; v < 3; v++) {
    const int bx = x + 1 + v * (bw + 6);
    bool any = false;
    for (int i = 0; i < nsid; i++) any |= Level(i, v) > 0;
    if (!any) {  // off: struck through
      gfxFrame(bx, by, bw, bh);
      gfxLine(bx, by + bh - 1, bx + bw - 1, by);
      continue;
    }
    const int pw = (bw - 2 * (parts - 1)) / parts;  // (2 pixels between the parts)
    for (int i = 0; i < parts; i++) {
      const int px = bx + i * (pw + 2);
      gfxFrame(px, by, pw, bh);
      if (parts > 1 && !Level(i, v)) { gfxLine(px, by + bh - 1, px + pw - 1, by); continue; }  // this chip's off
      const int e = parts > 1 ? int(sids[i].voice[v].envelope.envelope_counter) : Envelope(v);
      const int len = playing ? e * pw / 255 : 0;
      if (len > 2) gfxRect(px + 1, by + 1, len - 2, bh - 2);
      // the chip the L knob has picked: a dot inside its part, at the right
      // (inverted: dark where the bar has reached it)
      if (parts > 1 && i == mode[V1 + v]) gfxInvert(px + pw - 4, by + bh / 2 - 1, 2, 2);
    }
  }
}

FLASHMEM void SidDevice::CvTick() {
  const bool on = ready && playing;
  const float clock = sidfile_clock(&sf) == SID_CLOCK_NTSC ? float(C64_NTSC_HZ) : float(C64_PAL_HZ);
  for (int v = 0; v < 3; v++) {
    // the chip V1-V3 have picked (more chips), else the one
    const int c = multi ? constrain(int(mode[V1 + v]), 0, nsid - 1) : 0;
    const sid_voice_t& vo = sids[c].voice[v];
    // the pitch is kept while the voice is quiet (a release goes on sounding)
    if (on && vo.wave.freq) {
      const float hz = vo.wave.freq * clock * (1.0f / 16777216.0f);
      cv_semis[v] = std::max(0.0f, 12.0f * log2f(hz * (1.0f / 16.3516f)));  // C0 = 0 V
    }
    const uint32_t s = gate_seq[c][v];
    if (c != cv_chip[v]) { cv_chip[v] = c; gate_seen[v] = s; }  // another chip: from its gate on
    const bool g = on && vo.envelope.gate && Level(c, v) > 0;
    const bool again = s != gate_seen[v] && cv_gate[v];  // started again while high
    gate_seen[v] = s;
    cv_gate[v] = g && !again;
  }
}

FLASHMEM void SidDevice::DrawMini(int x, int y, int w, int h) {
  // the three voices' envelopes, side by side
  const int bw = std::max(1, (w - 2) / 3);
  for (int v = 0; v < 3; v++) {
    const int e = ready && playing ? Envelope(v) : 0;
    const int len = e * h / 255;
    if (len > 0) gfxRect(x + v * (bw + 1), y + h - len, bw, len);
    else gfxLine(x + v * (bw + 1), y + h - 1, x + v * (bw + 1) + bw - 1, y + h - 1);
  }
}

// the C64 runs the tune in chunks of 2048 cycles; every SID write it makes
// lands on the chip at its own cycle, and the chips make the samples
// between them
// each voice's share of L / R: its level, then its pan (the far side
// fades, the near one stays: C is the voice as the chip mixes it). Two or
// three chips: each where the tune means it (the first left, the second
// right, a third in the middle), each voice at its own chip's level
FLASHMEM void SidDevice::VoiceGains() {
  static constexpr float SIDE[3] = {-1.0f, 1.0f, 0.0f};
  for (int i = 0; i < C64_NSID; i++)
    for (int v = 0; v < 3; v++) {
      const float lv = Level(i, v) * 0.01f;
      const float pan = multi ? SIDE[i] : (mode[V1 + v] - 10) * 0.1f;
      vg[i][v][0] = int32_t(lv * std::min(1.0f, 1.0f - pan) * 32768.0f);
      vg[i][v][1] = int32_t(lv * std::min(1.0f, 1.0f + pan) * 32768.0f);
    }
}

// a voice's chips muted where their level is 0 (no CPU for them in the mix)
FLASHMEM void SidDevice::Mutes() {
  for (int i = 0; i < C64_NSID; i++)
    for (int v = 0; v < 3; v++) sids[i].vmute[v] = Level(i < nsid ? i : 0, v) == 0;
}

// a tune for more chips (or back to one): V1-V3 change what they hold.
// Going to more: their level is chip 1's, their PAN kept aside, chip 1
// picked; back to one: chip 1's level, the PAN back
FLASHMEM void SidDevice::SetMulti(bool m) {
  if (m == multi) {
    if (m) for (int v = 0; v < 3; v++) {  // (fewer chips than the last tune: on one there is)
      if (mode[V1 + v] >= nsid) { mode[V1 + v] = 0; val[V1 + v] = chip_lv[0][v]; Refresh(V1 + v); }
    }
    return;
  }
  for (int v = 0; v < 3; v++) {
    const int p = V1 + v;
    if (m) {
      chip_lv[0][v] = uint8_t(val[p]);
      pan_keep[v] = mode[p];
      mode[p] = 0;
    } else {
      chip_lv[constrain(int(mode[p]), 0, C64_NSID - 1)][v] = uint8_t(val[p]);
      val[p] = chip_lv[0][v];
      mode[p] = pan_keep[v];
    }
  }
  multi = m;
  for (int v = 0; v < 3; v++) Refresh(V1 + v);
}

// V1-V3 with more chips: the L knob picks the chip, R then sets its level
FLASHMEM void SidDevice::NudgeMode(int p, int dir) {
  if (!multi || p < V1 || p > V3) { Device::NudgeMode(p, dir); return; }
  const int v = p - V1;
  chip_lv[mode[p]][v] = uint8_t(val[p]);
  mode[p] = uint8_t(constrain(mode[p] + dir, 0, nsid - 1));
  mode_named |= uint16_t(1u << p);
  val[p] = chip_lv[mode[p]][v];
  Refresh(p);
}

FLASHMEM const char* SidDevice::Name(int p) const {
  if (!multi || p < V1 || p > V3) return Device::Name(p);
  static char nm[3][6];
  snprintf(nm[p - V1], sizeof(nm[0]), "S%dV%d", mode[p] + 1, p - V1 + 1);
  return nm[p - V1];
}

// V1-V3: their PAN, L100 .. C .. R100; FILT: CV OFF / ON
FLASHMEM void SidDevice::FormatMode(int p, char* buf, size_t n) const {
  if (multi && p >= V1 && p <= V3) {
    snprintf(buf, n, "S%dV%d", mode[p] + 1, p - V1 + 1);
  } else if (p >= V1 && p <= V3) {
    const int pan = (mode[p] - 10) * 10;
    if (!pan) snprintf(buf, n, "C");
    else snprintf(buf, n, "%c%d", pan < 0 ? 'L' : 'R', abs(pan));
  } else if (p == FILT) {
    snprintf(buf, n, "%s", mode[FILT] ? "ON" : "OFF");
  } else {
    Device::FormatMode(p, buf, n);
  }
}

// the voices, each on its own (stems: 3 a sample, per chip)
DMAMEM static int16_t sid_stems[C64_NSID][3 * AUDIO_BLOCK_SAMPLES];

// the C64 runs the tune in chunks of 2048 cycles; every SID write it makes
// lands on the chip at its own cycle, and the chips make the samples
// between them (bufs: each chip's own mix; the stems go to sid_stems)
FASTRUN void SidDevice::Render(int16_t* const bufs[C64_NSID], int n) {
  int done = 0;
  while (done < n) {
    if (pos >= chunk_len) {
      chunk_len = c64_run(&c64, 2048);
      if (!chunk_len) {  // the 6510 jammed: silence from here
        for (int i = 0; i < nsid; i++) memset(sid_stems[i] + 3 * done, 0, 3 * (n - done) * sizeof(int16_t));
        ready = false;
        fault = "the tune crashed";
        return;
      }
      nlog = c64.nlog;
      log_i = 0;
      pos = 0;
    }
    while (log_i < nlog && c64.log[log_i].cycle <= pos) {
      const c64_write_t& x = c64.log[log_i++];
      if (x.chip >= nsid) continue;
      const int v = x.reg / 7;  // a voice's control register: its gate, for the CV outputs
      const bool ctrl = x.reg < 21 && x.reg % 7 == 4;
      sid_voice_t& vo = sids[x.chip].voice[ctrl ? v : 0];
      const uint32_t was = ctrl ? vo.envelope.gate : 1;
      sid_write(&sids[x.chip], x.reg, x.val);
      if (ctrl && !was && vo.envelope.gate) gate_seq[x.chip][v] = gate_seq[x.chip][v] + 1;
    }
    const uint32_t target = log_i < nlog ? c64.log[log_i].cycle : chunk_len;
    int got = 0;
    for (int i = 0; i < nsid; i++) {
      int d = int(target - pos);
      sids[i].stem = sid_stems[i] + 3 * done;
      got = sid_clock_render(&sids[i], &d, bufs[i] + done, n - done, 1);
      if (i == nsid - 1) pos = target - uint32_t(d);
    }
    done += got;
  }
}

FASTRUN void SidDevice::update() {
  audio_block_t* o[2] = {allocate(), allocate()};
  if (o[0] && o[1]) {
    int16_t* l = o[0]->data;
    int16_t* r = o[1]->data;
    if (ready && playing) {
      static int16_t b3[AUDIO_BLOCK_SAMPLES];
      int16_t* const bufs[C64_NSID] = {l, r, b3};  // (the chips' own mixes: not used, the stems are)
      Render(bufs, AUDIO_BLOCK_SAMPLES);
      played = played + AUDIO_BLOCK_SAMPLES;
      // each voice to L / R at its level and pan, times LEVEL; a third chip
      // (in the middle, on both sides) takes some room
      const float g = gain * (nsid >= 3 ? 0.67f : 1.0f);
      int32_t gl[C64_NSID][3], gr[C64_NSID][3];
      // (Q12: nine voices at full level stay inside 32 bits)
      for (int i = 0; i < nsid; i++)
        for (int v = 0; v < 3; v++) { gl[i][v] = int32_t(vg[i][v][0] * g) >> 3; gr[i][v] = int32_t(vg[i][v][1] * g) >> 3; }
      int32_t pk = 0;
      for (int k = 0; k < AUDIO_BLOCK_SAMPLES; k++) {
        int32_t a = 0, b = 0;
        for (int i = 0; i < nsid; i++) {
          const int16_t* st = sid_stems[i] + 3 * k;
          a += st[0] * gl[i][0] + st[1] * gl[i][1] + st[2] * gl[i][2];
          b += st[0] * gr[i][0] + st[1] * gr[i][1] + st[2] * gr[i][2];
        }
        a >>= 12;
        b >>= 12;
        l[k] = int16_t(constrain(a, -32768, 32767));
        r[k] = int16_t(constrain(b, -32768, 32767));
        pk = std::max(pk, std::max(abs(a), abs(b)));
      }
      peak = int16_t(std::min<int32_t>(pk, 32767));
    } else {
      memset(l, 0, sizeof(o[0]->data));
      memset(r, 0, sizeof(o[1]->data));
    }
    transmit(o[0], 0);
    transmit(o[1], 1);
  }
  for (auto b : o) if (b) release(b);
}

}  // namespace loom
