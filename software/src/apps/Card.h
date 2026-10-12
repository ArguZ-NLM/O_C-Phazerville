#pragma once

// CARD: the SD card over USB (MTP), as an app of its own. Nothing of it runs
// until it is opened; Loom is paused meanwhile and has let go of its big
// buffers. On a Mac the card shows up in OpenMTP (macOS's photo import may
// have to be kept out of the way: card-mode.sh). The screen lists the card's
// folders and files, refreshed every 2 s, so copied files can be seen
// arriving; R enc scrolls, R push opens a folder, Y goes up, X held for 2 s
// deletes the selected file. B listens to a WAV (as in PLAYER's file list:
// once through, straight from the card), B held: AUTO, every file the
// cursor rests on plays. An MP3 copied anywhere onto the card becomes a WAV
// of the same name (the MP3 goes once that worked); the top line shows how
// far it is.

#include <MTP_Teensy.h>
#include "../Mp3ToWav.h"

// the quick listen is Loom's (Loom is paused while CARD is open)
#if defined(ENABLE_APP_LOOM) && defined(ARDUINO_TEENSY41)
#define CARD_LISTEN
#endif

OC_APP_CLASS(AppCard, TWOCCS("CR"), "Card", "SD card over USB") {
public:
  OC_APP_INTERFACE_DECLARE(AppCard, 0);

private:
  static constexpr int MAX = 64, NAME = 48;
  bool started = false;  // MTP set up (once, on first open)
  bool serving = false;  // open: answering the computer
  // the list
  char dir[96] = "/";
  char names[MAX][NAME] = {};
  uint32_t sizes[MAX] = {};
  bool folder[MAX] = {};
  int count = 0, sel = 0;
  uint32_t listed_at = 0;
  // X held on a file: delete it after 2 s
  bool del_held = false;
  uint32_t del_since = 0, deleted_at = 0;
  void List();
  // MP3s on the card become WAVs, one at a time: an MP3 is taken once its
  // size stayed the same between two looks (copied in full)
  Mp3ToWav conv;
  char mp3_seen[160] = {};
  uint32_t mp3_seen_size = 0, mp3_looked_at = 0;
  static constexpr int MAX_BAD = 16;
  char mp3_bad[MAX_BAD][48] = {};  // couldn't be converted: not tried again
  int nbad = 0;
  uint32_t failed_at = 0;  // shown in the top line for a while
  uint32_t mtp_at = 0;  // the computer last answered (while converting)
  char failed_name[24] = {};
  bool FindMp3(const char* dir, int depth, char* out, size_t n, uint32_t& size) const;
  void LookForMp3();
  void Converted();
  bool IsWav(int i) const {
    const size_t l = strlen(names[i]);
    return i < count && !folder[i] && l > 4 && !strcasecmp(names[i] + l - 4, ".WAV");
  }
#ifdef CARD_LISTEN
  loom::LoomPreview* preview = nullptr;  // Loom's, while open
  bool autoplay = false, auto_due = false;
  uint32_t auto_at = 0;
  void Listen(int i) {  // the file under the cursor, or quiet (a folder)
    if (!preview) return;
    if (i < count && IsWav(i)) { if (!preview->Is(dir, names[i])) preview->Start(dir, names[i]); }
    else preview->Stop();
  }
  void Moved() { if (autoplay) { auto_due = true; auto_at = millis(); } }
#else
  void Moved() {}
#endif
};

FLASHMEM void AppCard::Init() {}

// alphabetical, but Loom's own folders first: BOLTS, SPOOLS, THRUMS
static int Order(const char* a, const char* b) {
  // BOLTS, SPOOLS, THRUMS (in that order), then the rest by name
  auto rank = [](const char* n) {
    return !strcasecmp(n, "BOLTS") ? 0 : !strcasecmp(n, "SPOOLS") ? 1 : !strcasecmp(n, "THRUMS") ? 2 : 3;
  };
  if (rank(a) != rank(b)) return rank(a) - rank(b);
  return strcasecmp(a, b);
}

// folders first (sorted), then files (sorted), with their sizes
FLASHMEM void AppCard::List() {
  listed_at = millis();
  const int keep = sel;
  count = 0;
  if (strcmp(dir, "/")) { snprintf(names[count], NAME, ".."); folder[count] = true; sizes[count++] = 0; }
  File d = SDcard_Ready ? SD.open(dir) : File();
  if (!d) { sel = 0; return; }
  for (int pass = 0; pass < 2; pass++) {
    d.rewindDirectory();
    const int first = count;
    while (count < MAX) {
      File e = d.openNextFile();
      if (!e) break;
      const char* nm = e.name();
      const bool isdir = e.isDirectory();
      // (a .loom is the settings of the WAV next to it: not listed, it goes
      // with the WAV; nor Loom's session count, SESSION.TXT)
      const size_t nl = strlen(nm);
      const bool meta = !isdir && ((nl > 5 && !strcasecmp(nm + nl - 5, ".loom")) || !strcasecmp(nm, "SESSION.TXT"));
      if (nm[0] != '.' && !meta && nl < NAME - 1 && isdir == (pass == 0)) {
        snprintf(names[count], NAME, "%s", nm);
        folder[count] = isdir;
        sizes[count++] = isdir ? 0 : uint32_t(e.size());
      }
      e.close();
    }
    for (int i = first + 1; i < count; i++)
      for (int j = i; j > first && Order(names[j - 1], names[j]) > 0; j--) {
        char t[NAME];
        memcpy(t, names[j], NAME); memcpy(names[j], names[j - 1], NAME); memcpy(names[j - 1], t, NAME);
        std::swap(sizes[j], sizes[j - 1]);
        std::swap(folder[j], folder[j - 1]);
      }
  }
  d.close();
  sel = constrain(keep, 0, std::max(0, count - 1));
}

// the first MP3 on the card (folders up to 4 deep), not one that failed
FLASHMEM bool AppCard::FindMp3(const char* dir, int depth, char* out, size_t n, uint32_t& size) const {
  File d = SD.open(dir);
  if (!d) return false;
  bool found = false;
  while (!found) {
    File e = d.openNextFile();
    if (!e) break;
    const char* nm = e.name();
    const size_t l = strlen(nm);
    char path[160];
    if (nm[0] != '.' && strcmp(nm, "System Volume Information") &&
        snprintf(path, sizeof(path), "%s/%s", strcmp(dir, "/") ? dir : "", nm) < int(sizeof(path))) {
      if (e.isDirectory()) {
        if (depth < 4) found = FindMp3(path, depth + 1, out, n, size);
      } else if (l > 4 && !strcasecmp(nm + l - 4, ".MP3")) {
        bool bad = false;
        for (int i = 0; i < nbad; i++) bad |= !strncmp(mp3_bad[i], path, sizeof(mp3_bad[i]) - 1);
        if (!bad) {
          snprintf(out, n, "%s", path);
          size = uint32_t(e.size());
          found = true;
        }
      }
    }
    e.close();
  }
  d.close();
  return found;
}

FLASHMEM void AppCard::LookForMp3() {
  mp3_looked_at = millis();
  char p[160];
  uint32_t sz = 0;
  if (!SDcard_Ready || !FindMp3("/", 0, p, sizeof(p), sz)) { mp3_seen[0] = 0; return; }
  if (sz && sz == mp3_seen_size && !strcmp(p, mp3_seen)) {
    mp3_seen[0] = 0;
    Serial.printf("CARD: converting %s\n", p);
    if (!conv.Start(p)) Converted();
  } else {
    snprintf(mp3_seen, sizeof(mp3_seen), "%s", p);
    mp3_seen_size = sz;
  }
}

// one done: the computer is told (the WAV in, the MP3 out), or it failed
FLASHMEM void AppCard::Converted() {
  if (conv.state() == Mp3ToWav::DONE) {
    MtpReplace(conv.Mp3Path(), conv.WavPath());
  } else {
    if (nbad < MAX_BAD) snprintf(mp3_bad[nbad++], sizeof(mp3_bad[0]), "%s", conv.Mp3Path());
    const char* nm = strrchr(conv.Mp3Path(), '/');
    snprintf(failed_name, sizeof(failed_name), "%s", nm ? nm + 1 : conv.Mp3Path());
    failed_at = millis() | 1;
  }
  Serial.printf("CARD: %s %s\n", conv.state() == Mp3ToWav::DONE ? "converted" : "could not convert", conv.Mp3Path());
  List();
  mp3_looked_at = millis() - 2000;  // more of them: the next one straight away
}

FLASHMEM void AppCard::HandleAppEvent(OC::AppEvent event) {
  if (event == OC::APP_EVENT_RESUME) {
    if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
    if (!started) {
      MTP.begin();
      if (SDcard_Ready) MTP.addFilesystem(SD, "SD_Card");
      started = true;
    }
    serving = true;
    // every time it opens: in the LOOM folder, where the sound files and
    // the recordings are (".." goes up)
    snprintf(dir, sizeof(dir), "%s", SDcard_Ready && SD.exists("/LOOM") ? "/LOOM" : "/");
    // SPOOL's loops were in RECORDINGS: now SPOOLS
    if (SDcard_Ready && !SD.exists("/LOOM/SPOOLS") && SD.exists("/LOOM/RECORDINGS"))
      SD.rename("/LOOM/RECORDINGS", "/LOOM/SPOOLS");
    // and the thrums were in WAVES for a day: now THRUMS
    if (SDcard_Ready && !SD.exists("/LOOM/THRUMS") && SD.exists("/LOOM/WAVES"))
      SD.rename("/LOOM/WAVES", "/LOOM/THRUMS");
    sel = 0;
    List();
    mp3_seen[0] = 0;
    nbad = 0;  // those that failed: tried again
    mp3_looked_at = millis() - 2000;  // MP3s put on the card elsewhere: now
#ifdef CARD_LISTEN
    preview = loom::g_preview;  // Loom's (made at start-up)
    if (preview) {
      preview->level = 0.8f;  // as loud as in Loom (there its mixer has the 0.8)
      OC::AudioIO::ClaimOutput(preview->stream);
    }
#endif
    // The computer only asks for the card when the module turns up on USB,
    // while Loom was running and nobody answered. So: leave the bus for a
    // moment and come back, like pulling and putting back the cable.
    USB1_USBCMD &= ~USB_USBCMD_RS;
    delay(150);
    USB1_USBCMD |= USB_USBCMD_RS;
  } else if (event == OC::APP_EVENT_SUSPEND) {
    serving = false;
    conv.Cancel();  // (its MP3 stays: done next time)
#ifdef CARD_LISTEN
    autoplay = false;
    if (preview) { preview->Halt(); preview->level = 1.0f; }
    OC::AudioIO::ReleaseOutput();
#endif
  }
}

FLASHMEM void AppCard::Loop() {
  if (!serving) return;
  // while an MP3 is converted, the computer is answered once a second only:
  // answering it all the time slowed the card down ~15 times (3:36 took
  // 3.5 min instead of 14 s)
  if (conv.state() != Mp3ToWav::BUSY || millis() - mtp_at >= 1000) {
    mtp_at = millis();
    MTP.loop();
  }
#ifdef CARD_LISTEN
  if (autoplay && auto_due && millis() - auto_at >= 100) { auto_due = false; Listen(sel); }
  if (preview) preview->Idle(true);
#endif
  if (del_held && millis() - del_since >= 2000) {
    del_held = false;
    char path[160];
    snprintf(path, sizeof(path), "%s/%s", strcmp(dir, "/") ? dir : "", names[sel]);
    if (SD.remove(path)) {
      deleted_at = millis() | 1;  // the computer sees it on its next refresh
      // a WAV's settings (name.loom) go with it
      char* dot = strrchr(path, '.');
      if (dot && !strcasecmp(dot, ".wav")) {
        strcpy(dot, ".loom");
        SD.remove(path);
      }
    }
    List();
  }
  if (conv.state() == Mp3ToWav::BUSY) {
    if (conv.Step(20) != Mp3ToWav::BUSY) Converted();
  } else if (millis() - mp3_looked_at > 2000) {
    LookForMp3();
  }
  // new files show up by themselves (not while converting: opening the
  // files it reads and writes made the card refuse it, every 2 s)
  if (conv.state() != Mp3ToWav::BUSY && millis() - listed_at > 2000) List();
}

// the top line: whether a program on the computer has the card open (an MTP
// session; it can't tell which program - macOS's photo import counts too)
FLASHMEM void AppCard::DrawMenu() const {
  // a session open, and the USB line really there: cable in (VBUS), the bus
  // not asleep - pulling the cable doesn't end the session by itself
  const bool line = (USB1_OTGSC & (1u << 11)) && !(USB1_PORTSC1 & USB_PORTSC1_SUSP);  // OTGSC bit 11: B-session valid
  const bool on = serving && line && static_cast<bool>(MTP);
  if (conv.state() == Mp3ToWav::BUSY) {
    // an MP3 being converted: its name, and the line fills up as it goes
    const char* p = strrchr(conv.Mp3Path(), '/');
    p = p ? p + 1 : conv.Mp3Path();
    const int nl = std::max(0, int(strlen(p)) - 4);  // without .mp3
    char t[24];
    snprintf(t, sizeof(t), "MP3>WAV %3d%% %.*s", conv.Percent(), std::min(nl, 8), p);
    gfxPrint(0, 1, t);
    gfxInvert(0, 0, 128 * conv.Percent() / 100, 9);
  } else if (failed_at && millis() - failed_at < 4000) {
    char t[24];
    snprintf(t, sizeof(t), "MP3 FAILED %.10s", failed_name);
    gfxPrint(0, 1, t);
    gfxInvert(0, 0, 128, 9);
  } else {
    gfxPrint(0, 1, !SDcard_Ready ? "CARD  no SD card" : on ? "CARD  connected" : "CARD  waiting...");
    if (on) gfxInvert(0, 0, 128, 9);
  }
  gfxLine(0, 10, 127, 10);
  if (!SDcard_Ready) return;
  if (!on) {  // waiting: how to connect, instead of the list
    gfxPrint(0, 16, "Open the card on the");
    gfxPrint(0, 25, "computer (Mac: in");
    gfxPrint(0, 34, "OpenMTP).");
#ifdef LOOM_ONLY
    // the version and build number (the boot picture shows only the version)
    gfxPrint(0, 43, OC::Strings::VERSION);
    gfxPrint(6 * int(strlen(OC::Strings::VERSION)), 43, OC::Strings::BUILD_TAG);
    gfxPrint(0, 52, "back: Z + R push");
#else
    gfxPrint(0, 52, "back: app menu");
#endif
    return;
  }
  // connected: the folder, then 5 entries around the selection
  char t[24];
  const size_t l = strlen(dir);
#ifdef CARD_LISTEN
  const size_t room = autoplay ? 16 : 21;  // AUTO takes the right end
#else
  const size_t room = 21;
#endif
  snprintf(t, sizeof(t), "%s", l > room ? dir + l - room : dir);
  gfxPrint(0, 13, t);
#ifdef CARD_LISTEN
  if (autoplay) {
    gfxPrint(103, 13, "AUTO");
    gfxInvert(102, 12, 26, 9);
  }
#endif
  if (!count) { gfxPrint(4, 30, "(empty)"); return; }
  constexpr int rows = 5;
  const int first = constrain(sel - rows / 2, 0, std::max(0, count - rows));
  for (int r = 0; r < rows && first + r < count; r++) {
    const int i = first + r;
    const int ry = 23 + r * 8;
    char sz[8] = {};
    if (!folder[i]) {
      const uint32_t b = sizes[i];
      if (b < 1024) snprintf(sz, sizeof(sz), "%luB", (unsigned long)b);
      else if (b < 1024 * 1024) snprintf(sz, sizeof(sz), "%luK", (unsigned long)(b >> 10));
      else snprintf(sz, sizeof(sz), "%luM", (unsigned long)(b >> 20));
    }
    const int fit = 20 - (sz[0] ? int(strlen(sz)) + 1 : 0) - (folder[i] ? 1 : 0);
    char nm[28];
    if (!strcmp(names[i], "..")) {
      snprintf(nm, sizeof(nm), "..");
    } else {
      // shortened in the middle like in PLAYER; sound files without .wav
      const int nl = int(strlen(names[i])) - (IsWav(i) ? 4 : 0);
      if (nl <= fit) snprintf(nm, sizeof(nm), "%.*s", nl, names[i]);
      else snprintf(nm, sizeof(nm), "%.*s..%.*s", (fit - 2 + 1) / 2, names[i], (fit - 2) / 2, names[i] + nl - (fit - 2) / 2);
      if (folder[i]) strncat(nm, "/", sizeof(nm) - strlen(nm) - 1);
    }
    gfxPrint(2, ry, nm);
#ifdef CARD_LISTEN
    if (!folder[i] && preview && preview->Is(dir, names[i])) {
      // listening: a small arrow where the size is
      graphics.clearRect(128 - 6 * int(strlen(sz)), ry - 1, 6 * int(strlen(sz)), 9);
      for (int k = 0; k < 3; k++) gfxLine(123 + k, ry + k, 123 + k, ry + 6 - k);
    } else
#endif
    if (sz[0]) gfxPrint(128 - 6 * int(strlen(sz)), ry, sz);
    if (i == sel) gfxInvert(0, ry - 1, 128, 9);
  }
  // deleting: a box with the name and a bar filling over the 2 s
  if (del_held || (deleted_at && millis() - deleted_at < 1000)) {
    graphics.clearRect(8, 26, 112, 24);
    gfxFrame(8, 26, 112, 24);
    if (del_held) {
      char nm[20];
      const int nl = int(strlen(names[sel])) - (IsWav(sel) ? 4 : 0);
      if (nl <= 17) snprintf(nm, sizeof(nm), "%.*s", nl, names[sel]);
      else snprintf(nm, sizeof(nm), "%.8s..%.7s", names[sel], names[sel] + nl - 7);
      gfxPrint(12, 29, "DELETE");
      gfxPrint(12, 38, nm);
      const int fill = std::min<int>(106, (millis() - del_since) * 106 / 2000);
      gfxInvert(11, 28, fill, 20);
    } else {
      gfxPrint(40, 34, "deleted");
      gfxInvert(9, 27, 110, 22);
    }
  }
}
FLASHMEM void AppCard::DrawScreensaver() const { DrawMenu(); }
void AppCard::Process(OC::IOFrame*) {}
void AppCard::GetIOConfig(OC::IOConfig&) const {}
void AppCard::DrawDebugInfo() const {}
size_t AppCard::SaveAppData(util::StreamBufferWriter&) const { return 0; }
size_t AppCard::RestoreAppData(util::StreamBufferReader&) { return 0; }

FLASHMEM void AppCard::HandleEncoderEvent(const UI::Event& event) {
  const int was = sel;
  sel = constrain(sel + event.value, 0, std::max(0, count - 1));
  if (sel != was) Moved();
}

// R push: open the folder; Y: up; B: listen (held: AUTO); X held: delete
// the file
FLASHMEM void AppCard::HandleButtonEvent(const UI::Event& event) {
#ifdef CARD_LISTEN
  if (event.control == OC::CONTROL_BUTTON_B) {
    if (event.type == UI::EVENT_BUTTON_LONG_PRESS) {
      autoplay = !autoplay;
      auto_due = false;
      if (autoplay) Listen(sel); else if (preview) preview->Stop();
    } else if (event.type == UI::EVENT_BUTTON_PRESS && preview && sel < count && IsWav(sel)) {
      if (preview->Is(dir, names[sel])) preview->Stop();
      else preview->Start(dir, names[sel]);
    }
    return;
  }
#endif
  if (event.control == OC::CONTROL_BUTTON_X) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      del_held = sel < count && !folder[sel];
      del_since = millis();
#ifdef CARD_LISTEN
      // the file may be the one playing: it fades and is closed well
      // before the 2 s are up
      autoplay = false;
      if (del_held && preview) preview->Stop();
#endif
    } else if (event.type == UI::EVENT_BUTTON_PRESS || event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      del_held = false;  // let go early: nothing happens
    }
    return;
  }
  if (event.type != UI::EVENT_BUTTON_PRESS) return;
  const bool up = event.control == OC::CONTROL_BUTTON_Y ||
                  (event.control == OC::CONTROL_BUTTON_R && sel < count && !strcmp(names[sel], ".."));
  if (up && strcmp(dir, "/")) {
    char* slash = strrchr(dir, '/');
    if (slash == dir) dir[1] = 0; else if (slash) *slash = 0;
    sel = 0;
    List();
    Moved();
  } else if (event.control == OC::CONTROL_BUTTON_R && sel < count && folder[sel]) {
    const size_t l = strlen(dir);
    if (l + strlen(names[sel]) + 2 < sizeof(dir)) {
      snprintf(dir + l, sizeof(dir) - l, "%s%s", l > 1 ? "/" : "", names[sel]);
      sel = 0;
      List();
      Moved();
    }
  }
}
