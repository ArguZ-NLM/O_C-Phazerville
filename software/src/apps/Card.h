#pragma once

// CARD: the SD card over USB (MTP), as an app of its own. Nothing of it runs
// until it is opened; Loom is paused meanwhile and has let go of its big
// buffers. On a Mac the card shows up in OpenMTP (macOS's photo import may
// have to be kept out of the way: card-mode.sh). The screen lists the card's
// folders and files, refreshed every 2 s, so copied files can be seen
// arriving; R enc scrolls, R push opens a folder, B (or Y) goes up, X held
// for 2 s deletes the selected file.

#include <MTP_Teensy.h>

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
};

FLASHMEM void AppCard::Init() {}

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
      if (nm[0] != '.' && strlen(nm) < NAME - 1 && isdir == (pass == 0)) {
        snprintf(names[count], NAME, "%s", nm);
        folder[count] = isdir;
        sizes[count++] = isdir ? 0 : uint32_t(e.size());
      }
      e.close();
    }
    for (int i = first + 1; i < count; i++)
      for (int j = i; j > first && strcasecmp(names[j - 1], names[j]) > 0; j--) {
        char t[NAME];
        memcpy(t, names[j], NAME); memcpy(names[j], names[j - 1], NAME); memcpy(names[j - 1], t, NAME);
        std::swap(sizes[j], sizes[j - 1]);
        std::swap(folder[j], folder[j - 1]);
      }
  }
  d.close();
  sel = constrain(keep, 0, std::max(0, count - 1));
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
    List();
    // The computer only asks for the card when the module turns up on USB,
    // while Loom was running and nobody answered. So: leave the bus for a
    // moment and come back, like pulling and putting back the cable.
    USB1_USBCMD &= ~USB_USBCMD_RS;
    delay(150);
    USB1_USBCMD |= USB_USBCMD_RS;
  } else if (event == OC::APP_EVENT_SUSPEND) {
    serving = false;
  }
}

FLASHMEM void AppCard::Loop() {
  if (!serving) return;
  MTP.loop();
  if (del_held && millis() - del_since >= 2000) {
    del_held = false;
    char path[160];
    snprintf(path, sizeof(path), "%s/%s", strcmp(dir, "/") ? dir : "", names[sel]);
    if (SD.remove(path)) deleted_at = millis() | 1;  // the computer sees it on its next refresh
    List();
  }
  if (millis() - listed_at > 2000) List();  // new files show up by themselves
}

// the top line: whether a program on the computer has the card open (an MTP
// session; it can't tell which program - macOS's photo import counts too)
FLASHMEM void AppCard::DrawMenu() const {
  // a session open, and the USB line really there: cable in (VBUS), the bus
  // not asleep - pulling the cable doesn't end the session by itself
  const bool line = (USB1_OTGSC & (1u << 11)) && !(USB1_PORTSC1 & USB_PORTSC1_SUSP);  // OTGSC bit 11: B-session valid
  const bool on = serving && line && static_cast<bool>(MTP);
  gfxPrint(0, 1, !SDcard_Ready ? "CARD  no SD card" : on ? "CARD  connected" : "CARD  waiting...");
  if (on) gfxInvert(0, 0, 128, 9);
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
  snprintf(t, sizeof(t), "%s", l > 21 ? dir + l - 21 : dir);
  gfxPrint(0, 13, t);
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
      // shortened in the middle like in PLAYER (but with its extension)
      const int nl = int(strlen(names[i]));
      if (nl <= fit) snprintf(nm, sizeof(nm), "%s", names[i]);
      else snprintf(nm, sizeof(nm), "%.*s..%s", (fit - 2 + 1) / 2, names[i], names[i] + nl - (fit - 2) / 2);
      if (folder[i]) strncat(nm, "/", sizeof(nm) - strlen(nm) - 1);
    }
    gfxPrint(2, ry, nm);
    if (sz[0]) gfxPrint(128 - 6 * int(strlen(sz)), ry, sz);
    if (i == sel) gfxInvert(0, ry - 1, 128, 9);
  }
  // deleting: a box with the name and a bar filling over the 2 s
  if (del_held || (deleted_at && millis() - deleted_at < 1000)) {
    graphics.clearRect(8, 26, 112, 24);
    gfxFrame(8, 26, 112, 24);
    if (del_held) {
      char nm[20];
      const int nl = int(strlen(names[sel]));
      if (nl <= 17) snprintf(nm, sizeof(nm), "%s", names[sel]);
      else snprintf(nm, sizeof(nm), "%.8s..%s", names[sel], names[sel] + nl - 7);
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
  sel = constrain(sel + event.value, 0, std::max(0, count - 1));
}

// R push: open the folder; B (or Y): up; X held: delete the file
FLASHMEM void AppCard::HandleButtonEvent(const UI::Event& event) {
  if (event.control == OC::CONTROL_BUTTON_X) {
    if (event.type == UI::EVENT_BUTTON_DOWN) {
      del_held = sel < count && !folder[sel];
      del_since = millis();
    } else if (event.type == UI::EVENT_BUTTON_PRESS || event.type == UI::EVENT_BUTTON_LONG_RELEASE) {
      del_held = false;  // let go early: nothing happens
    }
    return;
  }
  if (event.type != UI::EVENT_BUTTON_PRESS) return;
  const bool up = event.control == OC::CONTROL_BUTTON_B || event.control == OC::CONTROL_BUTTON_Y ||
                  (event.control == OC::CONTROL_BUTTON_R && sel < count && !strcmp(names[sel], ".."));
  if (up && strcmp(dir, "/")) {
    char* slash = strrchr(dir, '/');
    if (slash == dir) dir[1] = 0; else if (slash) *slash = 0;
    sel = 0;
    List();
  } else if (event.control == OC::CONTROL_BUTTON_R && sel < count && folder[sel]) {
    const size_t l = strlen(dir);
    if (l + strlen(names[sel]) + 2 < sizeof(dir)) {
      snprintf(dir + l, sizeof(dir) - l, "%s%s", l > 1 ? "/" : "", names[sel]);
      sel = 0;
      List();
    }
  }
}
