#pragma once

// CARD: the SD card over USB (MTP), as an app of its own. Nothing of it runs
// until it is opened; Sculpt is paused meanwhile and has let go of its big
// buffers. On a Mac the card shows up in OpenMTP or Android File Transfer.

#include <MTP_Teensy.h>

OC_APP_CLASS(AppCard, TWOCCS("CR"), "Card", "SD card over USB") {
public:
  OC_APP_INTERFACE_DECLARE(AppCard, 0);

private:
  bool started = false;  // MTP set up (once, on first open)
  bool serving = false;  // open: answering the computer
};

FLASHMEM void AppCard::Init() {}

FLASHMEM void AppCard::HandleAppEvent(OC::AppEvent event) {
  if (event == OC::APP_EVENT_RESUME) {
    if (!SDcard_Ready) SDcard_Ready = SD.begin(BUILTIN_SDCARD);
    if (!started) {
      MTP.begin();
      if (SDcard_Ready) MTP.addFilesystem(SD, "SD_Card");
      started = true;
    }
    serving = true;
  } else if (event == OC::APP_EVENT_SUSPEND) {
    serving = false;
  }
}

FLASHMEM void AppCard::Loop() {
  if (serving) MTP.loop();
}

FLASHMEM void AppCard::DrawMenu() const {
  gfxHeader("CARD");
  gfxPrint(0, 15, SDcard_Ready ? "SD card on USB" : "no SD card");
  gfxPrint(0, 27, "Mac: OpenMTP or");
  gfxPrint(0, 36, "Android File Transfer");
  gfxPrint(0, 52, "back: app menu");
}
FLASHMEM void AppCard::DrawScreensaver() const { DrawMenu(); }
void AppCard::Process(OC::IOFrame*) {}
void AppCard::GetIOConfig(OC::IOConfig&) const {}
void AppCard::DrawDebugInfo() const {}
size_t AppCard::SaveAppData(util::StreamBufferWriter&) const { return 0; }
size_t AppCard::RestoreAppData(util::StreamBufferReader&) { return 0; }
FLASHMEM void AppCard::HandleButtonEvent(const UI::Event&) {}
FLASHMEM void AppCard::HandleEncoderEvent(const UI::Event&) {}
