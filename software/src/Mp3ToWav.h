#pragma once

// MP3 to WAV on the module: an .mp3 on the SD card becomes a 16-bit WAV of
// the same name next to it (at the MP3's own rate, mono or stereo as it
// was), a little at a time, so the CARD app keeps answering the computer
// meanwhile. The decoder is minimp3 (public domain), src/minimp3.h.

#include <stdint.h>

class Mp3ToWav {
public:
  enum State : uint8_t { IDLE, BUSY, DONE, FAILED };

  // begins with the .mp3 at path; false if it can't (no memory, no file)
  bool Start(const char* path);
  // decodes for about ms milliseconds; BUSY until it is through
  State Step(uint32_t ms);
  // gives up (the half-made WAV goes)
  void Cancel();

  State state() const { return st; }
  int Percent() const { return in_size ? int(uint64_t(in_read) * 100 / in_size) : 0; }
  const char* Mp3Path() const { return mp3; }
  const char* WavPath() const { return wav; }

private:
  static constexpr uint32_t IN_SIZE = 16 * 1024;  // holds 10 frames, as minimp3 wants
  State st = IDLE;
  char mp3[160] = {}, wav[160] = {}, tmp[160] = {};
  struct Work;  // the decoder and its buffers, only while busy
  Work* w = nullptr;
  uint32_t in_size = 0, in_read = 0;
  void Finish(bool ok);
};

// the MP3 gone, the WAV there: deletes `gone` and tells the computer, if
// one is connected over MTP, about both, so they show without reconnecting
void MtpReplace(const char* gone, const char* added);

// what the converter did last (USB 'P')
extern char g_mp3_report[200];
