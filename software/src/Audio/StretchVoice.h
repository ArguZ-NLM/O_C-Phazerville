#pragma once
// Mutable Instruments Clouds' pitch / time engine (its WSOLA sample player
// and correlator, MIT, in src/extern/mi) for the Loom STRETCH device,
// behind a small interface (the Clouds / stmlib headers stay in
// StretchVoice.cpp: their macros clash with the firmware's).
//
// The input is recorded into a stereo buffer (PSRAM) unless frozen and
// played back as overlapping windows, each placed where it lines up best
// with the last one (the correlator, run from the main loop), so the pitch
// changes without the speed and, frozen, the speed without the pitch.

#include <stddef.h>
#include <stdint.h>

class StretchVoice {
public:
  bool Init(float seconds);  // takes the buffer (PSRAM) and the rest; false if there is none
  void Free();
  bool ok() const { return ready_; }

  // n stereo frames, interleaved (L R L R ...)
  void Process(const float* in, float* out, size_t n);
  // main loop: the correlator's search (a slice at a time)
  void Idle();

  // position 0..1 (how far back, from now to the oldest), size 0..1 (window
  // length), pitch in semitones, speed (frozen: how fast it plays through
  // the buffer, 1 = as recorded, < 0 backwards)
  void Set(float position, float size, float pitch, float speed, bool freeze) {
    position_ = position; size_ = size; pitch_ = pitch; speed_ = speed; freeze_ = freeze;
  }
  void Trigger() { trigger_ = true; }
  float Scrub() const { return scrub_; }  // where the frozen playback is (0..1, for the picture)

private:
  void* impl_ = nullptr;  // the Clouds objects
  int16_t* buf_ = nullptr;
  bool ready_ = false;
  float position_ = 0.2f, size_ = 0.5f, pitch_ = 0.0f, speed_ = 1.0f;
  float scrub_ = 0.0f;   // frozen: the moving part of the position
  bool freeze_ = false;
  volatile bool trigger_ = false;
};
