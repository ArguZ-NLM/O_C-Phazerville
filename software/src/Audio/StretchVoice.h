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
  // switched out of the chain: only record (cheap), so the buffer is up to
  // date when it comes back; Restart() then begins with fresh windows
  void Record(const float* in, size_t n);
  void Restart();
  // main loop: the correlator's search (a slice at a time)
  void Idle();

  // position 0..1 (how far back, from now to the oldest), size 0..1 (window
  // length), pitch in semitones, speed (frozen: how fast it plays through
  // the buffer, 1 = as recorded, < 0 backwards)
  void Set(float position, float size, float pitch, float speed, bool freeze) {
    position_ = position; size_ = size; pitch_ = pitch; speed_ = speed; freeze_ = freeze;
  }
  // random variation, drawn anew for each window (roughly): pitch +-semitones,
  // size and position 0..1, and stereo: each window's place left / right 0..1
  void SetSpread(float pitch, float size, float pos, float stereo) {
    pitch_sp_ = pitch; size_sp_ = size; pos_sp_ = pos; stereo_sp_ = stereo;
  }
  void Trigger() { trigger_ = true; }
  float Scrub() const { return scrub_; }  // where the frozen playback is (0..1, for the picture)
  // this window's variation (for the picture)
  float DPitch() const { return d_pitch_; }
  float DPos() const { return d_pos_; }
  float DSize() const { return d_size_; }
  float Pan() const { return pan_; }

private:
  void* impl_ = nullptr;  // the Clouds objects
  int16_t* buf_ = nullptr;
  bool ready_ = false;
  float position_ = 0.2f, size_ = 0.5f, pitch_ = 0.0f, speed_ = 1.0f;
  float scrub_ = 0.0f;   // frozen: the moving part of the position
  bool freeze_ = false;
  float pitch_sp_ = 0.0f, size_sp_ = 0.0f, pos_sp_ = 0.0f, stereo_sp_ = 0.0f;
  float d_pitch_ = 0.0f, d_size_ = 0.0f, d_pos_ = 0.0f;  // this window's variation
  float pan_ = 0.0f, pan_to_ = 0.0f, pan_step_ = 0.0f;   // -1 .. 1, ramped over a hop
  int32_t hop_left_ = 0;
  uint32_t rng_ = 0x9E3779B9u;
  float Rand();  // -1 .. 1
  void NextWindow();
  volatile bool trigger_ = false;
};
