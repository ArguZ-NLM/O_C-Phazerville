#pragma once
// One Mutable Instruments Rings voice (rings::Part and its strummer, MIT,
// in src/extern/mi) for the Loom MODAL device, behind a small interface so
// the Rings / stmlib headers stay in RingsVoice.cpp (their macros clash with
// the firmware's).
//
// Mono in (the exciter), two outputs like the module: odd partials / strings
// on "out", even ones on "aux". Runs at the Teensy audio rate (Rings ran at
// 48 kHz; its sample rate constant is set to ours).

#include <stddef.h>
#include <stdint.h>

class RingsVoice {
public:
  // the models, as on the module
  enum Model { MODAL, SYMPATHETIC, STRING, FM, SYMPATHETIC_QUANTIZED, STRING_REVERB, MODELS };

  bool Init();   // takes its memory (RAM2 heap + PSRAM for the reverb); false if there is none
  void Free();
  bool ok() const { return part_ != nullptr; }

  // n frames of mono input into out / aux (n any size: done in Rings' blocks)
  void Process(const float* in, float* out, float* aux, size_t n);

  // note: semitones above C0 (0..72); the rest 0..1 like the module's knobs
  void SetPatch(float note, float structure, float brightness, float damping, float position) {
    note_ = note; structure_ = structure; brightness_ = brightness; damping_ = damping; position_ = position;
  }
  void SetModel(int m);
  void Strike() { strike_ = true; }  // a hit by hand: the internal exciter for one block

private:
  void* part_ = nullptr;
  void* strummer_ = nullptr;
  uint16_t* reverb_ = nullptr;
  float note_ = 36.0f, structure_ = 0.5f, brightness_ = 0.5f, damping_ = 0.5f, position_ = 0.5f;
  int model_ = MODAL;
  volatile bool strike_ = false;
  float in_level_ = 0.0f;
};
