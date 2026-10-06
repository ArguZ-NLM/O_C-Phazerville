// One Rings voice for the Loom MODAL device, see RingsVoice.h.
// The Rings DSP itself is Mutable Instruments' code (Emilie Gillet, MIT),
// unchanged apart from its sample rate; this file only drives it the way
// the module's main loop did (noise gate, strummer, Part::Process).

#include <Arduino.h>
#include <new>

#include "RingsVoice.h"
#include "rings/dsp/part.h"
#include "rings/dsp/strummer.h"


extern "C" void* extmem_malloc(size_t);
extern "C" void extmem_free(void*);
extern "C" uint8_t external_psram_size;

namespace {
constexpr float kNoiseGateThreshold = 0.00003f;  // as on the module

// RAM2 first; if that's full, the PSRAM (slower, but it works)
void* Take(size_t n, bool& in_ext) {
  void* p = malloc(n);
  in_ext = false;
  if (!p && external_psram_size) { p = extmem_malloc(n); in_ext = p != nullptr; }
  return p;
}
void Give(void* p, bool in_ext) { if (in_ext) extmem_free(p); else free(p); }
}

FLASHMEM bool RingsVoice::Init() {
  if (part_) return true;
  void* pm = Take(sizeof(rings::Part), part_ext_);
  void* sm = Take(sizeof(rings::Strummer), strum_ext_);
  if (!pm || !sm) {
    if (pm) Give(pm, part_ext_);
    if (sm) Give(sm, strum_ext_);
    return false;
  }
  auto* part = new (pm) rings::Part();
  auto* strummer = new (sm) rings::Strummer();
  part->Init(nullptr);  // no reverb memory: the string + reverb model isn't offered
  part->set_polyphony(1);
  part->set_model(static_cast<rings::ResonatorModel>(model_));
  strummer->Init(0.01f, rings::kSampleRate / rings::kMaxBlockSize);
  strummer_ = strummer;
  part_ = part;  // last: the audio side checks it
  return true;
}

FLASHMEM void RingsVoice::Free() {
  if (!part_) return;
  void* p = part_;
  part_ = nullptr;
  static_cast<rings::Part*>(p)->~Part();
  static_cast<rings::Strummer*>(strummer_)->~Strummer();
  Give(p, part_ext_);
  Give(strummer_, strum_ext_);
  strummer_ = nullptr;
}

FLASHMEM void RingsVoice::SetModel(int m) {
  model_ = m < 0 ? 0 : m >= MODELS ? MODELS - 1 : m;
  if (part_) static_cast<rings::Part*>(part_)->set_model(static_cast<rings::ResonatorModel>(model_));
}

void RingsVoice::Process(const float* in, float* out, float* aux, size_t n) {
  auto* part = static_cast<rings::Part*>(part_);
  auto* strummer = static_cast<rings::Strummer*>(strummer_);
  if (!part) {
    for (size_t i = 0; i < n; i++) out[i] = aux[i] = 0.0f;
    return;
  }
  rings::Patch patch;
  patch.structure = structure_;
  patch.brightness = brightness_;
  patch.damping = damping_;
  patch.position = position_;

  float gated[rings::kMaxBlockSize];
  for (size_t done = 0; done < n;) {
    const size_t size = std::min(n - done, rings::kMaxBlockSize);
    // the module's noise gate on the input
    for (size_t i = 0; i < size; i++) {
      const float s = in[done + i];
      const float error = s * s - in_level_;
      in_level_ += error * (error > 0.0f ? 0.1f : 0.0001f);
      const float gain = in_level_ <= kNoiseGateThreshold ? (1.0f / kNoiseGateThreshold) * in_level_ : 1.0f;
      gated[i] = gain * s;
    }
    rings::PerformanceState ps;
    ps.strum = false;
    ps.internal_exciter = false;  // the track is the exciter
    ps.internal_strum = true;     // strums from the input's onsets
    ps.internal_note = true;      // the pitch is NOTE alone
    ps.tonic = 12.0f + note_;     // C0 = MIDI 12
    ps.note = 0.0f;
    ps.fm = 0.0f;
    int chord = int(structure_ * (rings::kNumChords - 1) + 0.5f);
    ps.chord = chord < 0 ? 0 : chord >= rings::kNumChords ? rings::kNumChords - 1 : chord;
    strummer->Process(gated, size, &ps);
    if (strike_) {  // a hit by hand: the internal exciter, strummed
      strike_ = false;
      ps.internal_exciter = true;
      ps.strum = true;
    }
    part->Process(ps, patch, gated, out + done, aux + done, size);
    done += size;
  }
}
