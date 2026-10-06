// Clouds' pitch / time engine for the Loom STRETCH device, see
// StretchVoice.h. The DSP is Mutable Instruments' code (Emilie Gillet,
// MIT), unchanged; this file drives it the way Clouds' processor did in its
// STRETCH mode (record with a crossfade unless frozen, play the WSOLA
// player, search with the correlator from the main loop).

#include <Arduino.h>
#include <new>

#include "StretchVoice.h"
#include "clouds/dsp/audio_buffer.h"
#include "clouds/dsp/correlator.h"
#include "clouds/dsp/wsola_sample_player.h"

extern "C" void* extmem_malloc(size_t);
extern "C" void extmem_free(void*);

namespace {
constexpr size_t kCorrelatorBlock = (clouds::kMaxWSOLASize / 32) + 2;
constexpr size_t kChunk = 32;  // Clouds' block size

struct Impl {
  clouds::AudioBuffer<clouds::RESOLUTION_16_BIT> buffer[2];
  int16_t tail[2][::kCrossFadeSize];
  clouds::Correlator correlator;
  uint32_t correlator_data[kCorrelatorBlock * 3];
  clouds::WSOLASamplePlayer player;
  int32_t frames = 0;
};
}

FLASHMEM bool StretchVoice::Init(float seconds) {
  if (ready_) return true;
  const int32_t frames = int32_t(seconds * 44117.647f);
  void* im = malloc(sizeof(Impl));
  buf_ = static_cast<int16_t*>(extmem_malloc(size_t(frames) * 2 * sizeof(int16_t)));
  if (!im || !buf_) {
    free(im);
    if (buf_) extmem_free(buf_);
    buf_ = nullptr;
    return false;
  }
  auto* p = new (im) Impl();
  p->frames = frames;
  for (int c = 0; c < 2; c++) p->buffer[c].Init(buf_ + size_t(c) * frames, frames, p->tail[c]);
  p->correlator.Init(&p->correlator_data[0], &p->correlator_data[kCorrelatorBlock]);
  p->player.Init(&p->correlator, 2);
  impl_ = p;
  ready_ = true;  // last: the audio side checks it
  return true;
}

FLASHMEM void StretchVoice::Free() {
  if (!ready_) return;
  ready_ = false;
  auto* p = static_cast<Impl*>(impl_);
  p->~Impl();
  free(p);
  impl_ = nullptr;
  extmem_free(buf_);
  buf_ = nullptr;
}

FLASHMEM void StretchVoice::Idle() {
  if (!ready_) return;
  auto* p = static_cast<Impl*>(impl_);
  p->player.LoadCorrelator(p->buffer);
  p->correlator.EvaluateSomeCandidates();
}

void StretchVoice::Process(const float* in, float* out, size_t n) {
  if (!ready_) {
    for (size_t i = 0; i < 2 * n; i++) out[i] = 0.0f;
    return;
  }
  auto* p = static_cast<Impl*>(impl_);
  for (size_t done = 0; done < n;) {
    const size_t size = std::min(n - done, kChunk);
    // record (with Clouds' crossfade when freezing / thawing)
    for (int c = 0; c < 2; c++) p->buffer[c].WriteFade(&in[2 * done + c], int32_t(size), 2, !freeze_);
    // frozen: play through the buffer at SPEED (the position moves; live,
    // the head moves and the position stays)
    if (freeze_) {
      const float limit = float(p->frames) * 0.9f;
      scrub_ -= speed_ * size / limit;
      scrub_ -= floorf(scrub_);
    } else {
      scrub_ = 0.0f;
    }
    float pos = position_ + scrub_;
    pos -= floorf(pos);
    clouds::Parameters prm = {};
    prm.position = pos;
    prm.size = size_;
    prm.pitch = pitch_;
    prm.freeze = freeze_;
    prm.trigger = trigger_;
    trigger_ = false;
    p->player.Play(p->buffer, prm, out + 2 * done, size);
    done += size;
  }
}
