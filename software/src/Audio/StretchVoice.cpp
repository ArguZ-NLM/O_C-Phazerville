// Loom: compiled for speed (the build is -Os; this is per-sample DSP)
#pragma GCC optimize("O2")
// Clouds' pitch / time engine for the Loom STRETCH device, see
// StretchVoice.h. The DSP is Mutable Instruments' code (Emilie Gillet,
// MIT), unchanged; this file drives it the way Clouds' processor did in its
// STRETCH mode (record with a crossfade unless frozen, play the WSOLA
// player, search with the correlator from the main loop, as Clouds did).

#include <Arduino.h>
#include "PsramAlloc.h"
#include <Audio.h>
#include <new>

#include "StretchVoice.h"
#include "clouds/dsp/audio_buffer.h"
#include "clouds/dsp/correlator.h"
#include "clouds/dsp/wsola_sample_player.h"


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
  const int32_t frames = int32_t(seconds * AUDIO_SAMPLE_RATE_EXACT);
  void* im = malloc(sizeof(Impl));
  buf_ = static_cast<int16_t*>(PsramAlloc(size_t(frames) * 2 * sizeof(int16_t)));
  if (!im || !buf_) {
    free(im);
    if (buf_) PsramFree(buf_);
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

// the audio lets go first (an instant); clearing the 4 s buffer takes ~30
// ms and runs with the audio going (with it held off, that was a gap)
FLASHMEM void StretchVoice::Free() {
  if (!ready_) return;
  AudioNoInterrupts();
  ready_ = false;
  auto* p = static_cast<Impl*>(impl_);
  int16_t* b = buf_;
  impl_ = nullptr;
  buf_ = nullptr;
  AudioInterrupts();
  p->~Impl();
  free(p);
  PsramFree(b);
}

void StretchVoice::Work(uint32_t budget_us) {
  if (!ready_) return;
  auto* p = static_cast<Impl*>(impl_);
  // a few candidates at a time, looking at the clock in between (Clouds'
  // EvaluateSomeCandidates does a quarter of a search in one go, which
  // can take longer than the timer's turn); nothing when no search waits
  // (the correlator's size is garbage before the first one)
  const uint32_t t0 = micros();
  while (micros() - t0 < budget_us) {
    p->player.LoadCorrelator(p->buffer);
    if (p->correlator.done()) break;
    for (int k = 0; k < 16 && !p->correlator.done(); k++) p->correlator.EvaluateNextCandidate();
  }
}

float StretchVoice::Rand() {
  rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
  return int32_t(rng_) * (1.0f / 2147483648.0f);
}

// a new set of variations, held for about one window hop (the player picks
// up position / size / pitch when it starts a window); the stereo place
// glides to its new value over the hop, so it moves without clicks
void StretchVoice::NextWindow() {
  d_pitch_ = Rand() * pitch_sp_;
  d_size_ = Rand() * size_sp_ * 0.5f;
  d_pos_ = Rand() * pos_sp_;
  const float size = std::min(1.0f, std::max(0.0f, size_ + d_size_));
  // the player's window: kMaxWSOLASize at size 1, 5 octaves shorter at 0
  hop_left_ = std::max<int32_t>(64, int32_t(clouds::kMaxWSOLASize * exp2f((size - 1.0f) * 5.0f)) / 2);
  pan_to_ = Rand() * stereo_sp_;
  pan_step_ = (pan_to_ - pan_) / hop_left_;
}

void StretchVoice::Record(const float* in, size_t n) {
  if (!ready_) return;
  auto* p = static_cast<Impl*>(impl_);
  for (size_t done = 0; done < n; done += kChunk) {
    const size_t size = std::min(n - done, kChunk);
    for (int c = 0; c < 2; c++) p->buffer[c].WriteFade(&in[2 * done + c], int32_t(size), 2, !freeze_);
  }
}

// the windows that were playing point at audio recorded over since
void StretchVoice::Restart() {
  if (!ready_) return;
  auto* p = static_cast<Impl*>(impl_);
  p->player.Init(&p->correlator, 2);
  hop_left_ = 0;
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
    if (hop_left_ <= 0) NextWindow();
    hop_left_ -= int32_t(size);
    // frozen, it runs round the held buffer; recording, the window stops at
    // now and at the oldest sound (wrapping, it jumped from one to the other,
    // into what is being written)
    float pos = position_ + scrub_ + d_pos_;
    if (freeze_) pos -= floorf(pos);
    else pos = std::min(0.98f, std::max(0.0f, pos));
    clouds::Parameters prm = {};
    prm.position = pos;
    prm.size = std::min(1.0f, std::max(0.0f, size_ + d_size_));
    prm.pitch = pitch_ + d_pitch_;
    prm.freeze = freeze_;
    prm.trigger = trigger_;
    trigger_ = false;
    p->player.Play(p->buffer, prm, out + 2 * done, size);
    // stereo spread: the output moved left / right (equal power, the
    // middle untouched)
    if (stereo_sp_ > 0.0f || pan_ != 0.0f) {
      float* o = out + 2 * done;
      for (size_t i = 0; i < size; i++, o += 2) {
        pan_ += pan_step_;
        if ((pan_step_ > 0.0f && pan_ > pan_to_) || (pan_step_ < 0.0f && pan_ < pan_to_)) pan_ = pan_to_;
        const float a = (pan_ + 1.0f) * 0.25f * float(M_PI);
        o[0] *= cosf(a) * 1.41421356f;
        o[1] *= sinf(a) * 1.41421356f;
      }
    }
    done += size;
  }
}
