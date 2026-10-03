#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>

// Resonant filter bank for the Sculpt RING device, after the S4's RING.
//
// BANDS two-pole band-pass resonators (Chamberlin state variable), fed with
// the track's input summed to mono; even bands go left, odd bands right.
// The owner sets each band's frequency coefficient, gain and damping; the
// bank only runs them. Bands set to gain 0 (or too high to be stable) are
// skipped. CHOKE: the damping rises when the input goes quiet, so the bank
// stops ringing with the input instead of sustaining.
class AudioEffectRing : public AudioStream {
public:
  static constexpr int BANDS = 48;

  struct Viz {
    float band[BANDS];  // peak per band since the last read
    float out;
  };

  AudioEffectRing() : AudioStream(2, input_queue_array) {}

  // --- from the UI thread: the whole bank at once ---
  void SetBank(const float* f, const float* gain, float damp, bool choke, float wet) {
    AudioNoInterrupts();
    for (int b = 0; b < BANDS; b++) {
      coef[b] = f[b];
      target_gain[b] = gain[b];
    }
    damp_ = damp;
    choke_ = choke;
    wet_ = wet;
    AudioInterrupts();
  }

  void Reset() {
    for (int b = 0; b < BANDS; b++) lp[b] = bp[b] = 0.0f;
  }

  void ReadViz(Viz& v) {
    AudioNoInterrupts();
    v = viz;
    viz = Viz{};
    AudioInterrupts();
  }

  void update() override;

private:
  audio_block_t* input_queue_array[2];
  float coef[BANDS] = {}, target_gain[BANDS] = {}, gain[BANDS] = {};
  float lp[BANDS] = {}, bp[BANDS] = {};
  float damp_ = 0.1f, wet_ = 1.0f, env = 0.0f;
  bool choke_ = false;
  Viz viz = {};
};

// Runs from flash like the other Sculpt engines (RAM1 is full).
FLASHMEM void AudioEffectRing::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }

  // gains glide a step per block so CUTOFF / SLOPE moves don't click
  int active[BANDS], na = 0;
  for (int b = 0; b < BANDS; b++) {
    gain[b] += 0.3f * (target_gain[b] - gain[b]);
    if (gain[b] > 1e-4f && coef[b] > 0.0f) active[na++] = b;
    else lp[b] = bp[b] = 0.0f;
  }

  constexpr float q = 1.0f / 32768.0f;
  const float wet = wet_, base_damp = damp_;
  const bool choke = choke_;
  float pk[BANDS] = {};
  float peak_out = 0.0f;

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const float dl = in[0] ? in[0]->data[n] * q : 0.0f;
    const float dr = in[1] ? in[1]->data[n] * q : 0.0f;
    const float x = 0.5f * (dl + dr);
    float damp = base_damp;
    if (choke) {
      // follow the input; when it is quiet the bands get damped hard
      const float a = fabsf(x);
      env = a > env ? a : env * 0.9995f;
      damp += (1.0f - std::min(1.0f, env * 20.0f)) * 0.5f;
    }
    float sum[2] = {0.0f, 0.0f};
    for (int i = 0; i < na; i++) {
      const int b = active[i];
      const float f = coef[b];
      lp[b] += f * bp[b];
      const float hp = x - lp[b] - damp * bp[b];
      bp[b] += f * hp;
      // damp * bp: unity gain at the band's centre
      const float y = bp[b] * damp * gain[b];
      sum[b & 1] += y;
      if ((n & 7) == 0) pk[b] = std::max(pk[b], fabsf(y));
    }
    const float ol = dl + wet * (sum[0] - dl);
    const float orr = dr + wet * (sum[1] - dr);
    peak_out = std::max(peak_out, std::max(fabsf(ol), fabsf(orr)));
    out[0]->data[n] = __SSAT(int32_t(ol * 32767.0f), 16);
    out[1]->data[n] = __SSAT(int32_t(orr * 32767.0f), 16);
  }

  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);
  for (int b = 0; b < BANDS; b++) viz.band[b] = std::max(viz.band[b], pk[b]);
  viz.out = std::max(viz.out, peak_out);
}
