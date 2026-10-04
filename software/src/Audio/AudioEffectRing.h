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
  for (int b = 0; b < BANDS; b++) gain[b] += 0.3f * (target_gain[b] - gain[b]);

  constexpr float q = 1.0f / 32768.0f;
  const float wet = wet_;

  // the mono input and the damping for each sample of the block
  float x[AUDIO_BLOCK_SAMPLES], dmp[AUDIO_BLOCK_SAMPLES];
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const float dl = in[0] ? in[0]->data[n] * q : 0.0f;
    const float dr = in[1] ? in[1]->data[n] * q : 0.0f;
    x[n] = 0.5f * (dl + dr);
    float damp = damp_;
    if (choke_) {
      // follow the input; when it is quiet the bands get damped hard
      const float a = fabsf(x[n]);
      env = a > env ? a : env * 0.9995f;
      damp += (1.0f - std::min(1.0f, env * 20.0f)) * 0.5f;
    }
    dmp[n] = damp;
  }

  // one band at a time over the whole block, so its state stays in
  // registers; bands far below the loudest one are not heard and skipped
  float gmax = 0.0f;
  for (int b = 0; b < BANDS; b++) gmax = std::max(gmax, gain[b]);
  float sum[2][AUDIO_BLOCK_SAMPLES] = {};
  for (int b = 0; b < BANDS; b++) {
    const float f = coef[b], g = gain[b];
    if (g <= gmax * 1e-3f || g < 1e-4f || f <= 0.0f) {
      lp[b] = bp[b] = 0.0f;
      continue;
    }
    float l = lp[b], v = bp[b], pk = 0.0f;
    float* acc = sum[b & 1];
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      l += f * v;
      const float d = dmp[n];
      v += f * (x[n] - l - d * v);
      // damp * bp: unity gain at the band's centre
      const float y = v * d * g;
      acc[n] += y;
      pk = std::max(pk, fabsf(y));
    }
    lp[b] = l;
    bp[b] = v;
    viz.band[b] = std::max(viz.band[b], pk);
  }

  float peak_out = 0.0f;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const float dl = in[0] ? in[0]->data[n] * q : 0.0f;
    const float dr = in[1] ? in[1]->data[n] * q : 0.0f;
    const float ol = dl + wet * (sum[0][n] - dl);
    const float orr = dr + wet * (sum[1][n] - dr);
    peak_out = std::max(peak_out, std::max(fabsf(ol), fabsf(orr)));
    out[0]->data[n] = __SSAT(int32_t(ol * 32767.0f), 16);
    out[1]->data[n] = __SSAT(int32_t(orr * 32767.0f), 16);
  }

  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);
  viz.out = std::max(viz.out, peak_out);
}
