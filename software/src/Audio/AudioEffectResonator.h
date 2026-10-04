#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>

// Resonant filter bank for the Sculpt RESONATOR device.
//
// BANDS two-pole band-pass resonators (Chamberlin state variable), fed with
// the track's input summed to mono; even bands go left, odd bands right.
// The owner sets each band's frequency coefficient, gain and damping; the
// bank only runs them. Bands set to gain 0 (or too high to be stable) are
// skipped. CHOKE: the damping rises when the input goes quiet, so the bank
// stops ringing with the input instead of sustaining.
class AudioEffectResonator : public AudioStream {
public:
  static constexpr int BANDS = 48;

  struct Viz {
    float band[BANDS];  // peak per band since the last read
    float out;
  };

  AudioEffectResonator() : AudioStream(2, input_queue_array) {}

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
  // per-block scratch, kept here rather than on the small stack
  float x[AUDIO_BLOCK_SAMPLES], dmp[AUDIO_BLOCK_SAMPLES], sum[2][AUDIO_BLOCK_SAMPLES];
};

// Runs from flash like the other Sculpt engines (RAM1 is full).
FLASHMEM void AudioEffectResonator::update() {
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

  // bands far below the loudest one are not heard and skipped; the rest
  // run four at a time over the whole block (left and right side apart),
  // so their state stays in registers and the four can overlap
  float gmax = 0.0f;
  for (int b = 0; b < BANDS; b++) gmax = std::max(gmax, gain[b]);
  memset(sum, 0, sizeof(sum));
  for (int side = 0; side < 2; side++) {
    int act[BANDS / 2 + 3], na = 0;
    for (int b = side; b < BANDS; b += 2) {
      if (gain[b] <= gmax * 1e-3f || gain[b] < 1e-4f || coef[b] <= 0.0f) lp[b] = bp[b] = 0.0f;
      else act[na++] = b;
    }
    while (na & 3) act[na++] = -1;  // padding: a silent band
    float* acc = sum[side];
    for (int i = 0; i < na; i += 4) {
      float f[4], g[4], l[4], v[4], e[4] = {};
      for (int j = 0; j < 4; j++) {
        const int b = act[i + j];
        f[j] = b < 0 ? 0.0f : coef[b];
        g[j] = b < 0 ? 0.0f : gain[b];
        l[j] = b < 0 ? 0.0f : lp[b];
        v[j] = b < 0 ? 0.0f : bp[b];
      }
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        const float xn = x[n], d = dmp[n];
        float y = acc[n];
#pragma GCC unroll 4
        for (int j = 0; j < 4; j++) {
          l[j] += f[j] * v[j];
          v[j] += f[j] * (xn - l[j] - d * v[j]);
          y += g[j] * v[j];
          e[j] += v[j] * v[j];
        }
        acc[n] = y;
      }
      for (int j = 0; j < 4; j++) {
        const int b = act[i + j];
        if (b < 0) continue;
        lp[b] = l[j];
        bp[b] = v[j];
        // band level for the picture: about its peak, from the energy
        const float pk = sqrtf(2.0f * e[j] / AUDIO_BLOCK_SAMPLES) * damp_ * g[j];
        viz.band[b] = std::max(viz.band[b], pk);
      }
    }
  }
  // damp * bp: unity gain at a band's centre
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    sum[0][n] *= dmp[n];
    sum[1][n] *= dmp[n];
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
