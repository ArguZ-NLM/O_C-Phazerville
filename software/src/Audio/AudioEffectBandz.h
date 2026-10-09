#pragma once

#include <Audio.h>
#include "AudioSample.h"
#include <algorithm>
#include <cmath>

// Stereo 3-band tone + dynamics stage for the Loom MIX device, after
// Phazerville's 3-Bandz applet: the signal is split into low / mid / high
// (state-variable filters; mid = input - low - high, so the bands always sum
// back to the input), each band gets a gain and a compressor, then they are
// summed again. One BANDZ knob sets all three compressors (lower threshold,
// higher ratio). Auto make-up: a slow (~0.4 s) loudness match brings the
// compressed sum back to the level it had before compressing (at most
// +12 dB, held through silence), so COMP changes the feel, not the volume.
// Flat gains and BANDZ at 0 pass audio through.
class AudioEffectBandz : public AudioStream {
public:
  static constexpr int BANDS = 3;

  struct Viz {
    float level[BANDS];  // band peaks after gain, 0..1
    float gr_db[BANDS];  // gain reduction per band
  };

  AudioEffectBandz() : AudioStream(2, input_queue_array) {}

  // --- parameters, set from the UI thread ---
  void band_gain(int b, float db) { gain[b] = powf(10.0f, constrain(db, -24.0f, 24.0f) * 0.05f); Check(); }
  void split_low(float hz) { f_lo = Coeff(hz); }
  void set_bypass(bool b) { bypass = b; }  // fades to pass through, settings kept
  void split_high(float hz) { f_hi = Coeff(hz); }
  void compress(float c) {
    c = constrain(c, 0.0f, 1.0f);
    thresh_db = -6.0f - 24.0f * c;
    slope = c > 0.0f ? 1.0f - 1.0f / (1.0f + 5.0f * c) : 0.0f;
    Check();
  }

  void ReadViz(Viz& v) {
    AudioNoInterrupts();
    v = viz;
    viz = Viz{};
    AudioInterrupts();
  }

  // the last CAPTURE output samples (L+R) for the spectrum, oldest first:
  // enough for a 512-point FFT at a quarter of the sample rate (low end)
  static constexpr int CAPTURE = 2048;
  void ReadCapture(int16_t* dst) {
    AudioNoInterrupts();
    const int p = cap_pos;
    for (int i = 0; i < CAPTURE; i++) dst[i] = cap[(p + i) % CAPTURE];
    AudioInterrupts();
  }

  void update() override;

private:
  audio_block_t* input_queue_array[2];
  int16_t cap[CAPTURE] = {};
  int cap_pos = 0;
  void Capture(const audio_block_t* l, const audio_block_t* r) {
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      const int a = l ? l->data[n] : 0, b = r ? r->data[n] : 0;
      cap[cap_pos] = (a + b) >> 1;
      cap_pos = (cap_pos + 1) % CAPTURE;
    }
  }
  volatile float gain[BANDS] = {1.0f, 1.0f, 1.0f};
  volatile float f_lo = 0.03f, f_hi = 0.3f;
  volatile float thresh_db = -6.0f, slope = 0.0f;
  volatile bool flat = true, bypass = false;
  float mixw = 1.0f;  // processed share, fades with bypass

  float lo_l[2] = {}, lo_b[2] = {}, hi_l[2] = {}, hi_b[2] = {};
  float env[BANDS] = {}, cgain[BANDS] = {1.0f, 1.0f, 1.0f}, target[BANDS] = {1.0f, 1.0f, 1.0f};
  float e_ref = 0.0f, e_cmp = 0.0f, auto_gain = 1.0f;  // auto make-up
  Viz viz = {};

  static float Coeff(float hz) { // Chamberlin SVF frequency coefficient
    return 2.0f * sinf(float(M_PI) * std::min(hz, 7000.0f) / AUDIO_SAMPLE_RATE_EXACT);
  }
  void Check() {
    flat = slope == 0.0f && gain[0] == 1.0f && gain[1] == 1.0f && gain[2] == 1.0f;
  }
};

// Runs from RAM1 (ITCM) like the other Loom engines.
FASTRUN void AudioEffectBandz::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  // bypass fades over ~15 ms; nothing to do once it is all the way out
  const float mix_to = bypass ? 0.0f : 1.0f;
  if (flat || (mix_to == 0.0f && mixw == 0.0f)) {
    if (flat) mixw = mix_to;
    Capture(in[0], in[1]);
    // nothing to do: pass the blocks on as they are
    for (int c = 0; c < 2; c++) {
      if (!in[c]) continue;
      transmit(in[c], c);
      release(in[c]);
    }
    return;
  }
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }

  const float g[BANDS] = {gain[0], gain[1], gain[2]};
  const float fl = f_lo, fh = f_hi;
  const float th = thresh_db, sl = slope;
  const bool comp = sl > 0.0f;
  constexpr float att = 0.99584f, rel = 0.99986f; // 5 ms / 150 ms
  constexpr float q = 1.0f / 32768.0f, damp = 1.4f; // ~Butterworth

  float peak[BANDS] = {}, max_gr[BANDS] = {};
  // auto make-up for this block: from where the last block left it towards
  // the gain that matches compressed to uncompressed loudness
  float ag_to = 1.0f;
  if (comp) {
    if (e_ref > 1e-7f) ag_to = std::min(4.0f, sqrtf(e_ref / std::max(e_cmp, 1e-9f)));
    else ag_to = auto_gain;  // silence: hold
  }
  const float ag_step = (ag_to - auto_gain) / AUDIO_BLOCK_SAMPLES;
  float blk_ref = 0.0f, blk_cmp = 0.0f;
  constexpr float mix_step = 1.0f / (0.015f * AUDIO_SAMPLE_RATE_EXACT);
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    mixw = mix_to > mixw ? std::min(mix_to, mixw + mix_step) : std::max(mix_to, mixw - mix_step);
    float band[2][BANDS];
    float det[BANDS] = {};
    for (int c = 0; c < 2; c++) {
      const float x = in[c] ? in[c]->data[n] * q : 0.0f;
      // low: low-pass at the low split
      lo_l[c] += fl * lo_b[c];
      const float lh = x - lo_l[c] - damp * lo_b[c];
      lo_b[c] += fl * lh;
      // high: high-pass at the high split
      hi_l[c] += fh * hi_b[c];
      const float hh = x - hi_l[c] - damp * hi_b[c];
      hi_b[c] += fh * hh;
      band[c][0] = lo_l[c];
      band[c][2] = hh;
      band[c][1] = x - lo_l[c] - hh;
      for (int b = 0; b < BANDS; b++) det[b] = std::max(det[b], fabsf(band[c][b] * g[b]));
    }
    if (comp) {
      for (int b = 0; b < BANDS; b++) {
        env[b] = det[b] > env[b] ? att * env[b] + (1.0f - att) * det[b]
                                 : rel * env[b] + (1.0f - rel) * det[b];
      }
      if ((n & 15) == 0) {
        for (int b = 0; b < BANDS; b++) {
          const float over = 20.0f * log10f(env[b] + 1e-6f) - th;
          const float gr = over > 0.0f ? over * sl : 0.0f;
          max_gr[b] = std::max(max_gr[b], gr);
          target[b] = powf(10.0f, -gr * 0.05f);
        }
      }
      for (int b = 0; b < BANDS; b++) cgain[b] += 0.05f * (target[b] - cgain[b]);
    }
    auto_gain += ag_step;
    for (int c = 0; c < 2; c++) {
      float y = 0.0f, yr = 0.0f;
      for (int b = 0; b < BANDS; b++) {
        const float u = band[c][b] * g[b];
        const float v = u * (comp ? cgain[b] : 1.0f);
        yr += u;
        y += v;
        peak[b] = std::max(peak[b], fabsf(v * auto_gain));
      }
      blk_ref += yr * yr;
      blk_cmp += y * y;
      y *= auto_gain;
      const float x = in[c] ? in[c]->data[n] * q : 0.0f;
      out[c]->data[n] = ToSample16(x + mixw * (y - x));
    }
  }

  auto_gain = ag_to;
  // loudness before / after compressing, ~0.4 s
  constexpr float ea = 0.9933f;
  e_ref = ea * e_ref + (1.0f - ea) * blk_ref / AUDIO_BLOCK_SAMPLES;
  e_cmp = ea * e_cmp + (1.0f - ea) * blk_cmp / AUDIO_BLOCK_SAMPLES;

  Capture(out[0], out[1]);
  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);
  for (int b = 0; b < BANDS; b++) {
    viz.level[b] = std::max(viz.level[b], peak[b]);
    viz.gr_db[b] = std::max(viz.gr_db[b], max_gr[b]);
  }
}
