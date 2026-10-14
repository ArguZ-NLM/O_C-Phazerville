#pragma once

#include <Audio.h>
#include "AudioSample.h"
#include <algorithm>
#include <cmath>

// Stereo 3-band tone + dynamics stage for the Loom MIX device, after
// Phazerville's 3-Bandz applet: the signal is split into low / mid / high
// (state-variable filters; mid = input - low - high, so the bands always sum
// back to the input), each band gets its own compressor and then a level,
// and they are summed again. One amount per band sets its compressor (lower
// threshold, higher ratio); its speed is fixed per band: slow on the lows
// (no wobble on bass notes), fast on the highs (catches transients). Auto
// make-up per band: a slow (~0.4 s) loudness match brings each compressed
// band back to the level it had before (at most +12 dB, held through
// silence), so squashing changes the feel, not the volume; the band's level
// comes after that. All levels at 0 dB and no squashing pass audio through.
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
  // how hard band b is squashed, 0 (off) .. 1
  void split_low(float hz) { f_lo = Coeff(hz); }
  void set_bypass(bool b) { bypass = b; }  // fades to pass through, settings kept
  void split_high(float hz) { f_hi = Coeff(hz); }
  void compress(int b, float c) {
    c = constrain(c, 0.0f, 1.0f);
    thresh_db[b] = -6.0f - 24.0f * c;
    slope[b] = c > 0.0f ? 1.0f - 1.0f / (1.0f + 5.0f * c) : 0.0f;
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
  volatile float thresh_db[BANDS] = {-6.0f, -6.0f, -6.0f}, slope[BANDS] = {};
  volatile bool flat = true, bypass = false;
  float mixw = 1.0f;  // processed share, fades with bypass
  float lim_g = 1.0f; // the limiter's gain now

  float lo_l[2] = {}, lo_b[2] = {}, hi_l[2] = {}, hi_b[2] = {};
  float env[BANDS] = {}, cgain[BANDS] = {1.0f, 1.0f, 1.0f}, target[BANDS] = {1.0f, 1.0f, 1.0f};
  float e_ref[BANDS] = {}, e_cmp[BANDS] = {}, auto_gain[BANDS] = {1.0f, 1.0f, 1.0f};  // auto make-up
  Viz viz = {};

  // log2 / 2^x close enough for a compressor's gain (within 0.03 dB), without
  // the library calls (they ran from flash, 48 a block)
  static inline float FastLog2(float x) {
    union { float f; uint32_t i; } u = {x};
    const float e = float(int((u.i >> 23) & 255) - 128);
    u.i = (u.i & 0x007FFFFF) | 0x3F800000;  // mantissa as 1..2
    const float m = u.f;
    return e + (-0.34484843f * m + 2.02466578f) * m - 0.67487759f;
  }
  static inline float FastExp2(float x) {
    x = std::max(-126.0f, x);
    const float fl = floorf(x), fr = x - fl;
    union { uint32_t i; float f; } u = {uint32_t(int(fl) + 127) << 23};
    return u.f * (1.0f + fr * (0.6565f + fr * 0.3435f));
  }
  static float Coeff(float hz) { // Chamberlin SVF frequency coefficient
    return 2.0f * sinf(float(M_PI) * std::min(hz, 7000.0f) / AUDIO_SAMPLE_RATE_EXACT);
  }
  void Check() {
    flat = slope[0] == 0.0f && slope[1] == 0.0f && slope[2] == 0.0f
        && gain[0] == 1.0f && gain[1] == 1.0f && gain[2] == 1.0f;
  }
};

// Runs from RAM1 (ITCM) like the other Loom engines.
FASTRUN __attribute__((optimize("O2"))) void AudioEffectBandz::update() {  // (O2: the build is -Os)
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
  float yb[2][AUDIO_BLOCK_SAMPLES];  // the result, before the limiter
  const float fl = f_lo, fh = f_hi;
  float th[BANDS], sl[BANDS];
  bool comp[BANDS];
  for (int b = 0; b < BANDS; b++) { th[b] = thresh_db[b]; sl[b] = slope[b]; comp[b] = sl[b] > 0.0f; }
  // attack / release per band: low 20 / 300 ms, mid 8 / 150 ms, high 2 / 80 ms
  static constexpr float att[BANDS] = {0.998867f, 0.997170f, 0.988727f};
  static constexpr float rel[BANDS] = {0.999924f, 0.999849f, 0.999717f};
  constexpr float q = 1.0f / 32768.0f, damp = 1.4f; // ~Butterworth

  float peak[BANDS] = {}, max_gr[BANDS] = {};
  // auto make-up for this block, per band: from where the last block left
  // it towards the gain that matches the compressed band to its loudness
  // before
  float ag[BANDS], ag_to[BANDS], ag_step[BANDS];
  for (int b = 0; b < BANDS; b++) {
    ag[b] = auto_gain[b];
    ag_to[b] = 1.0f;
    if (comp[b]) {
      if (e_ref[b] > 1e-8f) ag_to[b] = std::min(4.0f, sqrtf(e_ref[b] / std::max(e_cmp[b], 1e-10f)));
      else ag_to[b] = auto_gain[b];  // silence: hold
    }
    ag_step[b] = (ag_to[b] - ag[b]) / AUDIO_BLOCK_SAMPLES;
  }
  float blk_ref[BANDS] = {}, blk_cmp[BANDS] = {};
  constexpr float mix_step = 1.0f / (0.015f * AUDIO_SAMPLE_RATE_EXACT);
  // no band squashed (only levels) and nothing gliding: the bands' levels
  // and the split only, without the compressors' per-sample work
  bool easy = mixw == mix_to;
  for (int b = 0; b < BANDS; b++)
    easy = easy && !comp[b] && fabsf(cgain[b] - 1.0f) < 1e-4f && fabsf(ag_to[b] - ag[b]) < 1e-4f;
  if (easy) {
    float gb[BANDS];
    for (int b = 0; b < BANDS; b++) { gb[b] = g[b] * ag[b]; cgain[b] = 1.0f; }
    for (int c = 0; c < 2; c++) {
      float ll = lo_l[c], lb = lo_b[c], hl = hi_l[c], hb = hi_b[c];
      float pk[BANDS] = {};
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        const float x = in[c] ? in[c]->data[n] * q : 0.0f;
        ll += fl * lb;
        const float lh = x - ll - damp * lb;
        lb += fl * lh;
        hl += fh * hb;
        const float hh = x - hl - damp * hb;
        hb += fh * hh;
        const float w0 = ll * gb[0], w1 = (x - ll - hh) * gb[1], w2 = hh * gb[2];
        pk[0] = std::max(pk[0], fabsf(w0));
        pk[1] = std::max(pk[1], fabsf(w1));
        pk[2] = std::max(pk[2], fabsf(w2));
        const float y = w0 + w1 + w2;
        yb[c][n] = x + mixw * (y - x);
      }
      lo_l[c] = ll; lo_b[c] = lb; hi_l[c] = hl; hi_b[c] = hb;
      for (int b = 0; b < BANDS; b++) peak[b] = std::max(peak[b], pk[b]);
    }
  }
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES && !easy; n++) {
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
      for (int b = 0; b < BANDS; b++) det[b] = std::max(det[b], fabsf(band[c][b]));
    }
    for (int b = 0; b < BANDS; b++) {
      ag[b] += ag_step[b];  // (glides back to 1 when the squashing is turned off)
      if (!comp[b]) { cgain[b] += 0.05f * (1.0f - cgain[b]); continue; }
      env[b] = det[b] > env[b] ? att[b] * env[b] + (1.0f - att[b]) * det[b]
                               : rel[b] * env[b] + (1.0f - rel[b]) * det[b];
      if ((n & 15) == 0) {
        const float over = 6.0206f * FastLog2(env[b] + 1e-6f) - th[b];  // dB
        const float gr = over > 0.0f ? over * sl[b] : 0.0f;
        max_gr[b] = std::max(max_gr[b], gr);
        target[b] = FastExp2(-gr * 0.16610f);  // 10^(-gr / 20)
      }
      cgain[b] += 0.05f * (target[b] - cgain[b]);
    }
    for (int c = 0; c < 2; c++) {
      float y = 0.0f;
      for (int b = 0; b < BANDS; b++) {
        const float u = band[c][b];
        const float v = u * cgain[b];
        blk_ref[b] += u * u;
        blk_cmp[b] += v * v;
        const float w = v * ag[b] * g[b];  // made up, then the band's level
        y += w;
        peak[b] = std::max(peak[b], fabsf(w));
      }
      const float x = in[c] ? in[c]->data[n] * q : 0.0f;
      yb[c][n] = x + mixw * (y - x);
    }
  }
  // a limiter (both sides together): band boosts can't push the track
  // into the soft clip (its knee is 90 %); down at once within 32 samples,
  // back up over ~150 ms
  {
    constexpr float TH = 0.9f;
    float pk = 0.0f;
    for (int c = 0; c < 2; c++)
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) pk = std::max(pk, fabsf(yb[c][n]));
    const float need = pk > TH ? TH / pk : 1.0f;
    const float from = lim_g;
    lim_g = need < lim_g ? need : std::min(need, lim_g + (1.0f - lim_g) * 0.02f);
    const int ramp = need < from ? 32 : AUDIO_BLOCK_SAMPLES;
    const float step = (lim_g - from) / ramp;
    for (int c = 0; c < 2; c++) {
      float gg = from;
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        if (n < ramp) gg += step;
        out[c]->data[n] = ToSample16(yb[c][n] * gg);
      }
    }
  }

  // loudness of each band before / after compressing, ~0.4 s
  constexpr float ea = 0.9933f;
  for (int b = 0; b < BANDS; b++) {
    auto_gain[b] = ag_to[b];
    e_ref[b] = ea * e_ref[b] + (1.0f - ea) * blk_ref[b] / AUDIO_BLOCK_SAMPLES;
    e_cmp[b] = ea * e_cmp[b] + (1.0f - ea) * blk_cmp[b] / AUDIO_BLOCK_SAMPLES;
  }

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
