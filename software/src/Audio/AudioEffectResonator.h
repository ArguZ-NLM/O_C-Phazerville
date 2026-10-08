#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>

// Resonant filter bank for the Loom RESONATOR device.
//
// BANDS two-pole band-pass resonators (Chamberlin state variable), fed with
// the track's input summed to mono; even bands go left, odd bands right.
// The owner sets each band's frequency coefficient, the bank's shape (where
// it peaks, its slope, the peak's height, the tilt) and the damping. A
// band's gain is applied to what goes in, so a band keeps ringing at its
// level when the shape moves on: with VARIATION, every hit (a jump in the
// input's level) moves the peak by some bands and the tilt by some amount
// at random, so a new hit rings other notes of the scale while the last
// ones ring out (arpeggios). Bands with no gain and nothing ringing are
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

  // the bank's shape: cut = the band it peaks at (fractional), slope -1..1
  // (bandpass .. lowpass / highpass), reso 0..100, tone -50..50, level
  struct Shape { float cut, slope, reso, tone, level; };

  // gain of band k (fractional for a picture) from CUT / SLOPE: 12 dB per
  // octave sides, 6 bands an octave
  static float Base(float k, float cut, float slope) {
    const float d = k - cut;
    const float g_lp = d <= 0.0f ? 1.0f : powf(10.0f, -2.0f * d / 20.0f);
    const float g_hp = d >= 0.0f ? 1.0f : powf(10.0f, 2.0f * d / 20.0f);
    const float g_bp = powf(10.0f, -2.0f * fabsf(d) / 20.0f);
    return slope < 0.0f ? g_bp + (-slope) * (g_lp - g_bp) : g_bp + slope * (g_hp - g_bp);
  }
  // the RESO bump around the cutoff, 0..1
  static float Bump(float k, float cut) {
    const float d = k - cut;
    return expf(-(d * d) / 2.25f);
  }

  // --- from the UI thread: the whole bank at once ---
  // cut_var: how many bands a hit may move the peak (either way); tone_var:
  // how far it may move the tilt
  void SetBank(const float* f, const Shape& sh, float cut_var, float tone_var, float damp, bool choke, float wet) {
    AudioNoInterrupts();
    for (int b = 0; b < BANDS; b++) coef[b] = f[b];
    shape = sh;
    if (cut_var <= 0.0f) hit_cut = 0.0f;
    if (tone_var <= 0.0f) hit_tone = 0.0f;
    cut_var_ = cut_var;
    tone_var_ = tone_var;
    damp_ = damp;
    choke_ = choke;
    wet_ = wet;
    reshape = true;
    AudioInterrupts();
  }
  // where the last hit put the peak (bands) and the tilt, for the picture
  float HitCut() const { return hit_cut; }
  float HitTone() const { return hit_tone; }

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
  Shape shape = {35.0f, -1.0f, 20.0f, 0.0f, 1.0f};
  float cut_var_ = 0.0f, tone_var_ = 0.0f;
  volatile float hit_cut = 0.0f, hit_tone = 0.0f;  // this hit's moves
  bool reshape = true;
  // the hit detector: the block's peak against a slower level
  float slow = 0.0f;
  int holdoff = 0;
  uint32_t rng = 0x9E3779B9u;
  float Rand() {  // -1..1
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return int32_t(rng) * (1.0f / 2147483648.0f);
  }
  void Gains() {
    const Shape& s = shape;
    const float cut = s.cut + hit_cut, tone = s.tone + hit_tone;
    for (int k = 0; k < BANDS; k++)
      target_gain[k] = Base(k, cut, s.slope) * (1.0f + s.reso * 0.06f * Bump(k, cut))
          * powf(4.0f, tone * 0.02f * (k - BANDS / 2) / float(BANDS / 2)) * s.level;  // +-12 dB at the ends
  }
  Viz viz = {};
  // per-block scratch, kept here rather than on the small stack
  float x[AUDIO_BLOCK_SAMPLES], dmp[AUDIO_BLOCK_SAMPLES], sum[2][AUDIO_BLOCK_SAMPLES];
};

// Runs from RAM1 (ITCM) like the other Loom engines.
FASTRUN void AudioEffectResonator::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }

  constexpr float q = 1.0f / 32768.0f;
  const float wet = wet_;

  // the mono input and the damping for each sample of the block
  float pk_in = 0.0f;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const float dl = in[0] ? in[0]->data[n] * q : 0.0f;
    const float dr = in[1] ? in[1]->data[n] * q : 0.0f;
    x[n] = 0.5f * (dl + dr);
    pk_in = std::max(pk_in, fabsf(x[n]));
    float damp = damp_;
    if (choke_) {
      // follow the input; when it is quiet the bands get damped hard
      const float a = fabsf(x[n]);
      env = a > env ? a : env * 0.9995f;
      damp += (1.0f - std::min(1.0f, env * 20.0f)) * 0.5f;
    }
    dmp[n] = damp;
  }

  // a hit: the level jumps well over where it has been (and at most one
  // every ~50 ms); with VARIATION it gets its own peak and tilt, at once
  bool hit = false;
  if (holdoff > 0) holdoff--;
  if (pk_in > 0.01f && pk_in > 2.0f * slow && holdoff == 0) { hit = true; holdoff = 18; }
  slow += (pk_in > slow ? 0.3f : 0.03f) * (pk_in - slow);
  if (hit && (cut_var_ > 0.0f || tone_var_ > 0.0f)) {
    hit_cut = cut_var_ > 0.0f ? roundf(Rand() * cut_var_) : 0.0f;  // whole bands: notes of the scale
    hit_tone = Rand() * tone_var_;
    reshape = true;
  }
  if (reshape) {
    reshape = false;
    Gains();
    if (hit) for (int b = 0; b < BANDS; b++) gain[b] = target_gain[b];
  }
  // gains glide a step per block so CUTOFF / SLOPE moves don't click
  for (int b = 0; b < BANDS; b++) gain[b] += 0.3f * (target_gain[b] - gain[b]);

  // bands far below the loudest one are not heard and skipped; the rest
  // run four at a time over the whole block (left and right side apart),
  // so their state stays in registers and the four can overlap
  // (a band still ringing runs on whatever its gain is now)
  float gmax = 0.0f;
  for (int b = 0; b < BANDS; b++) gmax = std::max(gmax, gain[b]);
  memset(sum, 0, sizeof(sum));
  for (int side = 0; side < 2; side++) {
    int act[BANDS / 2 + 3], na = 0;
    for (int b = side; b < BANDS; b += 2) {
      const bool quiet = gain[b] <= gmax * 1e-3f || gain[b] < 1e-4f;
      const bool ringing = fabsf(bp[b]) + fabsf(lp[b]) > 1e-5f;
      if (coef[b] <= 0.0f || (quiet && !ringing)) lp[b] = bp[b] = 0.0f;
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
          v[j] += f[j] * (g[j] * xn - l[j] - d * v[j]);  // the gain on the way in
          y += v[j];
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
        const float pk = sqrtf(2.0f * e[j] / AUDIO_BLOCK_SAMPLES) * damp_;
        viz.band[b] = std::max(viz.band[b], pk);
      }
    }
  }
  // damp * bp: unity gain at a band's centre; then a soft limit (unity
  // when quiet), as a note right on a ringing band can get loud
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    for (int c = 0; c < 2; c++) {
      const float v = sum[c][n] * dmp[n];
      sum[c][n] = v / sqrtf(1.0f + v * v);
    }
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
