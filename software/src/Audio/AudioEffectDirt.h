#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>

// Stereo dirt stage for the Loom TEXTURE device.
//
// Signal chain: tilt -> noise -> fold -> drive -> crush -> wet/dry.
//   TILT     two bands split at 80 Hz / 650 Hz / 2.5 kHz; + lifts the highs
//            and drops the lows, - the other way round; the level is
//            brought back so it sounds about as loud
//   NOISE    white, pink or dust (sparse clicks, like a record), its level
//            follows the input
//            (N.DCY = how long it trails), band-passed (N.TON)
//   FOLD     sine wavefolder, West-coast style, clean at 0
//   DRIVE    soft saturation, clean at 0; made up so a usual level stays
//            about as loud
//   CRUSH    bit depth (10 bits down to 1.5, smoothly), sample-rate
//            reduction with jitter, or both
class AudioEffectDirt : public AudioStream {
public:
  enum CrushMode : uint8_t { CRUSH_BIT, CRUSH_REDUX, CRUSH_BOTH };
  enum NoiseMode : uint8_t { NOISE_WHITE, NOISE_PINK, NOISE_DUST };

  struct Viz {
    float in, out;
    float noise;     // noise envelope level
  };

  AudioEffectDirt() : AudioStream(2, input_queue_array) {}

  // --- parameters, set from the UI thread ---
  void drive(float d) {
    drive_ = constrain(d, 0.0f, 1.0f);
    // DRIVE lifts quiet signals (up to 13x) and keeps full scale where it
    // is: bring a usual level (DRIVE_REF) back to where it was
    drive_gain = DRIVE_REF / Shape(DRIVE_REF, drive_);
  }
  void crush(float c, uint8_t mode) { crush_ = constrain(c, 0.0f, 1.0f); crush_mode = mode; }
  void fold(float f) {
    fold_ = constrain(f, 0.0f, 1.0f);
    // FOLD lifts a quieter signal (up to 7x before it folds back): bring a
    // sine at the usual level (DRIVE_REF) back to its own loudness (rms)
    float in2 = 0.0f, out2 = 0.0f;
    for (int i = 0; i < 64; i++) {
      const float x = DRIVE_REF * sinf(2.0f * float(M_PI) * (i + 0.5f) / 64.0f);
      const float y = Fold(x, fold_);
      in2 += x * x;
      out2 += y * y;
    }
    fold_gain = out2 > 0.0f ? sqrtf(in2 / out2) : 1.0f;
  }
  void tilt(float t, uint8_t xover) {
    static const float hz[3] = {80.0f, 650.0f, 2500.0f};
    t = constrain(t, -1.0f, 1.0f);
    tilt_lo = powf(10.0f, -12.0f * t / 20.0f);
    tilt_hi = powf(10.0f, 12.0f * t / 20.0f);
    // with low gain 1/g and high gain g the response crosses 0 dB at
    // corner / g, so put the corner at xover * g: the tilt pivots there
    const float corner = std::min(hz[xover % 3] * tilt_hi, 18000.0f);
    tilt_a = 1.0f - expf(-2.0f * float(M_PI) * corner / AUDIO_SAMPLE_RATE_EXACT);
    tilt_on = t != 0.0f;
    // loudness: most of a sound's energy sits low, so weight the two gains
    // by roughly how much of it is below the crossover
    static const float below[3] = {0.3f, 0.75f, 0.92f};
    const float wl = below[xover % 3];
    tilt_gain = 1.0f / sqrtf(wl * tilt_lo * tilt_lo + (1.0f - wl) * tilt_hi * tilt_hi);
    // lifting the highs: no correction (it already sounds quieter, as the
    // lows go down)
    if (t > 0.0f) tilt_gain = 1.0f;
  }
  void noise(float amt, uint8_t mode) { noise_amt = constrain(amt, 0.0f, 1.0f); noise_mode = mode; }
  void noise_decay(float secs) { noise_rel = expf(-1.0f / (std::max(secs, 0.001f) * AUDIO_SAMPLE_RATE_EXACT)); }
  void noise_tone(float t) { // 0..1 -> 100 Hz .. 10 kHz band-pass centre
    const float fc = 100.0f * powf(100.0f, constrain(t, 0.0f, 1.0f));
    svf_f = 2.0f * sinf(float(M_PI) * fc / AUDIO_SAMPLE_RATE_EXACT);
  }
  void wet(float w) { wet_ = constrain(w, 0.0f, 1.0f); }

  // fold curve, shared with the picture: sin() of the boosted signal folds
  // it back from the edges; crossfaded in so 0 is clean
  static inline float Fold(float x, float fold) {
    if (fold <= 0.0f) return x;
    const float k = 1.0f + 6.0f * fold;
    const float f = sinf(0.5f * float(M_PI) * constrain(x * k, -8.0f, 8.0f));
    return x + std::min(1.0f, fold * 4.0f) * (f - x);
  }

  // drive curve, shared with the picture
  static inline float Shape(float x, float drive) {
    if (drive <= 0.0f) return x;
    const float d = 1.0f + 12.0f * drive * drive;
    const float s = d * x / (1.0f + fabsf(d * x));      // rational soft clip
    const float norm = (1.0f + d) / d;                  // full scale stays full scale
    return x + std::min(1.0f, drive * 4.0f) * (s * norm - x);
  }
  // bit depth as levels per side of zero: 10 bits at the start (just
  // audible on quiet sounds) down to 1.5 bits, without steps between
  static inline float CrushLevels(float c) { return exp2f(9.0f - 8.5f * c); }  // 512 .. 1.4
  static constexpr float DRIVE_REF = 0.3f;
  static inline int ReduxHold(float c) { return 1 + int(c * c * 31.0f); }     // 1 .. 32

  void ReadViz(Viz& v) {
    AudioNoInterrupts();
    v = viz;
    viz = Viz{};
    AudioInterrupts();
  }

  void update() override;

private:
  audio_block_t* input_queue_array[2];

  volatile float drive_ = 0.0f, crush_ = 0.0f, wet_ = 1.0f;
  volatile uint8_t crush_mode = CRUSH_BIT, noise_mode = NOISE_PINK;
  volatile float fold_ = 0.0f, fold_gain = 1.0f;
  volatile bool tilt_on = false;
  volatile float tilt_lo = 1.0f, tilt_hi = 1.0f, tilt_a = 0.1f, tilt_gain = 1.0f;
  volatile float drive_gain = 1.0f;
  volatile float noise_amt = 0.0f, noise_rel = 0.999f, svf_f = 0.1f;

  // state
  uint32_t rng = 0x12345678;
  float tilt_z[2] = {}, env_in = 0.0f;
  float pink[2][3] = {}, svf_lo[2] = {}, svf_bp[2] = {};
  float dust_z[2] = {};  // DUST: the click ringing down, per side
  static constexpr float NOISE_ATTACK = 0.0104f;  // ~2 ms: the noise comes in softly
  static constexpr float DUST_THRESH = 1.0f - 2.0f * 30.0f / 48000.0f;  // Rand() above: ~30 clicks / s
  static constexpr float DUST_GAIN = 6.0f;
  int hold_count = 0;
  float hold_val[2] = {};
  Viz viz = {};

  inline float Rand() { // -1..1
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return int32_t(rng) * (1.0f / 2147483648.0f);
  }
};

// Runs from RAM1 (ITCM) like the other Loom engines.
FASTRUN void AudioEffectDirt::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }

  const float drv = drive_, crs = crush_, w = wet_, namt = noise_amt;
  const uint8_t cmode = crush_mode, nmode = noise_mode;
  const bool do_bits = crs > 0.0f && cmode != CRUSH_REDUX;
  const bool do_redux = crs > 0.0f && cmode != CRUSH_BIT;
  const float levels = CrushLevels(crs), inv_levels = 1.0f / levels;
  const float dg = drive_gain, tg = tilt_gain;
  const int hold = ReduxHold(crs);
  const float tl = tilt_lo, th = tilt_hi, ta = tilt_a;
  const bool tilt = tilt_on;
  const float nrel = noise_rel, sf = svf_f, fld = fold_, fold_gain_now = fold_gain;

  float peak_in = 0.0f, peak_out = 0.0f;
  constexpr float q = 1.0f / 32768.0f;

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    float x[2] = {in[0] ? in[0]->data[n] * q : 0.0f, in[1] ? in[1]->data[n] * q : 0.0f};
    const float dry[2] = {x[0], x[1]};
    const float lvl = std::max(fabsf(x[0]), fabsf(x[1]));
    peak_in = std::max(peak_in, lvl);

    // noise level follows the input: instant rise, N.DCY fall
    env_in = lvl > env_in ? env_in + NOISE_ATTACK * (lvl - env_in) : env_in * nrel;

    // the redux sample-and-hold runs for both channels together
    bool take = true;
    if (do_redux) {
      if (--hold_count > 0) take = false;
      else hold_count = hold + int((Rand() + 1.0f) * 0.25f * hold); // jitter
    }

    for (int c = 0; c < 2; c++) {
      float v = x[c];
      if (tilt) {
        tilt_z[c] += ta * (v - tilt_z[c]);
        v = (tilt_z[c] * tl + (v - tilt_z[c]) * th) * tg;
      }
      if (namt > 0.0f) {
        float nz = Rand();
        if (nmode == NOISE_PINK) {
          float* b = pink[c];
          b[0] = 0.99765f * b[0] + nz * 0.0990460f;
          b[1] = 0.96300f * b[1] + nz * 0.2965164f;
          b[2] = 0.57000f * b[2] + nz * 1.0526913f;
          nz = (b[0] + b[1] + b[2] + nz * 0.1848f) * 0.25f;
        } else if (nmode == NOISE_DUST) {
          // about 30 clicks a second on each side, at random; most quiet,
          // now and then a loud one (cubed), either sign; each dies away
          // in ~0.3 ms
          if (Rand() > DUST_THRESH) {
            const float u = 0.5f * (Rand() + 1.0f);
            dust_z[c] += (Rand() > 0.0f ? 1.0f : -1.0f) * u * u * u * DUST_GAIN;
          }
          nz = dust_z[c];
          dust_z[c] *= 0.93f;
        }
        // N.TON: band-pass (state variable)
        const float hp = nz - svf_lo[c] - svf_bp[c];
        svf_bp[c] += sf * hp;
        svf_lo[c] += sf * svf_bp[c];
        v += svf_bp[c] * env_in * namt * 2.0f;
      }
      v = Fold(v, fld) * fold_gain_now;
      v = Shape(v, drv) * dg;
      if (do_bits) v = roundf(constrain(v, -1.0f, 1.0f) * levels) * inv_levels;
      if (do_redux) {
        if (take) hold_val[c] = v;
        v = hold_val[c];
      }
      x[c] = v;
    }

    for (int c = 0; c < 2; c++) {
      const float o = dry[c] + w * (x[c] - dry[c]);
      peak_out = std::max(peak_out, fabsf(o));
      out[c]->data[n] = __SSAT(int32_t(o * 32767.0f), 16);
    }
  }

  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);

  viz.in = std::max(viz.in, peak_in);
  viz.out = std::max(viz.out, peak_out);
  viz.noise = std::max(viz.noise, env_in * namt);
}
