#pragma once

#include <Audio.h>
#include "AudioSample.h"
#include <algorithm>
#include <cmath>

// Stereo dirt stage for the Loom TEXTURE device.
//
// Signal chain: tilt -> noise -> fold -> drive -> crush -> wet/dry.
//   TILT     two bands split at 80 Hz / 650 Hz / 2.5 kHz; + lifts the highs
//            and drops the lows, - the other way round; the level is
//            brought back so it sounds about as loud
//   NOISE    white, pink or dust (sparse clicks, like a record), its level
//            follows the input above a threshold that NOISE sets too: a
//            little NOISE only answers the loudest hits, more answers
//            quieter playing as well, and louder (N.DCY = how long it
//            trails), band-passed (N.TON)
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
    auto energy = [&](float wl) { return sqrtf(wl * tilt_lo * tilt_lo + (1.0f - wl) * tilt_hi * tilt_hi); };
    tilt_gain = 1.0f / energy(below[xover % 3]);
    // lifting the highs: measured against the 650 Hz crossover, which
    // sounds right uncorrected (80 Hz lifts nearly everything, 2k5 drops
    // most of it)
    if (t > 0.0f) tilt_gain = energy(below[1]) / energy(below[xover % 3]);
  }
  void noise(float amt, uint8_t mode) { noise_amt = constrain(amt, 0.0f, 1.0f); noise_mode = mode; }
  void noise_decay(float secs) { noise_rel = expf(-1.0f / (std::max(secs, 0.001f) * AUDIO_SAMPLE_RATE_EXACT)); }
  void noise_tone(float t) { // 0..1 -> 100 Hz .. 10 kHz band-pass centre
    const float fc = 100.0f * powf(100.0f, constrain(t, 0.0f, 1.0f));
    svf_f = 2.0f * sinf(float(M_PI) * fc / AUDIO_SAMPLE_RATE_EXACT);
  }
  void wet(float w) { wet_ = constrain(w, 0.0f, 1.0f); }

  // HEDDLE (after the rest): a frequency shifter, after Bode. Every
  // frequency moves by the same Hz, up or down (a pitch shifter multiplies
  // instead), so harmonics turn inharmonic: bells, metal, and at a few Hz
  // the endless barber-pole sweep. The sound split into two 90 degrees
  // apart (two all-pass chains), times a sine / cosine pair: their sum is
  // one side, their difference the other.
  enum { SIDE_UP, SIDE_DOWN, SIDE_BOTH };  // BOTH: up on the left, down on the right
  void shift(float hz, int side, float fdbk, float mix) {
    shift_hz = hz;
    shift_side = uint8_t(side);
    shift_fb = constrain(fdbk, 0.0f, 0.95f);
    shift_mix = constrain(mix, 0.0f, 1.0f);
  }

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

  volatile float shift_hz = 0.0f, shift_fb = 0.0f, shift_mix = 0.0f;
  volatile uint8_t shift_side = SIDE_UP;
  // HEDDLE's state: the oscillator (cos, sin), the all-passes (per side,
  // 4 + 4 sections of x[n-1], x[n-2], y[n-1], y[n-2]), the one-sample delay
  // of the first chain, the last output (feedback)
  float osc_c = 1.0f, osc_s = 0.0f;
  float hb[2][8][4] = {};
  float hb_d[2] = {}, hb_fb[2] = {};
  float mix_now = 0.0f;
  // Olli Niemitalo's coefficients (squared): 90 degrees apart within ~0.1
  // degree from ~20 Hz to ~20 kHz at 48 kHz
  static constexpr float HB_A[8] = {
    0.6923878f * 0.6923878f, 0.9360654322959f * 0.9360654322959f, 0.9882295226860f * 0.9882295226860f, 0.9987488452737f * 0.9987488452737f,
    0.4021921162426f * 0.4021921162426f, 0.8561710882420f * 0.8561710882420f, 0.9722909545651f * 0.9722909545651f, 0.9952884791278f * 0.9952884791278f};

  // state
  uint32_t rng = 0x12345678;
  float tilt_z[2] = {}, env_in = 0.0f;
  float det = 0.0f;  // the input's level (peaks, ~30 ms fall) for the noise threshold
  float pink[2][3] = {}, svf_lo[2] = {}, svf_bp[2] = {};
  float dust_z[2] = {};  // DUST: the click ringing down, per side
  static constexpr float NOISE_ATTACK = 0.0104f;  // ~2 ms: the noise comes in softly
  static constexpr float DET_FALL = 0.99931f;     // ~30 ms
  static constexpr float GATE_KNEE = 1.8f;        // fully open ~9 dB over the threshold
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
FASTRUN __attribute__((optimize("O2"))) void AudioEffectDirt::update() {  // (O2: the build is -Os)
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
  // the noise threshold: -12 dB at the bottom of NOISE, down to -50 dB at
  // the top (as 1 / threshold)
  const float inv_thr = 1.0f / (0.25f * expf(-38.0f * 0.11512925f * namt));

  float peak_in = 0.0f, peak_out = 0.0f;
  constexpr float q = 1.0f / 32768.0f;

  // HEDDLE: the oscillator turns by this a sample (a rotation); its MIX
  // glides; nothing to do while it is off
  const float sh_w = 2.0f * float(M_PI) * shift_hz / AUDIO_SAMPLE_RATE_EXACT;
  const float rc = cosf(sh_w), rs = sinf(sh_w);
  const float sh_fb = shift_fb, sh_mix_to = shift_mix;
  const uint8_t sh_side = shift_side;
  const bool shifting = sh_mix_to > 0.0f || mix_now > 1e-4f;

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    float x[2] = {in[0] ? in[0]->data[n] * q : 0.0f, in[1] ? in[1]->data[n] * q : 0.0f};
    const float dry[2] = {x[0], x[1]};
    const float lvl = std::max(fabsf(x[0]), fabsf(x[1]));
    peak_in = std::max(peak_in, lvl);

    // noise level follows the input over the threshold, faded in across
    // the knee (no chatter at the edge): ~2 ms rise, N.DCY fall
    det = lvl > det ? lvl : det * DET_FALL;
    const float open = std::min(1.0f, std::max(0.0f, (det * inv_thr - 1.0f) * (1.0f / (GATE_KNEE - 1.0f))));
    const float want = det * open;
    env_in = want > env_in ? env_in + NOISE_ATTACK * (want - env_in) : env_in * nrel;

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

    float o2[2];
    for (int c = 0; c < 2; c++) o2[c] = dry[c] + w * (x[c] - dry[c]);
    if (shifting) {
      mix_now += 0.002f * (sh_mix_to - mix_now);
      const float oc = osc_c * rc - osc_s * rs;  // the oscillator a step on
      osc_s = osc_c * rs + osc_s * rc;
      osc_c = oc;
      for (int c = 0; c < 2; c++) {
        float u = o2[c] + sh_fb * hb_fb[c];
        u = u / (1.0f + fabsf(u) * 0.25f);  // (the feedback can't run away)
        float re = u, im = u;
        for (int k = 0; k < 4; k++) {  // y = a (x + y[n-2]) - x[n-2]
          float* st = hb[c][k];
          const float y = HB_A[k] * (re + st[3]) - st[1];
          st[1] = st[0]; st[0] = re; st[3] = st[2]; st[2] = y; re = y;
        }
        for (int k = 4; k < 8; k++) {
          float* st = hb[c][k];
          const float y = HB_A[k] * (im + st[3]) - st[1];
          st[1] = st[0]; st[0] = im; st[3] = st[2]; st[2] = y; im = y;
        }
        const float red = hb_d[c];  // the first chain a sample later
        hb_d[c] = re;
        const bool up = sh_side == SIDE_UP || (sh_side == SIDE_BOTH && c == 0);
        const float sh = up ? red * osc_c - im * osc_s : red * osc_c + im * osc_s;
        hb_fb[c] = sh;
        o2[c] += mix_now * (sh - o2[c]);
      }
    }
    for (int c = 0; c < 2; c++) {
      peak_out = std::max(peak_out, fabsf(o2[c]));
      out[c]->data[n] = ToSample16(o2[c]);
    }
  }
  if (shifting) {  // keep the oscillator on the circle
    const float m = 1.0f / sqrtf(osc_c * osc_c + osc_s * osc_s);
    osc_c *= m;
    osc_s *= m;
  }

  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);

  viz.in = std::max(viz.in, peak_in);
  viz.out = std::max(viz.out, peak_out);
  viz.noise = std::max(viz.noise, env_in * namt);
}
