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
//   NOISE    white, pink or spec (sparse clicks, like a record), its level
//            follows the input above a threshold that NOISE sets too: a
//            little NOISE only answers the loudest hits, more answers
//            quieter playing as well, and louder (N.DCY = how long it
//            trails), band-passed (N.TON)
//   FOLD     wavefolder after the Buchla 259 (five folding cells), clean at 0
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
  // FOLD's target: the engine glides there (~40 ms), no steps as it turns
  void fold(float f) { fold_ = constrain(f, 0.0f, 1.0f); }
  // the folds take loudness away (or add it): what brings a sine back to its
  // own loudness (rms); the level follower makes any level look the same
  static float FoldGain(float fold) {
    if (fold <= 0.0f) return 1.0f;
    float out2 = 0.0f;
    for (int i = 0; i < 16; i++) {  // a quarter of a sine is enough (symmetric)
      const float y = Fold(sinf(0.5f * float(M_PI) * (i + 0.5f) / 16.0f), fold);
      out2 += y * y;
    }
    return out2 > 0.0f ? sqrtf(0.5f * 16.0f / out2) : 1.0f;
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
  // a burst of the noise (its colour, N.TON, N.DCY), whatever comes in or
  // NOICE is (a snare-like hit)
  void noise_burst() { burst_req = true; }
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
    phasing = false;
    shift_hz = hz;
    shift_side = uint8_t(side);
    shift_fb = constrain(fdbk, 0.0f, 0.95f);
    shift_mix = constrain(mix, 0.0f, 1.0f);
  }
  // ... or PHASE: an endless phaser (as the Buchla 297's barber pole). Three
  // phasers of `stages` all-passes glide through 8 octaves (40 Hz .. 10
  // kHz), a third of the way apart, each fading in at the bottom and out
  // at the top: the notches rise (or fall) for ever, without a jump.
  // rate: octaves a second (signed); reso: feedback; depth 0..1 (1: the
  // deepest notches)
  static constexpr int PH_MAX = 20, PH_STREAMS = 3;
  static inline volatile bool lite = false;  // the CPU short: PHASE on at most 8 stages
  void phase(float oct_per_s, int stages, float reso, float depth) {
    phasing = true;
    ph_rate = oct_per_s;
    ph_stages = uint8_t(constrain(stages, 2, PH_MAX));
    shift_fb = constrain(reso, 0.0f, 0.9f);
    shift_mix = constrain(depth, 0.0f, 1.0f);
  }
  float PhasePos() const { return ph_pos; }  // (the picture)
  // on the clock: nudged so its octaves turn over on the note values
  void PullPhase(float d) { float p = ph_pos + d; ph_pos = p - floorf(p); }

  // FOLD after the Buchla 259's timbre circuit (Esqueda, Pontynen,
  // Valimaki, Parker: "Virtual Analog Buchla 259 Wavefolder", DAFx-17):
  // five folding cells side by side with the straight path, each one
  // joining in above its own threshold (in volts), summed with the circuit's
  // weights, then the circuit's gentle low-pass. FOLD sets how hard the
  // signal drives it (as the 259's timbre knob: the input level).
  //   cell k: Gk * sgn(v) * (|v| - Tk) above Tk, else 0
  //   out    = 5 v + sum Wk * cell k        (here as Ck = Wk * Gk)
  static constexpr float B259_T[5] = {0.6f, 1.8f, 2.994f, 4.08f, 5.46f};
  static constexpr float B259_C[5] = {-12.0f * 0.8333f, 17.647f * 0.5743f, -27.777f * 0.3768f,
                                      36.363f * 0.2673f, -21.428f * 0.2829f};
  static constexpr float B259_PEAK = 3.0f;  // the folds' tops, in volts: full scale out
  static inline float B259(float v) {  // the circuit's curve, volts in and out
    const float a = fabsf(v);
    float y = 5.0f * a;
    for (int k = 0; k < 5; k++) if (a > B259_T[k]) y += B259_C[k] * (a - B259_T[k]);
    return v < 0.0f ? -y : y;
  }
  static inline float B259Int(float v) {  // its integral (for the anti-aliasing)
    const float a = fabsf(v);
    float y = 2.5f * a * a;
    for (int k = 0; k < 5; k++) if (a > B259_T[k]) y += 0.5f * B259_C[k] * (a - B259_T[k]) * (a - B259_T[k]);
    return y;
  }
  // The volts are the circuit's own, nothing in Loom's audio: they only
  // set where the cells start. As in the Music Easel (the same five cells),
  // the folder always sees its sine at the same strength and the timbre
  // knob is a level in front of it (a JFET): here the sound is taken
  // relative to its own peak (a level follower), so FOLD alone says how
  // many cells work, loud or quiet: 0 = none (0.6, the first just starts),
  // 50 = three, 100 = all five (6.5; past the last one the curve runs off)
  static inline float B259Volts(float fold) { return 0.6f + 5.9f * fold; }
  // the Easel's two diodes across the summing amp: the folds' tips round
  // off (a soft clip, tanh-like, full scale kept)
  static inline float Diodes(float u) {
    const float a = 1.3f * u;
    return a * (27.0f + a * a) / (27.0f + 9.0f * a * a) * 1.1317f;  // (1 stays 1)
  }
  // fold curve on a sound whose peak is 1 (the picture's sine; the engine
  // divides by the level first), without the anti-aliasing and the
  // low-pass; crossfaded in so 0 is clean
  static inline float Fold(float x, float fold) {
    if (fold <= 0.0f) return x;
    const float f = Diodes(B259(x * B259Volts(fold)) * (1.0f / B259_PEAK));
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
  // bit depth as levels per side of zero: 10 bits at the start down to 2,
  // without steps between. The steps are cut from the sound's own level
  // (a peak follower), not from full scale: a quiet sound gets as many
  // bits as a loud one, so it keeps its grit instead of breaking up into
  // a few clicks and silence (it did: rounded from full scale, a sound at
  // -10 dB was down to one step or none from about 70 % up)
  static inline float CrushLevels(float c) { return exp2f(9.0f - 8.0f * c); }  // 512 .. 2
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
  volatile float fold_ = 0.0f;
  float fold_now = 0.0f, fold_gain = 1.0f;  // FOLD as it glides, its make-up
  volatile bool tilt_on = false;
  volatile float tilt_lo = 1.0f, tilt_hi = 1.0f, tilt_a = 0.1f, tilt_gain = 1.0f;
  volatile float drive_gain = 1.0f;
  volatile float noise_amt = 0.0f, noise_rel = 0.999f, svf_f = 0.1f;
  volatile bool burst_req = false;
  float burst = 0.0f;  // its level now
  bool burst_up = false;

  volatile float shift_hz = 0.0f, shift_fb = 0.0f, shift_mix = 0.0f;
  volatile uint8_t shift_side = SIDE_UP;
  volatile bool phasing = false;
  // SHIFT <-> PHASE: the effect fades out (~5 ms), the engine switches, it
  // fades back in (switched at once it clicked)
  bool ph_now = false;     // what runs
  float sw_g = 1.0f;       // the effect's level for the switch
  static constexpr float SW_STEP = 0.5f;  // a block's share of the fade
  volatile float ph_rate = 0.0f;
  volatile uint8_t ph_stages = 12;
  float ph_pos = 0.0f;                       // 0..1 through the 8 octaves (the first phaser)
  float ph_z[2][PH_STREAMS][PH_MAX] = {};    // the all-passes' state
  float ph_y[2][PH_STREAMS] = {};            // each phaser's last output (RESO)
  // a new stage count fades in over PH_FADE blocks (~17 ms) while the old
  // chain, on a copy of its state, fades out: switched at once it clicked
  static constexpr int PH_FADE = 6;
  uint8_t ph_n_was = 0, ph_n_old = 0, ph_fade = 0;
  float ph_z_old[2][PH_STREAMS][PH_MAX] = {};
  float ph_y_old[2][PH_STREAMS] = {};
  // HEDDLE's state: the oscillator (cos, sin), the all-passes (per side,
  // 4 + 4 sections of x[n-1], x[n-2], y[n-1], y[n-2]), the one-sample delay
  // of the first chain, the last output (feedback)
  float osc_c = 1.0f, osc_s = 0.0f;
  float hb[2][8][4] = {};
  float hb_d[2] = {}, hb_fb[2] = {};
  float side_g[2] = {1.0f, 1.0f};  // +1 up, -1 down: glides when SIDE changes (no click)
  float mix_now = 0.0f;
  // HEDDLE's level kept as it came in (Sascha: SHIFT with FDBK got ~2 dB
  // louder, peaks 3.5 dB; PHASE ~2 dB quieter): the energy in and out,
  // smoothed, and a gain gliding (~0.1 s) to match them, +-10 dB at most;
  // each side its own (SIDE BOTH: one side came out ~1.5 dB louder)
  float hl_in[2] = {}, hl_out[2] = {}, hg[2] = {1.0f, 1.0f};
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
  float fold_env = 0.0f;  // FOLD: the sound's level (both sides)
  float fold_v[2] = {}, fold_F[2] = {}, fold_x[2] = {}, fold_lp[2] = {};  // FOLD: last sample (volts, integral, dry), low-pass
  static constexpr float FOLD_LP = 1.0f - 0.65227f;  // one pole at ~3 kHz (exp(-2 pi 3000 / 44117))
  float crush_env = 0.0f;  // CRUSH: the sound's level (both sides), ~150 ms fall
  float dust_z[2] = {};  // SPEC: the click ringing down, per side
  static constexpr float NOISE_ATTACK = 0.0104f;  // ~2 ms: the noise comes in softly
  static constexpr float DET_FALL = 0.99931f;     // ~30 ms
  // a soft gate: starts opening 6 dB under the threshold, fully open 6 dB
  // over it, on an S curve (was a straight 5 dB ramp from the threshold up)
  static constexpr float GATE_LO = 0.5f, GATE_HI = 2.0f;
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

// one phaser of N first-order all-passes over a block, added into acc by
// w; its state held in registers (N known at build time)
template <int N>
FASTRUN __attribute__((optimize("O2"))) static void PhaseChain(const float* in, float* acc, float a, float w,
                                                              float fbk, float* zs, float& fb) {
  float z[N];
  for (int j = 0; j < N; j++) z[j] = zs[j];
  float f = fb;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    // (RESO stays under 0.9 with an all-pass gain of 1: it can't run away;
    // the clamp only guards against a stray value)
    float v = constrain(in[n] + fbk * f, -4.0f, 4.0f);
#pragma GCC unroll 20
    for (int j = 0; j < N; j++) {  // y = a x + z, z = x - a y
      const float y = a * v + z[j];
      z[j] = v - a * y;
      v = y;
    }
    f = v;
    acc[n] += w * v;
  }
  for (int j = 0; j < N; j++) zs[j] = z[j];
  fb = f;
}

// a chain of n stages (even, 4 .. 20), each count built on its own
FASTRUN static void RunChain(int n, const float* in, float* acc, float a, float w,
                             float fbk, float* zs, float& fb) {
  switch (n) {
    case 4:  PhaseChain<4>(in, acc, a, w, fbk, zs, fb); break;
    case 6:  PhaseChain<6>(in, acc, a, w, fbk, zs, fb); break;
    case 8:  PhaseChain<8>(in, acc, a, w, fbk, zs, fb); break;
    case 10: PhaseChain<10>(in, acc, a, w, fbk, zs, fb); break;
    case 12: PhaseChain<12>(in, acc, a, w, fbk, zs, fb); break;
    case 14: PhaseChain<14>(in, acc, a, w, fbk, zs, fb); break;
    case 16: PhaseChain<16>(in, acc, a, w, fbk, zs, fb); break;
    case 18: PhaseChain<18>(in, acc, a, w, fbk, zs, fb); break;
    default: PhaseChain<20>(in, acc, a, w, fbk, zs, fb); break;
  }
}

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
  if (burst_req) { burst_req = false; burst_up = true; }
  const bool do_bits = crs > 0.0f && cmode != CRUSH_REDUX;
  const bool do_redux = crs > 0.0f && cmode != CRUSH_BIT;
  const float levels = CrushLevels(crs), inv_levels = 1.0f / levels;
  const float dg = drive_gain, tg = tilt_gain;
  const int hold = ReduxHold(crs);
  const float tl = tilt_lo, th = tilt_hi, ta = tilt_a;
  const bool tilt = tilt_on;
  const float nrel = noise_rel, sf = svf_f;
  // FOLD glides from where it was to (part way to) its target over the
  // block: drive, share and make-up move with it sample by sample
  const float fld0 = fold_now, g0 = fold_gain;
  {
    const float to = fold_;
    fold_now += 0.07f * (to - fold_now);
    if (fabsf(to - fold_now) < 1e-4f) fold_now = to;
    if (fold_now != fld0) fold_gain = FoldGain(fold_now);
  }
  const float fld = std::max(fld0, fold_now);  // (on while either end folds)
  const float fA0 = B259Volts(fld0), fm0 = std::min(1.0f, fld0 * 4.0f);
  const float dfA = (B259Volts(fold_now) - fA0) * (1.0f / AUDIO_BLOCK_SAMPLES);
  const float dfm = (std::min(1.0f, fold_now * 4.0f) - fm0) * (1.0f / AUDIO_BLOCK_SAMPLES);
  const float dg_f = (fold_gain - g0) * (1.0f / AUDIO_BLOCK_SAMPLES);
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
  // PHASE: this block's all-pass coefficient and level per phaser
  if (phasing != ph_now && sw_g <= 0.0f) {  // faded out: switch, from silence
    ph_now = phasing;
    hb_fb[0] = hb_fb[1] = 0.0f;
    memset(ph_y, 0, sizeof(ph_y));
  }
  const float sw_from = sw_g;
  sw_g = phasing == ph_now ? std::min(1.0f, sw_g + SW_STEP) : std::max(0.0f, sw_g - SW_STEP);
  const float sw_d = (sw_g - sw_from) / AUDIO_BLOCK_SAMPLES;
  const bool ph = ph_now;
  float ph_a[PH_STREAMS] = {}, ph_w[PH_STREAMS] = {};
  const int ph_n = lite ? std::min<int>(ph_stages, 8) : ph_stages;
  if (ph && shifting) {
    constexpr float PH_OCTS = 8.0f, F_LO = 40.0f;
    ph_pos += ph_rate / PH_OCTS * (AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_EXACT);
    ph_pos -= floorf(ph_pos);
    float wsum = 0.0f;
    for (int k = 0; k < PH_STREAMS; k++) {
      float u = ph_pos + float(k) / PH_STREAMS;
      u -= floorf(u);
      const float f = F_LO * exp2f(PH_OCTS * u);
      const float t = tanf(float(M_PI) * std::min(f, 0.45f * AUDIO_SAMPLE_RATE_EXACT) / AUDIO_SAMPLE_RATE_EXACT);
      ph_a[k] = (t - 1.0f) / (t + 1.0f);
      const float sw = sinf(float(M_PI) * u);  // in at the bottom, out at the top
      ph_w[k] = sw * sw;
      wsum += ph_w[k];
    }
    for (auto& v : ph_w) v /= wsum;
  }

  float pre[2][AUDIO_BLOCK_SAMPLES];  // PHASE: the sound before the phasers
  float e_in[2] = {}, e_out[2] = {};   // HEDDLE's energy in / out this block, per side (the level match)
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    float x[2] = {in[0] ? in[0]->data[n] * q : 0.0f, in[1] ? in[1]->data[n] * q : 0.0f};
    const float dry[2] = {x[0], x[1]};
    const float lvl = std::max(fabsf(x[0]), fabsf(x[1]));
    peak_in = std::max(peak_in, lvl);

    // noise level follows the input over the threshold, faded in across
    // the knee (no chatter at the edge): ~2 ms rise, N.DCY fall
    det = lvl > det ? lvl : det * DET_FALL;
    const float u = std::min(1.0f, std::max(0.0f, (det * inv_thr - GATE_LO) * (1.0f / (GATE_HI - GATE_LO))));
    const float open = u * u * (3.0f - 2.0f * u);
    const float want = det * open;
    env_in = want > env_in ? env_in + NOISE_ATTACK * (want - env_in) : env_in * nrel;
    // a burst: in over ~2 ms, out over N.DCY
    if (burst_up) { burst += NOISE_ATTACK * 4.0f * (1.0f - burst); if (burst > 0.95f) burst_up = false; }
    else burst *= nrel;
    const float nlev = env_in * namt * 2.0f + burst * 0.7f;

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
      if (nlev > 1e-5f) {
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
        v += svf_bp[c] * nlev;
      }
      if (fld > 0.0f) {
        // first-order anti-aliasing (the curve's integral across the step
        // from the last sample: its corners don't fold back as whistles),
        // the straight path taken at the same half sample, then the
        // circuit's low-pass (one pole, 1.33 kHz in the 259, here 3 kHz)
        // the sound against its own peak (both sides, at once up, ~0.6 s
        // down: a sound dying away gets fewer folds as it fades, as a fading
        // sine does in the circuit, instead of its tail pulled back up into
        // the folds; under -30 dB it is driven less, so hiss doesn't fold)
        const float fA = fA0 + dfA * n, fm = fm0 + dfm * n, fold_gain_now = g0 + dg_f * n;
        fold_env = std::max(fabsf(v), fold_env * 0.999962f);
        const float lvl = std::max(fold_env, 0.03f);
        const float vv = v * (fA / lvl), dv = vv - fold_v[c];
        const float Fi = B259Int(vv);
        const float f = Diodes((fabsf(dv) > 1e-5f ? (Fi - fold_F[c]) / dv : B259(0.5f * (vv + fold_v[c])))
                               * (1.0f / B259_PEAK)) * lvl;
        const float xm = 0.5f * (v + fold_x[c]);
        fold_v[c] = vv; fold_F[c] = Fi; fold_x[c] = v;
        fold_lp[c] += FOLD_LP * (f - fold_lp[c]);
        v = (xm + fm * (fold_lp[c] - xm)) * fold_gain_now;
      } else {
        fold_v[c] = 0.0f; fold_F[c] = 0.0f; fold_x[c] = v; fold_lp[c] = 0.0f;
      }
      v = Shape(v, drv) * dg;
      if (do_bits) {
        // mid-rise steps (no step at zero): quiet tails buzz on as the
        // smallest steps instead of being gated to silence; under -48 dB
        // it is left alone (no buzz out of the noise floor)
        crush_env = std::max(fabsf(v), crush_env * 0.99985f);
        if (crush_env > 0.004f) {
          const float step = crush_env * inv_levels;
          v = (floorf(constrain(v, -1.0f, 1.0f) / step) + 0.5f) * step;
        }
      }
      if (do_redux) {
        if (take) hold_val[c] = v;
        v = hold_val[c];
      }
      x[c] = v;
    }

    float o2[2];
    for (int c = 0; c < 2; c++) o2[c] = dry[c] + w * (x[c] - dry[c]);
    if (shifting && ph) {  // (the phasers run over the whole block below)
      pre[0][n] = o2[0];
      pre[1][n] = o2[1];
      continue;
    } else if (shifting) {
      e_in[0] += o2[0] * o2[0]; e_in[1] += o2[1] * o2[1];
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
        side_g[c] += 0.003f * ((up ? 1.0f : -1.0f) - side_g[c]);
        const float sh = red * osc_c + side_g[c] * im * osc_s;  // (measured: + is up)
        hb_fb[c] = sh;
        o2[c] += mix_now * (sw_from + sw_d * (n + 1)) * (sh - o2[c]);
      }
      e_out[0] += o2[0] * o2[0]; e_out[1] += o2[1] * o2[1];
      o2[0] *= hg[0]; o2[1] *= hg[1];
    }
    for (int c = 0; c < 2; c++) {
      peak_out = std::max(peak_out, fabsf(o2[c]));
      out[c]->data[n] = ToSample16(o2[c]);
    }
  }
  if (shifting && ph) {
    // PHASE: a phaser at a time over the block; its stage count fixed at
    // build time (even, 4 .. 20) so the whole chain stays in registers.
    // A new count: the old chain goes on from a copy of its state and fades
    // out; the new one's added stages start empty
    if (ph_n_was && ph_n != ph_n_was) {
      memcpy(ph_z_old, ph_z, sizeof(ph_z));
      memcpy(ph_y_old, ph_y, sizeof(ph_y));
      for (int c = 0; c < 2; c++)
        for (int k = 0; k < PH_STREAMS; k++)
          for (int j = ph_n_was; j < ph_n; j++) ph_z[c][k][j] = 0.0f;
      ph_n_old = ph_n_was;
      ph_fade = PH_FADE;
    }
    ph_n_was = uint8_t(ph_n);
    const float fd_from = ph_fade ? float(PH_FADE - ph_fade) / PH_FADE : 1.0f;
    const float fd_step = ph_fade ? 1.0f / (PH_FADE * AUDIO_BLOCK_SAMPLES) : 0.0f;
    for (int c = 0; c < 2; c++) {
      float yw[AUDIO_BLOCK_SAMPLES] = {}, yo[AUDIO_BLOCK_SAMPLES];
      for (int k = 0; k < PH_STREAMS; k++)
        RunChain(ph_n, pre[c], yw, ph_a[k], ph_w[k], sh_fb, ph_z[c][k], ph_y[c][k]);
      if (ph_fade) {  // the old count, fading out
        memset(yo, 0, sizeof(yo));
        for (int k = 0; k < PH_STREAMS; k++)
          RunChain(ph_n_old, pre[c], yo, ph_a[k], ph_w[k], sh_fb, ph_z_old[c][k], ph_y_old[c][k]);
        for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
          const float g = fd_from + fd_step * (n + 1);
          yw[n] = yo[n] + g * (yw[n] - yo[n]);
        }
      }
      float mn = mix_now;
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        mn += 0.002f * (sh_mix_to - mn);
        // the notches: the sound and its phased copy half and half at the deepest
        const float o0 = pre[c][n] + mn * 0.5f * (sw_from + sw_d * (n + 1)) * (yw[n] - pre[c][n]);
        e_in[c] += pre[c][n] * pre[c][n];
        e_out[c] += o0 * o0;
        const float o = o0 * hg[c];
        peak_out = std::max(peak_out, fabsf(o));
        out[c]->data[n] = ToSample16(o);
      }
      if (c == 1) mix_now = mn;
    }
    if (ph_fade) ph_fade--;
  }
  // the level match: smoothed energies (~30 ms), the gain gliding (~0.1 s)
  // to their ratio; held through silence; back to 1 with HEDDLE off
  for (int c = 0; c < 2; c++) {
    if (shifting) {
      if (e_in[c] > 0.5e-6f * AUDIO_BLOCK_SAMPLES) {
        hl_in[c] += 0.1f * (e_in[c] - hl_in[c]);
        hl_out[c] += 0.1f * (e_out[c] - hl_out[c]);
        if (hl_out[c] > 1e-9f) hg[c] += 0.03f * (constrain(sqrtf(hl_in[c] / hl_out[c]), 0.316f, 3.16f) - hg[c]);
      }
    } else {
      hg[c] += 0.05f * (1.0f - hg[c]);
      hl_in[c] = hl_out[c] = 0.0f;
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
  viz.noise = std::max(viz.noise, std::max(env_in * namt, burst * 0.35f));
}
