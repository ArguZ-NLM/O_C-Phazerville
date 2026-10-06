#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

// Delay + reverb core for the Loom ECHO device. Stereo in, stereo out.
//
// The delay lines live in extmem and are read a whole block at a time (taps
// are at least one block long), which suits PSRAM far better than reading
// sample by sample. Changing a tap time crossfades from the old read position
// to the new one. The write signal is input plus feedback from the last tap,
// filtered (TONE), diffused / cross-fed (SPREAD) and soft clipped. DELAY
// crossfades the taps against the dry signal, then REVERB crossfades that
// against an 8-line feedback delay network fed by it: a low cut and four
// diffusers at its input, a Householder mix (every line into every other),
// a damping lowpass per line set by TONE, four lines slowly wobbled so the
// tail doesn't ring metallic, and line lengths that glide when SIZE moves.
class AudioEffectEcho : public AudioStream {
public:
  static constexpr int TAPS = 4;
  static constexpr int MIN_TAP = AUDIO_BLOCK_SAMPLES;

  // Peak levels (0..1), held until the UI reads them with ReadViz()
  struct Viz {
    float tap[2][TAPS];
    float tail;
    float in;
    float out;
    uint32_t blocks;
  };

  explicit AudioEffectEcho(size_t delay_samples)
    : AudioStream(2, input_queue_array), dlen(delay_samples) {}

  // allocate and clear everything; call while the stream is disconnected
  bool Acquire() {
    if (!mem) mem = static_cast<float*>(calloc(MEM_TOTAL, sizeof(float)));
    if (!rv) rv = TakeLines();
    for (auto& b : dbuf) {
      if (!b) b = static_cast<int16_t*>(extmem_malloc(dlen * sizeof(int16_t)));
    }
    Reset();
    return ready();
  }

  void Release() {
    free(mem);
    mem = nullptr;
    GiveLines(rv);
    rv = nullptr;
    for (auto& b : dbuf) {
      if (b) extmem_free(b);
      b = nullptr;
    }
  }

  bool ready() const { return mem && rv && dbuf[0] && dbuf[1]; }

  void Reset() {
    if (mem) std::fill(mem, mem + MEM_TOTAL, 0.0f);
    if (rv) memset(rv, 0, RV_TOTAL * sizeof(int16_t));
    for (auto& w : rv_wi) w = 0;
    for (auto& z : rv_lp) z = 0.0f;
    rv_hp_z[0] = rv_hp_z[1] = 0.0f;
    for (auto b : dbuf) if (b) memset(b, 0, dlen * sizeof(int16_t));
    for (auto& i : ix) i = 0;
    for (int c = 0; c < 2; c++) lp_z[c] = hp_z[c] = 0.0f;
  }

  // --- parameters, set from the UI thread ---
  // tap delay in samples, at least MIN_TAP
  void tap(int ch, int t, int samples) {
    tap_[ch][t].target = constrain(samples, MIN_TAP, int(dlen) - 1);
  }
  void feedback(float f) { fb_target = constrain(f, 0.0f, 1.05f); }
  void delay_mix(float m) { dmix_target = constrain(m, 0.0f, 1.0f); }
  void reverb_mix(float m) { rmix_target = constrain(m, 0.0f, 1.0f); }
  // SEND instead of MIX: the dry signal stays at full level and the echoes /
  // the reverb are added on top (MIX crossfades them against it)
  void delay_send(bool s) { dsend = s; }
  void reverb_send(bool s) { rsend = s; }
  void pingpong(float s) { spread_target = constrain(s, 0.0f, 1.0f); }
  void diffuse(float d) { ap_g = 0.7f * constrain(d, 0.0f, 1.0f); }
  void set_freeze(bool f) { freeze = f; }
  bool frozen() const { return freeze; }

  // -1 = dark (LP down to ~400 Hz), 0 = flat, +1 = thin (HP up to ~2.5 kHz)
  void tone(float t) {
    t = constrain(t, -1.0f, 1.0f);
    const float lp_hz = t < 0.0f ? 18000.0f * powf(400.0f / 18000.0f, -t) : 18000.0f;
    const float hp_hz = t > 0.0f ? 15.0f * powf(2500.0f / 15.0f, t) : 15.0f;
    lp_a = OnePoleCoeff(lp_hz);
    hp_a = OnePoleCoeff(hp_hz);
    // the reverb tail: dark 1.8 kHz .. 6 kHz (flat) .. bright 14 kHz
    const float rv_hz = t < 0.0f ? 6000.0f * powf(1800.0f / 6000.0f, -t) : 6000.0f * powf(14000.0f / 6000.0f, t);
    rv_damp_a = OnePoleCoeff(rv_hz);
  }

  // 0..1: the room, 0.35 .. 1x the line lengths (they glide there)
  void size(float s) { rv_scale_target = 0.35f + 0.65f * constrain(s, 0.0f, 1.0f); }

  // approximate RT60 of the reverb tail in seconds
  void decay(float secs) { decay_secs = constrain(secs, 0.2f, 30.0f); }

  void ReadViz(Viz& out) {
    AudioNoInterrupts();
    out = viz;
    viz = Viz{};
    viz.blocks = out.blocks;
    AudioInterrupts();
  }

  void update() override;

private:
  static constexpr float TAP_GAIN[TAPS] = {0.30f, 0.40f, 0.55f, 0.80f};
  static constexpr int XFADE = 4 * AUDIO_BLOCK_SAMPLES; // ~10 ms

  // allpasses (float, RAM2): 4 in the delay's feedback path (2 a side),
  // 8 diffusers at the reverb's input (4 a side)
  static constexpr int NUM_AP = 12;
  static constexpr int AP_FB_BASE = 0;
  static constexpr int AP_IN_BASE = 4;
  static constexpr int AP_LEN[NUM_AP] = {241, 467, 263, 503,
                                         142, 107, 379, 277, 151, 113, 397, 283};
  static constexpr int AP_OFS[NUM_AP] = {0, 241, 708, 971, 1474, 1616, 1723, 2102, 2379, 2530, 2643, 3040};
  static constexpr int MEM_TOTAL = 3323;
  static constexpr float AP_IN_G[4] = {0.75f, 0.70f, 0.65f, 0.60f};

  // the reverb's 8 lines (int16, RAM2): lengths with no common factors,
  // 35 .. 94 ms at full SIZE; each holds that plus room for the wobble
  static constexpr int RL = 8;
  static constexpr int RV_LEN[RL] = {1557, 1913, 2251, 2617, 3011, 3373, 3769, 4153};
  static constexpr int RV_MOD = 24;  // wobble depth room, samples
  static constexpr int RV_CAP[RL] = {1585, 1941, 2279, 2645, 3039, 3401, 3797, 4181};  // RV_LEN + RV_MOD + 4
  static constexpr int RV_OFS[RL] = {0, 1585, 3526, 5805, 8450, 11489, 14890, 18687};
  static constexpr int RV_TOTAL = 22868;
  static_assert(RV_TOTAL == 22868, "the RAM1 pool below has this size");
  static constexpr float RV_Q = 8192.0f;  // int16 per 1.0 (headroom x4)
  // the lines live in RAM1 (no cache misses: the reverb costs half): room
  // for two reverbs (one per Loom track), more from the heap
  static int16_t* TakeLines();
  static void GiveLines(int16_t* p);

  struct Tap {
    volatile int target = MIN_TAP;
    int cur = MIN_TAP;   // read position now
    int next = MIN_TAP;  // fading towards this one
    int phase = XFADE;   // XFADE = not fading
  };

  audio_block_t* input_queue_array[2];
  const size_t dlen;
  int16_t* dbuf[2] = {nullptr, nullptr};
  size_t wpos = 0;
  Tap tap_[2][TAPS];

  float* mem = nullptr;
  int ix[NUM_AP] = {};
  int16_t* rv = nullptr;
  int rv_wi[RL] = {};               // each line's write position
  float rv_scale = 1.0f;            // the lengths now (gliding to the target)
  volatile float rv_scale_target = 1.0f;
  volatile float rv_damp_a = 0.5f;
  float rv_lp[RL] = {};
  float rv_hp_z[2] = {};
  float lfo_ph[4] = {0.0f, 0.25f, 0.5f, 0.75f};

  volatile float fb_target = 0.0f, dmix_target = 0.4f, rmix_target = 0.3f;
  volatile float spread_target = 0.0f, ap_g = 0.0f;
  volatile float lp_a = 1.0f, hp_a = 0.0f;
  volatile bool freeze = false;
  volatile float decay_secs = 3.0f;

  float fb = 0.0f, dmix = 0.4f, rmix = 0.3f, spread_ = 0.0f, freeze_amt = 0.0f;
  volatile bool dsend = false, rsend = false;
  float lp_z[2] = {}, hp_z[2] = {};

  Viz viz = {};

  static float OnePoleCoeff(float hz) {
    return 1.0f - expf(-2.0f * float(M_PI) * hz / AUDIO_SAMPLE_RATE_EXACT);
  }

  static inline int16_t ToQ15(float x) {
    return static_cast<int16_t>(__SSAT(static_cast<int32_t>(x * 32767.0f), 16));
  }

  // cubic soft clip, unity gain near 0, flat at +-1 from +-1.5 on
  static inline float SoftClip(float x) {
    x = constrain(x, -1.5f, 1.5f);
    return x - (4.0f / 27.0f) * x * x * x;
  }

  __attribute__((always_inline)) inline float Allpass(float x, int i, float g) {
    float* buf = mem + AP_OFS[i];
    int& p = ix[i];
    const float d = buf[p];
    const float v = x + g * d;
    buf[p] = v;
    if (++p >= AP_LEN[i]) p = 0;
    return d - g * v;
  }

  // copy one block from `delay` samples back (wrapping around the buffer)
  void ReadBlock(int ch, int delay, int16_t* out) const {
    const int16_t* b = dbuf[ch];
    size_t start = (wpos + dlen - delay) % dlen;
    const size_t first = std::min<size_t>(AUDIO_BLOCK_SAMPLES, dlen - start);
    memcpy(out, b + start, first * sizeof(int16_t));
    if (first < AUDIO_BLOCK_SAMPLES)
      memcpy(out + first, b, (AUDIO_BLOCK_SAMPLES - first) * sizeof(int16_t));
  }

  void ReadTap(int ch, int t, int16_t* out);
};

static int16_t rv_pool[2][22868];  // RAM1 (= AudioEffectEcho::RV_TOTAL)
static bool rv_pool_used[2] = {false, false};

FLASHMEM int16_t* AudioEffectEcho::TakeLines() {
  for (int k = 0; k < 2; k++)
    if (!rv_pool_used[k]) { rv_pool_used[k] = true; memset(rv_pool[k], 0, sizeof(rv_pool[k])); return rv_pool[k]; }
  return static_cast<int16_t*>(calloc(RV_TOTAL, sizeof(int16_t)));
}

FLASHMEM void AudioEffectEcho::GiveLines(int16_t* p) {
  for (int k = 0; k < 2; k++)
    if (p == rv_pool[k]) { rv_pool_used[k] = false; return; }
  free(p);
}

// Reads one tap's block; crossfades when its time has changed.
FLASHMEM void AudioEffectEcho::ReadTap(int ch, int t, int16_t* out) {
  Tap& tp = tap_[ch][t];
  if (tp.phase >= XFADE && tp.target != tp.cur) {
    tp.next = tp.target;
    tp.phase = 0;
  }
  ReadBlock(ch, tp.cur, out);
  if (tp.phase >= XFADE) return;

  int16_t other[AUDIO_BLOCK_SAMPLES];
  ReadBlock(ch, tp.next, other);
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const int w = tp.phase + n; // 0..XFADE linear
    out[n] = static_cast<int16_t>((out[n] * (XFADE - w) + other[n] * w) / XFADE);
  }
  tp.phase += AUDIO_BLOCK_SAMPLES;
  if (tp.phase >= XFADE) tp.cur = tp.next;
}

// Runs from RAM1 (ITCM) like the other Loom engines.
FASTRUN void AudioEffectEcho::update() {
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1] || !ready()) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }

  // per-block parameter smoothing
  constexpr float k = 0.15f;
  fb += k * (fb_target - fb);
  dmix += k * (dmix_target - dmix);
  rmix += k * (rmix_target - rmix);
  spread_ += k * (spread_target - spread_);
  freeze_amt += 0.06f * ((freeze ? 1.0f : 0.0f) - freeze_amt);

  const float frz = freeze_amt;
  const float live = 1.0f - frz;
  const float s = spread_;
  const float g = ap_g * live;
  const float la = lp_a, ha = hp_a;
  // the reverb, per block: SIZE glides (~0.15 s), the lines' loop gains
  // follow their lengths and DECAY, frozen = no loss, no damping
  rv_scale += 0.02f * (rv_scale_target - rv_scale);
  float lg[RL], len[RL];
  const float rt = decay_secs * AUDIO_SAMPLE_RATE_EXACT;
  for (int i = 0; i < RL; i++) {
    len[i] = RV_LEN[i] * rv_scale;
    const float gi = exp2f(-9.966f * len[i] / rt);  // 10^(-3 len / rt)
    lg[i] = gi + (1.0f - gi) * frz;
  }
  const float rda = rv_damp_a + (1.0f - rv_damp_a) * frz;
  const float rv_hp_a = OnePoleCoeff(90.0f);  // the low cut at the input
  // the wobble: 4 lines, 0.3 .. 0.9 Hz, up to ~8 .. 16 samples
  static constexpr float LFO_HZ[4] = {0.31f, 0.47f, 0.69f, 0.89f};
  static constexpr float LFO_DEPTH[4] = {11.0f, 8.0f, 14.0f, 9.0f};
  // (worked out per block, straight lines in between: it moves slowly)
  float wob[4], wob_step[4];
  for (int k = 0; k < 4; k++) {
    const float a = LFO_DEPTH[k] * sinf(2.0f * float(M_PI) * lfo_ph[k]);
    lfo_ph[k] += LFO_HZ[k] * AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_EXACT;
    lfo_ph[k] -= floorf(lfo_ph[k]);
    const float b = LFO_DEPTH[k] * sinf(2.0f * float(M_PI) * lfo_ph[k]);
    wob[k] = a;
    wob_step[k] = (b - a) / AUDIO_BLOCK_SAMPLES;
  }
  // the line lengths with their wobble, per sample: start + n * step
  float dl0[RL], dls[RL];
  for (int i = 0; i < RL; i++) {
    const int wk = i == 0 ? 0 : i == 2 ? 1 : i == 5 ? 2 : i == 7 ? 3 : -1;
    dl0[i] = len[i] + (wk >= 0 ? wob[wk] : 0.0f);
    dls[i] = wk >= 0 ? wob_step[wk] : 0.0f;
  }

  // --- read the taps a block at a time ---
  float tsum[2][AUDIO_BLOCK_SAMPLES];
  int16_t last[2][AUDIO_BLOCK_SAMPLES];
  int16_t blk[AUDIO_BLOCK_SAMPLES];
  int peak_tap[2][TAPS] = {};
  for (int c = 0; c < 2; c++) {
    float* ts = tsum[c];
    for (int t = 0; t < TAPS; t++) {
      int16_t* dst = (t == TAPS - 1) ? last[c] : blk;
      ReadTap(c, t, dst);
      const float tg = TAP_GAIN[t] * (1.0f / 32768.0f);
      int pk = 0;
      if (t == 0) {
        for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
          ts[n] = dst[n] * tg;
          pk = std::max(pk, abs(dst[n]));
        }
      } else {
        for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
          ts[n] += dst[n] * tg;
          pk = std::max(pk, abs(dst[n]));
        }
      }
      peak_tap[c][t] = pk;
    }
  }

  static const int16_t silence[AUDIO_BLOCK_SAMPLES] = {};
  const int16_t* dry_in[2] = {
    in[0] ? in[0]->data : silence, in[1] ? in[1]->data : silence
  };
  int16_t wr[2][AUDIO_BLOCK_SAMPLES];
  float peak_tail = 0.0f;
  int peak_in = 0, peak_out = 0;
  constexpr float q = 1.0f / 32768.0f;

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const int16_t d0 = dry_in[0][n], d1 = dry_in[1][n];
    peak_in = std::max(peak_in, std::max(abs(d0), abs(d1)));
    const float dry[2] = {d0 * q, d1 * q};
    const float lst[2] = {last[0][n] * q, last[1][n] * q};

    // --- delay write: input + processed feedback ---
    // ping-pong 0 = straight stereo, 1 = mono into L, L<->R in the loop
    const float mono = 0.5f * (dry[0] + dry[1]);
    const float inj[2] = {
      live * (dry[0] + s * (mono - dry[0])),
      live * (dry[1] * (1.0f - s)),
    };
    const float fbsrc[2] = {
      lst[0] + s * (lst[1] - lst[0]),
      lst[1] + s * (lst[0] - lst[1]),
    };
    for (int c = 0; c < 2; c++) {
      float x = fbsrc[c] * fb;
      lp_z[c] += la * (x - lp_z[c]);
      x = lp_z[c];
      hp_z[c] += ha * (x - hp_z[c]);
      x -= hp_z[c];
      if (g > 0.0f) {
        x = Allpass(x, AP_FB_BASE + c * 2, g);
        x = Allpass(x, AP_FB_BASE + c * 2 + 1, g);
      }
      x = SoftClip(x);
      // frozen: loop the last tap untouched
      wr[c][n] = ToQ15(inj[c] + x * live + fbsrc[c] * frz);
    }

    // DELAY: crossfade dry -> echoes
    const float x1[2] = {
      dry[0] + dmix * (tsum[0][n] - (dsend ? 0.0f : dry[0])),
      dry[1] + dmix * (tsum[1][n] - (dsend ? 0.0f : dry[1])),
    };

    // --- REVERB: 8-line FDN fed by the delay stage ---
    float rin[2];
    for (int c = 0; c < 2; c++) {
      float x = live * 0.5f * x1[c];
      rv_hp_z[c] += rv_hp_a * (x - rv_hp_z[c]);  // low cut (no mud)
      x -= rv_hp_z[c];
      for (int k = 0; k < 4; k++) x = Allpass(x, AP_IN_BASE + c * 4 + k, AP_IN_G[k]);
      rin[c] = x;
    }
    float y[RL];
    float ysum = 0.0f;
    for (int i = 0; i < RL; i++) {
      // read len samples back (lines 0, 2, 5, 7 wobbled), interpolated
      const float dl = dl0[i] + n * dls[i];
      const int cap = RV_CAP[i];
      float rp = float(rv_wi[i]) - dl;
      if (rp < 0.0f) rp += cap;
      const int i0 = int(rp);
      const int i1 = i0 + 1 >= cap ? 0 : i0 + 1;
      const float fr = rp - i0;
      const int16_t* line = rv + RV_OFS[i];
      const float d = (line[i0] + fr * (line[i1] - line[i0])) * (1.0f / RV_Q);
      rv_lp[i] += rda * (d - rv_lp[i]);  // damping (TONE)
      y[i] = rv_lp[i] * lg[i];
      ysum += y[i];
    }
    // Householder: every line into every other, lossless
    const float hh = ysum * (2.0f / RL);
    for (int i = 0; i < RL; i++) {
      const float v = y[i] - hh + rin[i & 1] * ((i & 2) ? -0.5f : 0.5f);
      rv[RV_OFS[i] + rv_wi[i]] = int16_t(__SSAT(int32_t(v * RV_Q), 16));
      if (++rv_wi[i] >= RV_CAP[i]) rv_wi[i] = 0;
    }
    const float tail[2] = {0.42f * (y[0] - y[2] + y[4] - y[6]), 0.42f * (y[1] - y[3] + y[5] - y[7])};
    peak_tail = std::max(peak_tail, std::max(fabsf(tail[0]), fabsf(tail[1])));

    for (int c = 0; c < 2; c++) {
      const int16_t o = ToQ15(x1[c] + rmix * (tail[c] - (rsend ? 0.0f : x1[c])));
      out[c]->data[n] = o;
      peak_out = std::max(peak_out, abs(o));
    }
  }

  // write this block into the delay lines
  for (int c = 0; c < 2; c++) {
    int16_t* b = dbuf[c];
    const size_t first = std::min<size_t>(AUDIO_BLOCK_SAMPLES, dlen - wpos);
    memcpy(b + wpos, wr[c], first * sizeof(int16_t));
    if (first < AUDIO_BLOCK_SAMPLES)
      memcpy(b, wr[c] + first, (AUDIO_BLOCK_SAMPLES - first) * sizeof(int16_t));
  }
  wpos = (wpos + AUDIO_BLOCK_SAMPLES) % dlen;

  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);

  for (int c = 0; c < 2; c++)
    for (int t = 0; t < TAPS; t++)
      viz.tap[c][t] = std::max(viz.tap[c][t], peak_tap[c][t] * q);
  viz.tail = std::max(viz.tail, peak_tail);
  viz.in = std::max(viz.in, peak_in * q);
  viz.out = std::max(viz.out, peak_out * q);
  viz.blocks++;
}
