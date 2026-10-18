#pragma once

#include <Audio.h>
#include "PsramAlloc.h"
#include "AudioSample.h"
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
  // the CPU is short (Loom sets it): the reverb runs on 4 of its 8 lines,
  // the other 4 fading out (and in again, from silence, when it is back)
  static inline volatile bool lite = false;
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
    // the pre-delay in RAM2 (it was in PSRAM: a read and a write per sample
    // there made it the reverb's most costly part)
    if (!pre) {
      pre = static_cast<float*>(malloc(2 * PRE_H * sizeof(float)));
      if (pre) std::fill(pre, pre + 2 * PRE_H, 0.0f);
    }
    if (!rv) rv = TakeLines();
    if (!sh) sh = static_cast<int16_t*>(malloc(2 * SH_LEN * sizeof(int16_t)));
    if (!shd) shd = static_cast<int16_t*>(PsramAlloc(2 * SHD_LEN * sizeof(int16_t)));
    for (auto& b : dbuf) {
      if (!b) b = static_cast<int16_t*>(PsramAlloc(dlen * sizeof(int16_t)));
    }
    Reset();
    return ready();
  }

  void Release() {
    free(mem);
    mem = nullptr;
    free(pre);
    pre = nullptr;
    GiveLines(rv);
    rv = nullptr;
    free(sh);
    sh = nullptr;
    if (shd) PsramFree(shd);
    shd = nullptr;
    for (auto& b : dbuf) {
      if (b) PsramFree(b);
      b = nullptr;
    }
  }

  // parked (bypassed and silent): give back what sits in RAM2 (the
  // network's state, the pre-delay, the shifter's ring: ~47 KB), so the
  // other track's REPEAT finds room; Acquire takes it again. (Called from
  // the main loop: update() runs in the audio interrupt, never halfway
  // while this runs, and finds the pointers either all there or gone)
  void Shrink() {
    float* m = mem; float* p = pre; int16_t* s = sh;
    __disable_irq();
    mem = nullptr; pre = nullptr; sh = nullptr;
    __enable_irq();
    free(m); free(p); free(s);
  }
  bool ready() const { return mem && rv && pre && sh && dbuf[0] && dbuf[1]; }
  bool HoldsRam2() const { return mem || pre || sh; }
  // what it holds, for the status: m r p s d d (a dash: missing)
  void Holds(char* b) const { b[0] = mem ? 'm' : '-'; b[1] = rv ? 'r' : '-'; b[2] = pre ? 'p' : '-'; b[3] = sh ? 's' : '-'; b[4] = dbuf[0] ? 'd' : '-'; b[5] = dbuf[1] ? 'd' : '-'; b[6] = 0; }

  void Reset() {
    if (mem) std::fill(mem, mem + MEM_TOTAL, 0.0f);
    if (rv) memset(rv, 0, RV_TOTAL * sizeof(int16_t));
    for (auto& w : rv_wi) w = 0;
    if (pre) std::fill(pre, pre + 2 * PRE_H, 0.0f);
    pre_w = 0;
    for (auto& z : rv_lp) z = 0.0f;
    rv_hp_z[0] = rv_hp_z[1] = 0.0f;
    rv_t[0] = rv_t[1] = 0.0f;
    if (sh) memset(sh, 0, 2 * SH_LEN * sizeof(int16_t));
    if (shd) memset(shd, 0, 2 * SHD_LEN * sizeof(int16_t));
    sh_hp2[0] = sh_hp2[1] = 0.0f;
    for (auto& t : sh_tail) for (auto& v : t) v = 0.0f;
    sh_lp[0] = sh_lp[1] = sh_dc[0] = sh_dc[1] = sh_env = 0.0f;
    sh_up[0] = sh_up[1] = -1;
    for (auto b : dbuf) if (b) memset(b, 0, dlen * sizeof(int16_t));
    for (auto& i : ix) i = 0;
    for (int c = 0; c < 2; c++) lp_z[c] = hp_z[c] = 0.0f;
  }

  // --- parameters, set from the UI thread ---
  // tap delay in samples, at least MIN_TAP
  void tap(int ch, int t, int samples) {
    tap_[ch][t].target = constrain(samples, MIN_TAP, int(dlen) - 1);
  }
  // a head's level in the echoes, 0 (off) .. 1
  void tap_gain(int t, float g) { if (t >= 0 && t < TAPS) tap_gt[t] = constrain(g, 0.0f, 1.0f); }
  void feedback(float f) { fb_target = constrain(f, 0.0f, 1.05f); }
  void delay_mix(float m) { dmix_target = constrain(m, 0.0f, 1.0f); }
  void reverb_mix(float m) { rmix_target = constrain(m, 0.0f, 1.0f); }
  // SEND instead of MIX: the dry signal stays at full level and the echoes /
  // the reverb are added on top (MIX crossfades them against it)
  void delay_send(bool s) { dsend = s; }
  void reverb_send(bool s) { rsend = s; }
  // GLINT (a shimmer): the reverb's tail pitched and fed back into it, so it climbs
  // with every pass. amount 0..1 (0: off); the recipe: below
  // A recipe, as Eventide's ShimmerVerb: the pitch of the left and the
  // right shifter (cents), how long the tail waits before them (ms) and how
  // much of its low / mid / high band goes round again (0..1)
  void shimmer(float amount) { sh_amt_target = constrain(amount, 0.0f, 1.0f); }
  // a new recipe: while the shimmer is silent, at once; while it sounds, it
  // waits until the recipe has stayed the same for ~1/4 s (a knob turned
  // through DEEP, DOVE, DRIFT on its way to DUST: only DUST), then the
  // shimmer's feed fades out (~40 ms), the recipe changes and it fades in
  // again (~100 ms): no cut, no jump in pitch or delay
  void shimmer_recipe(float cents_l, float cents_r, float ms, float lo, float mid, float hi, float fb) {
    Recipe r;
    r.fb = constrain(fb, 0.0f, 1.2f);
    r.ra[0] = exp2f(cents_l / 1200.0f);
    r.ra[1] = exp2f(cents_r / 1200.0f);
    r.dn = constrain(int(ms * 0.001f * AUDIO_SAMPLE_RATE_EXACT * 0.5f), 1, SHD_LEN - 1);
    r.lv[0] = lo; r.lv[1] = mid; r.lv[2] = hi;
    if (sh_amt <= 0.0f && sh_amt_target <= 0.0f) { UseRecipe(r); pend = false; return; }
    if (!pend && r.Same(Current())) return;            // (PRE / ROOM call it again: nothing new)
    if (pend && r.Same(pend_r)) return;
    __disable_irq();
    pend_r = r; pend_age = 0; pend = true;
    __enable_irq();
  }
  // a part switched off: its input closes, the dry passes whole and what
  // it holds rings out
  void delay_open(bool o) { d_open = o; }
  void reverb_open(bool o) { r_open = o; }
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

  // 0..1: the room, 0.35 .. 1x the line lengths (a new size is crossfaded
  // in, so the tail keeps its pitch)
  void size(float s) { rv_scale_target = 0.35f + 0.65f * constrain(s, 0.0f, 1.0f); }

  // approximate RT60 of the reverb tail in seconds
  void decay(float secs) { decay_secs = constrain(secs, 0.2f, 30.0f); }

  // the gap before the reverb, 0..150 ms (a change crossfades, no bend)
  void pre_delay(float ms) { pre_target = constrain(int(ms * 0.001f * AUDIO_SAMPLE_RATE_EXACT), 0, PRE_MAX - 1); }

  // how quickly the echoes melt into a wash: 0 = audible reflections,
  // 1 = an instant soft cloud (the input diffusers' strength)
  void density(float d) { rv_diff = 0.4f + 0.6f * constrain(d, 0.0f, 1.0f); }

  void ReadViz(Viz& out) {
    AudioNoInterrupts();
    out = viz;
    viz = Viz{};
    viz.blocks = out.blocks;
    AudioInterrupts();
  }

  void update() override;

private:
  // each head's level: what the UI asks for (tap_gain) and where it is now,
  // gliding there over ~45 ms so a head coming in or going out never clicks.
  // A head at 0 isn't read at all (it costs nothing). The last head is the
  // one that feeds back, so it is always read, whatever its level.
  volatile float tap_gt[TAPS] = {0.0f, 0.0f, 0.0f, 1.0f};
  float tap_g[TAPS] = {0.0f, 0.0f, 0.0f, 1.0f};
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
  static constexpr int PRE_MAX = 7201;    // 150 ms of pre-delay, a side
  static constexpr int PRE_H = PRE_MAX / 2 + 1;  // the same at half the rate
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
  float* pre = nullptr;             // pre-delay at half the rate, 2 x PRE_H
  int pre_w = 0, pre_cur = 0, pre_next = 0, pre_xf = 0;
  volatile int pre_target = 0;
  volatile float rv_diff = 1.0f;
  float rv_sc_a = 1.0f, rv_sc_b = 1.0f;  // the lengths now / the ones fading in
  int rv_xf = 0;                    // samples left of that crossfade (0: none)
  static constexpr int RV_XF = 2048;  // ~46 ms
  volatile float rv_scale_target = 1.0f;
  volatile float rv_damp_a = 0.5f;
  float rv_lp[RL] = {};
  float rv_hi = 1.0f;       // lines 4..7's share: 1 = all 8 lines, 0 = 4 (lite)
  float rv_t[2] = {};  // half rate: the last step's tail
  // SHIMMER: a pitch shifter at half the rate, fed with the last block's
  // tail. Per side two read heads play the tail faster (up) or slower
  // (down) than it was written; each plays a grain of about SH_WIN samples,
  // fading in and out (sin^2), the two half a grain apart, so one is always
  // loud while the other jumps back and starts again.
  // Where a head lands when it starts again matters: on a pure tone (a
  // TWILL bell) a head landing out of phase with the other one cancels the
  // tone during the crossfade, about 16 times a second, and the octave turns
  // into a flat, warbling cluster. So a new grain looks around its landing
  // place, up to SH_ALIGN samples either way (a whole period down to ~75 Hz),
  // for the spot that lines up best with what the other head plays now.
  static constexpr int SH_LEN = 2048, SH_WIN = 1536, SH_ALIGN = 160;
  int16_t* sh = nullptr;       // 2 x SH_LEN: the tail as it came, a ring a side
  int sh_w = 0;
  float sh_hd[2][2] = {};      // each head's delay behind the writing, samples
  float sh_hs[2][2] = {};      // where its grain started
  int8_t sh_up[2] = {-1, -1};  // which way that side shifts (-1: heads not placed yet)
  float sh_tail[2][AUDIO_BLOCK_SAMPLES / 2] = {};
  float sh_lp[2] = {}, sh_dc[2] = {}, sh_hp2[2] = {}, sh_env = 0.0f;
  // the pitched sound waits ~150 ms before it goes back in (as Eventide's
  // pitch delay): each step up comes as its own bloom instead of one smear
  static constexpr int SHD_LEN = 8820;  // at the half rate: up to 400 ms
  int16_t* shd = nullptr;  // 2 x SHD_LEN, PSRAM (none: no wait)
  int shd_w = 0;
  volatile int shd_n = 1102;  // the wait (50 ms)
  volatile float sh_ra[2] = {2.0f, 2.0f};  // the shifters' ratios, left / right
  volatile float sh_lv[3] = {0.4f, 0.9f, 0.8f};  // low / mid / high round again
  volatile float sh_fb = 0.8f;  // how strongly it goes round
  struct Recipe {
    float fb = 0.0f, ra[2] = {1.0f, 1.0f}, lv[3] = {0.0f, 0.0f, 0.0f}; int dn = 1;
    bool Same(const Recipe& o) const {
      return fb == o.fb && ra[0] == o.ra[0] && ra[1] == o.ra[1] && dn == o.dn && lv[0] == o.lv[0] && lv[1] == o.lv[1] && lv[2] == o.lv[2];
    }
  };
  Recipe Current() const { Recipe r; r.fb = sh_fb; r.ra[0] = sh_ra[0]; r.ra[1] = sh_ra[1]; r.dn = shd_n; for (int i = 0; i < 3; i++) r.lv[i] = sh_lv[i]; return r; }
  void UseRecipe(const Recipe& r) {
    sh_fb = r.fb; sh_ra[0] = r.ra[0]; sh_ra[1] = r.ra[1]; shd_n = r.dn;
    for (int i = 0; i < 3; i++) sh_lv[i] = r.lv[i];
  }
  Recipe pend_r;
  volatile bool pend = false;
  int pend_age = 0;     // blocks the pending recipe has stayed the same
  float sh_xf = 1.0f;   // the shimmer feed's fade for a recipe change
  volatile float sh_amt_target = 0.0f;
  float sh_amt = 0.0f;
  bool rv_hi_cold = false;  // lines 4..7 were left out: cleared before they come back
  float rv_hp_z[2] = {};
  const float rv_hp_a = OnePoleCoeff(180.0f);  // 90 Hz at half the rate: the low cut at the input
  float lfo_ph[4] = {0.0f, 0.25f, 0.5f, 0.75f};

  volatile float fb_target = 0.0f, dmix_target = 0.4f, rmix_target = 0.3f;
  volatile float spread_target = 0.0f, ap_g = 0.0f;
  volatile float lp_a = 1.0f, hp_a = 0.0f;
  volatile bool freeze = false;
  volatile bool d_open = true, r_open = true;
  float dopen = 1.0f, ropen = 1.0f;
  volatile float decay_secs = 3.0f;

  float fb = 0.0f, dmix = 0.4f, rmix = 0.3f, spread_ = 0.0f, freeze_amt = 0.0f;
  volatile bool dsend = false, rsend = false;
  float lp_z[2] = {}, hp_z[2] = {};
  bool rv_cold = false;  // the reverb was off (LEVEL 0): cleared before it starts again

  Viz viz = {};

  static float OnePoleCoeff(float hz) {
    return 1.0f - expf(-2.0f * float(M_PI) * hz / AUDIO_SAMPLE_RATE_EXACT);
  }

  __attribute__((always_inline)) static inline int16_t ToQ15(float x) {
    return SoftClip16(x * 32767.0f);
  }

  // cubic soft clip, unity gain near 0, flat at +-1 from +-1.5 on
  __attribute__((always_inline)) static inline float SoftClip(float x) {
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

  // the reverb's input diffusers run at half the rate: half the length is
  // the same time
  __attribute__((always_inline)) inline float AllpassHalf(float x, int i, float g) {
    float* buf = mem + AP_OFS[i];
    int& p = ix[i];
    const float d = buf[p];
    const float v = x + g * d;
    buf[p] = v;
    if (++p >= AP_LEN[i] / 2) p = 0;
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
  // a shifter head on side c starting a new grain: the delay in [lo, hi]
  // whose sound (the 64 samples before it) lines up best with the 64 before
  // the other head's, plus the other head's fraction, so both read in step
  float ShPlace(int c, int lo, int hi, float other) const {
    const int16_t* b = sh + c * SH_LEN;
    const int ro = sh_w - int(other);
    int best = (lo + hi) / 2;
    float bs = -1e30f;
    for (int dd = lo; dd <= hi; dd++) {
      const int rn = sh_w - dd;
      float sc = 0.0f;
      for (int i = 0; i < 64; i += 2) {
        int a = rn - i; if (a < 0) a += SH_LEN;
        int o = ro - i; if (o < 0) o += SH_LEN;
        sc += float(b[a]) * float(b[o]);
      }
      if (sc > bs) { bs = sc; best = dd; }
    }
    return float(best) + (other - floorf(other));
  }
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
// (O2: the build is -Os, which left the per-sample helpers as calls)
FASTRUN __attribute__((optimize("O2"))) void AudioEffectEcho::update() {
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
  dopen += k * ((d_open ? 1.0f : 0.0f) - dopen);
  ropen += k * ((r_open ? 1.0f : 0.0f) - ropen);

  const float frz = freeze_amt;
  const float live = 1.0f - frz;
  // a part whose LEVEL is 0 is not heard: it isn't computed (the delay
  // still records the input, without its echoes; the reverb starts from
  // silence when its LEVEL opens again)
  if (dmix_target <= 0.0f && dmix < 1e-4f) dmix = 0.0f;
  if (rmix_target <= 0.0f && rmix < 1e-4f) rmix = 0.0f;
  const bool dly_on = dmix > 0.0f || dmix_target > 0.0f || freeze || frz > 1e-4f;
  const bool rv_on = rmix > 0.0f || rmix_target > 0.0f;
  if (!rv_on) {
    rv_cold = true;
  } else if (rv_cold) {
    rv_cold = false;
    memset(rv, 0, RV_TOTAL * sizeof(int16_t));
    std::fill(pre, pre + 2 * PRE_H, 0.0f);
    for (auto& z : rv_lp) z = 0.0f;
    rv_hp_z[0] = rv_hp_z[1] = 0.0f;
    rv_t[0] = rv_t[1] = 0.0f;
    std::fill(mem + AP_OFS[AP_IN_BASE], mem + MEM_TOTAL, 0.0f);
  }
  const float s = spread_;
  const float g = ap_g * live;
  const float la = lp_a, ha = hp_a;
  // the reverb, per block: SIZE glides (~0.15 s), the lines' loop gains
  // follow their lengths and DECAY, frozen = no loss, no damping
  // a new SIZE: read the lines at the old and the new lengths at once and
  // crossfade (a glide would bend the pitch)
  if (rv_xf == 0 && fabsf(rv_scale_target - rv_sc_a) > 0.002f) {
    rv_sc_b = rv_scale_target;
    rv_xf = RV_XF;
  }
  float lg[RL], len[RL], lenb[RL];
  const float rt = decay_secs * AUDIO_SAMPLE_RATE_EXACT;
  for (int i = 0; i < RL; i++) {
    len[i] = RV_LEN[i] * rv_sc_a;
    lenb[i] = RV_LEN[i] * rv_sc_b;
    const float gi = exp2f(-9.966f * (rv_xf ? lenb[i] : len[i]) / rt);  // 10^(-3 len / rt)
    lg[i] = gi + (1.0f - gi) * frz;
  }
  // the reverb runs at half the rate (HALF steps a block; its tail is
  // damped to 6 kHz unless TONE is bright, so nothing is lost but CPU): the
  // lengths in its own samples, the damping and the low cut for its rate
  constexpr int HALF = AUDIO_BLOCK_SAMPLES / 2;
  const float xf0 = rv_xf ? float(RV_XF - rv_xf) / RV_XF : 0.0f;  // the new one's share at k = 0
  const float xfs = rv_xf ? 2.0f / RV_XF : 0.0f;
  const float rda1 = rv_damp_a + (1.0f - rv_damp_a) * frz;
  const float rda = 1.0f - (1.0f - rda1) * (1.0f - rda1);  // one step = two of the full rate
  // pre-delay: a new time crossfades in over ~12 ms
  constexpr int PRE_XF = 512;
  if (pre_xf == 0 && pre_target != pre_cur) { pre_next = pre_target; pre_xf = PRE_XF; }
  const float dg = rv_diff;
  // the wobble: 4 lines, 0.3 .. 0.9 Hz, up to ~8 .. 16 samples
  static constexpr float LFO_HZ[4] = {0.31f, 0.47f, 0.69f, 0.89f};
  static constexpr float LFO_DEPTH[4] = {11.0f, 8.0f, 14.0f, 9.0f};
  // (worked out per block, straight lines in between: it moves slowly)
  float wob[4], wob_step[4];
  for (int k = 0; k < 4; k++) {
    const float a = 0.5f * LFO_DEPTH[k] * sinf(2.0f * float(M_PI) * lfo_ph[k]);  // (half-rate samples)
    lfo_ph[k] += LFO_HZ[k] * AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_EXACT;
    lfo_ph[k] -= floorf(lfo_ph[k]);
    const float b = 0.5f * LFO_DEPTH[k] * sinf(2.0f * float(M_PI) * lfo_ph[k]);
    wob[k] = a;
    wob_step[k] = (b - a) / HALF;
  }
  // the line lengths with their wobble, per sample: start + n * step
  float dl0[RL], dlb[RL], dls[RL];
  for (int i = 0; i < RL; i++) {
    const int wk = i == 0 ? 0 : i == 2 ? 1 : i == 5 ? 2 : i == 7 ? 3 : -1;
    const float wb = wk >= 0 ? wob[wk] : 0.0f;
    dl0[i] = 0.5f * len[i] + wb;
    dlb[i] = 0.5f * lenb[i] + wb;
    dls[i] = wk >= 0 ? wob_step[wk] : 0.0f;
  }

  // --- read the taps a block at a time ---
  float tsum[2][AUDIO_BLOCK_SAMPLES];
  int16_t last[2][AUDIO_BLOCK_SAMPLES];
  int16_t blk[AUDIO_BLOCK_SAMPLES];
  int peak_tap[2][TAPS] = {};
  // each head's level at the start and the end of this block (a ramp in
  // between): one step of the glide towards what the UI asked for
  float g0[TAPS], g1[TAPS];
  for (int t = 0; t < TAPS; t++) {
    g0[t] = tap_g[t];
    const float to = tap_gt[t], step = 1.0f / 16.0f;
    g1[t] = to > g0[t] ? std::min(to, g0[t] + step) : std::max(to, g0[t] - step);
    if (dly_on) tap_g[t] = g1[t];
  }
  for (int c = 0; c < 2 && dly_on; c++) {
    float* ts = tsum[c];
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) ts[n] = 0.0f;
    for (int t = 0; t < TAPS; t++) {
      const bool fb_head = t == TAPS - 1;
      if (!fb_head && g0[t] <= 0.0f && g1[t] <= 0.0f) continue;  // a silent head: not read
      int16_t* dst = fb_head ? last[c] : blk;
      ReadTap(c, t, dst);
      const float ga = g0[t] * (1.0f / 32768.0f);
      const float gs = (g1[t] - g0[t]) * (1.0f / 32768.0f) / AUDIO_BLOCK_SAMPLES;
      int pk = 0;
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        ts[n] += dst[n] * (ga + gs * n);
        pk = std::max(pk, abs(dst[n]));
      }
      peak_tap[c][t] = (g0[t] > 0.0f || g1[t] > 0.0f) ? pk : 0;
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

  // The work below is done in passes over the block, the two sides side by
  // side: the CPU runs in order and waits on every result it needs next, so
  // one side alone keeps it waiting most of the time; two (or two reverb
  // lines) fill those gaps. Same steps, same numbers as one sample at a time.

  // the reverb lines' outputs for the whole block first: every line is
  // longer than a block, so nothing read here is written in it. Read len
  // back (lines 0, 2, 5, 7 wobbled), interpolated, damped (TONE), loop gain.
  // lite: lines 4..7 fade out over ~45 ms and are then left out
  if (!lite && rv_hi <= 0.0f && rv_hi_cold) {
    memset(rv + RV_OFS[RL / 2], 0, (RV_TOTAL - RV_OFS[RL / 2]) * sizeof(int16_t));
    for (int i = RL / 2; i < RL; i++) rv_lp[i] = 0.0f;
    rv_hi_cold = false;
  }
  const float hi0 = rv_hi;
  const float hi1 = std::max(0.0f, std::min(1.0f, hi0 + (lite ? -1.0f : 1.0f) / 16.0f));
  rv_hi = hi1;
  const int nl = hi0 > 0.0f || hi1 > 0.0f ? RL : RL / 2;  // lines computed
  if (nl < RL) rv_hi_cold = true;
  const float hi_step = (hi1 - hi0) * (1.0f / HALF);
  static float yb[RL][HALF];
  if (rv_on) {
    // read `back` behind the write position (always more than a block, so
    // never past it), interpolated
    auto read = [](const int16_t* line, int cap, float rp) __attribute__((always_inline)) {
      if (rp < 0.0f) rp += cap;
      const int i0 = int(rp);
      const int i1 = i0 + 1 >= cap ? 0 : i0 + 1;
      const float fr = rp - i0;
      return (line[i0] + fr * (line[i1] - line[i0])) * (1.0f / RV_Q);
    };
    for (int i = 0; i < nl; i += 2) {  // two lines at a time
      const int16_t* la_ = rv + RV_OFS[i];
      const int16_t* lb_ = rv + RV_OFS[i + 1];
      const int ca = RV_CAP[i], cb = RV_CAP[i + 1];
      const float wa = float(rv_wi[i]), wb = float(rv_wi[i + 1]);
      const float ga = lg[i], gb = lg[i + 1];
      const float da = dl0[i], db = dl0[i + 1], dba = dlb[i], dbb = dlb[i + 1];
      const float sa_ = 1.0f - dls[i], sb_ = 1.0f - dls[i + 1];  // the read moves on by this a step
      float za = rv_lp[i], zb = rv_lp[i + 1];
      if (!rv_xf) {
        for (int n = 0; n < HALF; n++) {
          const float a = read(la_, ca, wa - da + n * sa_);
          const float b = read(lb_, cb, wb - db + n * sb_);
          za += rda * (a - za);
          zb += rda * (b - zb);
          yb[i][n] = za * ga;
          yb[i + 1][n] = zb * gb;
        }
      } else {  // crossfading to the new size
        for (int n = 0; n < HALF; n++) {
          const float wgt = std::min(1.0f, xf0 + n * xfs);
          float a = read(la_, ca, wa - da + n * sa_);
          float b = read(lb_, cb, wb - db + n * sb_);
          a += wgt * (read(la_, ca, wa - dba + n * sa_) - a);
          b += wgt * (read(lb_, cb, wb - dbb + n * sb_) - b);
          za += rda * (a - za);
          zb += rda * (b - zb);
          yb[i][n] = za * ga;
          yb[i + 1][n] = zb * gb;
        }
      }
      rv_lp[i] = za;
      rv_lp[i + 1] = zb;
    }
  }

  // --- the delay: the write signal (input + processed feedback) and
  // DELAY's crossfade dry -> echoes (x1, what the reverb is fed)
  float x1b[2][AUDIO_BLOCK_SAMPLES];
  const float dm = dmix;
  const float dsub = dsend ? 0.0f : dopen;  // (off: the dry whole)
  const float dlive = live * dopen;          // (off: nothing new in)
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const int16_t d0 = dry_in[0][n], d1 = dry_in[1][n];
    peak_in = std::max(peak_in, std::max(abs(d0), abs(d1)));
  }
  if (!dly_on) {  // the input only
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      const float dry0 = dry_in[0][n] * q, dry1 = dry_in[1][n] * q;
      const float mono = 0.5f * (dry0 + dry1);
      wr[0][n] = ToQ15(dlive * (dry0 + s * (mono - dry0)));
      wr[1][n] = ToQ15(dlive * (dry1 * (1.0f - s)));
      x1b[0][n] = dry0;
      x1b[1][n] = dry1;
    }
  } else {
    float lz0 = lp_z[0], lz1 = lp_z[1], hz0 = hp_z[0], hz1 = hp_z[1];
    int p[4] = {ix[AP_FB_BASE], ix[AP_FB_BASE + 1], ix[AP_FB_BASE + 2], ix[AP_FB_BASE + 3]};
    float* const ab = mem + AP_OFS[AP_FB_BASE];
    auto ap = [&](float x, int j) __attribute__((always_inline)) {
      float* b = ab + (AP_OFS[AP_FB_BASE + j] - AP_OFS[AP_FB_BASE]);
      const float d = b[p[j]];
      const float v = x + g * d;
      b[p[j]] = v;
      if (++p[j] >= AP_LEN[AP_FB_BASE + j]) p[j] = 0;
      return d - g * v;
    };
    auto clip = [](float x) __attribute__((always_inline)) {  // SoftClip, without branches
      x = fminf(fmaxf(x, -1.5f), 1.5f);
      return x - (4.0f / 27.0f) * x * x * x;
    };
    auto run = [&](auto diffuse) __attribute__((always_inline)) {
      for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
        const float dry0 = dry_in[0][n] * q, dry1 = dry_in[1][n] * q;
        const float l0 = last[0][n] * q, l1 = last[1][n] * q;
        // ping-pong 0 = straight stereo, 1 = mono into L, L<->R in the loop
        const float mono = 0.5f * (dry0 + dry1);
        const float inj0 = dlive * (dry0 + s * (mono - dry0));
        const float inj1 = dlive * (dry1 * (1.0f - s));
        const float f0 = l0 + s * (l1 - l0);
        const float f1 = l1 + s * (l0 - l1);
        float x0 = f0 * fb, y0 = f1 * fb;
        lz0 += la * (x0 - lz0);
        lz1 += la * (y0 - lz1);
        hz0 += ha * (lz0 - hz0);
        hz1 += ha * (lz1 - hz1);
        x0 = lz0 - hz0;
        y0 = lz1 - hz1;
        if constexpr (decltype(diffuse)::value) {
          x0 = ap(x0, 0);
          y0 = ap(y0, 2);
          x0 = ap(x0, 1);
          y0 = ap(y0, 3);
        }
        x0 = clip(x0);
        y0 = clip(y0);
        // frozen: loop the last tap untouched
        wr[0][n] = ToQ15(inj0 + x0 * live + f0 * frz);
        wr[1][n] = ToQ15(inj1 + y0 * live + f1 * frz);
        x1b[0][n] = dry0 + dm * (tsum[0][n] - dsub * dry0);
        x1b[1][n] = dry1 + dm * (tsum[1][n] - dsub * dry1);
      }
    };
    if (g > 0.0f) run(std::true_type{});
    else run(std::false_type{});
    lp_z[0] = lz0; lp_z[1] = lz1; hp_z[0] = hz0; hp_z[1] = hz1;
    for (int j = 0; j < 4; j++) ix[AP_FB_BASE + j] = p[j];
  }

  if (!rv_on) {
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      const int16_t o0 = ToQ15(x1b[0][n]), o1 = ToQ15(x1b[1][n]);
      out[0]->data[n] = o0;
      out[1]->data[n] = o1;
      peak_out = std::max(peak_out, std::max(abs(o0), abs(o1)));
    }
  } else {
    // --- REVERB: 8-line FDN fed by the delay stage, at half the rate: a
    // step takes two samples (their average)
    float rin[2][HALF];
    // SHIMMER: the last block's tail, only its mids and highs (~350 Hz ..
    // 6 kHz: the lows going round again piled up into a wall), pitched by
    // one or two voices, held back ~150 ms and fed in again, gently
    // enough that it dies away (it was strong enough to keep itself going,
    // held only by the guard: massive)
    float shim[2][HALF];
    sh_amt += 0.1f * (sh_amt_target - sh_amt);
    // a recipe change (see shimmer_recipe): wait ~1/4 s of no change, fade
    // out, change it (the heads placed afresh for it), fade in
    if (pend) {
      if (++pend_age > 86) {
        sh_xf -= 0.07f;
        if (sh_xf <= 0.0f) { sh_xf = 0.0f; UseRecipe(pend_r); pend = false; sh_up[0] = sh_up[1] = -1; }
      }
    } else if (sh_xf < 1.0f) {
      sh_xf = std::min(1.0f, sh_xf + 0.03f);
    }
    if (sh_amt_target <= 0.0f && sh_amt < 1e-3f) sh_amt = 0.0f;
    const bool shim_on = sh && sh_amt > 0.0f;
    if (shim_on) {
      // a shifter a side, as Eventide's (the left tail through A, the right
      // through B), each with the recipe's pitch
      const float ra[2] = {sh_ra[0], sh_ra[1]};
      const float lv0 = sh_lv[0], lv1 = sh_lv[1], lv2 = sh_lv[2];
      const int dn = shd_n;
      // (the guard only once the tail is loud: past 0.25 it holds it there;
      // x1 and 0.15 were too quiet a shimmer)
      const float guard = sh_env > 0.25f ? 0.25f / sh_env : 1.0f;
      const float g = sh_amt * sh_fb * 1.5f * guard * sh_xf;
      float pk = 0.0f;
      for (int k = 0; k < HALF; k++) {
        for (int c = 0; c < 2; c++) {
          // three bands (~300 Hz / ~3 kHz at the half rate), each at its level
          const float x = sh_tail[c][k];
          sh_dc[c] += 0.0819f * (x - sh_dc[c]);  // below ~300 Hz
          sh_lp[c] += 0.575f * (x - sh_lp[c]);   // below ~3 kHz
          const float y = lv0 * sh_dc[c] + lv1 * (sh_lp[c] - sh_dc[c]) + lv2 * (x - sh_lp[c]);
          sh[c * SH_LEN + sh_w] = int16_t(constrain(y * RV_Q, -32767.0f, 32767.0f));
          pk = std::max(pk, fabsf(x));
        }
        float o[2] = {0.0f, 0.0f};
        for (int c = 0; c < 2; c++) {  // side c through its own shifter
          const float r = ra[c];
          const int8_t up = r >= 1.0f ? 1 : 0;
          float* hd = sh_hd[c];
          float* hs = sh_hs[c];
          if (sh_up[c] != up) {  // the heads placed afresh: at the start, or the shift turned round
            sh_up[c] = up;
            for (int h = 0; h < 2; h++) {
              hs[h] = up ? 2.0f + SH_WIN : 2.0f;
              hd[h] = up ? hs[h] - h * (SH_WIN * 0.5f) : hs[h] + h * (SH_WIN * 0.5f);
            }
          }
          const int16_t* b = sh + c * SH_LEN;
          float acc = 0.0f, wsum = 0.0f;
          for (int h = 0; h < 2; h++) {
            hd[h] += 1.0f - r;  // up: the head catches up with the writing; down: falls behind
            // how far through its grain, 0 .. 1 (up: until it reaches the
            // writing; down: a grain's length on from where it started)
            float t = up ? 1.0f - (hd[h] - 2.0f) / (hs[h] - 2.0f) : (hd[h] - hs[h]) * (1.0f / SH_WIN);
            if (t >= 1.0f) {  // a new grain, in step with the other head
              hd[h] = up ? ShPlace(c, 2 + SH_WIN - 2 * SH_ALIGN, 2 + SH_WIN, hd[1 - h])
                         : ShPlace(c, 2, 2 + 2 * SH_ALIGN, hd[1 - h]);
              hs[h] = hd[h];
              t = 0.0f;
            }
            const float sw = sinf(float(M_PI) * std::max(0.0f, t));
            const float wgt = sw * sw;
            float rp = float(sh_w) - hd[h];
            if (rp < 0.0f) rp += SH_LEN;
            const int i0 = int(rp), i1 = i0 + 1 < SH_LEN ? i0 + 1 : 0;
            const float fr = rp - i0;
            acc += wgt * (b[i0] + fr * (b[i1] - b[i0]));
            wsum += wgt;
          }
          // the two heads' weights don't quite add up to 1 once a landing
          // has moved: divided by their sum, the level stays even
          o[c] = acc / std::max(wsum, 0.3f) * (1.0f / RV_Q);
        }
        if (++sh_w >= SH_LEN) sh_w = 0;
        if (shd) {  // held back: out what went in dn steps ago
          int r = shd_w - dn;
          if (r < 0) r += SHD_LEN;
          for (int c = 0; c < 2; c++) {
            shd[c * SHD_LEN + shd_w] = int16_t(constrain(o[c] * RV_Q, -32767.0f, 32767.0f));
            o[c] = shd[c * SHD_LEN + r] * (1.0f / RV_Q);
          }
          if (++shd_w >= SHD_LEN) shd_w = 0;
        }
        shim[0][k] = g * o[0];
        shim[1][k] = g * o[1];
      }
      sh_env = pk > sh_env ? pk : sh_env * 0.995f;
    }
    {
      float* pb0 = pre;
      float* pb1 = pre + PRE_H;
      int pw = pre_w;
      const int bc = pre_cur >> 1, bn = pre_next >> 1;
      float h0 = rv_hp_z[0], h1 = rv_hp_z[1];
      int p[8];
      for (int j = 0; j < 8; j++) p[j] = ix[AP_IN_BASE + j];
      float gj[4];
      for (int j = 0; j < 4; j++) gj[j] = AP_IN_G[j] * dg;
      float* const ab = mem + AP_OFS[AP_IN_BASE];
      // (half the length is the same time at half the rate)
      auto ap = [&](float x, int j, float gg) __attribute__((always_inline)) {
        float* b = ab + (AP_OFS[AP_IN_BASE + j] - AP_OFS[AP_IN_BASE]);
        const float d = b[p[j]];
        const float v = x + gg * d;
        b[p[j]] = v;
        if (++p[j] >= AP_LEN[AP_IN_BASE + j] / 2) p[j] = 0;
        return d - gg * v;
      };
      for (int k = 0; k < HALF; k++) {
        const int n = 2 * k + 1;
        float x0 = live * ropen * 0.25f * (x1b[0][n - 1] + x1b[0][n]);  // (0.5 x their average)
        float x1 = live * ropen * 0.25f * (x1b[1][n - 1] + x1b[1][n]);
        // pre-delay (a ring a side)
        pb0[pw] = x0;
        pb1[pw] = x1;
        int r = pw - bc;
        if (r < 0) r += PRE_H;
        x0 = pb0[r];
        x1 = pb1[r];
        if (pre_xf) {  // a new time crossfades in
          const float wgt = std::min(1.0f, float(PRE_XF - pre_xf + n) * (1.0f / PRE_XF));
          int rn = pw - bn;
          if (rn < 0) rn += PRE_H;
          x0 += wgt * (pb0[rn] - x0);
          x1 += wgt * (pb1[rn] - x1);
        }
        if (++pw >= PRE_H) pw = 0;
        if (shim_on) { x0 += shim[0][k]; x1 += shim[1][k]; }  // (before the low cut: no DC builds up)
        h0 += rv_hp_a * (x0 - h0);  // low cut (no mud)
        h1 += rv_hp_a * (x1 - h1);
        x0 -= h0;
        x1 -= h1;
        for (int j = 0; j < 4; j++) {
          x0 = ap(x0, j, gj[j]);
          x1 = ap(x1, 4 + j, gj[j]);
        }
        rin[0][k] = x0;
        rin[1][k] = x1;
      }
      pre_w = pw;
      rv_hp_z[0] = h0;
      rv_hp_z[1] = h1;
      for (int j = 0; j < 8; j++) ix[AP_IN_BASE + j] = p[j];
    }

    // Householder: every line into every other, lossless; what goes back
    // into the lines (kept in yb) and the tail out
    float tb[2][HALF];
    float hn = 2.0f / (RL / 2 + RL / 2 * (nl == RL ? hi0 : 0.0f));  // (2 / the lines in it)
    for (int k = 0; k < HALF; k++) {
      float y[RL] = {};
      float ysum = 0.0f;
      const float w = nl == RL ? hi0 + k * hi_step : 0.0f;  // lines 4..7's share
      if (hi_step != 0.0f) hn = 2.0f / (RL / 2 + RL / 2 * w);
      for (int i = 0; i < nl; i++) {
        y[i] = yb[i][k] * (i < RL / 2 ? 1.0f : w);
        ysum += y[i];
      }
      const float hh = ysum * hn;
      for (int i = 0; i < nl; i++)
        yb[i][k] = (y[i] - hh + rin[i & 1][k] * ((i & 2) ? -0.5f : 0.5f)) * RV_Q;
      const float ta = 0.42f * (1.0f + 0.414f * (1.0f - w));  // half the lines: x sqrt 2
      tb[0][k] = ta * (y[0] - y[2] + y[4] - y[6]);
      tb[1][k] = ta * (y[1] - y[3] + y[5] - y[7]);
    }
    for (int i = 0; i < nl; i++) {
      int16_t* line = rv + RV_OFS[i];
      const int cap = RV_CAP[i];
      int wi = rv_wi[i];
      for (int k = 0; k < HALF; k++) {
        line[wi] = SoftClip16(yb[i][k]);
        if (++wi >= cap) wi = 0;
      }
      rv_wi[i] = wi;
    }

    // the tail out is the last step's, halfway to the new one on odd samples
    const float rm = rmix;
    const float rsub = rsend ? 0.0f : ropen;
    float t0 = rv_t[0], t1 = rv_t[1];
    for (int k = 0; k < HALF; k++) {
      for (int h = 0; h < 2; h++) {
        const int n = 2 * k + h;
        float e0 = t0, e1 = t1;
        if (h) {
          e0 = 0.5f * (t0 + tb[0][k]);
          e1 = 0.5f * (t1 + tb[1][k]);
          t0 = tb[0][k];
          t1 = tb[1][k];
        }
        peak_tail = std::max(peak_tail, std::max(fabsf(e0), fabsf(e1)));
        const float a = x1b[0][n], b = x1b[1][n];
        const int16_t o0 = ToQ15(a + rm * (e0 - rsub * a));
        const int16_t o1 = ToQ15(b + rm * (e1 - rsub * b));
        out[0]->data[n] = o0;
        out[1]->data[n] = o1;
        peak_out = std::max(peak_out, std::max(abs(o0), abs(o1)));
      }
    }
    rv_t[0] = t0;
    rv_t[1] = t1;
    for (int k = 0; k < HALF; k++) { sh_tail[0][k] = tb[0][k]; sh_tail[1][k] = tb[1][k]; }
  }
  if (rv_xf) {
    rv_xf -= AUDIO_BLOCK_SAMPLES;
    if (rv_xf <= 0) { rv_xf = 0; rv_sc_a = rv_sc_b; }
  }
  if (pre_xf) {
    pre_xf -= AUDIO_BLOCK_SAMPLES;
    if (pre_xf <= 0) { pre_xf = 0; pre_cur = pre_next; }
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
