#pragma once

#include <Audio.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>

// Delay + reverb core for the Sculpt VAST device.
//
// The delay lines are two AudioDelayExt streams (buffers in extmem) that are
// patched around this object. Vast builds the signal written into them (input
// plus filtered, diffused feedback), crossfades the taps against the dry
// signal (DELAY), then sends that through a 4-line FDN reverb (REVERB).
// AudioDelayExt's own feedback stays at 0, so the tone filter and the L/R
// cross-feed can sit inside the loop.
//
// Inputs:  0,1 = dry L/R, 2..2+TAPS-1 = L taps, then TAPS R taps
// Outputs: 0,1 = out L/R, 2,3 = delay write L/R
//
// The loop runs through the audio graph, so it is one block longer than the
// tap time. The owner subtracts LOOP_LATENCY_SECS when setting tap times.
class AudioEffectVast : public AudioStream {
public:
  static constexpr int TAPS = 4;
  static constexpr int NUM_INPUTS = 2 + 2 * TAPS;
  static constexpr float LOOP_LATENCY_SECS
    = AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_EXACT;

  // Peak levels (0..1), held until the UI reads them with ReadViz()
  struct Viz {
    float tap[2][TAPS];
    float tail;
    float in;
    float out;
    uint32_t blocks;
  };

  AudioEffectVast() : AudioStream(NUM_INPUTS, input_queue_array) {}

  bool Acquire() {
    if (!mem) {
      mem = static_cast<float*>(calloc(MEM_TOTAL, sizeof(float)));
    }
    Reset();
    return mem != nullptr;
  }

  void Release() {
    float* m = mem;
    mem = nullptr;
    free(m);
  }

  void Reset() {
    if (mem) std::fill(mem, mem + MEM_TOTAL, 0.0f);
    for (auto& i : ix) i = 0;
    for (int c = 0; c < 2; c++) lp_z[c] = hp_z[c] = 0.0f;
    for (auto& z : damp_z) z = 0.0f;
  }

  // --- parameters, set from the UI thread ---
  void feedback(float f) { fb_target = constrain(f, 0.0f, 1.05f); }
  void delay_mix(float m) { dmix_target = constrain(m, 0.0f, 1.0f); }
  void reverb_mix(float m) { rmix_target = constrain(m, 0.0f, 1.0f); }
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
  }

  // 0..1, scales the reverb line lengths
  void size(float s) {
    s = constrain(s, 0.0f, 1.0f);
    for (int i = 0; i < LINES; i++) {
      line_len[i] = static_cast<int>(LINE_LEN[i] * (0.3f + 0.7f * s));
    }
    decay(decay_secs);
  }

  // approximate RT60 of the reverb tail in seconds
  void decay(float secs) {
    decay_secs = constrain(secs, 0.2f, 30.0f);
    for (int i = 0; i < LINES; i++) {
      line_g[i] = powf(10.0f, -3.0f * line_len[i] / (decay_secs * AUDIO_SAMPLE_RATE_EXACT));
    }
  }

  void ReadViz(Viz& out) {
    AudioNoInterrupts();
    out = viz;
    viz = Viz{};
    viz.blocks = out.blocks;
    AudioInterrupts();
  }

  bool ready() const { return mem != nullptr; }

  void update() override;

private:
  static constexpr float TAP_GAIN[TAPS] = {0.30f, 0.40f, 0.55f, 0.80f};
  static constexpr float REV_DAMP = 0.35f;

  // all delay memory lives in one RAM2 allocation
  static constexpr int LINES = 4;
  static constexpr int NUM_AP = 8; // 4 in the feedback path, 4 at reverb input
  static constexpr int AP_FB_BASE = 0;
  static constexpr int AP_IN_BASE = 4;
  static constexpr int LINE_LEN[LINES] = {2371, 2711, 3137, 3539};
  static constexpr int AP_LEN[NUM_AP] = {241, 467, 263, 503, 142, 379, 107, 277};
  static constexpr int LINE_OFS[LINES] = {
    0, 2371, 2371 + 2711, 2371 + 2711 + 3137
  };
  static constexpr int LINES_TOTAL = 2371 + 2711 + 3137 + 3539;
  static constexpr int AP_OFS[NUM_AP] = {
    LINES_TOTAL,
    LINES_TOTAL + 241,
    LINES_TOTAL + 241 + 467,
    LINES_TOTAL + 241 + 467 + 263,
    LINES_TOTAL + 241 + 467 + 263 + 503,
    LINES_TOTAL + 241 + 467 + 263 + 503 + 142,
    LINES_TOTAL + 241 + 467 + 263 + 503 + 142 + 379,
    LINES_TOTAL + 241 + 467 + 263 + 503 + 142 + 379 + 107,
  };
  static constexpr int MEM_TOTAL = AP_OFS[NUM_AP - 1] + 277;

  audio_block_t* input_queue_array[NUM_INPUTS];
  float* mem = nullptr;
  int ix[LINES + NUM_AP] = {};
  int line_len[LINES] = {LINE_LEN[0], LINE_LEN[1], LINE_LEN[2], LINE_LEN[3]};

  volatile float fb_target = 0.0f, dmix_target = 0.4f, rmix_target = 0.3f;
  volatile float spread_target = 0.0f, ap_g = 0.0f;
  volatile float lp_a = 1.0f, hp_a = 0.0f;
  volatile bool freeze = false;
  float line_g[LINES] = {0.8f, 0.8f, 0.8f, 0.8f};
  float decay_secs = 3.0f;

  float fb = 0.0f, dmix = 0.4f, rmix = 0.3f, spread_ = 0.0f, freeze_amt = 0.0f;
  float lp_z[2] = {}, hp_z[2] = {}, damp_z[LINES] = {};

  Viz viz = {};

  static float OnePoleCoeff(float hz) {
    return 1.0f - expf(-2.0f * float(M_PI) * hz / AUDIO_SAMPLE_RATE_EXACT);
  }

  static inline float Sample(const audio_block_t* b, int n) {
    return b ? b->data[n] * (1.0f / 32768.0f) : 0.0f;
  }

  static inline int16_t ToQ15(float x) {
    int32_t v = static_cast<int32_t>(x * 32767.0f);
    return static_cast<int16_t>(__SSAT(v, 16));
  }

  // tanh-like, unity gain near 0, keeps runaway feedback bounded
  static inline float SoftClip(float x) {
    x = constrain(x, -3.0f, 3.0f);
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
  }

  __attribute__((always_inline)) inline float Allpass(float x, int i, float g) {
    float* buf = mem + AP_OFS[i];
    int& p = ix[LINES + i];
    const float d = buf[p];
    const float v = x + g * d;
    buf[p] = v;
    if (++p >= AP_LEN[i]) p = 0;
    return d - g * v;
  }
};

// Runs from flash: RAM1 is full in the T41_audio build. It stays in the
// I-cache while Sculpt is running.
FLASHMEM void AudioEffectVast::update() {
  audio_block_t* in[NUM_INPUTS];
  for (int i = 0; i < NUM_INPUTS; i++) in[i] = receiveReadOnly(i);

  audio_block_t* out[4];
  for (auto& o : out) o = allocate();
  if (!out[0] || !out[1] || !out[2] || !out[3] || !mem) {
    // still keep the delays fed with silence when we can
    for (int i = 0; i < 4; i++) {
      if (out[i]) {
        if (i >= 2) {
          std::fill(out[i]->data, out[i]->data + AUDIO_BLOCK_SAMPLES, 0);
          transmit(out[i], i);
        }
        release(out[i]);
      }
    }
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
  const float damp = REV_DAMP * live;
  float lg[LINES];
  int len[LINES];
  for (int i = 0; i < LINES; i++) {
    lg[i] = line_g[i] + (1.0f - line_g[i]) * frz;
    len[i] = line_len[i];
    if (ix[i] >= len[i]) ix[i] = 0; // SIZE got smaller
  }

  float peak_tap[2][TAPS] = {};
  float peak_tail = 0.0f, peak_in = 0.0f, peak_out = 0.0f;

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    float dry[2], tap[2][TAPS], tapsum[2];
    for (int c = 0; c < 2; c++) {
      dry[c] = Sample(in[c], n);
      peak_in = std::max(peak_in, fabsf(dry[c]));
      tapsum[c] = 0.0f;
      for (int t = 0; t < TAPS; t++) {
        tap[c][t] = Sample(in[2 + c * TAPS + t], n);
        tapsum[c] += TAP_GAIN[t] * tap[c][t];
        peak_tap[c][t] = std::max(peak_tap[c][t], fabsf(tap[c][t]));
      }
    }

    // --- delay write: input + processed feedback ---
    // ping-pong 0 = straight stereo, 1 = mono into L, L<->R in the loop
    const float mono = 0.5f * (dry[0] + dry[1]);
    const float inj[2] = {
      live * (dry[0] + s * (mono - dry[0])),
      live * (dry[1] * (1.0f - s)),
    };
    const float last[2] = {tap[0][TAPS - 1], tap[1][TAPS - 1]};
    const float fbsrc[2] = {
      last[0] + s * (last[1] - last[0]),
      last[1] + s * (last[0] - last[1]),
    };
    for (int c = 0; c < 2; c++) {
      float x = fbsrc[c] * fb;
      lp_z[c] += lp_a * (x - lp_z[c]);
      x = lp_z[c];
      hp_z[c] += hp_a * (x - hp_z[c]);
      x -= hp_z[c];
      x = Allpass(x, AP_FB_BASE + c * 2, g);
      x = Allpass(x, AP_FB_BASE + c * 2 + 1, g);
      x = SoftClip(x);
      // frozen: loop the last tap untouched
      const float w = inj[c] + x * live + fbsrc[c] * frz;
      out[2 + c]->data[n] = ToQ15(w);
    }

    // DELAY: crossfade dry -> echoes
    float x1[2];
    for (int c = 0; c < 2; c++) x1[c] = dry[c] + dmix * (tapsum[c] - dry[c]);

    // --- REVERB: 4-line FDN fed by the delay stage ---
    float rin[2];
    for (int c = 0; c < 2; c++) {
      float x = live * 0.5f * x1[c];
      x = Allpass(x, AP_IN_BASE + c * 2, 0.6f);
      rin[c] = Allpass(x, AP_IN_BASE + c * 2 + 1, 0.6f);
    }
    float y[LINES];
    for (int i = 0; i < LINES; i++) {
      const float d = mem[LINE_OFS[i] + ix[i]];
      damp_z[i] += (1.0f - damp) * (d - damp_z[i]);
      y[i] = damp_z[i] * lg[i];
    }
    // Hadamard mix keeps the loop lossless when frozen
    const float a = y[0] + y[1], b = y[0] - y[1];
    const float cc = y[2] + y[3], dd = y[2] - y[3];
    const float m[LINES] = {
      0.5f * (a + cc), 0.5f * (b + dd), 0.5f * (a - cc), 0.5f * (b - dd)
    };
    const float feed[LINES] = {rin[0], rin[1], rin[0], -rin[1]};
    for (int i = 0; i < LINES; i++) {
      mem[LINE_OFS[i] + ix[i]] = m[i] + feed[i];
      if (++ix[i] >= len[i]) ix[i] = 0;
    }
    const float tail[2] = {0.9f * (y[0] + y[2]), 0.9f * (y[1] - y[3])};

    for (int c = 0; c < 2; c++) {
      peak_tail = std::max(peak_tail, fabsf(tail[c]));
      const float o = x1[c] + rmix * (tail[c] - x1[c]);
      peak_out = std::max(peak_out, fabsf(o));
      out[c]->data[n] = ToQ15(o);
    }
  }

  for (int i = 0; i < 4; i++) {
    transmit(out[i], i);
    release(out[i]);
  }
  for (auto b : in) if (b) release(b);

  for (int c = 0; c < 2; c++)
    for (int t = 0; t < TAPS; t++)
      viz.tap[c][t] = std::max(viz.tap[c][t], peak_tap[c][t]);
  viz.tail = std::max(viz.tail, peak_tail);
  viz.in = std::max(viz.in, peak_in);
  viz.out = std::max(viz.out, peak_out);
  viz.blocks++;
}
