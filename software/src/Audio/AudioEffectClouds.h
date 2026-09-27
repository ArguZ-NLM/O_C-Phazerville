#pragma once

#include "AudioBuffer.h"
#include "../dsputils.h"
#include "../dsputils_arm.h"
#include "../src/extern/stmlib_utils_random.h"
#include <Audio.h>

// Thin subclass of ExtAudioBuffer exposing write position and raw buffer
// access needed by AudioEffectClouds for absolute-position grain reads.
template <typename T = int16_t>
class CloudsCircBuffer : public ExtAudioBuffer<T> {
public:
    using ExtAudioBuffer<T>::ExtAudioBuffer;
    size_t GetWriteIx() const { return this->write_ix; }
    T*     RawBuffer()  const { return this->buffer; }
    bool   IsReady()    const { return this->buffer != nullptr; }
};

// Clouds-inspired live granular processor.
//
// Improvements over AudioEffectMist:
//   Texture   — 0: short fades; →0.5 longer fade-out; →1.0 longer fade-in (Hann)
//               Uses a 256-entry Q15 Hann LUT (512 bytes flash) — no sinf in hot loop.
//   Output    — normalised by the number of overlapping grains, then soft-limited
//   Density   — centred: 0=silence, <0=regular periodic, >0=stochastic cloud
//   Feedback  — fraction of grain output fed back into the record buffer
//
// Hot-loop design:
//   Single grain inner loop (no switch) — texture morph is 6 float ops per sample.
//   Hann via LUT read + one fmul: no transcendental functions in the hot path.
//   Hermite interpolation: 6 fmul + 5 fadd — single-cycle on Cortex-M7 FPU.
//   Feedback and accumulators are member arrays (not stack) to avoid stack pressure.
//   Stereo: each grain reads the left or the right buffer (odds set by the grain
//   source control) and is placed by a random pan scaled by the stereo spread.
class AudioEffectClouds : public AudioStream {
public:
    static const size_t CLOUDS_BUFFER_SAMPLES = AUDIO_SAMPLE_RATE; // ~1 sec
    static const int    MAX_GRAINS = 12;
    static const int    MAX_CHANNELS = 2;

    AudioEffectClouds(size_t buf_len = CLOUDS_BUFFER_SAMPLES, int channels = 1)
        : AudioStream(MAX_CHANNELS, input_queue_array),
          g_buffer{ CloudsCircBuffer<int16_t>(buf_len), CloudsCircBuffer<int16_t>(buf_len) },
          nch_(channels > 1 ? 2 : 1) {}

    void Acquire() { for (int c = 0; c < nch_; c++) g_buffer[c].Acquire(); }
    void Release() { for (int c = 0; c < nch_; c++) g_buffer[c].Release(); }
    bool IsReady() const {
        for (int c = 0; c < nch_; c++) if (!g_buffer[c].IsReady()) return false;
        return true;
    }

    // Setters — called from Controller() at ISR rate (~16.6 kHz).
    // update() snapshots them once per block.
    void setPosition(float p)        { pos_      = p; }
    // density in Hz, signed: 0=silence, neg=periodic, pos=stochastic
    void setDensity(float d)         { density_  = d; }
    void setSize(float secs)         { size_     = secs; }
    void setSpray(float s)           { spray_    = s; }
    void setPitch(float r)           { pitch_    = r; }
    void setPitchSpread(float semis) { psprd_    = semis; }
    // texture ∈ [0, 1]: 0=rect, 0.5=triangle, 1.0=Hann
    void setTexture(float t)         { texture_  = t; }
    // feedback ∈ [0, 1]: fraction of grain cloud fed back into record buffer
    void setFeedback(float f)        { feedback_ = f; }
    void setFreeze(bool f)           { freeze_   = f; }
    // stereo only. Every grain reads one input: source ∈ [−1, +1] sets the odds,
    // −1 = always left, 0 = 50/50, +1 = always right. Placement comes from spread.
    void setGrainSource(float b)     { source_   = b; }
    // stereo only. spread ∈ [0, 1]: random pan per grain (0 = keep the image)
    void setStereoSpread(float s)    { sspread_  = s; }

    uint8_t ActiveGrainCount() const {
        uint8_t n = 0;
        for (const auto& g : grains) if (g.active) n++;
        return n;
    }

    void update() override {
        audio_block_t* in[MAX_CHANNELS]  = {};
        audio_block_t* out[MAX_CHANNELS] = {};
        bool ok = true;
        for (int c = 0; c < nch_; c++) {
            in[c]  = receiveReadOnly(c);
            out[c] = allocate();
            if (!out[c]) ok = false;
        }
        if (!ok) { release_blocks(in, out); return; }

        // Snapshot volatile params once for this block.
        const float  cur_pos      = pos_;
        const float  cur_density  = density_;
        const float  cur_size     = size_;
        const float  cur_spray    = spray_;
        const float  cur_pitch    = pitch_;
        const float  cur_psprd    = psprd_;
        const float  cur_texture  = texture_;
        const float  cur_feedback = feedback_;
        const bool   cur_freeze   = freeze_;
        const size_t buf_size     = g_buffer[0].NumSamples;

        if (!IsReady()) {
            // No PSRAM — pass through unchanged.
            for (int c = 0; c < nch_; c++) {
                if (in[c]) memcpy(out[c]->data, in[c]->data, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
                else       memset(out[c]->data, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
                transmit(out[c], c);
            }
            release_blocks(in, out);
            return;
        }

        int16_t* buf[MAX_CHANNELS] = {};
        for (int c = 0; c < nch_; c++) buf[c] = g_buffer[c].RawBuffer();

        // ── Write incoming audio (+ feedback from previous block) unless frozen ─
        // All channels are written together so their record heads stay in step.
        bool any_in = false;
        for (int c = 0; c < nch_; c++) if (in[c]) any_in = true;
        const bool recorded = !cur_freeze && any_in;
        if (recorded) {
            for (int c = 0; c < nch_; c++) {
                // Mix live input with previous block's feedback, then write.
                int16_t tmp[AUDIO_BLOCK_SAMPLES];
                for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                    int32_t s = in[c] ? (int32_t)in[c]->data[i] : 0;
                    if (cur_feedback > 0.0f) s += Clip16(feedback_buf_[c][i]);
                    tmp[i] = (int16_t)(s > 32767 ? 32767 : (s < -32768 ? -32768 : s));
                }
                g_buffer[c].Write(tmp);
            }
        }

        // ── Pass 1: grain scheduling ───────────────────────────────────────────
        // density=0 → no grains. neg → periodic. pos → stochastic: each sample
        // starts a grain with probability rate/fs (random gaps, same average).
        if (cur_density != 0.0f) {
            const float abs_density = cur_density < 0.0f ? -cur_density : cur_density;
            const bool  stochastic  = cur_density > 0.0f;

            // Cap density so avg concurrent grains stays < MAX_GRAINS.
            const float capped = (abs_density * cur_size < (float)(MAX_GRAINS - 1))
                ? abs_density : (float)(MAX_GRAINS - 1) / cur_size;

            const float advance = capped / AUDIO_SAMPLE_RATE_EXACT;
            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                bool seed;
                if (stochastic) {
                    seed = stmlib::Random::GetFloat() < advance;
                } else {
                    spawn_phase_ += advance;
                    seed = spawn_phase_ >= 1.0f;
                    if (seed) spawn_phase_ -= 1.0f;
                }
                if (seed) {
                    // record head as it was at sample i of this block
                    const size_t head = (g_buffer[0].GetWriteIx() + buf_size
                        - (recorded ? AUDIO_BLOCK_SAMPLES - i : 0)) % buf_size;
                    spawnGrain(cur_pos, cur_size, cur_spray, cur_pitch,
                               cur_psprd, cur_texture, buf_size, head, i);
                }
            }
        }

        // ── Pass 2: grain processing (grain-outer, sample-inner) ───────────────
        //
        // Window — attack and release shaped separately, no sinf:
        //   x = ramp position of the current half (0 at the edge, 1 at the centre)
        //   w = x < len ? half-Hann(x / len) : 1
        //   (1/len per side precomputed at spawn — 0 per-sample cost).
        //
        // Hann: read Q15 LUT, convert with one fmul. No sinf in the hot path.
        memset(accum_, 0, sizeof(accum_));
        const uint8_t active_at_start = ActiveGrainCount();
        const bool stereo = nch_ > 1;

        for (auto& g : grains) {
            if (!g.active) continue;
            // grains start on the sample they were scheduled, not the block start
            const int first = g.pre_delay;
            g.pre_delay = 0;
            float       t      = (float)g.phase * g.inv_grain_len;
            const float t_step = g.inv_grain_len;

            for (int i = first; i < AUDIO_BLOCK_SAMPLES; i++) {
                // ── Window ────────────────────────────────────────────────────
                const bool rising = t < 0.5f;
                const float x = rising ? (2.0f * t) : (2.0f - 2.0f * t);
                const float u = x * (rising ? g.inv_attack : g.inv_release);
                float w = 1.0f;
                if (u < 1.0f) {
                    // half-Hann rise: hann(u/2) = sin²(πu/2)
                    uint8_t lut_idx = (uint8_t)(u * 127.5f + 0.5f);
                    w = (float)hann_lut_[lut_idx] * (1.0f / 32767.0f);
                }
                t += t_step;

                // ── Hermite interpolated read ──────────────────────────────────
                size_t idx  = (size_t)g.read_ptr;
                float  frac = g.read_ptr - (float)idx;
                size_t im1  = (idx == 0)             ? buf_size - 1       : idx - 1;
                size_t i1   = (idx + 1 >= buf_size)  ? 0                  : idx + 1;
                size_t i2   = (idx + 2 >= buf_size)  ? idx + 2 - buf_size : idx + 2;
                if (stereo) {
                    // one input per grain, placed by its pan
                    const int16_t *b = buf[g.source];
                    const float m = InterpHermite((float)b[im1], (float)b[idx],
                                                  (float)b[i1],  (float)b[i2], frac) * w;
                    accum_[0][i] += m * g.gain_l;
                    accum_[1][i] += m * g.gain_r;
                } else {
                    accum_[0][i] += InterpHermite((float)buf[0][im1], (float)buf[0][idx],
                                                  (float)buf[0][i1],  (float)buf[0][i2], frac) * w;
                }

                g.read_ptr += g.pitch;
                if (g.read_ptr >= (float)buf_size) g.read_ptr -= (float)buf_size;
                if (g.read_ptr < 0.0f)             g.read_ptr += (float)buf_size;
                if (++g.phase >= g.grain_len) { g.active = false; break; }
            }
        }

        // ── Pass 3: normalize, soft-limit, capture feedback for next block ─────
        // Gain follows the (smoothed) number of overlapping grains: 1/(n-1)
        // above two grains — twice Clouds' 1/sqrt(n-1) in dB.
        {
            const float n = (float)active_at_start;
            num_grains_ += (n - num_grains_) * (n > num_grains_ ? 0.95f : 0.5f);
        }
        const float gain_target = num_grains_ > 2.0f
            ? 1.0f / (num_grains_ - 1.0f) : 1.0f;
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            gain_ += (gain_target - gain_) * 0.01f;
            for (int c = 0; c < nch_; c++) {
                float x = accum_[c][i] * gain_ * (1.0f / 32768.0f);
                if (x > 3.0f) x = 3.0f;
                if (x < -3.0f) x = -3.0f;
                x = x * (27.0f + x * x) / (27.0f + 9.0f * x * x); // soft limit
                const float scaled = x * 32767.0f;
                out[c]->data[i] = Clip16(scaled);
                feedback_buf_[c][i] = scaled * cur_feedback;
            }
        }

        for (int c = 0; c < nch_; c++) transmit(out[c], c);
        release_blocks(in, out);
    }

private:
    // Overlap normalisation state (audio ISR only).
    float num_grains_ = 0.0f;
    float gain_       = 1.0f;

    // 256-entry Q15 Hann window LUT: round(sin²(π×i/255) × 32767), i = 0..255.
    // const static → placed in .rodata (flash) by the linker. 512 bytes, zero SRAM cost.
    static const int16_t hann_lut_[256];

    struct Grain {
        bool   active        = false;
        float  read_ptr      = 0.0f;
        float  pitch         = 1.0f;
        size_t grain_len     = 0;
        size_t phase         = 0;
        float  inv_grain_len = 0.0f;
        float  inv_attack    = 1.0f; // 1 / fade-in length (fraction of half grain)
        float  inv_release   = 1.0f; // 1 / fade-out length
        int    pre_delay     = 0;    // samples into the first block before starting
        float  gain_l        = 1.0f; // stereo pan gains
        float  gain_r        = 1.0f;
        int8_t source        = 0;    // stereo: 0 = left input, 1 = right input
    } grains[MAX_GRAINS];


    CloudsCircBuffer<int16_t> g_buffer[MAX_CHANNELS];
    audio_block_t* input_queue_array[MAX_CHANNELS];
    const int nch_;

    // Feedback and grain sums — member arrays (not stack) to avoid stack pressure.
    float feedback_buf_[MAX_CHANNELS][AUDIO_BLOCK_SAMPLES] = {};
    float accum_[MAX_CHANNELS][AUDIO_BLOCK_SAMPLES] = {};

    void release_blocks(audio_block_t** in, audio_block_t** out) {
        for (int c = 0; c < MAX_CHANNELS; c++) {
            if (in[c])  release(in[c]);
            if (out[c]) release(out[c]);
        }
    }

    // Grain scheduling state (audio ISR only — not volatile).
    float spawn_phase_ = 0.0f;

    // Volatile params: written from Controller() ISR, read from audio interrupt.
    volatile float pos_      = 0.5f;
    volatile float density_  = 0.0f;  // −1..+1 (0=silence)
    volatile float size_     = 0.1f;  // seconds
    volatile float spray_    = 0.0f;
    volatile float pitch_    = 1.0f;  // ratio
    volatile float psprd_    = 0.0f;  // semitones spread
    volatile float texture_  = 0.5f;  // 0=rect, 0.5=tri, 1.0=Hann
    volatile float feedback_ = 0.0f;  // 0..1
    volatile bool  freeze_   = false;
    volatile float source_   = 0.0f;  // −1..+1 (stereo)
    volatile float sspread_  = 0.0f;  // 0..1 (stereo)

    __attribute__((noinline)) void spawnGrain(float cur_pos, float cur_size, float cur_spray,
                    float cur_pitch, float cur_psprd, float cur_texture,
                    size_t buf_size, size_t head, int pre_delay);
};

// Out of the class body on purpose: under LTO, FLASHMEM is dropped from
// functions defined inside a class, and this one is not time-critical.
FLASHMEM void AudioEffectClouds::spawnGrain(float cur_pos, float cur_size, float cur_spray,
                  float cur_pitch, float cur_psprd, float cur_texture,
                  size_t buf_size, size_t head, int pre_delay) {
    Grain* g = nullptr;
    for (auto& gr : grains) {
        if (!gr.active) { g = &gr; break; }
    }
    if (!g) return; // all slots busy

    // Position scatter — same as Mist.
    float max_spray = cur_pos < 1.0f - cur_pos ? cur_pos : 1.0f - cur_pos;
    float eff_spray = cur_spray < max_spray ? cur_spray : max_spray;
    float scatter   = eff_spray * (stmlib::Random::GetFloat() * 2.0f - 1.0f);
    float eff_pos   = cur_pos + scatter;
    if (eff_pos < 0.0f) eff_pos = 0.0f;
    if (eff_pos > 1.0f) eff_pos = 1.0f;

    float grain_pitch = cur_pitch;
    if (cur_psprd > 0.0f) {
        float rand_semis = cur_psprd * (stmlib::Random::GetFloat() * 2.0f - 1.0f);
        grain_pitch *= SemitonesToRatio(rand_semis);
    }

    float grain_size = cur_size * AUDIO_SAMPLE_RATE_EXACT;
    if (grain_size < (float)AUDIO_BLOCK_SAMPLES) grain_size = AUDIO_BLOCK_SAMPLES;
    if (grain_size > buf_size * 0.5f)            grain_size = buf_size * 0.5f;
    // a fast play head must not run through the whole buffer
    if (grain_pitch > 1.0f && grain_size > buf_size * 0.25f / grain_pitch)
        grain_size = buf_size * 0.25f / grain_pitch;
    const size_t glen = (size_t)grain_size;

    // Start position as in Clouds: the grain's play head and the record
    // head never cross during the grain. pos=0 → ends at the live input,
    // pos=1 → oldest audio that survives the grain.
    const float eaten_by_play_head = grain_size * grain_pitch;
    float available = (float)buf_size - eaten_by_play_head - grain_size;
    if (available < 0.0f) available = 0.0f;
    float rptr = (float)head - (eff_pos * available + eaten_by_play_head);
    while (rptr < 0.0f) rptr += (float)buf_size;

    // Window: texture 0 = short fades both sides (never a hard edge),
    // 0 → 0.5 lengthens only the fade-out, 0.5 → 1 then the fade-in,
    // 1 = full Hann.
    constexpr float kMinFade = 0.02f;
    float rel = cur_texture * 2.0f;          if (rel > 1.0f) rel = 1.0f;
    float att = cur_texture * 2.0f - 1.0f;   if (att < 0.0f) att = 0.0f;
    float release_len = kMinFade + (1.0f - kMinFade) * rel;
    float attack_len  = kMinFade + (1.0f - kMinFade) * att;
    // short grains: keep every fade at least 1.5 ms, or the edges click
    float min_len = (0.0015f * AUDIO_SAMPLE_RATE_EXACT) / (grain_size * 0.5f);
    if (min_len > 1.0f) min_len = 1.0f;
    if (release_len < min_len) release_len = min_len;
    if (attack_len  < min_len) attack_len  = min_len;

    g->read_ptr      = rptr;
    g->pitch         = grain_pitch;
    g->grain_len     = glen;
    g->phase         = 0;
    g->inv_grain_len = 1.0f / (float)(glen - 1);
    g->inv_attack    = 1.0f / attack_len;
    g->inv_release   = 1.0f / release_len;
    g->pre_delay     = pre_delay;
    // stereo spread: random pan per grain (Clouds' formula)
    const float pan = 0.5f + sspread_ * (stmlib::Random::GetFloat() - 0.5f);
    if (pan < 0.5f) { g->gain_l = 1.0f; g->gain_r = 2.0f * pan; }
    else            { g->gain_r = 1.0f; g->gain_l = 2.0f * (1.0f - pan); }
    // grain source: left or right input, odds set by source_ (0 = 50/50)
    const float p_left = 0.5f - 0.5f * source_;
    g->source = (stmlib::Random::GetFloat() < p_left) ? 0 : 1;
    g->active        = true;
}

// ── Hann LUT ──────────────────────────────────────────────────────────────────
// round(sin²(π × i/255) × 32767) for i = 0..255. 512 bytes in .rodata (flash).
const int16_t AudioEffectClouds::hann_lut_[256] = {
       0,     5,    20,    45,    80,   124,   179,   243,
     317,   401,   495,   598,   711,   833,   965,  1106,
    1257,  1416,  1585,  1763,  1949,  2145,  2349,  2561,
    2782,  3011,  3249,  3494,  3747,  4008,  4276,  4552,
    4834,  5124,  5421,  5724,  6034,  6350,  6672,  7000,
    7334,  7673,  8018,  8367,  8722,  9081,  9444,  9812,
   10184, 10559, 10938, 11321, 11706, 12094, 12485, 12879,
   13274, 13671, 14070, 14470, 14872, 15274, 15677, 16081,
   16484, 16888, 17291, 17694, 18096, 18497, 18897, 19295,
   19691, 20085, 20477, 20867, 21254, 21638, 22019, 22396,
   22770, 23139, 23505, 23866, 24223, 24575, 24922, 25264,
   25601, 25932, 26257, 26576, 26889, 27195, 27495, 27789,
   28075, 28354, 28626, 28891, 29148, 29397, 29638, 29871,
   30096, 30313, 30521, 30721, 30912, 31094, 31267, 31432,
   31587, 31732, 31869, 31996, 32114, 32222, 32320, 32409,
   32488, 32557, 32617, 32666, 32706, 32736, 32756, 32766,
   32766, 32756, 32736, 32706, 32666, 32617, 32557, 32488,
   32409, 32320, 32222, 32114, 31996, 31869, 31732, 31587,
   31432, 31267, 31094, 30912, 30721, 30521, 30313, 30096,
   29871, 29638, 29397, 29148, 28891, 28626, 28354, 28075,
   27789, 27495, 27195, 26889, 26576, 26257, 25932, 25601,
   25264, 24922, 24575, 24223, 23866, 23505, 23139, 22770,
   22396, 22019, 21638, 21254, 20867, 20477, 20085, 19691,
   19295, 18897, 18497, 18096, 17694, 17291, 16888, 16484,
   16081, 15677, 15274, 14872, 14470, 14070, 13671, 13274,
   12879, 12485, 12094, 11706, 11321, 10938, 10559, 10184,
    9812,  9444,  9081,  8722,  8367,  8018,  7673,  7334,
    7000,  6672,  6350,  6034,  5724,  5421,  5124,  4834,
    4552,  4276,  4008,  3747,  3494,  3249,  3011,  2782,
    2561,  2349,  2145,  1949,  1763,  1585,  1416,  1257,
    1106,   965,   833,   711,   598,   495,   401,   317,
     243,   179,   124,    80,    45,    20,     5,     0,
};
