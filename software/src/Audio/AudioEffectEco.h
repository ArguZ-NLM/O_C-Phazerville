#pragma once
// ECO: the grain rules of ECHOFIELD (Philip Bos's granular sampler firmware
// for the M-VAVE FM-1, its engine.c, CLOUD mode) played on FIBRE's live
// buffer. Included at the end of AudioEffectClouds.h: the recording, the
// buffer, FREEZE, FDBK, the picture's history and grain view are FIBRE's;
// what a grain does is ECHOFIELD's:
//   - a grain's level set at its birth by how many overlap (0.75 / sqrt of
//     the overlap), not the whole cloud turned down by the grains counted
//     (FIBRE's 1 / (n - 1)): dense clouds stay full
//   - every grain a little detuned (+-0.08 st, more with DRIFT): the chorus
//   - REVRS: odds a grain plays backwards
//   - HARM: odds a grain sounds an octave / fifth away (quieter)
//   - TONE: a one-pole filter on each grain, left darker, right thinner
//   - DRIFT: the place and pitch wander (a random walk) and a slow swing,
//     the random rate breathes
//   - SHAPE: five windows, blended: plucked (0), flat (25), round (50),
//     soft (75), swelling (100)
//   - up to 20 grains at once, the random rate with ECHOFIELD's jitter
// Read with FIBRE's Hermite interpolation from a copy of each grain's span
// (as FIBRE does: PSRAM read in one go). All of it runs from flash
// (FLASHMEM): RAM1 has no room.
//
// ECHOFIELD FM-1: Copyright (c) 2026 Philip Bos, MIT License:
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions: The above copyright notice and this permission
// notice shall be included in all copies or substantial portions of the
// Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

struct AudioEffectClouds::EcoState {
    static const int WIN_N = 256;
    struct G {
        bool    active;
        bool    rev;        // plays backwards
        uint8_t tone;       // 0 none, 1 lowpass, 2 highpass
        uint8_t wa;         // window table, blended with wa + 1 by wmix
        int8_t  source;     // 0 left input, 1 right
        size_t  idx;        // read position: whole samples
        float   frac;       //                + fraction
        size_t  inc_int;
        float   inc_frac;
        size_t  len, phase;
        float   wmix, gl, gr, ta, z;
        int     pre_delay;
        float   scatter, semis;
    } g[ECO_MAX];
    float win[5][WIN_N + 2];
    float spawn_wait, rate_est;
    uint32_t since;          // samples since the last grain
    // DRIFT: walks and swings (ECHOFIELD's walk_pos / walk_pitch / lfo1 / lfo2)
    float walk_pos, walk_v1, walk_pitch, walk_v2, lfo1, lfo2_cur, lfo2_tgt, lfo2_t;
    // this block's settings, for EcoSpawn
    float rev, harm, tone, drift;
};

static inline __attribute__((always_inline)) float EcoRnd() { return stmlib::Random::GetFloat(); }
static inline __attribute__((always_inline)) float EcoRndb() { return stmlib::Random::GetFloat() * 2.0f - 1.0f; }

// ECHOFIELD's windows (ef_window), 0..1 -> 0..1
FLASHMEM static float EcoWindowShape(int w, float x) {
    const float PI_F = 3.14159265f, TWOPI_F = 6.28318531f;
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    switch (w) {
    case 0: {  // plucked: a quick rise, an exponential fall (ECHOFIELD's PERC)
        const float att = 0.03f;
        float v = x < att ? sinf(0.5f * PI_F * x / att) : expf(-5.0f * (x - att) / (1.0f - att));
        if (x > 0.92f) v *= 0.5f + 0.5f * cosf(PI_F * (x - 0.92f) / 0.08f);
        return v;
    }
    case 1: {  // flat: Tukey, 30 % fades
        const float a = 0.3f;
        if (x < a * 0.5f) return 0.5f - 0.5f * cosf(TWOPI_F * x / a);
        if (x > 1.0f - a * 0.5f) return 0.5f - 0.5f * cosf(TWOPI_F * (1.0f - x) / a);
        return 1.0f;
    }
    default:
    case 2:  // round: Hann
        return 0.5f - 0.5f * cosf(TWOPI_F * x);
    case 3: {  // soft: Gauss
        const float d = (x - 0.5f) / 0.16f, e = 0.00752f;
        const float g = (expf(-0.5f * d * d) - e) / (1.0f - e);
        return g < 0.0f ? 0.0f : g > 1.0f ? 1.0f : g;
    }
    case 4:  // swelling: the plucked one backwards
        return EcoWindowShape(0, 1.0f - x);
    }
}

FLASHMEM bool AudioEffectClouds::EcoAcquire() {
    if (eco_) return true;
    // in PSRAM (zeroed): RAM2's heap has next to nothing left with every
    // device in use; each grain's span is read into FIBRE's span_ (unused here)
    EcoState* e = static_cast<EcoState*>(PsramAlloc(sizeof(EcoState)));
    if (!e) return false;
    for (int w = 0; w < 5; w++) {
        for (int i = 0; i <= EcoState::WIN_N; i++) e->win[w][i] = EcoWindowShape(w, (float)i / EcoState::WIN_N);
        e->win[w][EcoState::WIN_N + 1] = 0.0f;
    }
    AudioNoInterrupts();
    eco_ = e;
    for (auto& g : grains) g.active = false;
    AudioInterrupts();
    return true;
}

FLASHMEM void AudioEffectClouds::EcoRelease() {
    AudioNoInterrupts();
    EcoState* e = eco_;
    eco_ = nullptr;
    for (auto& g : grains) g.active = false;
    AudioInterrupts();
    if (e) PsramFree(e);
}

// one window value: table a and the next blended by mix, at t 0..1
static inline __attribute__((always_inline)) float EcoWin(const float* a, const float* b, float mix, float t) {
    float x = t * 256.0f;
    if (x < 0.0f) x = 0.0f;
    int i = (int)x;
    if (i > 255) { i = 256; x = 256.0f; }
    const float f = x - (float)i;
    const float va = a[i] + (a[i + 1] - a[i]) * f;
    const float vb = b[i] + (b[i + 1] - b[i]) * f;
    return va + (vb - va) * mix;
}

__attribute__((noinline)) FLASHMEM void AudioEffectClouds::EcoSpawn(float cur_pos, float cur_size, float cur_spray,
                  float cur_pitch, float cur_psprd, float cur_texture, float rate,
                  size_t buf_size, size_t head, int pre_delay) {
    EcoState& e = *eco_;
    EcoState::G* g = nullptr;
    int active = 0;
    const int limit = eco_limit < ECO_MAX ? eco_limit : ECO_MAX;
    for (int k = 0; k < ECO_MAX; k++) {
        if (e.g[k].active) active++;
        else if (!g && k < limit) g = &e.g[k];
    }
    if (!g || active >= limit) return;

    // the rate the cloud's grains come at (for their level): the setting,
    // or (on the clock, by hand) as they really came
    const float interval = (float)(int32_t)(e.since + (uint32_t)pre_delay);
    if (interval > 0.5f) e.rate_est += 0.3f * (AUDIO_SAMPLE_RATE_EXACT / interval - e.rate_est);
    e.since = (uint32_t)(-pre_delay);  // (counted from this grain's own sample)
    if (rate <= 0.0f) rate = e.rate_est;

    // place: as on FIBRE (the variation never past now or the oldest audio)
    float max_spray = cur_pos < 1.0f - cur_pos ? cur_pos : 1.0f - cur_pos;
    float eff_spray = cur_spray < max_spray ? cur_spray : max_spray;
    float scatter   = eff_spray * EcoRndb();
    float eff_pos   = cur_pos + scatter;
    if (eff_pos < 0.0f) eff_pos = 0.0f;
    if (eff_pos > 1.0f) eff_pos = 1.0f;

    // pitch: the spread, ECHOFIELD's small detune on every grain (more with
    // DRIFT), the walk, sometimes a harmony
    const float D = e.drift;
    float semis = 0.0f, amp = 1.0f;
    if (cur_psprd > 0.0f) semis += cur_psprd * EcoRndb();
    semis += EcoRndb() * (0.08f + 0.3f * D) + e.walk_pitch * D * 2.0f;
    if (EcoRnd() < e.harm * 0.65f) {
        static const float IV[8] = {12, 12, -12, 7, 19, 24, -5, 7};
        semis += IV[stmlib::Random::GetWord() & 7u];
        amp *= 0.7f;
    }
    float grain_pitch = cur_pitch * SemitonesToRatio(semis);
    if (grain_pitch > 4.0f) grain_pitch = 4.0f;          // (its span fits the copy)
    if (grain_pitch < 1.0f / 64.0f) grain_pitch = 1.0f / 64.0f;

    float grain_size = cur_size * AUDIO_SAMPLE_RATE_EXACT;
    const float ss = ssprd_;
    if (ss > 0.0f) grain_size *= 1.0f + ss * EcoRndb();
    if (grain_size < (float)AUDIO_BLOCK_SAMPLES) grain_size = AUDIO_BLOCK_SAMPLES;
    if (grain_size > buf_size * 0.5f)            grain_size = buf_size * 0.5f;
    if (grain_pitch > 1.0f && grain_size > buf_size * 0.25f / grain_pitch)
        grain_size = buf_size * 0.25f / grain_pitch;
    const size_t glen = (size_t)grain_size;

    // where it reads: FIBRE's way, the play head never crossing the record head
    const float eaten = grain_size * grain_pitch;
    float available = (float)buf_size - eaten - grain_size;
    if (available < 0.0f) available = 0.0f;
    const bool rev = EcoRnd() < e.rev;
    double rptr = (double)head - (eff_pos * available + eaten);
    if (rev) rptr += (double)eaten - 3.0;  // backwards: from the far end of the same stretch
    while (rptr < 0.0) rptr += (double)buf_size;
    while (rptr >= (double)buf_size) rptr -= (double)buf_size;

    g->idx       = (size_t)rptr;
    g->frac      = (float)(rptr - (double)g->idx);
    g->inc_int   = (size_t)grain_pitch;
    g->inc_frac  = grain_pitch - (float)g->inc_int;
    g->rev       = rev;
    g->len       = glen;
    g->phase     = 0;
    g->pre_delay = pre_delay;
    // SHAPE: five windows, each blended into the next
    float sh = cur_texture * 4.0f;
    if (sh < 0.0f) sh = 0.0f;
    int wa = (int)sh;
    if (wa > 3) wa = 3;
    g->wa   = (uint8_t)wa;
    g->wmix = sh - (float)wa;
    if (g->wmix > 1.0f) g->wmix = 1.0f;
    // level by the overlap (ECHOFIELD: 0.75 / sqrt), an equal-power pan
    float overlap = rate * (float)glen / AUDIO_SAMPLE_RATE_EXACT;
    if (overlap < 1.0f) overlap = 1.0f;
    const float a = amp * 0.75f / sqrtf(overlap);
    float pan = 0.5f + EcoRndb() * 0.5f * sspread_;
    if (pan < 0.0f) pan = 0.0f;
    if (pan > 1.0f) pan = 1.0f;
    g->gl = a * sqrtf(1.0f - pan) * 1.41421f;
    g->gr = a * sqrtf(pan) * 1.41421f;
    // TONE: left a lowpass (18 kHz .. 360 Hz), right a highpass (150 Hz .. 4.5 kHz)
    const float t = e.tone;
    g->z = 0.0f;
    if (fabsf(t) < 0.02f) {
        g->tone = 0;
        g->ta = 0.0f;
    } else {
        float hz = t < 0.0f ? 18000.0f * powf(0.02f, -t) : 150.0f * powf(30.0f, t);
        if (hz > 20000.0f) hz = 20000.0f;
        g->tone = t < 0.0f ? 1 : 2;
        g->ta = 1.0f - expf(-6.28318531f * hz / AUDIO_SAMPLE_RATE_EXACT);
    }
    const float p_left = 0.5f - 0.5f * source_;
    g->source  = (nch_ > 1 && EcoRnd() >= p_left) ? 1 : 0;
    g->scatter = eff_pos - cur_pos;
    g->semis   = semis;
    g->active  = true;
}

// (noinline: inlined into update() it would run from RAM1)
__attribute__((optimize("O2"), noinline))
FLASHMEM void AudioEffectClouds::UpdateEco(audio_block_t** in, audio_block_t** out) {
    EcoState& e = *eco_;
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
    e.rev = eco_rev_;
    e.harm = eco_harm_;
    e.tone = eco_tone_;
    e.drift = eco_drift_;

    if (!IsReady()) {
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

    // recording: as FIBRE (the live input + the feedback, unless frozen)
    bool any_in = false;
    for (int c = 0; c < nch_; c++) if (in[c]) any_in = true;
    const bool recorded = !cur_freeze && any_in;
    if (recorded) {
        for (int c = 0; c < nch_; c++) {
            int16_t tmp[AUDIO_BLOCK_SAMPLES];
            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                int32_t s = in[c] ? (int32_t)in[c]->data[i] : 0;
                if (cur_feedback > 0.0f) s += Clip16(feedback_buf_[c][i]);
                tmp[i] = SoftClip16(s);
                const int a = tmp[i] < 0 ? -tmp[i] : tmp[i];
                if (a > hist_peak_) hist_peak_ = a;
            }
            g_buffer[c].Write(tmp);
        }
        hist_blocks_ += (float)AUDIO_BLOCK_SAMPLES * HIST / (float)buf_size;
        if (hist_blocks_ >= 1.0f) {
            hist_blocks_ -= 1.0f;
            hist_[hist_pos_] = (uint8_t)(hist_peak_ >= 32640 ? 255 : hist_peak_ >> 7);
            hist_pos_ = (hist_pos_ + 1) % HIST;
            hist_peak_ = 0;
        }
    }

    if (rest) {
        for (auto& g : e.g) g.active = false;
        for (auto& g : grains) g.active = false;
        burst_ = 0;
        memset(feedback_buf_, 0, sizeof(feedback_buf_));
        release_blocks(in, out);
        return;
    }

    // DRIFT: the walks and swings move on (not while frozen: nothing moves)
    const float dt = AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_EXACT, D = e.drift;
    if (!cur_freeze) {
        e.lfo1 += dt * (0.03f + 0.3f * D);
        if (e.lfo1 >= 1.0f) e.lfo1 -= 1.0f;
        e.lfo2_t += dt * (0.1f + 1.5f * D);
        if (e.lfo2_t >= 1.0f) { e.lfo2_t -= 1.0f; e.lfo2_tgt = EcoRndb(); }
        float k = dt * (0.5f + 3.0f * D);
        if (k > 1.0f) k = 1.0f;
        e.lfo2_cur += (e.lfo2_tgt - e.lfo2_cur) * k;
        const float wr = (0.08f + 0.4f * D) * dt;
        e.walk_v1 += EcoRndb() * wr * 2.0f - e.walk_v1 * wr * 2.0f;
        e.walk_pos += e.walk_v1 * wr * 4.0f - e.walk_pos * wr * 0.3f;
        e.walk_pos = e.walk_pos < -1.0f ? -1.0f : e.walk_pos > 1.0f ? 1.0f : e.walk_pos;
        e.walk_v2 += EcoRndb() * wr * 2.0f - e.walk_v2 * wr * 2.0f;
        e.walk_pitch += e.walk_v2 * wr * 4.0f - e.walk_pitch * wr * 0.3f;
        e.walk_pitch = e.walk_pitch < -1.0f ? -1.0f : e.walk_pitch > 1.0f ? 1.0f : e.walk_pitch;
    }
    float pos = cur_pos + D * (0.15f * sinf(6.28318531f * e.lfo1) + 0.3f * e.walk_pos);
    pos = pos < 0.0f ? 0.0f : pos > 1.0f ? 1.0f : pos;

    // when the grains come: P on time, R with ECHOFIELD's jitter (and its
    // breathing with DRIFT)
    float rate = cur_density < 0.0f ? -cur_density : cur_density;
    if (cur_density > 0.0f) rate *= 1.0f + 0.4f * D * e.lfo2_cur;
    if (rate > 0.0f) {
        const float mean = AUDIO_SAMPLE_RATE_EXACT / rate, jitter = cur_density > 0.0f ? 0.8f : 0.0f;
        if (e.spawn_wait > mean) e.spawn_wait = mean;  // (from slow to fast at once)
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            e.spawn_wait -= 1.0f;
            if (e.spawn_wait > 0.0f) continue;
            e.spawn_wait += mean * (1.0f - jitter + jitter * 2.0f * EcoRnd());
            if (e.spawn_wait < 1.0f) e.spawn_wait = 1.0f;
            const size_t head = (g_buffer[0].GetWriteIx() + buf_size
                - (recorded ? AUDIO_BLOCK_SAMPLES - i : 0)) % buf_size;
            EcoSpawn(pos, cur_size, cur_spray, cur_pitch, cur_psprd, cur_texture, rate, buf_size, head, i);
        }
    }
    for (uint8_t b = burst_; b > 0; b--) {  // by hand (B, triggers, the clock)
        const size_t head = (g_buffer[0].GetWriteIx() + buf_size
            - (recorded ? AUDIO_BLOCK_SAMPLES : 0)) % buf_size;
        EcoSpawn(pos, cur_size, cur_spray, cur_pitch, cur_psprd, cur_texture, 0.0f, buf_size, head, 0);
    }
    burst_ = 0;
    e.since += AUDIO_BLOCK_SAMPLES;

    // the grains: each one's stretch copied from PSRAM, read with Hermite,
    // coloured, windowed (every 8 samples, glided between), panned
    memset(accum_, 0, sizeof(accum_));
    const bool stereo = nch_ > 1;
    for (auto& g : e.g) {
        if (!g.active) continue;
        const int first = g.pre_delay;
        g.pre_delay = 0;
        const int steps = std::min<int>(AUDIO_BLOCK_SAMPLES - first, (int)(g.len - g.phase));
        const float inc = (float)g.inc_int + g.inc_frac;
        const int16_t* src = stereo ? buf[g.source] : buf[0];
        int16_t* sp = span_;
        size_t start;
        int count, li;
        if (!g.rev) {
            count = (int)(steps * inc + g.frac) + 4;
            start = g.idx == 0 ? buf_size - 1 : g.idx - 1;
            li = 1;
        } else {
            const int back = (int)(steps * inc) + 3;
            count = back + 3;
            start = (g.idx + buf_size - (size_t)back) % buf_size;
            li = back;
        }
        if (count > SPAN_MAX) count = SPAN_MAX;
        {
            const size_t first_part = std::min<size_t>((size_t)count, buf_size - start);
            memcpy(sp, src + start, first_part * sizeof(int16_t));
            if (first_part < (size_t)count)
                memcpy(sp + first_part, src, (count - first_part) * sizeof(int16_t));
        }
        const float* wa = e.win[g.wa];
        const float* wb = e.win[g.wa + 1];
        const float wmix = g.wmix, inv_len = 1.0f / (float)g.len;
        const float fi = g.inc_frac, gl = g.gl, gr = g.gr, ta = g.ta;
        const size_t ii = g.inc_int, len = g.len;
        const uint8_t tone = g.tone;
        const bool rev = g.rev;
        float frac = g.frac, z = g.z;
        size_t phase = g.phase;
        int i = first;
        while (i < AUDIO_BLOCK_SAMPLES && phase < len) {
            const int n = std::min<int>(std::min(8, AUDIO_BLOCK_SAMPLES - i), (int)(len - phase));
            float w = EcoWin(wa, wb, wmix, (float)phase * inv_len);
            const float dw = (EcoWin(wa, wb, wmix, (float)(phase + n) * inv_len) - w) / n;
            for (int k = 0; k < n; k++, i++) {
                float m = CloudsHermite(sp[li - 1], sp[li], sp[li + 1], sp[li + 2], frac);
                if (tone) {
                    z += (m - z) * ta;
                    m = tone == 1 ? z : m - z;
                }
                m *= w;
                if (stereo) {
                    accum_[0][i] += m * gl;
                    accum_[1][i] += m * gr;
                } else {
                    accum_[0][i] += m * (gl + gr) * 0.5f;
                }
                w += dw;
                if (!rev) {
                    frac += fi;
                    if (frac >= 1.0f) { frac -= 1.0f; li++; }
                    li += (int)ii;
                } else {
                    frac -= fi;
                    if (frac < 0.0f) { frac += 1.0f; li--; }
                    li -= (int)ii;
                }
            }
            phase += n;
        }
        g.frac = frac;
        g.z = z;
        g.phase = phase;
        g.idx = (start + (size_t)li) % buf_size;
        if (phase >= len) g.active = false;
    }

    // out: soft-limited (FIBRE's curve), no turning down by the count (the
    // grains' levels took the overlap into account); the feedback taken
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        for (int c = 0; c < nch_; c++) {
            float x = accum_[c][i] * (1.0f / 32768.0f);
            if (x > 3.0f) x = 3.0f;
            if (x < -3.0f) x = -3.0f;
            x = x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
            const float scaled = x * 32767.0f;
            out[c]->data[i] = Clip16(scaled);
            feedback_buf_[c][i] = scaled * cur_feedback;
        }
    }

    // the picture: the first MAX_GRAINS of them, as FIBRE's grains
    {
        int k = 0;
        for (auto& g : e.g) {
            if (!g.active || k >= MAX_GRAINS) continue;
            Grain& v = grains[k++];
            v.active = true;
            v.scatter = g.scatter;
            v.semis = g.semis;
            v.grain_len = g.len;
            v.phase = g.phase;
        }
        for (; k < MAX_GRAINS; k++) grains[k].active = false;
    }

    for (int c = 0; c < nch_; c++) transmit(out[c], c);
    release_blocks(in, out);
}
