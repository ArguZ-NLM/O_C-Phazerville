#pragma once

#include <Audio.h>
#include "AudioSample.h"
#include <algorithm>
#include <cmath>

// Tuned resonators for the Loom CHIME device.
//
// Six note places, the notes of a scale SPREAD steps apart from NOTE, even
// places on the left, odd on the right. A scale is any set of steps in
// cents (12-note scales in several tunings, or microtonal ones with their
// own steps and period). The input (L + R) goes into the
// resonators of the places that are ON: NOTES of them in a row, starting at
// the rotation. Every resonator is two band-passes in a row on the same
// note (the second one sets the ring; the first keeps what is far from the
// note out of it: an octave away it lets through ~1/18000 where one alone
// lets ~1/600), so only the notes ring, clean.
//
// A hit (a jump in the input level) or STEP (A) moves the rotation (UP,
// DOWN or at random): the input goes to the new places' notes, faded over a
// few ms, and the notes it leaves ring out in their own resonators at their
// own pitch. So there are four times as many resonators as places (a
// change of all six notes leaves six ringing out): a new note takes one
// still ringing on it if there is one, else the quietest free one; a note
// that stays keeps its own. TUNE's
// variation moves every new note by up to +-n steps of the scale,
// DCAY's makes it ring shorter or longer, TONE's gives every hit its own
// tilt. MORPH: how long the input takes to move from the old notes to the
// new ones (a crossfade, the notes seem to slide apart).
class AudioEffectChime : public AudioStream {
public:
  static constexpr int PLACES = 6, VOICES = 24;
  // the CPU is short (Loom sets it): a resonator ringing out stops at
  // -60 dB (else -100 dB)
  static inline volatile bool lite = false;
  enum Rotate : uint8_t { ROT_OFF, ROT_UP, ROT_DOWN, ROT_RAND };

  struct Params {
    int root = 48;           // MIDI note of the first place
    // the scale: its steps in cents over the root, repeating every period
    // (1200: the octave; Bohlen-Pierce: 1902, the twelfth)
    float step[24] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100};
    int steps = 12;
    float period = 1200.0f;
    int spread = 2;          // scale steps between places
    int notes = 6;           // places on at once
    uint8_t rotate = ROT_OFF;
    int step_var = 0;        // scale steps, every new note at random
    float tone = 0.0f;       // -50..50: tilt low <-> high places, and above 0 some of the first pass
    float tone_var = 0.0f;
    float decay = 2.0f;      // s to -60 dB
    float decay_var = 0.0f;  // every new note's decay times 2^(+-this), at random
    float morph = 0.003f;    // s: the input handed from old notes to new ones
    float blend = 0.0f;      // 0..1: the wide first pass heard too (DCAY low: a filter)
    float wet = 1.0f;
  };

  struct Viz {
    float hz[VOICES];     // each resonator's note (0: free)
    float level[VOICES];  // its peak since the last read
    bool fed[VOICES];     // getting the input now
    float place_hz[PLACES];
    uint8_t on;           // bit per place: on now
    float out;
  };

  AudioEffectChime() : AudioStream(2, input_queue_array) {}

  // from the UI thread
  void Set(const Params& p) {
    AudioNoInterrupts();
    par = p;
    retune = true;
    AudioInterrupts();
  }
  void Step() { step_req = true; }  // A, a gate: rotate now
  void Strike() { strike_req = true; }  // B: a mallet on the chimes (they ring without any input)
  void Reset() {
    for (auto& v : vc) v = Voice{};
    for (auto& f : fading) f.g = 0.0f;
    retune = true;
  }
  void ReadViz(Viz& v) {
    AudioNoInterrupts();
    v = viz;
    for (auto& l : viz.level) l = 0.0f;
    viz.out = 0.0f;
    AudioInterrupts();
  }
  void update() override;

private:
  struct Voice {
    float hz = 0.0f, f = 0.0f, da = 0.5f, db = 0.5f, makeup = 0.0f;
    float la = 0.0f, ba = 0.0f, lb = 0.0f, bb = 0.0f;
    float in = 0.0f, in_to = 0.0f;  // input gain, faded
    float pan_l = 1.0f, pan_r = 0.0f, gain = 1.0f;
    float env = 0.0f;
    int place = -1;  // the place it sounds for while fed
    // last block's level and sides (glided to the new ones: a note that
    // stays may move to a place on the other side)
    float mk_was = -1.0f, gain_was = -1.0f, pl_was = -1.0f, pr_was = -1.0f;
  };
  // a ringing resonator taken for a new note hands its ring to one of
  // these, which fades it out over ~5 ms (cut at once it clicked: fast
  // NOTE turns ran out of free resonators)
  struct Fading { Voice v; float g = 0.0f; };
  Fading fading[PLACES];
  audio_block_t* input_queue_array[2];
  Params par;
  Voice vc[VOICES];
  volatile bool step_req = false, strike_req = false;
  bool retune = true;
  int rot = 0;
  float hit_tone = 0.0f;
  int place_var[PLACES] = {};  // this hit's PITCH variation per place
  float place_dec[PLACES] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};  // and DCAY's
  float slow = 0.0f;
  int holdoff = 0;
  // the level control: the wet sum's peak level, and the gain it sets
  float agc_env = 0.0f, agc_g = 1.0f;
  uint32_t rng = 0x2545F491u;
  Viz viz = {};
  float x[AUDIO_BLOCK_SAMPLES], sum[2][AUDIO_BLOCK_SAMPLES];

  float Rand() {  // -1..1
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return int32_t(rng) * (1.0f / 2147483648.0f);
  }
  // scale step k (any integer) from the root, in Hz
  float Degree(int k) const {
    const int n = std::max(1, std::min(24, par.steps));
    const int per = k >= 0 ? k / n : -((-k + n - 1) / n);
    const float cents = per * par.period + par.step[k - per * n];
    return 440.0f * powf(2.0f, (par.root - 69) / 12.0f + cents / 1200.0f);
  }
  float PlaceHz(int p) const {
    return std::min(7000.0f, Degree(p * par.spread + place_var[p]));
  }
  void Tune(Voice& v, float hz, float dec) {
    v.hz = hz;
    v.f = 2.0f * sinf(float(M_PI) * hz / AUDIO_SAMPLE_RATE_EXACT);
    // the second pass rings DECAY long (T60 = 6.9 Q / (pi f)); the first
    // is wide (Q at most 20) and only cleans up
    const float q = std::max(2.0f, par.decay * dec * float(M_PI) * hz / 6.9078f);
    v.db = 1.0f / q;
    v.da = 1.0f / std::min(q, 20.0f);
    // a hit's share of a narrow band shrinks with Q: made up (measured on
    // a plucked note: the ring then peaks about where the hit did)
    v.makeup = 1.5f * sqrtf(q) * powf(q / 60.0f, 0.2f);
  }
  void Assign();
};

// the places that are on get a resonator on their note: the one that
// already has it, or the quietest free one; the rest stop being fed and
// ring out
FASTRUN void AudioEffectChime::Assign() {
  const int n = constrain(par.notes, 1, PLACES);
  bool on[PLACES] = {};
  for (int j = 0; j < n; j++) on[((rot + j) % PLACES + PLACES) % PLACES] = true;
  viz.on = 0;
  for (int p = 0; p < PLACES; p++) {
    viz.place_hz[p] = PlaceHz(p);
    if (on[p]) viz.on |= 1 << p;
  }
  bool keep[VOICES] = {};
  for (int p = 0; p < PLACES; p++) {
    if (!on[p]) continue;
    const float hz = viz.place_hz[p];
    int use = -1;
    for (int i = 0; i < VOICES && use < 0; i++)  // already on that note
      if (!keep[i] && vc[i].in_to > 0.0f && fabsf(vc[i].hz - hz) < hz * 0.001f) use = i;
    for (int i = 0; i < VOICES && use < 0; i++)  // still ringing on that note: on from there
      if (!keep[i] && vc[i].in_to == 0.0f && vc[i].env > 0.0f && fabsf(vc[i].hz - hz) < hz * 0.001f) use = i;
    if (use < 0) {  // the quietest that isn't fed (one still fading out last)
      float lo = 1e9f;
      for (int i = 0; i < VOICES; i++) {
        const float e = vc[i].env + (vc[i].in > 0.0f ? 10.0f : 0.0f);
        if (!keep[i] && vc[i].in_to == 0.0f && e < lo) { lo = e; use = i; }
      }
    }
    if (use < 0) continue;
    Voice& v = vc[use];
    if (fabsf(v.hz - hz) >= hz * 0.001f) {  // a new note: from silence
      if (v.env > 1e-4f) {  // still ringing: faded out, not cut
        Fading* to = &fading[0];
        for (auto& f : fading) if (f.g < to->g) to = &f;
        to->v = v;
        to->v.in_to = 0.0f;
        to->g = 1.0f;
      }
      v.la = v.ba = v.lb = v.bb = 0.0f;
      v.mk_was = v.gain_was = v.pl_was = v.pr_was = -1.0f;
      v.env = 0.0f;
      Tune(v, hz, place_dec[p]);
    } else {
      Tune(v, hz, place_dec[p]);  // DECAY may have changed
    }
    v.place = p;
    v.pan_l = (p & 1) ? 0.35f : 1.0f;
    v.pan_r = (p & 1) ? 1.0f : 0.35f;
    // TONE: the higher places louder (or softer), +-12 dB across
    const float t = (par.tone + hit_tone) * 0.02f;
    v.gain = powf(4.0f, t * (p - 2.5f) / 2.5f);
    v.in_to = 1.0f;
    keep[use] = true;
  }
  for (int i = 0; i < VOICES; i++) if (!keep[i]) vc[i].in_to = 0.0f;
}

FASTRUN __attribute__((optimize("O2"))) void AudioEffectChime::update() {  // (O2: the build is -Os)
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }
  constexpr float q15 = 1.0f / 32768.0f;
  float pk_in = 0.0f;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const float dl = in[0] ? in[0]->data[n] * q15 : 0.0f;
    const float dr = in[1] ? in[1]->data[n] * q15 : 0.0f;
    x[n] = 0.5f * (dl + dr);
    pk_in = std::max(pk_in, fabsf(x[n]));
  }
  // struck (below, each note the six are on rung at once), a hit like any other
  const bool strike = strike_req;
  strike_req = false;

  // a hit: the level jumps well over where it has been (at most one every
  // ~50 ms); with ROTATE it moves the places on, and it gets its own
  // variation
  bool hit = false;
  if (holdoff > 0) holdoff--;
  if (pk_in > 0.01f && pk_in > 2.0f * slow && holdoff == 0) { hit = true; holdoff = 18; }
  if (strike) hit = true;
  slow += (pk_in > slow ? 0.3f : 0.03f) * (pk_in - slow);
  const bool step = step_req;
  step_req = false;
  if ((hit && par.rotate != ROT_OFF) || step) {
    switch (par.rotate) {
      case ROT_DOWN: rot--; break;
      case ROT_RAND: {
        const int was = rot;
        while (PLACES > 1 && rot == was) rot = int((Rand() + 1.0f) * 0.5f * PLACES) % PLACES;
      } break;
      default: rot++; break;  // UP (and STEP while ROTATE is off)
    }
    rot = (rot % PLACES + PLACES) % PLACES;
  }
  if (hit || step) {
    for (int p = 0; p < PLACES; p++) {
      place_var[p] = par.step_var ? int(roundf(Rand() * par.step_var)) : 0;
      place_dec[p] = par.decay_var > 0.0f ? powf(2.0f, Rand() * par.decay_var) : 1.0f;
    }
    hit_tone = Rand() * par.tone_var;
    retune = true;
  }
  if (retune) {
    retune = false;
    if (!par.step_var) for (auto& v : place_var) v = 0;
    if (par.decay_var <= 0.0f) for (auto& d : place_dec) d = 1.0f;
    if (par.tone_var <= 0.0f) hit_tone = 0.0f;
    Assign();
  }

  // the resonators: the input handed over from old notes to new ones over
  // MORPH (3 ms .. a few s)
  memset(sum, 0, sizeof(sum));
  const float fade = 1.0f / (std::max(0.003f, par.morph) * AUDIO_SAMPLE_RATE_EXACT);
  // the first pass heard as it is: DCAY low (a filter bank), and TONE above
  // 0 (breathier); at its own level, not the ring's make-up
  const float first = par.blend + std::max(0.0f, par.tone + hit_tone) * 0.006f;
  for (int i = 0; i < VOICES; i++) {
    Voice& v = vc[i];
    if (v.hz <= 0.0f) continue;
    const bool quiet = v.in == 0.0f && v.in_to == 0.0f && v.env < (lite ? 1e-3f : 1e-5f);
    if (quiet) { v.la = v.ba = v.lb = v.bb = 0.0f; viz.fed[i] = false; continue; }
    float la = v.la, ba = v.ba, lb = v.lb, bb = v.bb, g = v.in;
    const float f = v.f, da = v.da, db = v.db, to = v.in_to;
    // a new DCAY / TONE on a ringing note: its level glides over the block
    const float mk_to = v.makeup * v.gain;
    float mk = v.mk_was < 0.0f ? mk_to : v.mk_was, gn = v.gain_was < 0.0f ? v.gain : v.gain_was;
    const float mk_d = (mk_to - mk) * (1.0f / AUDIO_BLOCK_SAMPLES), gn_d = (v.gain - gn) * (1.0f / AUDIO_BLOCK_SAMPLES);
    float pl = v.pl_was < 0.0f ? v.pan_l : v.pl_was, pr = v.pr_was < 0.0f ? v.pan_r : v.pr_was;
    const float pl_d = (v.pan_l - pl) * (1.0f / AUDIO_BLOCK_SAMPLES), pr_d = (v.pan_r - pr) * (1.0f / AUDIO_BLOCK_SAMPLES);
    // struck: each fed note rung straight to about a quarter of full
    // level, however long its DCAY (a mallet, not the input's slow build)
    if (strike && to > 0.0f) bb += (bb >= 0.0f ? 1.0f : -1.0f) * 0.25f / std::max(1e-4f, db * mk_to);
    v.mk_was = mk_to;
    v.gain_was = v.gain;
    v.pl_was = v.pan_l;
    v.pr_was = v.pan_r;
    float pk = 0.0f;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      g += std::max(-fade, std::min(fade, to - g));
      mk += mk_d;
      gn += gn_d;
      pl += pl_d;
      pr += pr_d;
      la += f * ba;
      ba += f * (g * x[n] - la - da * ba);
      const float ya = ba * da;
      lb += f * bb;
      bb += f * (ya - lb - db * bb);
      const float y = bb * db * mk + first * ya * gn;
      pk = std::max(pk, fabsf(y));
      sum[0][n] += y * pl;
      sum[1][n] += y * pr;
    }
    v.la = la; v.ba = ba; v.lb = lb; v.bb = bb; v.in = g;
    v.env = std::max(pk, v.env * 0.9f);
    viz.hz[i] = v.hz;
    viz.level[i] = std::max(viz.level[i], pk);
    viz.fed[i] = v.in_to > 0.0f;
  }

  // rings handed over by resonators taken for new notes: out over ~5 ms
  constexpr float FADE_OUT = 1.0f / (0.005f * AUDIO_SAMPLE_RATE_EXACT);
  for (auto& fv : fading) {
    if (fv.g <= 0.0f) continue;
    Voice& v = fv.v;
    float la = v.la, ba = v.ba, lb = v.lb, bb = v.bb, g = v.in, fg = fv.g;
    const float f = v.f, da = v.da, db = v.db, mk = v.makeup * v.gain;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES && fg > 0.0f; n++) {
      g = std::max(0.0f, g - fade);
      la += f * ba;
      ba += f * (g * x[n] - la - da * ba);
      const float ya = ba * da;
      lb += f * bb;
      bb += f * (ya - lb - db * bb);
      const float y = (bb * db * mk + first * ya * v.gain) * fg;
      fg -= FADE_OUT;
      sum[0][n] += y * v.pan_l;
      sum[1][n] += y * v.pan_r;
    }
    v.la = la; v.ba = ba; v.lb = lb; v.bb = bb; v.in = g;
    fv.g = std::max(0.0f, fg);
  }

  // the level control: a held input builds the rings up far past a pluck
  // (the make-up is for plucks), so the sum is turned down smoothly when
  // its peaks go over ~-6 dB (down within ~1 ms, back up over ~0.3 s);
  // then a soft limit for what gets past, then dry / wet
  constexpr float AGC_TARGET = 0.5f;
  const float env_fall = 0.9995f, g_down = 0.02f, g_up = 0.00007f;
  const float wet = par.wet;
  float peak_out = 0.0f;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const float dl = in[0] ? in[0]->data[n] * q15 : 0.0f;
    const float dr = in[1] ? in[1]->data[n] * q15 : 0.0f;
    const float a = std::max(fabsf(sum[0][n]), fabsf(sum[1][n]));
    agc_env = a > agc_env ? a : agc_env * env_fall;
    const float want = agc_env > AGC_TARGET ? AGC_TARGET / agc_env : 1.0f;
    agc_g += (want < agc_g ? g_down : g_up) * (want - agc_g);
    const float xl = sum[0][n] * agc_g, xr = sum[1][n] * agc_g;
    const float sl = SoftLimit(xl);
    const float sr = SoftLimit(xr);
    const float ol = dl + wet * (sl - dl), orr = dr + wet * (sr - dr);
    peak_out = std::max(peak_out, std::max(fabsf(ol), fabsf(orr)));
    out[0]->data[n] = ToSample16(ol);
    out[1]->data[n] = ToSample16(orr);
  }
  transmit(out[0], 0);
  transmit(out[1], 1);
  for (auto b : out) release(b);
  for (auto b : in) if (b) release(b);
  viz.out = std::max(viz.out, peak_out);
}
