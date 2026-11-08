// SHEAR's engine (Loom, 2026-11-07): an MPC / REX style slicer. It cuts the
// loop a reel plays (SHUTTLE's file in memory, SPOOL's recording: their
// AudioEffectReel, read where it lies, never copied) into slices and plays
// them on Loom's clock.
//
// The grid (Sascha, 2026-11-07): the loop is a grid of G equal steps (8, 16
// or 32: the fewest that hold the slices); the slices always start on its
// lines. 8, 16, 32 slices: every line. Any other number: the lines spread
// over the grid as a Euclidean rhythm (or the loudest lines, HITS), so some
// slices are longer. A slice lasts as many steps as it is long: played in
// order, the whole loop comes out as it is (at PITCH 0; another pitch plays
// each slice faster or slower, with gaps or cut short). Each time round the
// grid starts again from the top, on the clock.
//
// Only the slices come out (no dry): the slot's fade mixes them with the
// source by MIX, as FIBRE's grains.
//
// Timing (Sascha 2026-11-08: mixed with the dry it must stay in step): while
// the source plays forwards, the grid rides on its own play head (the
// reel's position at the start of this very block), so at PITCH 0 in FWD
// the slices are the dry, sample for sample. Stopped (or backwards,
// PING), the grid runs on Loom's clock instead: the main loop hands over
// where the clock was (in beats) and when (micros). Either way the update
// works out where each step falls in its block, to the sample. RATE
// (factor) fires the steps that many times as often (x2: each slice cut
// to half, the order twice round a loop; /2: once round in two loops). A voice holds the slice it plays; a new slice
// takes the other voice and the old one fades out (3 ms): no clicks.
//
// The reel's buffer can move under it (SHUTTLE trims its memory to the loop
// when its page is left, or swaps in more): that happens with the audio
// stopped, its window moving along, so reading the buffer and the window in
// the update, every block, is always consistent. The slices are kept as grid
// steps, read as fractions of the window.

#pragma once

#include <Arduino.h>
#include <AudioStream.h>
#include "AudioEffectReel.h"

class AudioEffectShear : public AudioStream {
public:
  static constexpr int MAX_SLICES = 32;
  enum Order : uint8_t { FWD, BACK, RANDOM, KEYS };

  AudioEffectShear() : AudioStream(2, inq) {
    for (int i = 0; i <= MAX_SLICES; i++) start[i] = uint8_t(i);
    for (int i = 0; i < MAX_SLICES; i++) perm[i] = uint8_t(i);
  }

  // --- set from the main loop (Set: the cuts, with the audio stopped)
  const AudioEffectReel* volatile reel = nullptr;  // the loop it cuts (nullptr: silent)
  volatile float rate = 1.0f;       // source frames a sample (the file's rate, SHUTTLE's SYNC)
  volatile uint8_t nslices = 16, grid = 16;
  volatile uint8_t start[MAX_SLICES + 1];  // each slice's first grid step (start[n] = grid)
  volatile uint8_t perm[MAX_SLICES];       // the order the slices come in (shuffled by SHUF)
  volatile uint8_t order = FWD;
  volatile float factor = 1.0f;       // RATE: steps this many times as often as the loop's grid
  volatile float step_beats = 0.25f;  // on the clock: a grid step, in beats (RATE included)
  volatile float gate = 1.0f;         // a slice sounds for this share of its steps (at most its length)
  volatile float pitch = 1.0f;        // playback ratio
  volatile float spread = 0.0f;       // +- semitones, at random each slice
  volatile float p_rev = 0.0f, p_roll = 0.0f;  // odds a slice plays backwards / rolls
  volatile uint8_t roll_hits = 2;     // a roll: hits in the slice's time
  volatile bool running = true;       // A: play / stop
  volatile bool rest = false;         // parked
  // KEYS: a slice by hand (MIDI), its level 0..1 (played in the next update)
  void Pad(int s, float lvl) { pad_lvl = lvl; asm volatile("" ::: "memory"); pad = uint8_t(s); }
  // the clock, as the main loop last saw it (two copies: the update reads
  // the one not being written)
  struct ClockSnap { double beats; uint32_t us; float bpm; bool running; };
  void SetClock(double beats, uint32_t us, float bpm, bool run) {
    const uint8_t w = clk_i ^ 1;
    clk[w] = {beats, us, bpm, run};
    asm volatile("" ::: "memory");
    clk_i = w;
  }
  // --- read by the main loop (the picture)
  volatile int8_t playing_slice = -1;
  volatile uint32_t n_steps = 0, n_pads = 0;  // (USB status)
  volatile float play_frac = 0.0f;    // where in the window the head is (0..1)
  volatile bool following = false;    // on the source's head (else the clock)

  void update() override {
    for (int c = 0; c < 2; c++) if (audio_block_t* b = receiveReadOnly(c)) release(b);  // (no dry: the slot's fade has it)
    const AudioEffectReel* r = reel;
    const int16_t* src[2] = {r ? r->Data(0) : nullptr, r ? r->Data(1) : nullptr};
    const AudioEffectReel::State st = r ? r->state() : AudioEffectReel::EMPTY;
    const bool usable = src[0] && src[1] && (st == AudioEffectReel::PLAY || st == AudioEffectReel::STOPPED || st == AudioEffectReel::DUB);
    if (rest || !usable) { voice[0].on = voice[1].on = false; last_step = INT64_MIN; playing_slice = -1; return; }
    size_t ws, wl;
    r->Window(ws, wl);
    const size_t len = r->Length();
    if (wl < 2 || len < 2 || ws >= len) return;

    audio_block_t* out[2] = {allocate(), allocate()};
    if (!out[0] || !out[1]) { for (auto b : out) if (b) release(b); return; }

    // where the grid's steps fall in this block: now (in steps) and how
    // far a sample goes (in steps), on the source's head or on the clock
    int trig_at = -1;
    int64_t trig_k = 0;
    const int G = std::max<int>(1, grid);
    float h0 = 0.0f, hstep = 0.0f;
    const bool fol = order != KEYS && running && r->BlockHead(h0, hstep);
    double now = 0.0, dps = 0.0;
    bool timed = false;
    if (fol) {
      const float f = h0 - floorf(h0);  // (past 1 in the seam: the next time round already)
      if (!following) loops = 0;
      else if (f < last_frac - 0.5f) loops++;  // round again (or back to the start)
      last_frac = f;
      const double per = double(G) * factor;  // steps a time round
      now = (double(loops) + f) * per;
      dps = double(hstep) * per;
      head_rate = hstep * float(wl);  // (frames a sample: the voices play at the head's own speed)
      timed = dps > 1e-9;
    } else {
      const ClockSnap& k = clk[clk_i];
      if (k.running && running && order != KEYS && k.bpm > 1.0f) {
        const double beats = k.beats + double(uint32_t(micros() - k.us)) * (k.bpm / 60e6);
        const double sb = step_beats;
        const double spb = AUDIO_SAMPLE_RATE_EXACT * 60.0 / k.bpm;  // samples a beat
        now = beats / sb;
        dps = 1.0 / (sb * spb);
        head_rate = rate;
        timed = sb > 1e-6;
      }
    }
    if (fol != following) last_step = INT64_MIN;  // (from the clock to the head or back: start afresh)
    following = fol;
    if (timed) {
      step_samples = float(1.0 / dps);
      const int64_t s = int64_t(floor(now));
      if (last_step == INT64_MIN || s < last_step - 1 || s > last_step + 4) {
        // (started, or it jumped: this step if it has only just begun)
        last_step = s;
        if ((now - double(s)) / dps < 2.0 * AUDIO_BLOCK_SAMPLES) { trig_at = 0; trig_k = s; }
        cyc_pos = 0; next_at = -1;
      } else if (s > last_step) {
        last_step = s;  // (late: at once)
        trig_at = 0; trig_k = s;
      }
      if (trig_at < 0) {
        const double off = ceil((double(last_step + 1) - now) / dps);
        if (off >= 0.0 && off < AUDIO_BLOCK_SAMPLES) { trig_at = int(off); trig_k = ++last_step; }
      }
    } else last_step = INT64_MIN;
    const uint8_t pd = pad;
    if (pd != 0xFF) { pad = 0xFF; n_pads = n_pads + 1; Start(pd, -1, pad_lvl); }

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      if (i == trig_at) GridStep(trig_k);
      if (roll_left && --roll_in <= 0) { roll_left--; roll_in = roll_gap; Retrigger(); }
      float l = 0.0f, rr = 0.0f;
      for (auto& v : voice) {
        if (!v.on) continue;
        // the envelope: up in 1 ms, down in 3 ms (released: its time over, or a new slice)
        if (v.rel || v.left <= 0.0f) { v.env -= 1.0f / (0.003f * AUDIO_SAMPLE_RATE_EXACT); if (v.env <= 0.0f) { v.on = false; continue; } }
        else if (v.env < 1.0f) v.env = std::min(1.0f, v.env + 1.0f / (0.001f * AUDIO_SAMPLE_RATE_EXACT));
        // at its end it fades out (3 ms), reading on past it so it doesn't click
        // (backwards its end is its loudest: the hit)
        if (v.pos >= v.span - 1.0f) v.rel = true;
        const float p = v.rev ? v.span - 1.0f - v.pos : v.pos;
        const float at = std::max(0.0f, float(ws) + float(start[v.slice]) * float(wl) / float(grid) + p);  // (the window as it is now)
        const size_t i0 = std::min(size_t(at), len - 2);
        const float f = at - float(i0);
        const float g = v.env * v.amp;
        l += g * (src[0][i0] + (src[0][i0 + 1] - src[0][i0]) * f);
        rr += g * (src[1][i0] + (src[1][i0 + 1] - src[1][i0]) * f);
        v.pos += v.inc;
        v.left -= 1.0f;
      }
      out[0]->data[i] = int16_t(constrain(int32_t(l), -32768, 32767));
      out[1]->data[i] = int16_t(constrain(int32_t(rr), -32768, 32767));
    }
    // the picture
    const Voice& v = voice[cur_v];
    if (v.on && !v.rel) {
      playing_slice = int8_t(v.slice);
      const float p = v.rev ? v.span - v.pos : v.pos;
      play_frac = float(start[v.slice]) / float(grid) + p / float(wl);
    } else playing_slice = -1;
    transmit(out[0], 0);
    transmit(out[1], 1);
    release(out[0]);
    release(out[1]);
  }

private:
  audio_block_t* inq[2];
  struct Voice {
    bool on = false, rev = false, rel = false;
    uint8_t slice = 0;
    float pos = 0.0f, inc = 1.0f, span = 0.0f, env = 0.0f, left = 0.0f, amp = 1.0f;
  };
  Voice voice[2];
  uint8_t cur_v = 0;
  ClockSnap clk[2] = {};
  volatile uint8_t clk_i = 0;
  volatile uint8_t pad = 0xFF;
  volatile float pad_lvl = 1.0f;
  int64_t last_step = INT64_MIN;
  float step_samples = 0.0f;
  float head_rate = 1.0f;   // source frames a sample the voices play at (PITCH on top)
  uint32_t loops = 0;       // times round the loop, following its head
  float last_frac = 0.0f;
  int cyc_pos = 0;       // the next slice in the order, this time round
  int64_t next_at = -1;  // the grid step it starts on (-1: from the top)
  uint32_t rng = 0x9E3779B9u;
  int roll_left = 0;
  float roll_in = 0.0f, roll_gap = 0.0f;
  uint32_t Rand() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
  float Rand01() { return float(Rand() >> 8) * (1.0f / 16777216.0f); }
  int N() const { return std::max(1, std::min<int>(nslices, MAX_SLICES)); }
  int Units(int s) const { return std::max(1, int(start[s + 1]) - int(start[s])); }

  // a grid step k (counted on the clock): the top of the grid starts the
  // order again; a slice starts when the last one's steps are over
  void GridStep(int64_t k) {
    const int G = std::max<int>(1, grid);
    const int g = int(((k % G) + G) % G);
    if (g == 0 || next_at < 0) { cyc_pos = 0; next_at = k - g; }
    if (k < next_at) return;
    const int n = N();
    int s;
    if (order == RANDOM) s = int(Rand() % uint32_t(n));
    else {
      if (cyc_pos >= n) return;  // (all played: the rest of the grid quiet)
      const int at = order == BACK ? n - 1 - cyc_pos : cyc_pos;
      s = perm[at] % n;
      cyc_pos++;
    }
    const int u = Units(s);
    next_at = k + u;
    n_steps = n_steps + 1;
    Start(s, u, 1.0f);
    roll_left = 0;
    if (p_roll > 0.0f && Rand01() < p_roll && step_samples > 0.0f) {  // its time in hits
      const int hits = std::max(2, int(roll_hits));
      roll_gap = step_samples * float(u) / float(hits);
      roll_in = roll_gap;
      roll_left = hits - 1;
    }
  }

  // slice s, for u grid steps (-1: by hand, all of it)
  void Start(int s, int u, float lvl) {
    s = constrain(s, 0, N() - 1);
    const bool rev = p_rev > 0.0f && Rand01() < p_rev;
    float ratio = pitch;
    if (spread > 0.0f) ratio *= exp2f((Rand01() * 2.0f - 1.0f) * spread * (1.0f / 12.0f));
    const float inc = head_rate * ratio;
    size_t ws, wl;
    reel->Window(ws, wl);
    const float step = float(wl) / float(std::max<int>(1, grid));  // a grid step, in source frames
    const float span = float(Units(s)) * step;  // its length in source frames
    // it sounds for its gate share of its steps, at most its own length
    const float own = span / std::max(0.01f, inc);
    const float left = u < 0 || step_samples <= 0.0f ? own : std::min(own, gate * float(u) * step_samples);
    // the slice just where the playing one has got to (in order, forwards,
    // same speed): the same voice plays on, no fade (FWD at PITCH 0 is the
    // dry itself)
    Voice& o = voice[cur_v];
    if (u >= 0 && o.on && !o.rel && !o.rev && !rev && o.left > 0.0f && fabsf(o.inc - inc) < 1e-5f) {
      const float d = float(start[o.slice]) * step + o.pos - float(start[s]) * step;
      if (fabsf(d) <= inc + 2.0f) { o.slice = uint8_t(s); o.pos = d; o.span = span; o.left = left; o.amp = lvl; return; }
    }
    o.rel = true;  // (the old one fades out)
    cur_v ^= 1;
    Voice& v = voice[cur_v];
    v.on = true;
    v.rel = false;
    v.env = 0.0f;
    v.amp = lvl;
    v.slice = uint8_t(s);
    v.pos = 0.0f;
    v.rev = rev;
    v.inc = inc;
    v.span = span;
    v.left = left;
  }

  void Retrigger() {  // a roll's next hit: the same slice from its start
    const Voice keep = voice[cur_v];
    if (!keep.on) return;
    voice[cur_v].rel = true;
    cur_v ^= 1;
    Voice& v = voice[cur_v];
    v = keep;
    v.on = true;
    v.rel = false;
    v.env = 0.0f;
    v.pos = 0.0f;
    v.left = std::min(v.span / std::max(0.01f, v.inc), roll_gap * gate);
  }
};
