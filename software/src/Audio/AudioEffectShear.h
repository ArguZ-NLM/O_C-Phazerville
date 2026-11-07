// SHEAR's engine (Loom, 2026-11-07): an MPC / REX style slicer. It cuts the
// loop a reel plays (SHUTTLE's file in memory, SPOOL's recording: their
// AudioEffectReel, read where it lies, never copied) into slices and plays
// them on Loom's clock, a slice a step, in an order: as they lie, backwards,
// at random or shuffled. Each step may play its slice backwards, roll it
// (2-4 hits in the step) or move its pitch.
//
// Only the slices come out (no dry): the slot's fade mixes them with the
// source by MIX, as FIBRE's grains.
//
// Timing: the main loop hands over where the clock was (in beats) and when
// (micros); the audio update works out from that where each step falls in
// its block, to the sample. A voice holds the slice it plays; a new step
// takes the other voice and the old one fades out (3 ms), so steps never
// click.
//
// The reel's buffer can move under it (SHUTTLE trims its memory to the loop
// when its page is left, or swaps in more): that happens with the audio
// stopped, its window moving along, so reading the buffer and the window in
// the update, every block, is always consistent. The slices are kept as
// fractions of the window.

#pragma once

#include <Arduino.h>
#include <AudioStream.h>
#include "AudioEffectReel.h"

class AudioEffectShear : public AudioStream {
public:
  static constexpr int MAX_SLICES = 32;
  enum Order : uint8_t { STRAIGHT, BACK, RANDOM, SHUFFLE };

  AudioEffectShear() : AudioStream(2, inq) {
    for (int i = 0; i <= MAX_SLICES; i++) cut[i] = float(i) / MAX_SLICES;
    for (int i = 0; i < MAX_SLICES; i++) perm[i] = uint8_t(i);
  }

  // --- set from the main loop
  const AudioEffectReel* volatile reel = nullptr;  // the loop it cuts (nullptr: silent)
  volatile float rate = 1.0f;       // source frames a sample (the file's rate, SHUTTLE's SYNC)
  volatile uint8_t nslices = 16;
  volatile float cut[MAX_SLICES + 1];  // slice starts, fractions of the window (cut[n] = 1)
  volatile uint8_t perm[MAX_SLICES];   // SHUFFLE's order
  volatile uint8_t order = STRAIGHT;
  volatile float step_beats = 0.25f;  // a step (a 1/16: a quarter beat)
  volatile float gate = 1.0f;         // a slice sounds for this share of its step (at most its length)
  volatile float pitch = 1.0f;        // playback ratio
  volatile float spread = 0.0f;       // +- semitones, at random each step
  volatile float p_rev = 0.0f, p_roll = 0.0f;  // odds a step plays backwards / rolls
  volatile uint8_t roll_hits = 2;     // a roll: hits in the step
  volatile bool running = true;       // A: play / stop
  volatile bool rest = false;         // parked
  volatile uint8_t pad = 0xFF;        // a slice by hand (MIDI / B): played at once
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
  volatile float play_frac = 0.0f;    // where in the window the head is (0..1)
  volatile uint32_t steps = 0;

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
    if (!wl || ws >= len) return;

    audio_block_t* out[2] = {allocate(), allocate()};
    if (!out[0] || !out[1]) { for (auto b : out) if (b) release(b); return; }

    // where the steps fall in this block
    int trig_at = -1;
    const ClockSnap& k = clk[clk_i];
    if (k.running && running && k.bpm > 1.0f) {
      const double now = k.beats + double(uint32_t(micros() - k.us)) * (k.bpm / 60e6);
      const double sb = step_beats;
      const int64_t s = int64_t(floor(now / sb));
      const double spb = AUDIO_SAMPLE_RATE_EXACT * 60.0 / k.bpm;  // samples a beat
      if (last_step == INT64_MIN || s < last_step - 1 || s > last_step + 4) {
        last_step = s;  // (started, or the clock jumped: from the next step)
      } else if (s > last_step) {
        last_step = s;  // (late: at once)
        trig_at = 0;
      }
      if (trig_at < 0) {
        const int off = int(((double(last_step + 1) * sb) - now) * spb);
        if (off >= 0 && off < AUDIO_BLOCK_SAMPLES) { trig_at = off; last_step++; }
      }
      step_samples = float(sb * spb);
    }
    const uint8_t pd = pad;
    if (pd != 0xFF) { pad = 0xFF; Start(pd, 0, true); }

    const int n = std::max(1, std::min<int>(nslices, MAX_SLICES));
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      if (i == trig_at) Step(n);
      if (roll_left && --roll_in <= 0) { roll_left--; roll_in = roll_gap; Retrigger(); }
      float l = 0.0f, rr = 0.0f;
      for (auto& v : voice) {
        if (!v.on) continue;
        // the envelope: up in 1 ms, down in 3 ms (released: its gate over, or a new step)
        if (v.rel || v.left <= 0.0f) { v.env -= 1.0f / (0.003f * AUDIO_SAMPLE_RATE_EXACT); if (v.env <= 0.0f) { v.on = false; continue; } }
        else if (v.env < 1.0f) v.env = std::min(1.0f, v.env + 1.0f / (0.001f * AUDIO_SAMPLE_RATE_EXACT));
        const float p = v.rev ? v.span - 1.0f - v.pos : v.pos;
        if (p < 0.0f || p >= v.span - 1.0f) { v.rel = true; v.pos += v.inc; continue; }
        const float a0 = cut[v.slice] * float(wl);  // (from the window as it is now)
        const float at = float(ws) + a0 + p;
        const size_t i0 = std::min(size_t(at), len - 2);
        const float f = at - float(i0);
        l += v.env * (src[0][i0] + (src[0][i0 + 1] - src[0][i0]) * f);
        rr += v.env * (src[1][i0] + (src[1][i0 + 1] - src[1][i0]) * f);
        v.pos += v.inc;
        v.left -= 1.0f;
      }
      out[0]->data[i] = int16_t(constrain(int32_t(l), -32768, 32767));
      out[1]->data[i] = int16_t(constrain(int32_t(rr), -32768, 32767));
    }
    // the picture
    const Voice& v = voice[cur_v];
    if (v.on) {
      playing_slice = int8_t(v.slice);
      const float p = v.rev ? v.span - v.pos : v.pos;
      play_frac = cut[v.slice] + p / float(wl);
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
    float pos = 0.0f, inc = 1.0f, span = 0.0f, env = 0.0f, left = 0.0f;
  };
  Voice voice[2];
  uint8_t cur_v = 0;
  ClockSnap clk[2] = {};
  volatile uint8_t clk_i = 0;
  int64_t last_step = INT64_MIN;
  float step_samples = 0.0f;
  uint32_t rng = 0x9E3779B9u;
  int roll_left = 0;
  float roll_in = 0.0f, roll_gap = 0.0f;
  uint32_t Rand() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
  float Rand01() { return float(Rand() >> 8) * (1.0f / 16777216.0f); }

  // a step: which slice, and how
  void Step(int n) {
    const uint32_t k = steps++;
    int s;
    switch (order) {
      case BACK: s = n - 1 - int(k % uint32_t(n)); break;
      case RANDOM: s = int(Rand() % uint32_t(n)); break;
      case SHUFFLE: s = perm[k % uint32_t(n)] % n; break;
      default: s = int(k % uint32_t(n)); break;
    }
    Start(s, k, false);
    roll_left = 0;
    if (p_roll > 0.0f && Rand01() < p_roll && step_samples > 0.0f) {  // the rest of the step in hits
      const int hits = std::max(2, int(roll_hits));
      roll_gap = step_samples / float(hits);
      roll_in = roll_gap;
      roll_left = hits - 1;
    }
  }

  void Start(int s, uint32_t k, bool by_hand) {
    const int n = std::max(1, std::min<int>(nslices, MAX_SLICES));
    s = constrain(s, 0, n - 1);
    voice[cur_v].rel = true;  // (the old one fades out)
    cur_v ^= 1;
    Voice& v = voice[cur_v];
    v.on = true;
    v.rel = false;
    v.env = 0.0f;
    v.slice = uint8_t(s);
    v.pos = 0.0f;
    v.rev = !by_hand && p_rev > 0.0f && Rand01() < p_rev;
    float ratio = pitch;
    if (spread > 0.0f) ratio *= exp2f((Rand01() * 2.0f - 1.0f) * spread * (1.0f / 12.0f));
    v.inc = rate * ratio;
    size_t ws, wl;
    reel->Window(ws, wl);
    v.span = (cut[s + 1] - cut[s]) * float(wl);  // its length in source frames
    // it sounds for its gate share of the step, at most its own length
    const float own = v.span / std::max(0.01f, v.inc);
    v.left = by_hand || step_samples <= 0.0f ? own : std::min(own, gate * step_samples);
    (void)k;
  }

  void Retrigger() {  // a roll's next hit: the same slice from its start
    Voice& o = voice[cur_v];
    if (!o.on) return;
    const Voice keep = o;
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
