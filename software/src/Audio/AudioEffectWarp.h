#pragma once

#include <Arduino.h>
#include <AudioStream.h>
#include "AudioSample.h"

// Eight voices playing one sample (held in PSRAM) at the pitch of the notes
// played, for the Loom WARP device.
//
// A note starts a voice at START, pitched against the root (there the
// sample plays as recorded), and plays to END: once (LOOP OFF), looping
// START .. END, or back and forth (PING). LOOP's seam as SPOOL's FADE:
// CROSS blends what is around END into what is around START (where the
// sample has nothing beyond them: its last frames into the loop's first),
// INOUT dips out and back in. ATTACK fades it in; a note off lets it go over RELEASE.
// A ninth note takes the oldest voice, which hands its sound to a fader
// that lets it go over ~5 ms (cut, it clicked).
//
// The main loop sends notes through a small queue; the audio interrupt
// takes them at the start of each block. Whatever comes in (THREAD: track
// A's output) passes through under the voices.
class AudioEffectWarp : public AudioStream {
public:
  static constexpr int VOICES = 8;
  static constexpr int FADERS = 4;  // voices taken for new notes, fading out
  enum Loop : uint8_t { ONCE, LOOP, PING };

  struct Params {
    float start = 0.0f, end = 1.0f;  // 0..1 of the sample
    uint8_t loop = ONCE;
    float xfade = 0.01f;     // s: LOOP's seam
    bool cross = true;       // CROSS, else INOUT (a dip)
    float attack = 0.002f;   // s
    float decay = 0.1f;      // s, to sustain (-60 dB of the way)
    float sustain = 1.0f;    // 0..1
    float release = 0.2f;    // s, to -60 dB
    float level = 1.0f;      // 1: a note at full velocity as loud as the sample
    float vel_amt = 0.75f;   // 0: every note as loud; 1: velocity sets it all
    int root = 60;
    float rate = 1.0f;       // the file's sample rate / 48 kHz
  };

  // what the picture shows
  struct Viz {
    float pos[VOICES];   // 0..1 of the sample (-1: free)
    float env[VOICES];
    uint8_t note[VOICES];
  };

  AudioEffectWarp() : AudioStream(2, inq) {}

  // main loop: a new sample (nullptr / 0: none). All voices stop, unless
  // keep (the same sound again, recorded over: they play on in the new one)
  void SetSample(const int16_t* l, const int16_t* r, uint32_t frames, bool keep = false) {
    AudioNoInterrupts();
    sl = l;
    sr = r ? r : l;
    nframes = l ? frames : 0;
    for (auto& v : vc) {
      if (!keep || !nframes) v.on = false;
      else if (v.pos >= nframes - 1) v.pos = 0.0;
    }
    if (!keep) ev_rd = ev_wr;
    AudioInterrupts();
  }
  void Set(const Params& p) {
    AudioNoInterrupts();
    par = p;
    AudioInterrupts();
  }
  // main loop: notes for the audio (vel 0: note off)
  void NoteOn(int note, int vel, bool once = false) { Push(note, constrain(vel, 1, 127), once); }
  void NoteOff(int note) { Push(note, 0, false); }
  void AllOff() { Push(255, 0, false); }  // every voice let go
  int Sounding() const {
    int n = 0;
    for (int i = 0; i < VOICES; i++) n += vc[i].on;
    return n;
  }
  void ReadViz(Viz& z) {
    AudioNoInterrupts();
    for (int i = 0; i < VOICES; i++) {
      const Voice& v = vc[i];
      z.pos[i] = v.on && nframes ? float(v.pos / nframes) : -1.0f;
      z.env[i] = v.on ? v.env : 0.0f;
      z.note[i] = v.note;
    }
    AudioInterrupts();
  }
  void update() override;

private:
  struct Voice {
    bool on = false;
    bool held = false;   // the key is down
    bool once = false;   // TRIG: let go once it is in (LOOP / PING)
    uint8_t note = 0;
    uint8_t stage = 0;   // 0 attack, 1 decay / sustain, 2 release
    int8_t dir = 1;      // PING: the way it goes
    bool looped = false; // INOUT: came round once (the fade in)
    double pos = 0.0;    // in frames of the sample
    double step = 1.0;
    float env = 0.0f;
    float gain = 1.0f;
    float fade = 1.0f;   // a fader's own fade out (stolen voices)
    uint32_t age = 0;
  };
  struct Event { uint8_t note, vel; bool once; };

  audio_block_t* inq[2];
  const int16_t* sl = nullptr;
  const int16_t* sr = nullptr;
  uint32_t nframes = 0;
  Params par;
  Voice vc[VOICES + FADERS];
  uint32_t clock = 0;
  static constexpr int QUEUE = 32;
  Event queue[QUEUE];
  volatile uint32_t ev_wr = 0, ev_rd = 0;

  void Push(int note, int vel, bool once) {
    if (ev_wr - ev_rd >= QUEUE) return;  // full: dropped
    queue[ev_wr % QUEUE] = {uint8_t(note), uint8_t(vel), once};
    ev_wr = ev_wr + 1;
  }
  void Start(const Event& e);
  void Let(int note);
  // the sample at a place between frames (4-point Hermite)
  static float At(const int16_t* d, int32_t i, float t, uint32_t n) {
    auto s = [&](int32_t k) { return k < 0 || k >= int32_t(n) ? 0.0f : float(d[k]); };
    const float y0 = s(i - 1), y1 = s(i), y2 = s(i + 1), y3 = s(i + 2);
    const float c1 = 0.5f * (y2 - y0);
    const float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
    return ((c3 * t + c2) * t + c1) * t + y1;
  }
};

FASTRUN void AudioEffectWarp::Start(const Event& e) {
  if (!nframes) return;
  // a free voice; else the oldest, whose sound goes to a fader first
  int use = -1;
  for (int i = 0; i < VOICES && use < 0; i++) if (!vc[i].on) use = i;
  if (use < 0) {
    use = 0;
    for (int i = 1; i < VOICES; i++) if (vc[i].age < vc[use].age) use = i;
    Voice* f = &vc[VOICES];
    for (int i = VOICES; i < VOICES + FADERS; i++) if (!vc[i].on || vc[i].fade < f->fade) f = &vc[i];
    *f = vc[use];
    f->fade = 1.0f;
  }
  Voice& v = vc[use];
  const float a = constrain(par.start, 0.0f, 1.0f) * nframes;
  v = Voice{};
  v.on = true;
  v.held = true;
  v.once = e.once;
  v.note = e.note;
  v.pos = a;
  v.step = par.rate * powf(2.0f, (int(e.note) - par.root) / 12.0f);
  const float vel = e.vel / 127.0f;
  v.gain = 1.0f - par.vel_amt + par.vel_amt * vel;
  v.age = ++clock;
}

FASTRUN void AudioEffectWarp::Let(int note) {
  for (int i = 0; i < VOICES; i++)
    if (vc[i].on && vc[i].held && (note == 255 || vc[i].note == note)) {
      vc[i].held = false;
      vc[i].stage = 2;
    }
}

FASTRUN void AudioEffectWarp::update() {
  while (ev_rd != ev_wr) {
    const Event e = queue[ev_rd % QUEUE];
    ev_rd = ev_rd + 1;
    if (e.vel) Start(e);
    else Let(e.note);
  }
  audio_block_t* in[2] = {receiveReadOnly(0), receiveReadOnly(1)};
  bool any = false;
  for (auto& v : vc) any |= v.on;
  if (!any || !nframes) {  // no voice: what comes in, as it is
    for (int c = 0; c < 2; c++) if (in[c]) { transmit(in[c], c); release(in[c]); }
    return;
  }
  audio_block_t* out[2] = {allocate(), allocate()};
  if (!out[0] || !out[1]) {
    for (auto b : out) if (b) release(b);
    for (auto b : in) if (b) release(b);
    return;
  }
  float sum[2][AUDIO_BLOCK_SAMPLES] = {};
  const double a = double(constrain(par.start, 0.0f, 1.0f)) * nframes;
  double b = double(constrain(par.end, 0.0f, 1.0f)) * nframes;
  if (b < a + 64.0) b = std::min<double>(nframes, a + 64.0);
  const double len = b - a;
  // LOOP's seam, in frames of the sample: CROSS h either side of END into
  // h either side of START; else (no room, h 0) the last f before END into
  // the first f of the loop; INOUT a dip of f each side
  const double seam = par.xfade * 48000.0 * par.rate;
  double h = 0.0, f = 0.0;
  if (par.loop == LOOP) {
    if (par.cross) {
      h = std::min({seam * 0.5, len * 0.5, a, double(nframes) - b});
      if (h < 16.0) h = 0.0;
      f = h > 0.0 ? 0.0 : std::min(seam, len * 0.5);
    } else {
      f = std::min(seam * 0.5, len * 0.5);
    }
  }
  const float att = 1.0f / (std::max(0.001f, par.attack) * AUDIO_SAMPLE_RATE_EXACT);
  const float rel = expf(-6.9078f / (std::max(0.005f, par.release) * AUDIO_SAMPLE_RATE_EXACT));
  const float dec = expf(-6.9078f / (std::max(0.005f, par.decay) * AUDIO_SAMPLE_RATE_EXACT));
  const float sus = constrain(par.sustain, 0.0f, 1.0f);
  constexpr float FADER = 1.0f / (0.005f * AUDIO_SAMPLE_RATE_EXACT);
  const float g = par.level * (1.0f / 32768.0f);
  for (int i = 0; i < VOICES + FADERS; i++) {
    Voice& v = vc[i];
    if (!v.on) continue;
    const bool fader = i >= VOICES;
    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
      // the envelope
      if (v.stage == 0) {
        v.env += att;
        if (v.env >= 1.0f) {
          v.env = 1.0f;
          v.stage = v.once && par.loop != ONCE ? 2 : 1;
        }
      } else if (v.stage == 1) {
        v.env = sus + (v.env - sus) * dec;
        if (v.env < 1e-4f) { v.on = false; break; }  // SUS 0: over once it has decayed
      } else if (v.stage == 2) {
        v.env *= rel;
        if (v.env < 1e-4f) { v.on = false; break; }
      }
      float e = v.env * v.gain;
      if (fader) {
        v.fade -= FADER;
        if (v.fade <= 0.0f) { v.on = false; break; }
        e *= v.fade;
      }
      // where it reads, and the seam
      const int32_t k = int32_t(v.pos);
      const float t = float(v.pos - k);
      float l = At(sl, k, t, nframes), r = sr == sl ? l : At(sr, k, t, nframes);
      if (par.loop == LOOP && par.cross) {
        double q = -1.0;
        float m = 0.0f;
        if (h > 0.0 && v.pos > b - h) {  // centred on the edges
          q = v.pos - len;
          m = float((v.pos - (b - h)) / (2.0 * h));
        } else if (f >= 1.0 && v.pos > b - f) {  // the last f into the first f
          q = a + (v.pos - (b - f));
          m = float((v.pos - (b - f)) / f);
        }
        if (q >= 0.0) {
          const int32_t kq = int32_t(q);
          const float tq = float(q - kq);
          const float lq = At(sl, kq, tq, nframes), rq = sr == sl ? lq : At(sr, kq, tq, nframes);
          l += m * (lq - l);
          r += m * (rq - r);
        }
      } else if (par.loop == LOOP && f >= 1.0) {  // INOUT
        float d = float((b - v.pos) / f);
        if (v.looped) d = std::min(d, float((v.pos - a) / f));
        if (d < 1.0f) e *= std::max(0.0f, d);
      }
      if (par.loop == ONCE) {  // the last 2 ms before END fade out
        const float left = float((b - v.pos) / v.step);
        if (left < 96.0f) e *= std::max(0.0f, left / 96.0f);
      }
      sum[0][n] += l * e;
      sum[1][n] += r * e;
      // on
      v.pos += v.step * v.dir;
      if (par.loop == ONCE) {
        if (v.pos >= b) { v.on = false; break; }
      } else if (par.loop == LOOP) {
        const double wrap = h > 0.0 ? b + h : b;  // CROSS: past the blend
        if (v.pos >= wrap) {
          v.pos -= h > 0.0 ? len : (b - a) - (par.cross ? f : 0.0);  // where the seam left it
          v.looped = true;
        }
        if (v.pos < a - h) v.pos = a;
      } else {  // PING
        if (v.pos >= b - 1.0) { v.pos = 2.0 * (b - 1.0) - v.pos; v.dir = -1; }
        if (v.pos <= a) { v.pos = 2.0 * a - v.pos; v.dir = 1; }
      }
    }
  }
  constexpr float q = 1.0f / 32768.0f;
  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    const float tl = in[0] ? in[0]->data[n] * q : 0.0f, tr = in[1] ? in[1]->data[n] * q : 0.0f;
    out[0]->data[n] = ToSample16(SoftLimit(sum[0][n] * g + tl));
    out[1]->data[n] = ToSample16(SoftLimit(sum[1][n] * g + tr));
  }
  transmit(out[0], 0);
  transmit(out[1], 1);
  release(out[0]);
  release(out[1]);
  for (auto b : in) if (b) release(b);
}
