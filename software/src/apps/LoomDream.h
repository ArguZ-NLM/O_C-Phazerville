// DREAM: a generative Berlin-school piece as a Loom SOURCE.
//
// The engine is NACHTFAHRT's (Cranky Man / FartyPants, github.com/
// FartyPants/fm1-nachtfahrt, GPL-3.0; after DUB-1 and AMB-1): a Drone (a
// chord over a pedal, walking through the scale), a Sequence and a second
// one a step shorter, a gliding Lead with call-and-answer phrases, an
// Atmosphere, all moving through an arc of sections (Emerge, Flow, Solo,
// Bridge) with its own echo and plate. In src/extern/dream (see the README
// there); this file is the Loom device around it.
//
// Page 1: STYLE one of the sixteen pieces (then U1..U8, the user's own: B
// held on one keeps what plays there); TEMPO (the L knob: SYNC, the
// tempo follows Loom's clock); ROOT (the L knob: the MODE); LEVEL.
// Page 2: DRONE SEQ LEAD SEQ2 each layer's level, the L knob its voice.
// Page 3: MOTION (how much the arrangement moves), ENERGY and SPACE (the
// piece's density and its echo / plate, around the piece's own), CV.
// A: play (held for 2 s: stop), B: the next chord (held for 2 s: a fresh
// piece, Loom.h). MIDI notes in play the Lead in the piece's scale (and it
// answers them): the white keys the scale's notes in turn, C the root, any
// octave; the black keys hold five of the FM-1's effects (C# D# F# G# A#).
// KEYS (CV's L knob): the same from CV in 1 and 2.
//
// CV (its CV on, while DREAM is the track's source): A the Sequence's
// pitch, B Sequence 2's, C the Lead's, D the Drone's lowest note; E F a
// trigger on each sequence's notes (half a sixteenth), G the Lead's gate,
// H the arc: the Drone's filter moving through the sections, 0..10 V.
//
// Each track has an engine of its own (A's amb.c, B's the same source again
// as amb_b.c, its names renamed): A can play a piece while B's, at CV ONLY,
// plays none of its sound and drives the CV outputs alone (CV ONLY wins the
// outputs over CV ON).
//
// LINK (B's CV, the L knob; B's own default, with CV ONLY): B's DREAM follows
// A's: A's clock pulse by pulse (they start and stop together, B's A pressed
// while A plays joins on A's next bar), its tempo, key, mode and chord. B's
// own piece plays its sequences, its lead, its arc over them: the same music,
// told differently.

#pragma once

extern "C" {
#include <stdint.h>
#include "dream/seq.h"
#include "dream/dream_port.h"
}

namespace loom {

// what Loom needs of a DREAM, whichever engine it plays
class DreamBase : public Device, public AudioStream, public CvSource {
public:
  enum { PIECE, TEMPO, ROOT, LEVEL, DRONE, SEQ, LEAD, SEQ2, MOTION, ENERGY, SPACE, CV, ECHO, FDBK, PLATE, TONE };
  enum { CV_OFF, CV_ON, CV_ONLY };
  explicit DreamBase(const DeviceDef& d) : Device(d), AudioStream(0, nullptr) {}
  int8_t track = 0;
  virtual void Stop() = 0;    // A held (Loom.h)
  virtual void Regrow() = 0;  // B held (Loom.h)
  virtual void SpaceOut(bool on) = 0;  // REPEAT on the track: SPACE moves it
  // (ONLY claims the outputs while it plays: stopped, a CV ON elsewhere has them)
  bool CvOnly() const override { return cur[CV] == CV_ONLY && ok() && Playing(); }
  virtual bool Playing() const = 0;
  // the CPU it takes (the guard, Loom.h): next to none for the CV alone
  float Cost() const { return cur[CV] == CV_ONLY ? 4.0f : 38.0f; }
};

// LINK: track A's DREAM as it plays (A writes it, B's follows it)
struct DreamLink {
  volatile uint32_t pulses = 0;       // A's clock pulses (24 a beat) so far
  volatile uint32_t starts = 0, stops = 0;
  volatile uint32_t start_pulse = 0;  // the pulses at its last start: the bar's phase
  volatile bool playing = false;
  volatile int8_t deg = 0;            // its chord's scale degree
  volatile int16_t tempo = 96, root = 2;
  volatile uint8_t mode = 5;
  volatile uint32_t seen = 0;         // millis() A's main loop last wrote it (0: never)
  bool Alive() const { return seen && int32_t(millis() - seen) < 1000; }  // (seen: millis() | 1, a ms ahead at most)
  // B's side, for the USB status: linked, how often it linked or let go,
  // its chord's degree, the chords it was sent
  volatile bool b_linked = false;
  volatile uint32_t b_flips = 0, b_degs = 0;
  volatile int8_t b_deg = 0;
  // (USB status) the black keys' effects held (MIDI + CV 2, a bit each) and
  // the last MIDI key taken: its note and what it became (an effect, a degree)
  volatile uint16_t fx_held = 0;
  volatile int16_t key_note = -1, key_took = 0;
};
inline DreamLink g_dream_link;

// from Loom's screen (Loom.h): the setting grabbed (R turns it), and B held
// on DREAM so far (0..1, on which track)
inline Device* g_edit_dev = nullptr;
inline int8_t g_edit_p = -1;
inline float g_dream_hold = 0.0f;
inline int8_t g_dream_hold_track = -1;

// the user's own pieces, U1..U8 after the built-in ones (both tracks'): B
// held on one keeps what DREAM plays there: the piece it grew from, these
// settings (TEMPO .. SPACE, the voices and octaves with them) and the two
// sequences as they are; DREAMU.BIN on the internal flash
constexpr int DREAM_USER_N = 8;
struct DreamUserPiece {
  uint8_t used, base;
  int16_t val[16];
  uint8_t mode[16];
  uint8_t seq[82];   // (EN_SEQ_N)
};
struct DreamUserPiece1 {  // (DRU1, before the FX page: 12 settings)
  uint8_t used, base;
  int16_t val[12];
  uint8_t mode[12];
  uint8_t seq[82];
};
struct DreamUser {
  static constexpr char FILE_NAME[] = "DREAMU.BIN";
  DreamUserPiece slot[DREAM_USER_N];
  bool loaded = false;
  void Load() {
    if (loaded) return;
    loaded = true;
    memset(slot, 0, sizeof(slot));
    File f = PhzConfig::myfs.open(FILE_NAME, FILE_READ);
    if (!f) return;
    char mark[4] = {};
    if (f.read(mark, 4) != 4) { f.close(); return; }
    if (!memcmp(mark, "DRU2", 4)) {
      if (f.read(slot, sizeof(slot)) != int(sizeof(slot))) memset(slot, 0, sizeof(slot));
    } else if (!memcmp(mark, "DRU1", 4)) {  // (the FX page at the style's own)
      for (auto& s : slot) {
        DreamUserPiece1 o;
        if (f.read(&o, sizeof(o)) != int(sizeof(o))) break;
        s.used = o.used, s.base = o.base;
        memcpy(s.val, o.val, sizeof(o.val));
        memcpy(s.mode, o.mode, sizeof(o.mode));
        memcpy(s.seq, o.seq, sizeof(o.seq));
      }
    }
    f.close();
  }
  bool Save() const {
    File f = PhzConfig::myfs.open(FILE_NAME, FILE_WRITE_BEGIN);
    if (!f) return false;
    const bool ok = f.write("DRU2", 4) == 4 && f.write(slot, sizeof(slot)) == sizeof(slot);
    f.close();
    return ok;
  }
};
inline DreamUser g_dream_user;

}  // namespace loom

// track A's engine (amb.c)
#define DREAM_NS dream_a
#define DREAM_TRACK_B 0
#include "LoomDreamCore.h"
#undef DREAM_TRACK_B
#undef DREAM_NS

// track B's: the same again, every engine name renamed (amb_b.c)
#include "dream/dream_b_names.h"
#undef DREAM_AMB_H
#define DREAM_NS dream_b
#define DREAM_TRACK_B 1
#include "LoomDreamCore.h"
#undef DREAM_TRACK_B
#undef DREAM_NS
#include "dream/dream_b_unnames.h"
