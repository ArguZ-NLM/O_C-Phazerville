# dream: the engine behind Loom's DREAM source

Taken from [NACHTFAHRT](https://github.com/FartyPants/fm1-nachtfahrt) by Cranky Man / FartyPants
(`firmware/src/dsp`, main after 0.9.4), GPL-3.0. Its echo chain, plate, pieces, chord walk and UI
come from DUB-1 and AMB-1; the plate is Dattorro's; Choir's formants, Shimmer's shifter and the
platform from Felucca; Pluck after M-VAVE's fm1_open (DaisySP String, MIT); the noise after
Skiff; Freeze after JIANT's CLOUD (see NACHTFAHRT's LICENSING.md).

| Files | What |
| --- | --- |
| `amb.c`, `amb.h` | the engine: the Drone, two Sequences, the Lead, the Atmosphere, the arc, the echo and plate |
| `seq.h`, `seq_impl.h` (its `seq.c`), `fastmath.h` | the sequences' patterns, the fast float maths |
| `dream_port.h`, `dream_zero.inc` (new) | `DREAM_FLASH` / `DREAM_CONST` / `DREAM_RAM2` placement; the RAM2 / PSRAM state cleared once (neither is zeroed at start) |
| `amb_b.c`, `dream_b_names.h`, `dream_b_unnames.h` (new) | the engine again for track B: every exported name renamed `dreamb_*`, its state in PSRAM |

Changed for Loom:

- The Sequences' voices (all but Pluck) are a ladder voice after
  [FMini-1](https://github.com/ruppert/fmini-1)'s Minimoog filter, in float: four saturating
  stages, the feedback through a lag pole, solved ahead; each voice a drive and a character
  (`FAT[]`), its oscillators drifting a few cents.
- `en_silent` (CV ONLY: the piece plays on, nothing rendered), `en_follow` / `en_follow_deg` /
  `en_clock_now` / `en_deg` (LINK: track B's engine on track A's clock and chords),
  `en_drone_oct` (the Drone's octave), `en_gate` (LENGTH: the Sequences' notes held).
- Eight more pieces (Oxide .. Velocity) after the original eight.

GPL-3.0: a firmware built with these files is distributed under the GPL-3.0 as a whole.
