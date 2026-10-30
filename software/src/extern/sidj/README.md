# sidj: the C64 and SID engine behind Loom's SID source

Taken from [SiDJ](https://github.com/thomasj/fm1-sidj) by Thomas Jansson (commit in `.upstream`):

| Files | What | Licence |
| --- | --- | --- |
| `sid.c`, `sid.h`, `sid_tables.c`, `sid_tables.h` | fixed-point C port of reSID 0.16 (Dag Lem, 2004) | GPL-2.0-or-later |
| `c64.c`, `c64.h`, `c64regs.h`, `cpu6502.c`, `cpu6502.h`, `sidfile.c`, `sidfile.h`, `sidreg.h` | the C64: 6510, CIAs, VIC raster, KERNAL stand-in, PSID/RSID parser | GPL-3.0-only |

Changed for the Teensy 4.1, nothing else:

- `sidj_port.h` (new): `SIDJ_FLASH` puts every function in flash (built at -O2), `SIDJ_CONST`
  keeps the tables in flash, `SIDJ_RAM2` puts the 33 KB of combined-waveform running sums in
  RAM2. RAM1 has no room for them. On the host the macros are empty.
- Each `.c` includes `sidj_port.h`; function definitions carry `SIDJ_FLASH`, the tables in
  `sid_tables.c` `SIDJ_CONST`, `comb_rec` / `comb_pre` in `sid.c` `SIDJ_RAM2`.

Checked on the Mac: SiDJ's `host/sidplay.c` built against these files renders the same WAV,
byte for byte, as against SiDJ's own.

Because these files are GPL-3.0-only, a firmware built with them is distributed under the
GPL-3.0 as a whole.
