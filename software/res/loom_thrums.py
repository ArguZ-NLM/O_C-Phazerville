#!/usr/bin/env python3
# Builds src/apps/LoomThrums.h: Loom's starter set of single-cycle waves,
# which the firmware writes to /LOOM/THRUMS when the card doesn't have it.
#
# The waves are the Adventure Kid Waveforms (AKWF) by Kristoffer Ekstrand,
# CC0 1.0: https://github.com/KristofferKarlAxelEkstrand/AKWF-FREE
#
#   python3 res/loom_thrums.py <unzipped AKWF-FREE-main folder>
#
# Same pick as the set first put on the cards by hand (2026-10-09): 23
# folders, at most 32 waves each, evenly spread over the AKWF folder.

import os
import sys
import wave

PICK = [
    ('bw_perfectwaves', 'BASIC'), ('bw_sin', 'SINE'), ('bw_tri', 'TRIANGLE'),
    ('bw_saw', 'SAW'), ('bw_squ', 'SQUARE'), ('sinharm', 'SINHARM'),
    ('overtone', 'OVERTONE'), ('fmsynth', 'FM'), ('hvoice', 'VOICE'),
    ('eorgan', 'ORGAN'), ('epiano', 'EPIANO'), ('piano', 'PIANO'),
    ('flute', 'FLUTE'), ('clarinett', 'CLARINET'), ('oboe', 'OBOE'),
    ('altosax', 'SAX'), ('cello', 'CELLO'), ('violin', 'VIOLIN'),
    ('theremin', 'THEREMIN'), ('vgame', 'GAME'), ('oscchip', 'CHIP'),
    ('granular', 'GRANULAR'), ('distorted', 'DISTORT'),
]
SAMPLES = 600
RATE = 44100

src = os.path.join(sys.argv[1], 'AKWF')
out = os.path.join(os.path.dirname(__file__), '..', 'src', 'apps', 'LoomThrums.h')

folders, counts, waves = [], [], []
for d, name in PICK:
    path = os.path.join(src, 'AKWF_' + d)
    fs = sorted(f for f in os.listdir(path) if f.lower().endswith('.wav'))
    if len(fs) > 32:
        fs = [fs[round(i * (len(fs) - 1) / 31)] for i in range(32)]
    for f in fs:
        w = wave.open(os.path.join(path, f))
        assert (w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()) == (1, 2, RATE, SAMPLES), f
        raw = w.readframes(SAMPLES)
        waves.append([int.from_bytes(raw[i:i + 2], 'little', signed=True) for i in range(0, len(raw), 2)])
    folders.append(name)
    counts.append(len(fs))

with open(out, 'w') as h:
    h.write('#pragma once\n')
    h.write('// made by res/loom_thrums.py (do not edit): the Adventure Kid Waveforms\n')
    h.write('// (AKWF) by Kristoffer Ekstrand, CC0 1.0, as Loom writes them to\n')
    h.write('// /LOOM/THRUMS/<FOLDER>/<FOLDER>_nn.wav (mono, 16 bit, 44.1 kHz)\n\n')
    h.write('namespace loom {\n')
    h.write('constexpr int THRUM_FOLDERS = %d, THRUM_WAVES = %d, THRUM_SAMPLES = %d, THRUM_RATE = %d;\n'
            % (len(folders), len(waves), SAMPLES, RATE))
    h.write('PROGMEM static const char THRUM_NAME[THRUM_FOLDERS][9] = {\n')
    h.write(',\n'.join('  "%s"' % n for n in folders) + '\n};\n')
    h.write('PROGMEM static const uint8_t THRUM_COUNT[THRUM_FOLDERS] = {%s};\n'
            % ', '.join(str(c) for c in counts))
    h.write('PROGMEM static const int16_t THRUM_DATA[THRUM_WAVES][THRUM_SAMPLES] = {\n')
    for w in waves:
        h.write('{' + ','.join(str(v) for v in w) + '},\n')
    h.write('};\n}  // namespace loom\n')

print(len(folders), 'folders,', len(waves), 'waves ->', os.path.normpath(out))
