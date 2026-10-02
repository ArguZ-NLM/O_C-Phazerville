#pragma once

#include "../Audio/AudioEffectModalResonator.h"

// ModalResonatorApplet — modal synthesis resonator bank
//
// Inspired by Mutable Instruments Rings / Elements.  Takes any input signal as
// an "exciter" (trigger burst, noise, or external audio) and passes it through
// a bank of 12 parallel two-pole resonators tuned to a modal spectrum.
//
// Page 1: Pit, Inh, Brt, Dmp, Pos
// Page 2: Mix, Trg, Vel, Fin, Qnt
// Page 3 (stereo only): Spr
//
//   Pit — fundamental (semitones, V/Oct CV)    Fin — fine tune in cents
//   Inh — inharmonicity: 0 = harmonic series, 100 = bell/bar spectrum
//   Brt — mode amplitude taper, also how hard the built-in strike sounds
//   Dmp — decay time: 0 = metallic click, 100 = long sustain
//   Pos — excitation-point comb (Rings-style): 0=off, 50=odd harmonics
//   Mix — wet/dry blend
//   Trg — gate input for the built-in strike    Vel — strike velocity
//   Qnt — snap the pitch CV to semitones
//   Spr — stereo: L+R summed in, odd modes out left, even modes right
//
// AuxButton: fires a manual noise-burst strike (test without a patched exciter).
//
// The UI lives in this non-template base so it exists once and can sit in
// flash (FLASHMEM has no effect on template member functions).
class ModalResonatorBase : public HemisphereAudioApplet {
public:
    explicit ModalResonatorBase(bool stereo) : stereo_(stereo) {}

    const char* applet_name() { return "ModalRes"; }

    // --- Controller (~150 Hz, ISR context) -----------------------------------

    void Controller() override {
        // Pitch: base (128 units = 1 semitone) + V/Oct CV
        int cv = freq_cv.In();
        if (quantize) cv = ((cv + 64) >> 7) << 7;
        float freq_hz = PitchToRatio(pitch + cv) * C3;
        if (freq_hz < 20.0f)   freq_hz = 20.0f;
        if (freq_hz > 4000.0f) freq_hz = 4000.0f;

        float eff_structure = constrain(0.01f * structure + struct_cv.InF(),  0.0f, 1.0f);
        float eff_bright    = constrain(0.01f * brightness + bright_cv.InF(), 0.0f, 1.0f);
        float eff_damp      = constrain(0.01f * damping    + damp_cv.InF(),   0.0f, 1.0f);
        float eff_pos       = constrain(0.01f * position   + pos_cv.InF(),    0.0f, 1.0f);
        float eff_mix       = constrain(0.01f * mix        + mix_cv.InF(),    0.0f, 1.0f);
        float eff_spread    = constrain(0.01f * spread     + spread_cv.InF(), 0.0f, 1.0f);

        float dry_gain, wet_gain;
        EqualPowerFade(dry_gain, wet_gain, eff_mix);

        // Rising edge on strike_input gate
        bool cur_strike_gate = strike_input.Gate();
        bool gate_strike = cur_strike_gate && !prev_strike_gate_;
        prev_strike_gate_ = cur_strike_gate;

        resonator.updateCoeffs(
            freq_hz, eff_structure, eff_bright, eff_damp, eff_pos);
        resonator.setSpread(eff_spread);
        SetMixGains(dry_gain, wet_gain);

        if (manual_strike_ || gate_strike) {
            float vel = manual_strike_ ? 1.0f
                      : constrain(0.01f * velocity + vel_cv.InF(), 0.0f, 1.0f);
            resonator.strike(vel);
        }
        manual_strike_ = false;
    }

    FLASHMEM void View() override {
        // VU bars: excitation input (y=7), output wet mix (y=11)
        uint16_t peak = resonator.readPeak();
        if (peak > 0) {
            int bar_w = (int)((uint32_t)peak * 56 / 32767) + 1;
            gfxRect(1, 7, bar_w, 3);
        }
        uint16_t out_peak = resonator.readOutputPeak();
        if (out_peak > 0) {
            int bar_w = (int)((uint32_t)out_peak * 56 / 32767) + 1;
            gfxFrame(1, 11, bar_w, 3);
        }

        // Rows, 5 per page
        int8_t items[MAX_ITEMS], item_row[MAX_ITEMS];
        int8_t rows[ROW_COUNT];
        const int nrows = VisibleRows(rows);
        BuildItems(items, item_row);
        const int8_t id = items[cursor];
        const int page = item_row[cursor] / ROWS_PER_PAGE;
        const int last_page = (nrows - 1) / ROWS_PER_PAGE;

        char v[8];
        for (int r = page * ROWS_PER_PAGE; r < nrows && r < (page + 1) * ROWS_PER_PAGE; r++) {
            const Row &row = kRows[rows[r]];
            const int y = 15 + 10 * (r - page * ROWS_PER_PAGE);
            gfxPrint(1, y, row.label);
            if (row.val >= 0) {
                FormatValue(row.val, v, sizeof(v));
                DrawValue(y, v, id == row.val);
            }
            if (row.cv == STRIKE) {
                if (prev_strike_gate_) gfxInvert(1, y, 20, 8);
                gfxStartCursor(VAL_END, y);
                gfxPrint(strike_input);
                gfxEndCursor(id == STRIKE, true, strike_input.InputName());
            } else if (row.cv >= 0) {
                DrawCV(y, *CvMap(row.cv), id == row.cv);
            }
        }

        // Page indicator
        gfxPrint(58, 56, page < last_page ? ">" : "<");

        gfxDisplayInputMapEditor();
    }

    // --- AuxButton: manual test strike ---------------------------------------

    FLASHMEM void AuxButton() override {
        manual_strike_ = true;
        CancelEdit();
    }

    // --- Button / encoder ----------------------------------------------------

    FLASHMEM void OnButtonPress() override {
        int8_t items[MAX_ITEMS], item_row[MAX_ITEMS];
        BuildItems(items, item_row);
        const int8_t id = items[cursor];
        if (CheckEditInputMapPress(
                id,
                IndexedInput(FREQ_CV,   freq_cv),
                IndexedInput(STRUCT_CV, struct_cv),
                IndexedInput(BRIGHT_CV, bright_cv),
                IndexedInput(DAMP_CV,   damp_cv),
                IndexedInput(POS_CV,    pos_cv),
                IndexedInput(MIX_CV,    mix_cv),
                IndexedInput(STRIKE,    strike_input),
                IndexedInput(VEL_CV,    vel_cv),
                IndexedInput(SPREAD_CV, spread_cv)
            ))
            return;
        if (id == QUANT) {
            quantize ^= 1;
            return;
        }
        CursorToggle();
    }

    FLASHMEM void OnEncoderMove(int direction) override {
        int8_t items[MAX_ITEMS], item_row[MAX_ITEMS];
        const int nitems = BuildItems(items, item_row);
        if (!EditMode()) {
            MoveCursor(cursor, direction, nitems - 1);
            return;
        }
        if (EditSelectedInputMap(direction)) return;

        switch (items[cursor]) {
            case PITCH:
                pitch = constrain(pitch + direction * 128, MIN_PITCH, MAX_PITCH);
                break;
            case FINE:
                pitch = constrain(pitch + direction * 4, MIN_PITCH, MAX_PITCH);
                break;
            case FREQ_CV:    freq_cv.ChangeSource(direction);   break;
            case STRUCT:     structure  = constrain(structure  + direction, 0, 100); break;
            case STRUCT_CV:  struct_cv.ChangeSource(direction);  break;
            case BRIGHT:     brightness = constrain(brightness + direction, 0, 100); break;
            case BRIGHT_CV:  bright_cv.ChangeSource(direction);  break;
            case DAMP:       damping    = constrain(damping    + direction, 0, 100); break;
            case DAMP_CV:    damp_cv.ChangeSource(direction);    break;
            case POS:        position   = constrain(position   + direction, 0, 100); break;
            case POS_CV:     pos_cv.ChangeSource(direction);     break;
            case MIX:        mix        = constrain(mix        + direction, 0, 100); break;
            case MIX_CV:     mix_cv.ChangeSource(direction);     break;
            case STRIKE:     strike_input.ChangeSource(direction); break;
            case VEL:        velocity = constrain(velocity + direction, 0, 100); break;
            case VEL_CV:     vel_cv.ChangeSource(direction);       break;
            case SPREAD:     spread   = constrain(spread + direction, 0, 100); break;
            case SPREAD_CV:  spread_cv.ChangeSource(direction);    break;
            default: break;
        }
    }

    // --- Persistence ---------------------------------------------------------

#define MODAL_PARAMS pitch, structure, brightness, damping, position, mix, velocity
    FLASHMEM void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override {
        uint16_t dummy = 0;
        data[0] = PackPackables(MODAL_PARAMS);
        data[1] = PackPackables(freq_cv, struct_cv, bright_cv, damp_cv);
        data[2] = PackPackables(pos_cv, mix_cv, dummy, vel_cv);
        data[3] = PackPackables(strike_input, spread, quantize, spread_cv);
    }

    // presets from before Spr/Qnt load with Spr 0 (same sound as before)
    FLASHMEM void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override {
        uint16_t dummy = 0;
        UnpackPackables(data[0], MODAL_PARAMS);
        UnpackPackables(data[1], freq_cv, struct_cv, bright_cv, damp_cv);
        UnpackPackables(data[2], pos_cv, mix_cv, dummy, vel_cv);
        UnpackPackables(data[3], strike_input, spread, quantize, spread_cv);
        pitch    = constrain(pitch, MIN_PITCH, MAX_PITCH);
        spread   = constrain(spread, 0, 100);
        quantize = quantize ? 1 : 0;
    }
#undef MODAL_PARAMS

protected:
    void SetHelp() override {}

    virtual void SetMixGains(float dry, float wet) = 0;

    AudioEffectModalResonator resonator;

private:
    enum Cursor : int8_t {
        // Page 1
        PITCH = 0, FREQ_CV,
        STRUCT, STRUCT_CV,
        BRIGHT, BRIGHT_CV,
        DAMP, DAMP_CV,
        POS, POS_CV,
        // Page 2
        MIX, MIX_CV,
        STRIKE,
        VEL, VEL_CV,
        FINE,
        QUANT,
        // Page 3, stereo only
        SPREAD, SPREAD_CV,
    };

    // Screen rows in display order; stereo-only rows are skipped in mono.
    struct Row { const char *label; int8_t val; int8_t cv; bool stereo_only; };
    static constexpr int ROW_COUNT = 11;
    static constexpr int ROWS_PER_PAGE = 5;
    static constexpr int MAX_ITEMS = ROW_COUNT * 2;
    static constexpr Row kRows[ROW_COUNT] = {
        { "Pit:", PITCH,  FREQ_CV,   false },
        { "Inh:", STRUCT, STRUCT_CV, false },
        { "Brt:", BRIGHT, BRIGHT_CV, false },
        { "Dmp:", DAMP,   DAMP_CV,   false },
        { "Pos:", POS,    POS_CV,    false },
        { "Mix:", MIX,    MIX_CV,    false },
        { "Trg:", -1,     STRIKE,    false },
        { "Vel:", VEL,    VEL_CV,    false },
        { "Fin:", FINE,   -1,        false },
        { "Qnt:", QUANT,  -1,        false },
        { "Spr:", SPREAD, SPREAD_CV, true  },
    };

    int VisibleRows(int8_t *rows) const {
        int n = 0;
        for (int r = 0; r < ROW_COUNT; r++)
            if (stereo_ || !kRows[r].stereo_only) rows[n++] = r;
        return n;
    }
    // cursor = index into the list of selectable items (value, then CV icon)
    int BuildItems(int8_t *items, int8_t *item_row) const {
        int8_t rows[ROW_COUNT];
        const int nrows = VisibleRows(rows);
        int n = 0;
        for (int r = 0; r < nrows; r++) {
            const Row &row = kRows[rows[r]];
            if (row.val >= 0) { items[n] = row.val; item_row[n++] = r; }
            if (row.cv >= 0)  { items[n] = row.cv;  item_row[n++] = r; }
        }
        return n;
    }

    CVInputMap *CvMap(int8_t id) {
        switch (id) {
            case FREQ_CV:   return &freq_cv;
            case STRUCT_CV: return &struct_cv;
            case BRIGHT_CV: return &bright_cv;
            case DAMP_CV:   return &damp_cv;
            case POS_CV:    return &pos_cv;
            case MIX_CV:    return &mix_cv;
            case VEL_CV:    return &vel_cv;
            case SPREAD_CV: return &spread_cv;
            default:        return &freq_cv;
        }
    }

    static constexpr int VAL_END = 49; // x where the CV icons start
    // values end 2px into the CV icon's blank left margin
    static constexpr int VAL_SHIFT = 2;

    FLASHMEM void DrawValue(int y, const char *text, bool selected) {
        gfxStartCursor(VAL_END + VAL_SHIFT - 6 * (int)strlen(text), y);
        gfxPrint(text);
        gfxEndCursor(selected);
    }
    FLASHMEM void DrawCV(int y, CVInputMap &map, bool selected) {
        gfxStartCursor(VAL_END, y);
        gfxPrint(map);
        gfxEndCursor(selected, false, map.InputName());
    }

    // pitch 0 = C3; nearest semitone and the offset from it (1/128 semitone)
    int Semitone() const { return (pitch + 64) >> 7; }
    int FineOffset() const { return pitch - Semitone() * 128; }

    FLASHMEM void FormatValue(int8_t id, char *v, size_t n) {
        switch (id) {
            case PITCH:  snprintf(v, n, "%s", midi_note_numbers[48 + Semitone()]); break;
            case FINE: {
                const int cents = FineOffset() * 100 / 128;
                snprintf(v, n, cents > 0 ? "+%dc" : "%dc", cents);
                break;
            }
            case STRUCT: snprintf(v, n, "%d", structure); break;
            case BRIGHT: snprintf(v, n, "%d", brightness); break;
            case DAMP:   snprintf(v, n, "%d", damping); break;
            case POS:    snprintf(v, n, "%d", position); break;
            case MIX:    snprintf(v, n, "%d%%", mix); break;
            case VEL:    snprintf(v, n, "%d%%", velocity); break;
            case QUANT:  snprintf(v, n, quantize ? "on" : "off"); break;
            case SPREAD: snprintf(v, n, "%d%%", spread); break;
            default:     v[0] = 0; break;
        }
    }

    // C0 .. C8 (the resonator stops at 4 kHz, just below C8)
    static constexpr int MAX_PITCH = 5 * 12 * 128;
    static constexpr int MIN_PITCH = -3 * 12 * 128;

    const bool stereo_;
    int8_t cursor = 0;

    // Parameters
    int16_t pitch      = 1 * 12 * 128;  // C4 default
    int8_t  structure  = 50;            // 0=harmonic, 100=inharmonic
    int8_t  brightness = 70;            // mode amplitude taper
    int8_t  damping    = 50;            // decay time
    int8_t  position   = 25;            // excitation point
    int8_t  mix        = 100;           // wet/dry %
    int8_t  velocity   = 100;           // strike velocity 0–100%
    int8_t  spread     = 100;           // stereo odd/even split %
    int8_t  quantize   = 0;             // pitch CV snapped to semitones

    CVInputMap freq_cv;
    CVInputMap struct_cv;
    CVInputMap bright_cv;
    CVInputMap damp_cv;
    CVInputMap pos_cv;
    CVInputMap mix_cv;
    CVInputMap vel_cv;
    CVInputMap spread_cv;

    bool manual_strike_ = false;

    DigitalInputMap strike_input;
    bool prev_strike_gate_ = false;
};

template <AudioChannels Channels>
class ModalResonatorApplet : public ModalResonatorBase {
public:
    ModalResonatorApplet() : ModalResonatorBase(Channels > 1) {}

    void Start() override {
        resonator.Acquire();
        resonator.setSplit(Channels > 1);
        if (Channels > 1) {
            for (int ch = 0; ch < Channels; ch++) {
                PatchCable(input_stream, ch, input_mixer, ch);
                input_mixer.gain(ch, 0.5f);
            }
            PatchCable(input_mixer, 0, resonator, 0);
        } else {
            PatchCable(input_stream, 0, resonator, 0);
        }
        for (int ch = 0; ch < Channels; ch++) {
            PatchCable(input_stream, ch, wet_dry_mixer[ch], DRY_CH);
            PatchCable(resonator,    ch, wet_dry_mixer[ch], WET_CH);
            PatchCable(wet_dry_mixer[ch], 0, output_stream, ch);
        }
    }

    void Unload() override {
        resonator.Release();
        AllowRestart();
    }

    AudioStream* InputStream()  override { return &input_stream;  }
    AudioStream* OutputStream() override { return &output_stream; }

protected:
    void SetMixGains(float dry, float wet) override {
        for (int ch = 0; ch < Channels; ch++) {
            wet_dry_mixer[ch].gain(DRY_CH, dry);
            wet_dry_mixer[ch].gain(WET_CH, wet);
        }
    }

private:
    static const uint8_t DRY_CH = 0;
    static const uint8_t WET_CH = 1;

    // stereo: L+R summed into one resonator, odd modes left, even modes right
    AudioMixer<2> input_mixer;
    AudioMixer<2> wet_dry_mixer[Channels];

    AudioPassthrough<Channels> input_stream;
    AudioPassthrough<Channels> output_stream;
};
