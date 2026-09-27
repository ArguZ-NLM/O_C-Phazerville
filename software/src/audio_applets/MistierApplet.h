#pragma once

#include "../Audio/AudioEffectClouds.h"
#include <smalloc.h>

extern "C" uint8_t external_psram_size;

// MistierApplet — Clouds-inspired live granular audio processor.
//
// Records audio into a 1-second PSRAM circular buffer and plays it back as a
// cloud of overlapping grains. Differences from MistApplet:
//   • Texture — 0: short fades; up to 50 the fade-out grows, above 50 the fade-in
//   • Density — centred at 0 (silence). CCW → regular periodic. CW → stochastic.
//   • Feedback (Fdb) — grain output fed back into the record buffer
//   • No fixed grain shapes — shape driven by Texture
//
// I/O:
//   Input → [AudioEffectClouds] → wet ──────────────────────────────┐
//   Input →                     → dry ─────────────────────────────┤ AudioMixer<2> → Output
//
// Freeze:
//   • Frz on/off: cursor on the value, press the encoder to toggle
//   • AuxButton: toggles the same latch (performance use, no cable needed)
//   • Frz input: assign a hardware gate via input map editor (cursor on the icon → press button)
//   Latch and gate OR together.
//
// Parameters:
//   Mix, Den, [L+R, SSp — stereo only], Pos, Siz, Spr, PSp, Pit, Tex, Fdb, Frz, Buf
//   (5 rows per page)
//
// Everything shared by the mono and stereo versions lives in this non-template
// base, so it exists once and its UI code can sit in flash (FLASHMEM has no
// effect on template member functions).
class MistierBase : public HemisphereAudioApplet {
public:
    explicit MistierBase(bool stereo) : stereo_(stereo) {}

    const char* applet_name() { return "Misty"; }

    void Controller() override {
        // ── CV-modulated effective parameters ──────────────────────────────────
        float eff_pos     = constrain(0.01f * pos     + pos_cv.InF(),              0.0f, 1.0f);
        // density: −20..+20 Hz in 1 Hz steps (0=silence, >0=stochastic, <0=periodic)
        float eff_density = constrain(DensityHz() + density_cv.InF() * 20.0f, -20.0f, 20.0f);
        float eff_size    = constrain(0.01f * size    + size_cv.InF() * 0.49f,     0.01f, 0.5f);
        float eff_texture = constrain(0.01f * texture + texture_cv.InF(),           0.0f, 1.0f);
        float eff_spray   = constrain(0.01f * spray   + spray_cv.InF(),             0.0f, 1.0f);

        // Pitch: semitones + V/Oct CV.
        float eff_semis = (float)pitch + (float)pitch_cv.In() / 128.0f;
        float eff_pitch = SemitonesToRatio(eff_semis);

        // Pitch spread: quadratic curve for fine control at low values.
        float eff_psprd_raw   = constrain(0.01f * psprd + psprd_cv.InF(), 0.0f, 1.0f);
        float eff_psprd_semis = 12.0f * eff_psprd_raw * eff_psprd_raw;

        // Feedback and wet/dry.
        float eff_feedback = constrain(0.01f * fdb + fdb_cv.InF(), 0.0f, 1.0f);
        float eff_mix      = constrain(0.01f * mix + mix_cv.InF(), 0.0f, 1.0f);

        // Freeze: hardware gate OR manual latch.
        bool frozen = freeze_input.Gate() || manual_freeze_;

        float dry_gain, wet_gain;
        EqualPowerFade(dry_gain, wet_gain, eff_mix);

        grain().setPosition(eff_pos);
        grain().setDensity(eff_density);
        grain().setSize(eff_size);
        grain().setSpray(eff_spray);
        grain().setPitch(eff_pitch);
        grain().setPitchSpread(eff_psprd_semis);
        grain().setTexture(eff_texture);
        grain().setFeedback(eff_feedback);
        grain().setFreeze(frozen);
        grain().setGrainSource(0.01f * lr);
        grain().setStereoSpread(0.01f * sspread);
        SetMixGains(dry_gain, wet_gain);
    }

    FLASHMEM void View() override {
        if (!grain().IsReady()) {
            // allocation failed: PSRAM full, or (without PSRAM) RAM full
            gfxPrint(1, 15, external_psram_size ? "PSRAM full" : "no memory");
            return;
        }

        // ── Grain activity bar (y=7) ──────────────────────────────────────────
        uint8_t active = grain().ActiveGrainCount();
        for (uint8_t i = 0; i < AudioEffectClouds::MAX_GRAINS; i++) {
            if (i < active) gfxPixel(1 + i, 7);
        }

        // ── Rows, 5 per page ─────────────────────────────────────────────────
        int8_t items[MAX_ITEMS], item_row[MAX_ITEMS];
        int8_t rows[ROW_COUNT];
        const int nrows = VisibleRows(rows);
        BuildItems(items, item_row);
        const int8_t id = items[cursor];
        const int page = item_row[cursor] / ROWS_PER_PAGE;
        const int last_page = (nrows - 1) / ROWS_PER_PAGE;

        char v[8];
        const bool buf_was_shown = buf_page_shown_;
        buf_page_shown_ = false;
        for (int r = page * ROWS_PER_PAGE; r < nrows && r < (page + 1) * ROWS_PER_PAGE; r++) {
            const Row &row = kRows[rows[r]];
            const int y = 15 + 10 * (r - page * ROWS_PER_PAGE);
            gfxPrint(1, y, row.label);
            FormatValue(row.val, v, sizeof(v));
            DrawValue(y, v, id == row.val);
            if (row.cv == FREEZE) {
                gfxStartCursor(VAL_END, y);
                gfxPrint(freeze_input);
                gfxEndCursor(id == FREEZE, true, freeze_input.InputName());
            } else if (row.cv >= 0) {
                DrawCV(y, *CvMap(row.cv), id == row.cv);
            }
            if (row.val == BUFLEN) {
                if (!buf_was_shown) free_dirty_ = true; // page just opened: refresh
                buf_page_shown_ = true;
                DrawBufferInfo(y + 10);
            }
        }

        // Page indicator
        gfxPrint(58, 56, page < last_page ? ">" : "<");

        gfxDisplayInputMapEditor();
    }

    // AuxButton: latch/unlatch manual freeze (live performance, no cable needed).
    FLASHMEM void AuxButton() override {
        manual_freeze_ ^= 1;
        CancelEdit();
    }

    FLASHMEM void OnButtonPress() override {
        int8_t items[MAX_ITEMS], item_row[MAX_ITEMS];
        BuildItems(items, item_row);
        const int8_t id = items[cursor];
        if (CheckEditInputMapPress(
                id,
                IndexedInput(POS_CV,      pos_cv),
                IndexedInput(DENSITY_CV,  density_cv),
                IndexedInput(SIZE_CV,     size_cv),
                IndexedInput(SPRAY_CV,    spray_cv),
                IndexedInput(PSPRD_CV,    psprd_cv),
                IndexedInput(PITCH_CV,    pitch_cv),
                IndexedInput(FDB_CV,      fdb_cv),
                IndexedInput(TEXTURE_CV,  texture_cv),
                IndexedInput(MIX_CV,      mix_cv),
                IndexedInput(FREEZE,      freeze_input)
            ))
            return;
        if (id == FREEZE_LATCH) {
            manual_freeze_ ^= 1;
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
            case POS:        pos     = constrain(pos     + direction,   0, 100); break;
            case POS_CV:     pos_cv.ChangeSource(direction);                      break;
            case DENSITY:    SetDensityHz(DensityHz() + direction);             break;
            case DENSITY_CV: density_cv.ChangeSource(direction);                  break;
            case SIZE:       size    = constrain(size    + direction,   1,  50); break;
            case SIZE_CV:    size_cv.ChangeSource(direction);                     break;
            case SPRAY:      spray   = constrain(spray   + direction,   0, 100); break;
            case SPRAY_CV:   spray_cv.ChangeSource(direction);                    break;
            case PSPRD:      psprd   = constrain(psprd   + direction,   0, 100); break;
            case PSPRD_CV:   psprd_cv.ChangeSource(direction);                    break;
            case PITCH:      pitch   = constrain(pitch   + direction, -24,  24); break;
            case PITCH_CV:   pitch_cv.ChangeSource(direction);                    break;
            case FDB:        fdb     = constrain(fdb     + direction,   0, 100); break;
            case FDB_CV:     fdb_cv.ChangeSource(direction);                      break;
            case TEXTURE:    texture = constrain(texture + direction,   0, 100); break;
            case TEXTURE_CV: texture_cv.ChangeSource(direction);                  break;
            case MIX:        mix     = constrain(mix     + direction,   0, 100); break;
            case MIX_CV:     mix_cv.ChangeSource(direction);                      break;
            case FREEZE:     freeze_input.ChangeSource(direction);                break;
            case LR:         lr      = constrain(lr      + direction, -99,  99); break;
            case SSPREAD:    sspread = constrain(sspread + direction,   0,  99); break;
            case BUFLEN:     buf_idx = constrain(buf_idx + direction,   0, MAX_BUF_IDX); break;
            default: break;
        }
    }

#define MISTIER_PARAMS  pos, density, size, texture, pitch, psprd, fdb, mix
    FLASHMEM void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override {
        data[0] = PackPackables(MISTIER_PARAMS);
        data[1] = PackPackables(pos_cv, density_cv, size_cv, spray_cv);
        data[2] = PackPackables(pitch_cv, fdb_cv, texture_cv, mix_cv);
        data[3] = PackPackables(freeze_input, spray, psprd_cv);
        extra_   = PackPackables(lr, sspread, buf_idx);
    }

    FLASHMEM void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override {
        UnpackPackables(data[0], MISTIER_PARAMS);
        UnpackPackables(data[1], pos_cv, density_cv, size_cv, spray_cv);
        UnpackPackables(data[2], pitch_cv, fdb_cv, texture_cv, mix_cv);
        UnpackPackables(data[3], freeze_input, spray, psprd_cv);
        UnpackPackables(extra_, lr, sspread, buf_idx);
        buf_idx = constrain(buf_idx, 0, MAX_BUF_IDX);
        lr      = constrain(lr, -99, 99);
        sspread = constrain(sspread, 0, 99);
    }
#undef MISTIER_PARAMS

    // density is stored as 0–100 (50 = silence) for preset compatibility,
    // but edited and used in whole Hz: −20..+20, 41 steps
    int DensityHz() const {
        const float f = (density - 50) * 0.4f;
        return (int)(f < 0.0f ? f - 0.5f : f + 0.5f);
    }
    FLASHMEM void SetDensityHz(int hz) {
        hz = constrain(hz, -20, 20);
        const float f = hz * 2.5f;
        density = 50 + (int)(f < 0.0f ? f - 0.5f : f + 0.5f);
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

    FLASHMEM void FormatValue(int8_t id, char *v, size_t n) {
        switch (id) {
            case MIX:     snprintf(v, n, "%d%%", mix); break;
            case DENSITY: {
                // P = periodic, S = stochastic; letter stays in place: "S20", "P 4"
                const int d = DensityHz();
                snprintf(v, n, "%c%2d", d > 0 ? 'S' : (d < 0 ? 'P' : ' '), d < 0 ? -d : d);
                break;
            }
            case LR:
                // grain source odds: L99 .. O/O (50/50) .. R99
                if (lr == 0) snprintf(v, n, "O/O");
                else snprintf(v, n, "%c%2d", lr < 0 ? 'L' : 'R', lr < 0 ? -lr : lr);
                break;
            case SSPREAD: snprintf(v, n, "%d%%", sspread); break;
            case POS:     snprintf(v, n, "%d%%", pos); break;
            case SIZE:    snprintf(v, n, "%d", size * 10); break; // ms
            case SPRAY:   snprintf(v, n, "%d%%", spray); break;
            case PSPRD:   snprintf(v, n, "%d%%", psprd); break;
            case PITCH:   snprintf(v, n, pitch > 0 ? "+%d" : "%d", pitch); break;
            case TEXTURE: snprintf(v, n, "%d%%", texture); break;
            case FDB:     snprintf(v, n, "%d%%", fdb); break;
            case FREEZE_LATCH: snprintf(v, n, manual_freeze_ ? "on" : "off"); break;
            case BUFLEN:
                if (external_psram_size) snprintf(v, n, "%ds", 1 << buf_idx);
                else snprintf(v, n, "0.5s"); // fixed small buffer in RAM
                break;
            default:      v[0] = 0; break;
        }
    }

    // line under Buf: memory the buffers really use, or that PSRAM is missing
    // lines under Buf: memory these buffers use and PSRAM still free,
    // or that there is no PSRAM
    FLASHMEM void DrawBufferInfo(int y) {
        if (!external_psram_size) {
            gfxPrint(1, y, "no PSRAM");
            return;
        }
        char t[16];
        const uint32_t bytes = grain().BufferSamples() * sizeof(int16_t) * (stereo_ ? 2 : 1);
        FormatBytes(t, sizeof(t), bytes, "used");
        gfxPrint(1, y, t);
        if (psram_free_ < 0) snprintf(t, sizeof(t), "? free");
        else FormatBytes(t, sizeof(t), (uint32_t)psram_free_, "free");
        gfxPrint(1, y + 10, t);
    }
    static void FormatBytes(char *t, size_t n, uint32_t bytes, const char *what) {
        if (bytes < 1000000) {
            snprintf(t, n, "%lukB %s", (unsigned long)((bytes + 500) / 1000), what);
        } else {
            const uint32_t tenths = (bytes + 50000) / 100000; // MB with one decimal
            snprintf(t, n, "%lu.%luMB %s", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10), what);
        }
    }

    CVInputMap *CvMap(int8_t id) {
        switch (id) {
            case MIX_CV:     return &mix_cv;
            case DENSITY_CV: return &density_cv;
            case POS_CV:     return &pos_cv;
            case SIZE_CV:    return &size_cv;
            case SPRAY_CV:   return &spray_cv;
            case PSPRD_CV:   return &psprd_cv;
            case PITCH_CV:   return &pitch_cv;
            case TEXTURE_CV: return &texture_cv;
            case FDB_CV:     return &fdb_cv;
            default:         return &mix_cv;
        }
    }

    uint64_t *ExtraData() override { return &extra_; }

    // Buffer length changes happen here, in the main loop: allocating and
    // zeroing PSRAM must not run inside the audio or controller interrupts.
    void mainloop() override;

protected:
    void SetHelp() override {}

    virtual AudioEffectClouds& grain() = 0;
    virtual void SetMixGains(float dry, float wet) = 0;

private:
    enum Cursor : int8_t {
        // Page 1
        MIX = 0, MIX_CV,
        DENSITY, DENSITY_CV,
        POS, POS_CV,
        SIZE, SIZE_CV,
        SPRAY, SPRAY_CV,
        // Page 2
        PSPRD, PSPRD_CV,
        PITCH, PITCH_CV,
        TEXTURE, TEXTURE_CV,
        FDB, FDB_CV,
        FREEZE_LATCH,
        FREEZE,
        LR,
        SSPREAD,
        BUFLEN,
        CURSOR_LENGTH,
    };

    // Screen rows in display order; stereo-only rows are skipped in mono.
    struct Row { const char *label; int8_t val; int8_t cv; bool stereo_only; };
    static constexpr int ROW_COUNT = 13;
    static constexpr int ROWS_PER_PAGE = 5;
    static constexpr int MAX_ITEMS = ROW_COUNT * 2;
    static constexpr Row kRows[ROW_COUNT] = {
        { "Mix:", MIX,          MIX_CV,     false },
        { "Den:", DENSITY,      DENSITY_CV, false },
        { "L+R:", LR,           -1,         true  },
        { "SSp:", SSPREAD,      -1,         true  },
        { "Pos:", POS,          POS_CV,     false },
        { "Siz:", SIZE,         SIZE_CV,    false },
        { "Spr:", SPRAY,        SPRAY_CV,   false },
        { "PSp:", PSPRD,        PSPRD_CV,   false },
        { "Pit:", PITCH,        PITCH_CV,   false },
        { "Tex:", TEXTURE,      TEXTURE_CV, false },
        { "Fdb:", FDB,          FDB_CV,     false },
        { "Frz:", FREEZE_LATCH, FREEZE,     false },
        { "Buf:", BUFLEN,       -1,         false },
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
            items[n] = row.val; item_row[n++] = r;
            if (row.cv >= 0) { items[n] = row.cv; item_row[n++] = r; }
        }
        return n;
    }

    const bool stereo_;
    int8_t cursor = 0;

    // Parameters
    int8_t  pos      = 50;  // 0–100%
    CVInputMap pos_cv;
    int8_t  density  = 75;  // 0–100 (50=silence; 0.4*(val-50) Hz, >50=stochastic)
    CVInputMap density_cv;
    int8_t  size     = 15;  // 1–50 (×10ms = 10–500ms)
    CVInputMap size_cv;
    int8_t  spray    = 20;  // 0–100% position scatter
    CVInputMap spray_cv;
    int8_t  psprd    = 0;   // 0–100% pitch spread
    CVInputMap psprd_cv;
    int8_t  pitch    = 0;   // −24 to +24 semitones
    CVInputMap pitch_cv;
    int8_t  fdb      = 0;   // 0–100% grain feedback
    CVInputMap fdb_cv;
    int8_t  texture  = 50;  // 0–100% (0=short fades, 50=long fade-out, 100=Hann)
    CVInputMap texture_cv;
    int8_t  mix      = 80;  // 0–100% wet/dry
    CVInputMap mix_cv;
    DigitalInputMap freeze_input;

    int8_t  lr       = 0;   // grain source odds: L99 (left) .. 0 (50/50) .. R99 (right)
    int8_t  sspread  = 0;   // 0–99% stereo spread of grains, stereo
    int8_t  buf_idx  = 0;   // buffer length 1 << buf_idx seconds (1, 2, 4, 8)
    // free-PSRAM readout: counting it walks the whole pool (slow), so it is
    // only refreshed when the Buf page opens or the buffer changes
    int32_t psram_free_ = -1;
    bool    free_dirty_ = false;
    bool    buf_page_shown_ = false;
    static constexpr int MAX_BUF_IDX = 3;
    uint64_t extra_  = 0;   // 5th preset word: lr, sspread, buf_idx
    bool manual_freeze_ = false;  // latched by encoder press on Frz or AuxButton
};

FLASHMEM void MistierBase::mainloop() {
    if (!external_psram_size) return; // without PSRAM stay at the small buffer
    const size_t want = AudioEffectClouds::CLOUDS_BUFFER_SAMPLES << buf_idx;
    if (grain().BufferSamples() != want) {
        if (!grain().Resize(want)) {
            // not enough memory: go back to the length that is actually in use
            for (int i = 0; i <= MAX_BUF_IDX; i++)
                if ((AudioEffectClouds::CLOUDS_BUFFER_SAMPLES << i) == grain().BufferSamples())
                    buf_idx = i;
        }
        free_dirty_ = true;
    }
    if (free_dirty_ && buf_page_shown_) {
        size_t total = 0, user = 0, free_bytes = 0;
        int blocks = 0;
        sm_malloc_stats_pool(&extmem_smalloc_pool, &total, &user, &free_bytes, &blocks);
        psram_free_ = (int32_t)free_bytes;
        free_dirty_ = false;
    }
}

template <AudioChannels Channels>
class MistierApplet : public MistierBase {
public:
    MistierApplet() : MistierBase(Channels > 1) {}

    void Start() override {
        grain_stream.Acquire();
        for (int ch = 0; ch < Channels; ch++) {
            PatchCable(input_stream, ch, grain_stream, ch);
            PatchCable(input_stream, ch, mixer[ch],    DRY_CH);
            PatchCable(grain_stream, ch, mixer[ch],    WET_CH);
            PatchCable(mixer[ch],    0,  output_stream, ch);
        }
    }

    void Unload() override {
        grain_stream.Release();
        AllowRestart();
    }

    AudioStream* InputStream()  override { return &input_stream; }
    AudioStream* OutputStream() override { return &output_stream; }

protected:
    AudioEffectClouds& grain() override { return grain_stream; }
    void SetMixGains(float dry, float wet) override {
        for (int ch = 0; ch < Channels; ch++) {
            mixer[ch].gain(DRY_CH, dry);
            mixer[ch].gain(WET_CH, wet);
        }
    }

private:
    // DSP: one grain engine for all channels, plus a dry/wet mixer per channel.
    static const uint8_t DRY_CH = 0;
    static const uint8_t WET_CH = 1;

    AudioEffectClouds grain_stream{
        external_psram_size
            ? AudioEffectClouds::CLOUDS_BUFFER_SAMPLES
            : AudioEffectClouds::CLOUDS_BUFFER_SAMPLES / 2,
        Channels };
    AudioMixer<2> mixer[Channels];

    AudioPassthrough<Channels> input_stream;
    AudioPassthrough<Channels> output_stream;
};
