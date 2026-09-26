#include "synth_waveform.h"

// Per-oscillator detune scale factors.
// 12 oscillators split into 4 voices of 3 each.
// Detune: signed multiplier applied to the detune amount in Hz.
// Per-oscillator scale factors — file-scope constexpr, no linkage issues
static constexpr float HANDSAW_DETUNE[12] PROGMEM = {  0,  3, -2,   1,  4, -5,  -1,  2, -3,   6,  5, -4 };
static constexpr float HANDSAW_SWARM_HZ[12] PROGMEM = {
     0.31f, -0.07f, -0.24f,
    -0.29f,  0.13f,  0.16f,
     0.43f, -0.17f, -0.26f,
    -0.41f,  0.19f,  0.22f,
};

class HandSawApplet : public HemisphereAudioApplet {
    public:
        const char* applet_name() {
            return "HandSaw";
        }

        void Start() override {
            vca_level.Acquire();
            vca_level.Method(INTERPOLATION_LINEAR);
            swarm_timer = 0;

            // Making audio connections...
            // Call order follows signal flow: sources before sinks,
            // so the audio scheduler can process them in one pass.
            for (int i = 0; i < 12; i++) {
              PatchCable(synths[i], 0, outputMixer, i);
            }

            PatchCable(outputMixer,   0, vca,         0);
            PatchCable(vca_level,     0, vca,         1);

            PatchCable(vca,           0, final_out, 0);
            PatchCable(input_stream,  0, final_out, 1);

            final_out.gain(0, 1.0f); // voice
            final_out.gain(1, 1.0f); // passthru

            for (int i = 0; i < 12; i++) {
                synths[i].amplitude(1.0f);
            }

            // 3 oscillators per voice; normalise to 1/3
            // 4 voices; normalise to 1/4
            for (int ch = 0; ch < 12; ch++) {
                outputMixer.gain(ch, 0.25f / 3);
            }
        }

        void Unload() override {
            vca_level.Release();
            AllowRestart();
        }

        void Controller() override {
            const float dt = swarm_timer * 1e-6f;
            swarm_timer = 0;
            const float realign = 1.0f - expf(-dt / REALIGN_S);

            float detuneValue = constrain(detune + detune_cv.In() * 0.01f, -DETUNE_MAX, DETUNE_MAX);
            float detuneHz = DetuneHz(detuneValue);
            float amount = constrain((swarm + swarm_cv.In() * SWARM_MAX / 7680.0f) / SWARM_MAX, 0.0f, 1.0f);
            amount *= amount;

            for (int v = 0; v < 4; v++) {
                int cvmod = pitch_cv[v].In();
                if (!pitch_cv[v].enabled()) cvmod = pitch_cv[0].In();

                float freq = PitchToRatio(pitch[v] + cvmod) * C3;
                for (int o = 0; o < 3; o++) {
                    int idx = v * 3 + o;
                    float offset = 0.0f;
                    if (amount > 0.0f) {
                        offset = HANDSAW_SWARM_HZ[idx] * amount;
                        drift[idx] = WrapCycles(drift[idx] + offset * dt);
                    } else {
                        correction[idx] = WrapCycles(correction[idx] + WrapCycles(-drift[idx] - correction[idx]) * realign);
                    }
                    synths[idx].frequency(freq + offset + HANDSAW_DETUNE[idx] * detuneHz);
                    synths[idx].phase(WrapDegrees(correction[idx] * 360.0f));
                }
            }

            float m = amp < LVL_MIN_DB ? 0.0f : dbToScalar(amp);
            m += (amp_cv.InF() * amp_cv.InF());
            vca_level.Push(float_to_q15(m));
        }

        FLASHMEM void View() override {

            gfxPrint(0, 25, "Wave:");
            gfxStartCursor();
            gfxPrint(WAVEFORM_NAMES[waveform]);
            gfxEndCursor(cursor == WAVEFORM);

            // voices 2-4 follow voice 1, keeping their intervals
            gfxStartCursor(54, 25);
            if (link) gfxIcon(54, 25, LINK_ICON);
            else gfxPrint("-");
            gfxPos(62, 25);
            gfxEndCursor(cursor == LINK, false, link ? "Linked" : "Unlinked");

            // right-aligned so the hundredths digit stays put
            char hz[8];
            const float hz100 = DetuneHz(detune) * 100.0f;
            const int cents = int(hz100 < 0.0f ? hz100 - 0.5f : hz100 + 0.5f);
            snprintf(hz, sizeof(hz), "%d.%02d", abs(cents) / 100, abs(cents) % 100);
            gfxPrint(0, 35, "DT");
            gfxIcon(12, 35, HZ_ICON);
            gfxPrint(19, 35, ":");
            // numbers end 2px into the CV icon's blank left margin
            const int hz_x = 58 - 6 * strlen(hz);
            gfxStartCursor(cents < 0 ? hz_x - 3 : hz_x, 35);
            if (cents < 0) gfxLine(hz_x - 3, 38, hz_x - 1, 38); // half-width minus
            gfxPos(hz_x, 35);
            gfxPrint(hz);
            gfxEndCursor(cursor == DETUNE);

            gfxStartCursor(56, 35);
            gfxPrint(detune_cv);
            gfxEndCursor(cursor == DETUNE_CV, false, detune_cv.InputName());

            gfxPrint(1, 45, "Swarm:");
            gfxStartCursor(swarm < 10 ? 52 : 46, 45);
            graphics.printf("%d", swarm);
            gfxEndCursor(cursor == SWARM);

            gfxStartCursor(56, 45);
            gfxPrint(swarm_cv);
            gfxEndCursor(cursor == SWARM_CV, false, swarm_cv.InputName());

            gfxPrint(1, 55, "Amp:");
            gfxStartCursor(28, 55);
            gfxPrintDb(amp);
            gfxEndCursor(cursor == AMP);

            gfxStartCursor(56, 55);
            gfxPrint(amp_cv);
            gfxEndCursor(cursor == AMP_CV, false, amp_cv.InputName());

            // Draw voice pitch and CV maps last, so cursors sit on top
            for (int i = 0; i < 4; ++i) {
                gfxStartCursor(1 + 15*i, 15);
                gfxPrintTuningIndicator(pitch[i]);
                gfxEndCursor(cursor == PITCH1 + i);
                if (cursor == PITCH1 + i) {
                    gfxIcon(1 + 15*i, 25, UP_ICON, true);
                }
            }
            // CV mappings sit on top of the pitch row
            for (int i = 0; i < 4; ++i) {
                gfxStartCursor(10 + 15*i, 15);
                gfxPrint(pitch_cv[i]);
                gfxEndCursor(cursor == PITCH_CV1 + i, false, pitch_cv[i].InputName());
            }

            gfxDisplayInputMapEditor();
        }

    #define SWARM_OSC_PARAMS \
        pitch[0], pitch[1], pitch[2], pitch[3]

        FLASHMEM void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override {
            data[0] = PackPackables(SWARM_OSC_PARAMS);
            data[1] = PackPackables(pitch_cv[0], pitch_cv[1], pitch_cv[2], pitch_cv[3]);
            data[2] = PackPackables(detune_cv, swarm_cv, amp_cv);
            data[3] = PackPackables(waveform, detune, swarm, link, amp);
        }

        FLASHMEM void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override {
            UnpackPackables(data[0], SWARM_OSC_PARAMS);
            UnpackPackables(data[1], pitch_cv[0], pitch_cv[1], pitch_cv[2], pitch_cv[3]);
            UnpackPackables(data[2], detune_cv, swarm_cv, amp_cv);
            UnpackPackables(data[3], waveform, detune, swarm, link, amp);
            detune = constrain(detune, -DETUNE_MAX, DETUNE_MAX);
            swarm = constrain(swarm, 0, SWARM_MAX);
            SetWaveform(waveform);
        }

        FLASHMEM void AuxButton() override {
            switch (cursor) {
                case PITCH1:
                case PITCH2:
                case PITCH3:
                case PITCH4: {
                    // shortcut to snap to closest semitone
                    auto& p = pitch[cursor - PITCH1];
                    int old = p;
                    p = (((p + 63) >> 7) << 7);
                    if (link && cursor == PITCH1) ShiftLinked(p - old);
                    break;
                }
                case PITCH_CV1:
                case PITCH_CV2:
                case PITCH_CV3:
                case PITCH_CV4: {
                    // shortcut for auto-learn
                    auto& p = pitch_cv[cursor - PITCH_CV1];
                    p.AutoLearn();
                    break;
                }
            }
        }

        FLASHMEM void OnButtonLongPress() override {
            if (cursor < PITCH1 || cursor > PITCH4) return;
            auto& p = pitch[cursor - PITCH1];
            int old = p;
            p = DEFAULT_PITCH;
            if (link && cursor == PITCH1) ShiftLinked(p - old);
        }

        FLASHMEM void OnButtonPress() override {
            if (CheckEditInputMapPress(cursor,
                IndexedInput(DETUNE_CV, detune_cv),
                IndexedInput(SWARM_CV,  swarm_cv),
                IndexedInput(AMP_CV,    amp_cv)
            ))
                return;
            if (cursor == LINK) {
                link = !link;
                return;
            }
            CursorToggle();
        }

        static constexpr int max_pitch =  7 * 12 * 128;
        static constexpr int min_pitch = -3 * 12 * 128;

        // move voices 2-4 along with voice 1
        void ShiftLinked(int delta) {
            for (int v = 1; v < 4; v++) {
                pitch[v] = constrain(pitch[v] + delta, min_pitch, max_pitch);
            }
        }

        static float DetuneHz(float value) {
            float hz = DETUNE_MAX_HZ * (expf(DETUNE_TAPER * fabsf(value) / DETUNE_MAX) - 1.0f)
                                     / (expf(DETUNE_TAPER) - 1.0f);
            return value < 0.0f ? -hz : hz;
        }

        // wrap into -0.5 .. +0.5 cycles
        static double WrapCycles(double c) {
            return c - floor(c + 0.5);
        }

        static float WrapDegrees(float deg) {
            return deg < 0.0f ? deg + 360.0f : deg;
        }

        void SetWaveform(int wf) {
            waveform = constrain(wf, 0, 4);
            for (int i = 0; i < 12; i++) {
                synths[i].begin(WAVEFORMS[waveform]);
                if (2 == waveform) // saw
                  synths[i].pulseWidth(0.0f);
                else if (4 == waveform) // rev saw
                  synths[i].pulseWidth(1.0f);
                else // triangle or pulse
                  synths[i].pulseWidth(0.5f);
            }
        }

        FLASHMEM void OnEncoderMove(int direction) override {
            if (!EditMode()) {
                MoveCursor(cursor, direction, AMP_CV);
                return;
            }
            if (EditSelectedInputMap(direction)) return;

            switch (cursor) {
                case PITCH1: {
                    int old = pitch[0];
                    pitch[0] = constrain(pitch[0] + direction * 4, min_pitch, max_pitch);
                    if (link) ShiftLinked(pitch[0] - old);
                    break;
                }
                case PITCH_CV1:
                    pitch_cv[0].ChangeSource(direction);
                    break;
                case PITCH2:
                    pitch[1] = constrain(pitch[1] + direction * 4, min_pitch, max_pitch);
                    break;
                case PITCH_CV2:
                    pitch_cv[1].ChangeSource(direction);
                    break;
                case PITCH3:
                    pitch[2] = constrain(pitch[2] + direction * 4, min_pitch, max_pitch);
                    break;
                case PITCH_CV3:
                    pitch_cv[2].ChangeSource(direction);
                    break;
                case PITCH4:
                    pitch[3] = constrain(pitch[3] + direction * 4, min_pitch, max_pitch);
                    break;
                case PITCH_CV4:
                    pitch_cv[3].ChangeSource(direction);
                    break;
                case WAVEFORM:
                    SetWaveform(waveform + direction);
                    break;
                case DETUNE:
                    detune = constrain(detune + direction, -DETUNE_MAX, DETUNE_MAX);
                    break;
                case DETUNE_CV:
                    detune_cv.ChangeSource(direction);
                    break;
                case SWARM:
                    swarm = constrain(swarm + direction, 0, SWARM_MAX);
                    break;
                case SWARM_CV:
                    swarm_cv.ChangeSource(direction);
                    break;
                case AMP:
                    amp = constrain(amp + direction, LVL_MIN_DB - 1, 0);
                    break;
                case AMP_CV:
                    amp_cv.ChangeSource(direction);
                    break;
                default:
                    break;
            }
        }

        AudioStream* InputStream()  override { return &input_stream; }
        AudioStream* OutputStream() override { return &final_out; }

    protected:
        void SetHelp() override {}

    private:
        enum Cursor: int8_t {
            PITCH1,
            PITCH2,
            PITCH3,
            PITCH4,
            PITCH_CV1,
            PITCH_CV2,
            PITCH_CV3,
            PITCH_CV4,
            WAVEFORM,
            LINK,
            DETUNE,
            DETUNE_CV,
            SWARM,
            SWARM_CV,
            AMP,
            AMP_CV
        };

        static constexpr int8_t WAVEFORMS[5] = {
            WAVEFORM_SINE,
            WAVEFORM_TRIANGLE_VARIABLE, // actual triangle
            WAVEFORM_TRIANGLE_VARIABLE, // saw
            WAVEFORM_PULSE,
            WAVEFORM_TRIANGLE_VARIABLE, // reverse saw
        };
        static constexpr char const* WAVEFORM_NAMES[5] = {"SIN", "TRI", "SAW", "PLS", "SAWR"};

        uint8_t  waveform = WAVEFORM_SINE;
        int8_t   cursor   = PITCH1;
        static constexpr int16_t DEFAULT_PITCH = -1 * 12 * 128; // C2
        int16_t  pitch[4] = { DEFAULT_PITCH, DEFAULT_PITCH, DEFAULT_PITCH, DEFAULT_PITCH };

        int16_t detune = 0;
        int16_t swarm  = 0;
        int8_t  amp    = 0;
        int8_t  link   = 0;

        // TODO:
        // uint8_t pw = 50;

        CVInputMap pitch_cv[4];
        CVInputMap detune_cv;
        CVInputMap swarm_cv;
        CVInputMap amp_cv;

        AudioPassthrough<MONO>  input_stream;
        AudioSynthWaveform      synths[12];
        AudioMixer<12>          outputMixer;
        AudioVCA                vca;
        InterpolatingStream<>   vca_level;
        AudioMixer<2>           final_out;

        static constexpr int   DETUNE_MAX    = 99;
        static constexpr float DETUNE_MAX_HZ = 40.0f; // per unit of HANDSAW_DETUNE
        static constexpr float DETUNE_TAPER  = 4.4f;  // half knob = 1/10 of full detune
        static constexpr int   SWARM_MAX = 50;
        static constexpr float REALIGN_S = 0.2f;
        double drift[12] = {};
        double correction[12] = {};
        elapsedMicros swarm_timer;
};
