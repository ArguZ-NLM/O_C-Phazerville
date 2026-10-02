// Copyright (c) 2026, Sascha A W Haber
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Sculpt: full-screen stereo audio app. Step 1 is the VAST page: a long
// extmem delay with a tone filter in the feedback loop, ping-pong spread,
// diffusion and freeze, followed by a reverb.
//
// While Sculpt is the active app it claims the audio outputs from AudioIO
// (Input -> Vast -> Output). The Quadrants audio applet chain is left intact
// and comes back as soon as Sculpt is suspended.

#include "../HSApplication.h"
#include "../AudioIO.h"
#include "../OC_gpio.h"
#include "../Audio/AudioDelayExt.h"
#include "../Audio/AudioEffectVast.h"

namespace sculpt {
  // same template args as the Delay applet, so the code is shared
  using DelayStream = AudioDelayExt<9>;
  constexpr float MAX_DELAY_SECS = 8.0f;
  constexpr size_t DELAY_SAMPLES
    = static_cast<size_t>(MAX_DELAY_SECS * AUDIO_SAMPLE_RATE_EXACT) + 512;
  constexpr int TAPS = AudioEffectVast::TAPS;

  // order matters: Vast updates before the delays it feeds
  AudioEffectVast vast;
  DelayStream delays[2] = {DelayStream(DELAY_SAMPLES), DelayStream(DELAY_SAMPLES)};

  // patch cords live on the heap: no RAM1 until Sculpt is first opened
  struct Patch {
    AudioConnection in[2];
    AudioConnection write[2];
    AudioConnection taps[2][TAPS];
  };
  Patch* patch = nullptr;

  bool audio_hw() { return I2S2_Audio_ADC && I2S2_Audio_DAC; }
}

OC_APP_CLASS(AppSculpt, TWOCCS("SP"), "Sculpt", "Sculpt"),
  public HSApplication {
public:
  OC_APP_INTERFACE_DECLARE(AppSculpt, 0);

  enum Param : uint8_t {
    TIME, FEEDBACK, TONE, SPREAD, DIFFUSE, REVERB, DECAY, MIX, FREEZE,
    PARAM_COUNT
  };

  void Start();
  void Resume();
  void Suspend();
  void Controller() {}
  void MainLoop();

  void View() const;
  void DrawVast() const;

  /////////////////////////////////////////////////////////////////
  // Control handlers
  /////////////////////////////////////////////////////////////////
  void OnLeftEncoderMove(int direction);
  void OnRightEncoderMove(int direction);
  void OnRightButtonPress();

private:
  int8_t cursor = TIME;
  bool active = false;
  bool vast_mem = false, delay_mem = false;
  elapsedMillis stats_timer;

  // parameter values, in UI units
  int time_ms = 600;
  int8_t feedback = 45;
  int8_t tone = -10;   // <0 darker, >0 thinner
  int8_t spread = 30;  // ping-pong amount
  int8_t diffuse = 20;
  int8_t reverb = 35;
  uint8_t decay = 40;  // tenths of a second
  int8_t mix = 50;
  bool freeze = false;

  // decaying display copies of the viz levels
  mutable float disp_tap[2][sculpt::TAPS] = {};
  mutable float disp_tail = 0.0f;

  void ApplyAll();
  void Apply(int p);

  void PrintValue(int p, int x, int y) const;
};

FLASHMEM void AppSculpt::Start() {
  ApplyAll();
}

FLASHMEM void AppSculpt::Resume() {
  if (!sculpt::audio_hw() || active) return;
  using namespace sculpt;
  if (!patch) patch = new Patch;
  AudioNoInterrupts();
  vast_mem = vast.Acquire();
  for (auto& d : delays) d.Acquire();
  delay_mem = delays[0].ready() && delays[1].ready();
  for (int c = 0; c < 2; c++) {
    delays[c].taps(TAPS);
    patch->in[c].connect(OC::AudioIO::InputStream(), c, vast, c);
    patch->write[c].connect(vast, 2 + c, delays[c], 0);
    for (int t = 0; t < TAPS; t++) {
      patch->taps[c][t].connect(delays[c], t, vast, 2 + c * TAPS + t);
    }
  }
  AudioInterrupts();
  ApplyAll();
  OC::AudioIO::ClaimOutput(vast);
  active = true;
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
}

FLASHMEM void AppSculpt::Suspend() {
  if (!active) return;
  using namespace sculpt;
  OC::AudioIO::ReleaseOutput();
  AudioNoInterrupts();
  for (int c = 0; c < 2; c++) {
    patch->in[c].disconnect();
    patch->write[c].disconnect();
    for (auto& tc : patch->taps[c]) tc.disconnect();
  }
  AudioInterrupts();
  // disconnected streams no longer update, so freeing is safe now
  for (auto& d : delays) d.Release();
  vast.Release();
  active = false;
}

FLASHMEM void AppSculpt::MainLoop() {
  if (!active) return;
  if (stats_timer >= 1000) {
    stats_timer = 0;
    const float cpu = AudioProcessorUsageMax();
    Serial.printf("Sculpt: AudioProcessorUsageMax %d.%d%%  mem %u/%d blocks\n",
      int(cpu), int(cpu * 10) % 10, AudioMemoryUsageMax(), OC::AudioIO::AUDIO_MEMORY);
    AudioProcessorUsageMaxReset();
    AudioMemoryUsageMaxReset();
  }
}

FLASHMEM void AppSculpt::OnLeftEncoderMove(int direction) {
  cursor = constrain(cursor + direction, 0, PARAM_COUNT - 1);
}

FLASHMEM void AppSculpt::OnRightEncoderMove(int direction) {
  switch (cursor) {
    case TIME: {
      // ~3% per detent, so the whole 20 ms..8 s range is reachable
      const int step = std::max(1, time_ms / 32);
      time_ms = constrain(time_ms + direction * step, 20, 8000);
    } break;
    case FEEDBACK: feedback = constrain(feedback + direction, 0, 100); break;
    case TONE:     tone = constrain(tone + direction, -50, 50); break;
    case SPREAD:   spread = constrain(spread + direction, 0, 100); break;
    case DIFFUSE:  diffuse = constrain(diffuse + direction, 0, 100); break;
    case REVERB:   reverb = constrain(reverb + direction, 0, 100); break;
    case DECAY:    decay = constrain(decay + direction, 3, 200); break;
    case MIX:      mix = constrain(mix + direction, 0, 100); break;
    case FREEZE:   freeze = direction > 0; break;
  }
  Apply(cursor);
}

FLASHMEM void AppSculpt::OnRightButtonPress() {
  freeze = !freeze;
  Apply(FREEZE);
}

FLASHMEM void AppSculpt::ApplyAll() {
  for (int p = 0; p < PARAM_COUNT; p++) Apply(p);
}

FLASHMEM void AppSculpt::Apply(int p) {
  using namespace sculpt;
  switch (p) {
    case TIME: {
      const float secs = time_ms * 0.001f;
      for (int c = 0; c < 2; c++) {
        for (int t = 0; t < TAPS; t++) {
          delays[c].delay(t, secs * AudioEffectVast::TAP_POS[c][t]
                             - AudioEffectVast::LOOP_LATENCY_SECS);
        }
      }
    } break;
    case FEEDBACK: vast.feedback(feedback * 0.01f); break;
    case TONE:     vast.tone(tone * 0.02f); break;
    case SPREAD:   vast.spread(spread * 0.01f); break;
    case DIFFUSE:  vast.diffuse(diffuse * 0.01f); break;
    case REVERB:   vast.reverb(reverb * 0.01f); break;
    case DECAY:    vast.decay(decay * 0.1f); break;
    case MIX:      vast.mix(mix * 0.01f); break;
    case FREEZE:   vast.set_freeze(freeze); break;
  }
}

FLASHMEM void AppSculpt::PrintValue(int p, int x, int y) const {
  char buf[12];
  switch (p) {
    case TIME:
      if (time_ms < 1000) snprintf(buf, sizeof(buf), "%dms", time_ms);
      else snprintf(buf, sizeof(buf), "%d.%02ds", time_ms / 1000, (time_ms % 1000) / 10);
      break;
    case TONE:
      if (tone == 0) snprintf(buf, sizeof(buf), "flat");
      else snprintf(buf, sizeof(buf), "%s%d", tone < 0 ? "LP" : "HP", abs(tone));
      break;
    case DECAY:
      snprintf(buf, sizeof(buf), "%d.%ds", decay / 10, decay % 10);
      break;
    case FREEZE:
      snprintf(buf, sizeof(buf), "%s", freeze ? "ON" : "off");
      break;
    default: {
      const int8_t v[] = {0, feedback, 0, spread, diffuse, reverb, 0, mix};
      snprintf(buf, sizeof(buf), "%d%%", v[p]);
    } break;
  }
  gfxPrint(x - 6 * int(strlen(buf)), y, buf);
}

FLASHMEM void AppSculpt::DrawVast() const {
  using namespace sculpt;
  static const char* const names[PARAM_COUNT] = {
    "Time", "Feedback", "Tone", "Spread", "Diffuse", "Reverb", "Decay", "Mix", "Freeze"
  };

  AudioEffectVast::Viz v;
  vast.ReadViz(v);

  gfxHeader("Sculpt: VAST");
  if (freeze) {
    gfxPrint(104, 1, "FRZ");
    gfxInvert(103, 0, 21, 9);
  }

  // tap timeline: bar position = tap time, height = level (sqrt for a
  // rough dB feel), L filled, R outlined
  constexpr int base = 46, h_max = 32, x0 = 2, w = 92;
  gfxDottedLine(x0, base, x0 + w + 4, base, 2);
  if (!active) {
    gfxPrint(x0 + 10, 26, "no audio I/O");
  } else if (!vast_mem || !delay_mem) {
    gfxPrint(x0 + 10, 26, "out of memory");
  }
  for (int c = 0; c < 2; c++) {
    for (int t = 0; t < TAPS; t++) {
      float& d = disp_tap[c][t];
      d = std::max(sqrtf(v.tap[c][t]), d * 0.88f);
      const int h = constrain(int(d * h_max), 0, h_max);
      const int x = x0 + int(AudioEffectVast::TAP_POS[c][t] * w) - 3 + c * 4;
      if (h > 0) {
        if (c == 0) gfxRect(x, base - h, 3, h);
        else gfxFrame(x, base - h, 3, h);
      }
    }
  }

  // tail meter
  disp_tail = std::max(sqrtf(v.tail), disp_tail * 0.94f);
  const int th = constrain(int(disp_tail * h_max), 0, h_max);
  gfxFrame(108, base - h_max - 1, 14, h_max + 2);
  if (th > 0) gfxRect(110, base - th, 10, th);
  gfxPrint(103, base + 1, "tail");

  // parameter line
  gfxPrint(1, 55, names[cursor]);
  PrintValue(cursor, 98, 55);
  // position dots
  for (int p = 0; p < PARAM_COUNT; p++) {
    if (p == cursor) gfxRect(1 + p * 4, 63, 3, 1);
    else gfxPixel(2 + p * 4, 63);
  }
}

FLASHMEM void AppSculpt::View() const { DrawVast(); }

FLASHMEM void AppSculpt::Init() {
  BaseStart();
}

size_t AppSculpt::SaveAppData(util::StreamBufferWriter &) const { return 0; }
size_t AppSculpt::RestoreAppData(util::StreamBufferReader &) { return 0; }

void AppSculpt::Process(OC::IOFrame *ioframe) {
  BaseController(ioframe);
}

FLASHMEM void AppSculpt::HandleAppEvent(OC::AppEvent event) {
  if (event == OC::APP_EVENT_RESUME) Resume();
  if (event == OC::APP_EVENT_SUSPEND) Suspend();
}

FLASHMEM void AppSculpt::Loop() {
  MainLoop();
}

FLASHMEM void AppSculpt::GetIOConfig(OC::IOConfig &) const { }
void AppSculpt::DrawDebugInfo() const { }

FLASHMEM void AppSculpt::DrawMenu() const { BaseView(); }

FLASHMEM void AppSculpt::DrawScreensaver() const { BaseView(); }

FLASHMEM void AppSculpt::HandleButtonEvent(const UI::Event &event) {
  if (UI::EVENT_BUTTON_PRESS != event.type) return;
  switch (event.control) {
    case OC::CONTROL_BUTTON_R:
      OnRightButtonPress();
      break;
    default: break;
  }
}

FLASHMEM void AppSculpt::HandleEncoderEvent(const UI::Event &event) {
  if (event.control == OC::CONTROL_ENCODER_L) OnLeftEncoderMove(event.value);
  if (event.control == OC::CONTROL_ENCODER_R) OnRightEncoderMove(event.value);
}
