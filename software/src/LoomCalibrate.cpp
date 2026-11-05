// Loom firmware: the CV calibration, Phazerville's steps and data without its
// Settings app (that one costs RAM1 the Loom build doesn't have).
//
// Started by holding L at power-up (the splash screen's "[L] => Calibrate"),
// before Loom itself starts. The same steps, screens and stored data as
// Phazerville's (OC_calibration.cpp: calibration_steps, calibration_data,
// calibration_save), so a module calibrated here and one calibrated in
// Phazerville read the same values:
//   - start fresh? (yes: back to the defaults; DAC A's high point then
//     copied to B..H to start from)
//   - center display
//   - each CV output, its low and its high point: R turns the DAC value
//     until a voltmeter on the jack reads the voltage shown (X held: x16,
//     Y held: x32); L turns through the octaves in between; B held fills the
//     octaves between from the two points
//   - CV input 0 V offsets (B held: take them now), CV1 at +1 V and +3 V
//     (B held at each: take it)
//   - screen blank minutes, then save? yes / no
// R push: next step, L push: back (on the first page: leave without
// calibrating). Everything here runs from flash.

#include <Arduino.h>
#include "OC_strings.h"
#include "OC_apps.h"
#include "OC_calibration.h"
#include "OC_config.h"
#include "OC_core.h"
#include "OC_menus.h"
#include "OC_ui.h"
#include "OC_options.h"
#include "OC_DAC.h"
#include "OC_ADC.h"
#include "src/drivers/display.h"

#ifdef LOOM_ONLY

namespace loom {

namespace menu = OC::menu;

namespace {

struct Cal {
  OC::CalibrationState st;
  int octave = 0;          // the octave being set (CALIBRATE_OCTAVE)
  bool done = false;
};

// the step, as Phazerville's calibration_steps has it, but built here with
// its words in flash (that table's words would sit in RAM1, ~1 KB, and shift
// an aligned buffer by 3 KB more). Two slots: the step left and the one
// entered are both read while switching
FLASHMEM const OC::CalibrationStep* StepAt(int i) {
  static OC::CalibrationStep slot[2];
  static char title[2][24];
  static int k = 0;
  k ^= 1;
  OC::CalibrationStep& s = slot[k];
  s = OC::CalibrationStep{};
  s.step = OC::CALIBRATION_STEP(i);
  s.title = title[k];
  s.message = PSTR("");
  s.help = PSTR("[R] => Adjust");
  s.footer = PSTR("[PREV]         [NEXT]");
  s.value_str = nullptr;
  const char* hold = PSTR("Hold [B] to set");
  if (i >= OC::DAC_A_VOLT_MIN && i <= OC::DAC_H_VOLT_HIGH) {
    const int ch = (i - OC::DAC_A_VOLT_MIN) / 2;
    const bool high = (i - OC::DAC_A_VOLT_MIN) & 1;
    snprintf(title[k], sizeof(title[k]), high ? PSTR("DAC %c (high)") : PSTR("DAC %c (min)"), 'A' + ch);
    s.calibration_type = OC::CALIBRATE_OCTAVE;
    s.index = high ? 8 : 0;
    if (high) s.help = hold;
    s.min = 0; s.max = OC::DAC::MAX_VALUE;
    return &s;
  }
  auto name = [&](const char* n) { strlcpy(title[k], n, sizeof(title[k])); };
  switch (i) {
    case OC::HELLO:
      name(PSTR("Setup: Calibrate")); s.message = PSTR("Start fresh? ");
      s.help = PSTR("[R] => Select"); s.footer = PSTR("[CANCEL]         [OK]");
      s.calibration_type = OC::CALIBRATE_NONE; s.min = 0; s.max = 1;
      break;
    case OC::CENTER_DISPLAY:
      name(PSTR("Center Display")); s.message = PSTR("Pixel offset ");
      s.calibration_type = OC::CALIBRATE_DISPLAY; s.min = 0; s.max = 2;
      break;
    case OC::ADC_OFFSETS:
      name(PSTR("ADC input 0V offset")); s.help = hold;
      s.calibration_type = OC::CALIBRATE_ADC_OFFSET; s.min = 0; s.max = 4095;
      break;
    case OC::ADC_PITCH_C2:
      name(PSTR("ADC octave +1")); s.message = NorthernLightModular ? PSTR("CV1: Input 1.2V") : PSTR("CV1: Input 1.0V"); s.help = hold;
      s.calibration_type = OC::CALIBRATE_ADC_1V; s.index = 1;
      break;
    case OC::ADC_PITCH_C4:
      name(PSTR("ADC octave +3")); s.message = NorthernLightModular ? PSTR("CV1: Input 3.6V") : PSTR("CV1: Input 3.0V"); s.help = hold;  // (1.2 V an octave on the Buchla)
      s.calibration_type = OC::CALIBRATE_ADC_3V; s.index = 3;
      break;
    case OC::CALIBRATION_SCREENSAVER_TIMEOUT:
      name(PSTR("Screen Blank")); s.message = PSTR("(minutes)");
      s.calibration_type = OC::CALIBRATE_SCREENSAVER;
      s.min = (OC::Ui::kLongPressTicks * 2 + 500) / 1000; s.max = SCREENSAVER_TIMEOUT_MAX_S;
      break;
    default:  // CALIBRATION_EXIT
      name(PSTR("Calibration complete")); s.message = PSTR("Save values? ");
      s.help = PSTR("[R] => Select"); s.footer = PSTR("[PREV]         [EXIT]");
      s.calibration_type = OC::CALIBRATE_NONE; s.min = 0; s.max = 1;
      break;
  }
  return &s;
}

// the octave's voltage, as Phazerville shows it: octave 0 is kOctaveZero below 0 V
FLASHMEM int OctavePitch(int octave) {
  return (octave - OC::DAC::kOctaveZero) * (12 * (1 + DAC_20Vpp) << 7);
}

FLASHMEM void SetAllOctave(int octave) {
  const int32_t pitch = OctavePitch(octave);
  for (int c = 0; c < DAC_CHANNEL_COUNT; ++c) {
    const DAC_CHANNEL ch = DAC_CHANNEL(c);
    OC::DAC::set(ch, OC::DAC::PitchToScaledDAC(ch, pitch, OC::VOLTAGE_SCALING_1V_PER_OCT, false));
  }
}

FLASHMEM void Interpolate(Cal& c, int ch) {
  // from octave 0 up to the one set: even steps on to the top
  const int hi = c.octave;
  if (hi <= 0) return;
  uint32_t v = OC::calibration_data.dac.calibrated_octaves[ch][0];
  const uint16_t second = OC::calibration_data.dac.calibrated_octaves[ch][hi];
  const int interval = (int(second) - int(v)) / hi;
  for (int i = 1; i < OCTAVES + 1; ++i) {
    v += interval;
    if (v > 0xFFFF) v = 0xFFFF;
    OC::calibration_data.dac.calibrated_octaves[ch][i] = uint16_t(v);
  }
  c.st.auto_scale_set[ch] = true;
}

FLASHMEM void SwitchToStep(Cal& c, int dir) {
  const OC::CalibrationStep* cur = c.st.current_step;
  if (cur->calibration_type == OC::CALIBRATE_OCTAVE && !c.st.used_defaults) {
    // fine-tuning: L walks the octaves of this channel first
    const int o = c.octave + dir;
    if (!(o < 0 || o > min(OCTAVES, cur->index + 7))) {
      c.octave = o;
      c.st.encoder_value = OC::calibration_data.dac.calibrated_octaves[OC::step_to_channel(c.st.step)][c.octave];
      return;
    }
  }
  int idx = int(c.st.step) + dir;
  idx = constrain(idx, int(OC::CENTER_DISPLAY), int(OC::CALIBRATION_EXIT));
  if (idx == int(c.st.step)) return;
  const OC::CalibrationStep* next = StepAt(idx);
  c.st.step = OC::CALIBRATION_STEP(idx);

  // leaving the step
  if (c.st.used_defaults && cur->calibration_type == OC::CALIBRATE_OCTAVE && cur->index > 5)
    Interpolate(c, OC::step_to_channel(cur->step));
  switch (cur->step) {
    case OC::HELLO:
      if (c.st.encoder_value) {  // start fresh
        const uint32_t flags = OC::calibration_data.flags & OC::CALIBRATION_FLAG_ENCODER_MASK;
        OC::calibration_reset();
        OC::calibration_data.flags |= flags;  // (keeps the encoder setup)
        c.st.used_defaults = true;
      }
      break;
    case OC::DAC_A_VOLT_HIGH:
      if (c.st.used_defaults)  // A's points as the start for B..H
        for (int ch = 1; ch < DAC_CHANNEL_COUNT; ++ch)
          for (int i = 0; i < OCTAVES; ++i)
            OC::calibration_data.dac.calibrated_octaves[ch][i] = OC::calibration_data.dac.calibrated_octaves[0][i];
      break;
    case OC::ADC_PITCH_C4:
      if (c.st.adc_1v && c.st.adc_3v) OC::ADC::CalibratePitch(c.st.adc_1v, c.st.adc_3v);
      break;
    default: break;
  }

  // entering the next
  const DAC_CHANNEL ch = OC::step_to_channel(next->step);
  switch (next->calibration_type) {
    case OC::CALIBRATE_OCTAVE:
      c.octave = next->index;
      c.st.encoder_value = OC::calibration_data.dac.calibrated_octaves[ch][c.octave];
      break;
    case OC::CALIBRATE_ADC_OFFSET:
      if (c.st.used_defaults)
        for (int i = 0; i < ADC_CHANNEL_COUNT; ++i)
          OC::calibration_data.adc.offset[i] = OC::ADC::smoothed_raw_value(ADC_CHANNEL(i));
      break;
    case OC::CALIBRATE_DISPLAY:
      c.st.encoder_value = OC::calibration_data.display_offset;
      break;
    case OC::CALIBRATE_SCREENSAVER:
      c.st.encoder_value = OC::calibration_data.screensaver_timeout;
      break;
    default:
      if (next->step != OC::CALIBRATION_EXIT) c.st.encoder_value = 0;
      else c.st.encoder_value = (c.st.used_defaults && OC::calibration_data_loaded) ? 0 : 1;  // save? (fresh over saved data: no)
      break;
  }
  c.st.current_step = next;
}

// the DAC and the settings follow the step, as Phazerville's Controller()
FLASHMEM void Apply(Cal& c) {
  const OC::CalibrationStep* s = c.st.current_step;
  switch (s->calibration_type) {
    case OC::CALIBRATE_OCTAVE:
      OC::calibration_data.dac.calibrated_octaves[OC::step_to_channel(s->step)][c.octave] = uint16_t(c.st.encoder_value);
      SetAllOctave(c.octave);
      break;
    case OC::CALIBRATE_ADC_1V: SetAllOctave(OC::DAC::kOctaveZero + 1); break;
    case OC::CALIBRATE_ADC_3V: SetAllOctave(OC::DAC::kOctaveZero + 3); break;
    case OC::CALIBRATE_DISPLAY:
      OC::calibration_data.display_offset = c.st.encoder_value;
      display::AdjustOffset(OC::calibration_data.display_offset);
      SetAllOctave(OC::DAC::kOctaveZero);
      break;
    case OC::CALIBRATE_SCREENSAVER:
      OC::calibration_data.screensaver_timeout = c.st.encoder_value;
      SetAllOctave(OC::DAC::kOctaveZero);
      break;
    default:
      SetAllOctave(OC::DAC::kOctaveZero);  // 0 V
      break;
  }
}

FLASHMEM void Draw(const Cal& c) {
  const OC::CalibrationStep* s = c.st.current_step;
  GRAPHICS_BEGIN_FRAME(true);
  menu::DefaultTitleBar::Draw();
  graphics.print(s->title);
  weegfx::coord_t y = menu::CalcLineY(0);
  const weegfx::coord_t vx = menu::kDisplayWidth - 30;
  graphics.setPrintPos(menu::kIndentDx, y + 2);
  switch (s->calibration_type) {
    case OC::CALIBRATE_OCTAVE: {
      const int mv = (c.octave - OC::DAC::kOctaveZero) * 10 * (1 + DAC_20Vpp);
      graphics.printf(PSTR("-> %d.%d00V"), mv / 10, abs(mv % 10));
      if (c.st.auto_scale_set[OC::step_to_channel(s->step)]) graphics.print(PSTR(" *"));
      graphics.setPrintPos(vx, y + 2);
      graphics.print(c.st.encoder_value, 5);
      break;
    }
    case OC::CALIBRATE_SCREENSAVER:
      graphics.print(s->message);
      graphics.setPrintPos(vx, y + 2);
      graphics.print(c.st.encoder_value, 5);
      break;
    case OC::CALIBRATE_ADC_OFFSET:
      for (int i = 0; i < ADC_CHANNEL_COUNT; ++i) {
        graphics.setPrintPos(1 + i % 4 * 32, y + 10 * (i / 4));
        graphics.printf(PSTR("%3d"), int(OC::ADC::value(ADC_CHANNEL(i))));
      }
      y += 10;
      break;
    case OC::CALIBRATE_DISPLAY:
      graphics.print(s->message);
      graphics.setPrintPos(vx, y + 2);
      graphics.pretty_print(c.st.encoder_value, 2);
      graphics.drawFrame(0, 0, 128, 64);
      break;
    case OC::CALIBRATE_ADC_1V:
    case OC::CALIBRATE_ADC_3V:
      graphics.print(s->message);
      y += menu::kMenuLineH;
      graphics.setPrintPos(menu::kIndentDx, y + 2);
      graphics.print(int(OC::ADC::value(ADC_CHANNEL_1)), 2);
      if ((c.st.adc_1v && s->calibration_type == OC::CALIBRATE_ADC_1V) ||
          (c.st.adc_3v && s->calibration_type == OC::CALIBRATE_ADC_3V))
        graphics.print(PSTR("  (set)"));
      break;
    default:
      if (s->step != OC::CALIBRATION_EXIT) {
        graphics.print(s->message);
      } else {
        graphics.print(OC::calibration_data_loaded && c.st.used_defaults ? PSTR("Overwrite? ") : PSTR("Save? "));
      }
      if (s->max == 1) graphics.print(c.st.encoder_value ? PSTR("yes") : PSTR("no"));  // (start fresh? / save?)
      break;
  }
  y += menu::kMenuLineH;
  graphics.setPrintPos(menu::kIndentDx, y + 2);
  if (s->help) graphics.print(s->help);
  graphics.drawStr(1, menu::kDisplayHeight - menu::kFontHeight - 3, s->footer);
  const int w = (menu::kDisplayWidth << 8) / (OC::CALIBRATION_STEP_LAST - 1);
  graphics.drawRect(0, menu::kDisplayHeight - 2, (int(c.st.step) * w) >> 8, 2);
  GRAPHICS_END_FRAME();
}

FLASHMEM void Handle(Cal& c, const UI::Event& e) {
  using namespace OC;
  if (e.type == UI::EVENT_BUTTON_DOWN) {
    if (e.control == CONTROL_BUTTON_R) {  // next (act on press)
      if (c.st.step < CALIBRATION_EXIT) SwitchToStep(c, 1);
      else c.done = true;
      ui.SetButtonIgnoreMask();
    }
    return;
  }
  switch (e.control) {
    case CONTROL_BUTTON_L:
      if (e.type != UI::EVENT_BUTTON_PRESS) break;
      if (c.st.step == HELLO) { c.st.encoder_value = 0; c.done = true; }  // leave, nothing saved
      else if (c.st.step > CENTER_DISPLAY) SwitchToStep(c, -1);
      break;
    case CONTROL_ENCODER_L:
      if (c.st.step > HELLO) SwitchToStep(c, e.value);
      break;
    case CONTROL_ENCODER_R: {
      int d = e.value;
      if (e.mask & CONTROL_BUTTON_X) d *= 16;
      else if (e.mask & CONTROL_BUTTON_Y) d *= 32;
      c.st.encoder_value = constrain(c.st.encoder_value + d, c.st.current_step->min, c.st.current_step->max);
      break;
    }
    case CONTROL_BUTTON_UP:
    case CONTROL_BUTTON_DOWN: {
      const CalibrationStep* s = c.st.current_step;
      if (e.type == UI::EVENT_BUTTON_LONG_PRESS) {  // B held: take the reading / fill the octaves
        if (s->step == ADC_PITCH_C2) c.st.adc_1v = OC::ADC::value(ADC_CHANNEL_1);
        if (s->step == ADC_PITCH_C4) c.st.adc_3v = OC::ADC::value(ADC_CHANNEL_1);
        if (s->calibration_type == CALIBRATE_ADC_OFFSET)
          for (int i = 0; i < ADC_CHANNEL_COUNT; ++i)
            calibration_data.adc.offset[i] = OC::ADC::smoothed_raw_value(ADC_CHANNEL(i));
        if (s->calibration_type == CALIBRATE_OCTAVE && c.octave > 0) Interpolate(c, step_to_channel(s->step));
      } else if (e.type == UI::EVENT_BUTTON_PRESS && (c.st.step == HELLO || c.st.step == CALIBRATION_EXIT)) {
        ui.configure_encoders(calibration_data.next_encoder_config());  // (encoder direction)
      }
      break;
    }
    default: break;
  }
}

}  // namespace

FLASHMEM void RunCalibration() {
  static Cal c;  // (once, at power-up)
  c = Cal();
  c.st.step = OC::HELLO;
  c.st.current_step = StepAt(OC::HELLO);
  c.st.encoder_value = OC::calibration_data_loaded ? 0 : 1;  // start fresh? (nothing saved yet: yes)
  OC::ui.SetButtonIgnoreMask();  // (L, still held from the power-up, is not a press)
  OC::ui.FlushEvents();
  OC::ui.encoder_enable_acceleration(OC::CONTROL_ENCODER_R, true);
  while (!c.done) {
    UI::Event e;
    while (!c.done && OC::ui.NextEvent(e)) Handle(c, e);
    if (c.done) break;
    Apply(c);
    Draw(c);
    delay(2);
  }
  const bool save = c.st.current_step->step == OC::CALIBRATION_EXIT && c.st.encoder_value;
  if (save) OC::calibration_save();
  else if (c.st.used_defaults) OC::calibration_load();  // (left without saving: back to what was stored)
  OC::ui.set_screensaver_timeout(OC::calibration_data.screensaver_timeout);
  display::AdjustOffset(OC::calibration_data.display_offset);
  OC::ui.encoder_enable_acceleration(OC::CONTROL_ENCODER_R, false);
  OC::ui.SetButtonIgnoreMask();
  OC::ui.FlushEvents();
}

}  // namespace loom

#endif  // LOOM_ONLY
