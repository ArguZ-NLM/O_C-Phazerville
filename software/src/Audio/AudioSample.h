#pragma once
#include <Arduino.h>

// a sample in 16-bit steps (may be over full scale) to 16 bit: left alone up
// to 90 % of full scale, past that bent smoothly towards full scale, never
// cut off flat (digital clipping sounds harsh; Loom never clips hard)
static constexpr int32_t SOFT_KNEE16 = 29491;  // 0.9 * 32768
static inline int16_t SoftClip16(float s) {
  constexpr float k = float(SOFT_KNEE16), r = 32767.0f - k;
  const float a = fabsf(s);
  if (a > k) {
    const float y = (a - k) * (1.0f / r);
    s = copysignf(k + r * y / (1.0f + y), s);
  }
  return int16_t(int32_t(s + (s >= 0.0f ? 0.5f : -0.5f)));
}
static inline int16_t SoftClip16(int32_t v) {
  if (v <= SOFT_KNEE16 && v >= -SOFT_KNEE16) return int16_t(v);
  return SoftClip16(float(v));
}

// a float sample (1.0 = full scale, read in as data / 32768) back to 16 bit:
// rounded to the nearest step, overs bent (SoftClip16). (Cut off with * 32767
// every stage lost a few dB of signal to noise: the noise test, USB 'N'.)
static inline int16_t ToSample16(float v) {
  return SoftClip16(v * 32768.0f);
}

// a soft limit that leaves the sound alone up to 0.7 and only bends what
// goes past, smoothly towards full scale (never over). (v / sqrt(1 + v*v)
// bent everything: ~3 % distortion already at half scale, heard as
// overdrive on held notes.)
static inline float SoftLimit(float v) {
  constexpr float k = 0.7f, r = 1.0f - k;
  const float a = fabsf(v);
  if (a <= k) return v;
  const float y = (a - k) * (1.0f / r);
  const float o = k + r * y / (1.0f + y);
  return v < 0.0f ? -o : o;
}
