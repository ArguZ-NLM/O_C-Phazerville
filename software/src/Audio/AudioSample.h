#pragma once
#include <Arduino.h>

// a float sample (1.0 = full scale, read in as data / 32768) back to 16 bit:
// rounded to the nearest step. (Cut off with * 32767 every stage lost a few
// dB of signal to noise: the noise test, USB 'N'.)
static inline int16_t ToSample16(float v) {
  const float s = v * 32768.0f;
  return int16_t(__SSAT(int32_t(s + (s >= 0.0f ? 0.5f : -0.5f)), 16));
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
