#pragma once
#include <Arduino.h>

// a float sample (1.0 = full scale, read in as data / 32768) back to 16 bit:
// rounded to the nearest step. (Cut off with * 32767 every stage lost a few
// dB of signal to noise: the noise test, USB 'N'.)
static inline int16_t ToSample16(float v) {
  const float s = v * 32768.0f;
  return int16_t(__SSAT(int32_t(s + (s >= 0.0f ? 0.5f : -0.5f)), 16));
}
