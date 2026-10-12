// MP3 to WAV on the module, for the CARD app (see Mp3ToWav.h)
#ifdef CARD_APP

#include <Arduino.h>
#include <SD.h>
#include <new>
#include <MTP_Teensy.h>
#include "Mp3ToWav.h"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_CODE FLASHMEM  // in flash: RAM1 is Loom's
#define MINIMP3_ROM PROGMEM
#include "minimp3.h"

char g_mp3_report[200] = "MP3: nothing yet";

struct Mp3ToWav::Work {
  mp3dec_t dec;
  File in;
  FsFile out;  // (SdFat's own: it can set the space aside beforehand)
  alignas(32) uint8_t buf[IN_SIZE];
  uint32_t fill = 0, pos = 0;
  uint32_t end = 0;  // where the sound ends in the file (an ID3v1 tag after it)
  bool eof = false;
  mp3d_sample_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
  static constexpr int OUT = 8192;  // samples (16 KB), written to the card in one go
  alignas(32) int16_t obuf[OUT];
  int ofill = 0;
  int chans = 0, hz = 0;
  bool started = false;
  uint32_t data_bytes = 0;
  // the encoder's silence at the start and the end (LAME's tag says how
  // much), so loops stay seamless
  uint32_t skip = 0;
  uint64_t left = UINT64_MAX;
  uint32_t t_start = 0, us_decode = 0, us_read = 0, us_write = 0, frames = 0, steps = 0, said_at = 0;  // for the USB report
};

// from minimp3_ex.h: the Xing / Info frame at the start of an MP3 (no
// sound in it), with the number of frames and the encoder's delay and
// padding; 1 if found
FLASHMEM static int CheckVbrTag(const uint8_t* frame, int frame_size, uint32_t* frames, int* delay, int* padding) {
  bs_t bs[1];
  L3_gr_info_t gr_info[4];
  if (frame_size < (int)(HDR_SIZE + 8)) return 0;
  bs_init(bs, frame + HDR_SIZE, frame_size - HDR_SIZE);
  if (HDR_IS_CRC(frame)) get_bits(bs, 16);
  if (L3_read_side_info(bs, gr_info, frame) < 0) return 0;
  const size_t size = (size_t)frame_size;
  size_t off = HDR_SIZE + bs->pos / 8;
  if (off > size || size - off < 8) return 0;
  const uint8_t* tag = frame + off;
  if (memcmp("Xing", tag, 4) && memcmp("Info", tag, 4)) return 0;
  const int flags = tag[7];
  if (!(flags & 1)) return -1;  // a tag, but no frame count
  off += 8;
  if (size - off < 4) return 0;
  tag = frame + off;
  *frames = (uint32_t)(tag[0] << 24) | (tag[1] << 16) | (tag[2] << 8) | tag[3];
  off += 4;
  if (flags & 2) { if (size - off < 4) return 0; off += 4; }
  if (flags & 4) { if (size - off < 100) return 0; off += 100; }
  if (flags & 8) { if (size - off < 4) return 0; off += 4; }
  *delay = *padding = 0;
  if (size - off < 1) return 0;
  tag = frame + off;
  if (*tag) {  // LAME, Lavc, ...: the same layout
    if (size - off <= 35) return 0;
    tag = frame + off + 21;
    *delay = ((tag[0] << 4) | (tag[1] >> 4)) + (528 + 1);
    *padding = (((tag[1] & 0xF) << 8) | tag[2]) - (528 + 1);
  }
  return 1;
}

FLASHMEM static void PutLE(uint8_t* p, uint32_t v, int n) {
  for (int i = 0; i < n; i++) p[i] = uint8_t(v >> (8 * i));
}

// the 512 bytes in front of the samples: RIFF, fmt, a JUNK chunk to fill
// (every reader skips it), data. The samples start on a card sector, so
// the card takes them whole: block by block it was ten times slower.
constexpr uint32_t HEADER = 512;
FLASHMEM static void WavHeader(uint8_t* h, int chans, int hz, uint32_t data_bytes) {
  memset(h, 0, HEADER);
  memcpy(h, "RIFF", 4);
  PutLE(h + 4, HEADER - 8 + data_bytes, 4);
  memcpy(h + 8, "WAVEfmt ", 8);
  PutLE(h + 16, 16, 4);
  PutLE(h + 20, 1, 2);  // PCM
  PutLE(h + 22, chans, 2);
  PutLE(h + 24, hz, 4);
  PutLE(h + 28, hz * chans * 2, 4);
  PutLE(h + 32, chans * 2, 2);
  PutLE(h + 34, 16, 2);
  memcpy(h + 36, "JUNK", 4);
  PutLE(h + 40, HEADER - 36 - 8 - 8, 4);
  memcpy(h + HEADER - 8, "data", 4);
  PutLE(h + HEADER - 4, data_bytes, 4);
}

FLASHMEM bool Mp3ToWav::Start(const char* path) {
  if (w) Cancel();
  st = IDLE;
  snprintf(mp3, sizeof(mp3), "%s", path);
  // the WAV's name: the MP3's, or with -2, -3 ... when that is taken
  char base[160];
  snprintf(base, sizeof(base), "%s", path);
  if (char* dot = strrchr(base, '.')) *dot = 0;
  snprintf(wav, sizeof(wav), "%s.wav", base);
  for (int k = 2; SD.exists(wav) && k < 100; k++) snprintf(wav, sizeof(wav), "%s-%d.wav", base, k);
  snprintf(tmp, sizeof(tmp), "%s.part", base);

  // 43 KB: in the RAM if there is room (the card reads and writes from the
  // PSRAM failed now and then), else in the PSRAM
  void* mem = malloc(sizeof(Work));
  in_psram = !mem;
  if (!mem) mem = extmem_malloc(sizeof(Work));
  if (!mem) { Serial.printf("MP3: no memory for %s\n", mp3); st = FAILED; return false; }
  w = new (mem) Work();
  w->in = SD.open(mp3, FILE_READ);
  if (!w->in) { Serial.printf("MP3: can't open %s\n", mp3); Finish(false); return false; }
  in_size = w->end = uint32_t(w->in.size());
  in_read = 0;
  // an ID3v2 tag at the start (names, pictures): skipped
  uint8_t h[10];
  if (w->in.read(h, 10) == 10 && !memcmp(h, "ID3", 3)) {
    const uint32_t sz = ((h[6] & 0x7F) << 21) | ((h[7] & 0x7F) << 14) | ((h[8] & 0x7F) << 7) | (h[9] & 0x7F);
    in_read = std::min<uint32_t>(10 + sz + ((h[5] & 0x10) ? 10 : 0), in_size);
  }
  // an ID3v1 tag at the end: 128 bytes after the sound
  if (in_size >= in_read + 128 && w->in.seek(in_size - 128) && w->in.read(h, 3) == 3 && !memcmp(h, "TAG", 3))
    w->end = in_size - 128;
  w->in.seek(in_read);
  SD.remove(tmp);
  w->out = SD.sdfs.open(tmp, O_RDWR | O_CREAT | O_TRUNC);
  if (!w->out.isOpen()) { Serial.printf("MP3: can't write %s\n", tmp); Finish(false); return false; }
  // (the header follows with the first frame, when the size is known)
  mp3dec_init(&w->dec);
  w->t_start = millis();
  st = BUSY;
  return true;
}

FLASHMEM Mp3ToWav::State Mp3ToWav::Step(uint32_t ms) {
  if (st != BUSY || !w) return st;
  Work& k = *w;
  // the samples so far onto the card; false if it is full
  auto flush = [&]() {
    if (!k.ofill) return true;
    const size_t n = size_t(k.ofill) * 2;
    k.ofill = 0;
    const uint8_t* p = reinterpret_cast<uint8_t*>(k.obuf);
    size_t done = 0;
    // a write the card refuses (busy): again, from where it got to
    for (int tries = 0; done < n && tries < 6; tries++) {
      if (tries) {
        Serial.printf("MP3: write at %lu stopped after %u of %u bytes (err %d), again\n",
                      (unsigned long)(HEADER + k.data_bytes), unsigned(done), unsigned(n), k.out.getWriteError());
        k.out.clearWriteError();
        delay(20 << tries);
        k.out.seekSet(HEADER + k.data_bytes + done);
      }
      const uint32_t us = micros();
      size_t r = k.out.write(p + done, n - done);
      if (r > n - done) r = 0;  // (-1: an error)
      k.us_write += micros() - us;
      done += r;
    }
    k.data_bytes += done;
    if (done < n) {
      Serial.printf("MP3: the card takes no more (%llu MB free)\n",
                    (unsigned long long)((SD.totalSize() - SD.usedSize()) >> 20));
      return false;
    }
    return true;
  };
  const uint32_t t0 = millis();
  bool through = false;
  k.steps++;
  if (t0 - k.said_at >= 1000) {
    k.said_at = t0;
    snprintf(g_mp3_report, sizeof(g_mp3_report), in_psram ? "(PSRAM) " : "(RAM) ");
    snprintf(g_mp3_report + strlen(g_mp3_report), sizeof(g_mp3_report) - strlen(g_mp3_report), "MP3: %d%% after %lu ms: %lu steps, %lu frames, decoding %lu ms, reading %lu ms, writing %lu ms\n",
                  Percent(), (unsigned long)(t0 - k.t_start), (unsigned long)k.steps, (unsigned long)k.frames,
                  (unsigned long)(k.us_decode / 1000), (unsigned long)(k.us_read / 1000), (unsigned long)(k.us_write / 1000));
  }
  while (!through && millis() - t0 < ms) {
    // keep the buffer at least half full
    if (!k.eof && k.fill - k.pos < IN_SIZE / 2) {
      // what is left to the front, so that the read lands on a word in
      // memory and ends on a card sector: then the card reads it whole
      const uint32_t left = k.fill - k.pos, shift = (0u - left) & 3;
      memmove(k.buf + shift, k.buf + k.pos, left);
      k.pos = shift;
      k.fill = shift + left;
      uint32_t want = std::min<uint32_t>(IN_SIZE - k.fill, k.end - in_read);
      if (in_read + want < k.end && want > 512) want -= (in_read + want) & 511;
      const uint32_t us = micros();
      int got = want ? k.in.read(k.buf + k.fill, want) : 0;
      k.us_read += micros() - us;
      // the card gives nothing now and then (busy): again, from there
      for (int tries = 1; want && got <= 0 && tries < 6; tries++) {
        Serial.printf("MP3: read at %lu gave %d, again\n", (unsigned long)in_read, got);
        delay(20 << tries);
        k.in.seek(in_read);
        got = k.in.read(k.buf + k.fill, want);
      }
      if (got > 0) { k.fill += got; in_read += got; }
      if (got <= 0 || in_read >= k.end) {
        k.eof = true;
        if (in_read < k.end)
          Serial.printf("MP3: read at %lu of %lu gave %d (wanted %lu), pos %lu\n", (unsigned long)in_read,
                        (unsigned long)k.end, got, (unsigned long)want, (unsigned long)uint32_t(k.in.position()));
      }
    }
    const uint32_t avail = k.fill - k.pos;
    if (!avail) { through = true; Serial.printf("MP3: all read\n"); break; }
    mp3dec_frame_info_t info;
    const uint32_t us = micros();
    const int n = mp3dec_decode_frame(&k.dec, k.buf + k.pos, int(avail), k.pcm, &info);
    k.us_decode += micros() - us;
    k.frames++;
    if (!info.frame_bytes) {  // no frame in what is left
      if (k.eof) { through = true; Serial.printf("MP3: no frame in the last %lu bytes\n", (unsigned long)avail); break; }
      k.pos = k.fill;  // not sound: on to what comes next
      continue;
    }
    const uint8_t* frame = k.buf + k.pos + info.frame_offset;
    const int frame_size = info.frame_bytes - info.frame_offset;
    k.pos += info.frame_bytes;
    if (!n) continue;  // skipped something, or a frame that couldn't be decoded
    if (!k.started) {
      k.started = true;
      k.chans = info.channels;
      k.hz = info.hz;
      uint32_t frames = 0;
      int delay = 0, padding = 0;
      const bool tag = CheckVbrTag(frame, frame_size, &frames, &delay, &padding) > 0;
      if (tag) {
        // the tag frame is no sound
        k.skip = uint32_t(std::max(delay, 0));
        uint64_t total = uint64_t(frames) * n;
        total = total > k.skip ? total - k.skip : 0;
        if (padding > 0) total = total > uint64_t(padding) ? total - padding : 0;
        if (frames) k.left = total;
      }
      // the WAV's whole size set aside on the card in one piece, so the
      // card isn't searched for room at every step: ten times faster.
      // (Known from the tag, or from the bitrate with some to spare; what
      // isn't used is given back at the end.)
      const uint64_t secs_x1000 = info.bitrate_kbps ? uint64_t(k.end - in_read) * 8 / info.bitrate_kbps : 0;
      uint64_t sz = k.left != UINT64_MAX ? k.left : secs_x1000 * k.hz / 1000 * 5 / 4;
      sz = sz * k.chans * 2 + HEADER;
      if (sz < (1ull << 32) && !k.out.preAllocate(sz)) Serial.printf("MP3: no room set aside (%llu)\n", (unsigned long long)sz);
      uint8_t hdr[HEADER];
      WavHeader(hdr, k.chans, k.hz, 0);  // for now; the real one at the end
      if (k.out.write(hdr, HEADER) != HEADER) { Finish(false); return st; }
      if (tag) continue;  // the tag frame is no sound
    }
    // n frames (samples per channel); the first ones may be the encoder's
    // delay, the last ones its padding
    int from = 0;
    if (k.skip) { from = int(std::min<uint32_t>(k.skip, uint32_t(n))); k.skip -= from; }
    int to = n;
    if (k.left != UINT64_MAX) {
      if (uint64_t(to - from) > k.left) to = from + int(k.left);
      k.left -= (to - from);
    }
    for (int i = from; i < to; i++) {
      if (k.ofill + 2 > Work::OUT && !flush()) { Finish(false); return st; }
      if (k.chans == 2) {  // (a mono frame in a stereo file: on both sides)
        const int16_t l = k.pcm[i * info.channels];
        k.obuf[k.ofill++] = l;
        k.obuf[k.ofill++] = info.channels == 2 ? k.pcm[i * 2 + 1] : l;
      } else {
        k.obuf[k.ofill++] = info.channels == 2 ? int16_t((k.pcm[i * 2] + k.pcm[i * 2 + 1]) / 2) : k.pcm[i];
      }
    }
    if (k.left == 0) through = true;  // the rest is the encoder's padding
  }
  if (through) Finish(flush() && k.data_bytes > 0);
  return st;
}

FLASHMEM void Mp3ToWav::Finish(bool ok) {
  if (w) {
    if (ok) {
      uint8_t hdr[HEADER];
      WavHeader(hdr, w->chans, w->hz, w->data_bytes);
      ok = w->out.truncate(HEADER + uint64_t(w->data_bytes)) &&  // (what was set aside and not used)
           w->out.seekSet(0) && w->out.write(hdr, HEADER) == HEADER;
    }
    if (w->out.isOpen()) w->out.close();
    if (w->in) w->in.close();
    snprintf(g_mp3_report, sizeof(g_mp3_report),
             "MP3: %s, %d ch %d Hz, %lu bytes, %lu ms: %lu steps, %lu frames, decoding %lu ms, reading %lu ms, writing %lu ms\n",
             ok ? "ok" : "failed", w->chans, w->hz, (unsigned long)w->data_bytes, (unsigned long)(millis() - w->t_start),
             (unsigned long)w->steps, (unsigned long)w->frames, (unsigned long)(w->us_decode / 1000),
             (unsigned long)(w->us_read / 1000), (unsigned long)(w->us_write / 1000));
    Serial.print(g_mp3_report);
    w->~Work();
    extmem_free(w);  // (also frees what came from malloc)
    w = nullptr;
  }
  if (ok) ok = SD.rename(tmp, wav);
  if (ok) {
    st = DONE;  // (the MP3 goes in MtpReplace)
  } else {
    SD.remove(tmp);
    st = FAILED;
  }
}

FLASHMEM void Mp3ToWav::Cancel() {
  if (st == BUSY) Finish(false);
  st = IDLE;
}

// MTP_Teensy keeps the messages to the computer (a file added / removed)
// and its list of the card private; an explicit instantiation may name
// private members, so this reaches them
using MtpPathEvent = bool (MTP_class::*)(uint32_t, const char*);
using MtpIdEvent = int (MTP_class::*)(uint32_t);
using MtpStore = MTPStorage MTP_class::*;
template <MtpPathEvent ADD, MtpIdEvent REMOVED, MtpStore STORE>
struct MtpInside {
  friend MtpPathEvent MtpAdd() { return ADD; }
  friend MtpIdEvent MtpRemoved() { return REMOVED; }
  friend MtpStore MtpStorage() { return STORE; }
};
MtpPathEvent MtpAdd();
MtpIdEvent MtpRemoved();
MtpStore MtpStorage();
template struct MtpInside<&MTP_class::send_addObjectEvent, &MTP_class::send_removeObjectEvent, &MTP_class::storage_>;

FLASHMEM void MtpReplace(const char* gone, const char* added) {
  if (!static_cast<bool>(MTP)) { SD.remove(gone); return; }  // nobody connected
  MTPStorage& st = MTP.*MtpStorage();
  // the SD card is MTP's first (and only) store
  const uint32_t h = st.MapFileNameToIndex(0, gone, false, nullptr);
  if (h != 0xFFFFFFFFUL) {  // the computer knows it: deleted as if it had done it
    st.DeleteObject(h);
    (MTP.*MtpRemoved())(h);
  } else {
    SD.remove(gone);
  }
  (MTP.*MtpAdd())(0, added);
}

#endif  // CARD_APP
