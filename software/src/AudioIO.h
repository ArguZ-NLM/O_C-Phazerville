#pragma once

#include <Audio.h>

namespace OC {
  namespace AudioIO {
    // total block size including header is 260 bytes
    // 252 * 260 fits nicely into two 32KB pages
#ifdef LOOM_ONLY
    // Loom: its devices hand blocks on at once (no queues holding them); a
    // full patch used 15 at most (2026-11-07), so 96 leaves room, and the
    // 40 KB of RAM2 left over goes to the effects
    const int AUDIO_MEMORY = 96;
#else
    const int AUDIO_MEMORY = 252;
#endif
    AudioStream& InputStream(int interface = 0);
    AudioStream& OutputStream();
    void Init();

    // Lets a full-screen app feed the outputs directly. The applet chain keeps
    // its own connections and keeps running, it just isn't heard until the
    // output is released again. src channels 0 and 1 become L and R.
    void ClaimOutput(AudioStream& src);
    void ReleaseOutput();

    // Restart reading the I2S input at the start of a frame. Depending on
    // timing at power-up the input can start on the right channel's word, and
    // L and R stay swapped until the next restart; after this they come in the
    // same order every time.
    void ResyncInput();
  }
}
