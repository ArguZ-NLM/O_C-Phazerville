#pragma once

#include <Audio.h>

namespace OC {
  namespace AudioIO {
    // total block size including header is 260 bytes
    // 252 * 260 fits nicely into two 32KB pages
    const int AUDIO_MEMORY = 252;
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
