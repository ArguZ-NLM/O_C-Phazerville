#ifdef ARDUINO_TEENSY41

#include "AudioIO.h"
#include "AudioStream.h"
#include "Audio/AudioMixer.h"
#include "Audio/AudioPassthrough.h"
#include "PhzConfig.h"
#include "Audio/AudioPassthrough.h"
#include "usb_desc.h"

namespace OC {
  namespace AudioIO {
    AudioInputI2S2 input_stream;

    AudioOutputI2S2* output_stream = nullptr;
    AudioPassthrough<2> output_route;
#ifdef AUDIO_INTERFACE
    AudioInputUSB input_usb;
    AudioMixer<2> usbmix[2];
    AudioOutputUSB output_usb;
    AudioConnection out_conn_usbL{output_route, 0, output_usb, 0};
    AudioConnection out_conn_usbR{output_route, 1, output_usb, 1};
    AudioConnection out_conn_usbL2{input_stream, 0, output_usb, 2};
    AudioConnection out_conn_usbR2{input_stream, 1, output_usb, 3};

    AudioConnection in_conn_usbL{input_usb, 2, usbmix[0], 0};
    AudioConnection in_conn_usbR{input_usb, 3, usbmix[1], 0};
    AudioConnection in_conn_mixL{output_route, 0, usbmix[0], 1};
    AudioConnection in_conn_mixR{output_route, 1, usbmix[1], 1};
#endif

    AudioConnection out_conn[2];

    AudioStream& InputStream(int interface) {
      switch (interface) {
        default:
        case 0:
          return input_stream;
#ifdef AUDIO_INTERFACE
        case 1:
          return input_usb;
#endif
      }
    }

    FLASHMEM AudioStream& OutputStream() {
      // The output stream should be created after all other streams have been
      // created or we get an extra 3ms of latency. Hence, we initialize it
      // lazily. This is pretty hacky; if someone references this before all
      // other streams have been created it won't work. But it was the simplest
      // fix for now.
      if (output_stream == nullptr) {
        output_stream = new AudioOutputI2S2();
#ifdef AUDIO_INTERFACE
        out_conn[0].connect(usbmix[0], 0, *output_stream, 0);
        out_conn[1].connect(usbmix[1], 0, *output_stream, 1);
        usbmix[0].gain(0, 1.0f);
        usbmix[0].gain(1, 1.0f);
        usbmix[1].gain(0, 1.0f);
        usbmix[1].gain(1, 1.0f);
#else
        out_conn[0].connect(output_route, 0, *output_stream, 0);
        out_conn[1].connect(output_route, 1, *output_stream, 1);
#endif
      }
      return output_route;
    }

    void Init() {
      AudioMemory(AUDIO_MEMORY);
    }

    // used while an app has claimed the outputs; on the heap so they cost
    // no RAM1 until first used
    struct ClaimConns {
      AudioConnection out[2];
#ifdef AUDIO_INTERFACE
      AudioConnection usb[2];
#endif
    };
    ClaimConns* claim = nullptr;
    AudioStream* claimed_by = nullptr;

    FLASHMEM void ClaimOutput(AudioStream& src) {
      OutputStream(); // make sure the I2S output exists
      if (!claim) claim = new ClaimConns;
      AudioNoInterrupts();
      if (claimed_by) {
        for (auto& c : claim->out) c.disconnect();
#ifdef AUDIO_INTERFACE
        for (auto& c : claim->usb) c.disconnect();
#endif
      }
#ifdef AUDIO_INTERFACE
      // the applet chain normally feeds the hardware out (via usbmix) and USB
      in_conn_mixL.disconnect();
      in_conn_mixR.disconnect();
      out_conn_usbL.disconnect();
      out_conn_usbR.disconnect();
      claim->out[0].connect(src, 0, usbmix[0], 1);
      claim->out[1].connect(src, 1, usbmix[1], 1);
      claim->usb[0].connect(src, 0, output_usb, 0);
      claim->usb[1].connect(src, 1, output_usb, 1);
#else
      out_conn[0].disconnect();
      out_conn[1].disconnect();
      claim->out[0].connect(src, 0, *output_stream, 0);
      claim->out[1].connect(src, 1, *output_stream, 1);
#endif
      claimed_by = &src;
      AudioInterrupts();
    }

    FLASHMEM void ReleaseOutput() {
      if (!claimed_by) return;
      AudioNoInterrupts();
      for (auto& c : claim->out) c.disconnect();
#ifdef AUDIO_INTERFACE
      for (auto& c : claim->usb) c.disconnect();
      in_conn_mixL.connect(output_route, 0, usbmix[0], 1);
      in_conn_mixR.connect(output_route, 1, usbmix[1], 1);
      out_conn_usbL.connect(output_route, 0, output_usb, 0);
      out_conn_usbR.connect(output_route, 1, output_usb, 1);
#else
      out_conn[0].connect(output_route, 0, *output_stream, 0);
      out_conn[1].connect(output_route, 1, *output_stream, 1);
#endif
      claimed_by = nullptr;
      AudioInterrupts();
    }
  }
}
#endif
