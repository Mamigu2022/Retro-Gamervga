#pragma once
// ============================================================================
// audio_i2s.h — Audio directo por I2S+DAC interno (pin 25)
// Reemplaza fabgl SoundGenerator + EmuAudioGenerator + audioRing
// ============================================================================

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void     audioInit(void);
void     audioFeedSamples(const int16_t *samples, int count);
void     audioFeedStereoMixed(const int16_t *left, const int16_t *right, int count);
void     audioSetEnabled(bool enabled);
void     audioShutdown(void);

#ifdef __cplusplus
}
#endif
