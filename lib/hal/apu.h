// SPDX-License-Identifier: MIT
// Xbox APU Voice Processor driver — hardware mixing for up to 256 voices

#ifndef HAL_APU_H
#define HAL_APU_H

#if defined(__cplusplus)
extern "C" {
#endif

// Initialize the APU Voice Processor.
// Probes PCI, maps MMIO, allocates voice array + SGE table + PCM pool.
// Returns 0 on success, -1 on failure.
int  XApuInit(void);

// Shut down the APU VP, free all allocated memory.
void XApuShutdown(void);

// Allocate a hardware voice. Returns handle 0-255, or -1 if none free.
int  XApuVoiceAlloc(void);

// Free a previously allocated voice.
void XApuVoiceFree(int voice);

// Start playing PCM data on a voice.
// pcm: source PCM data (will be copied to contiguous memory)
// bytes: size of PCM data
// rate: sample rate in Hz (e.g. 11025, 22050, 44100, 48000)
// bits: 8 or 16
// channels: 1 (mono) or 2 (stereo)
// loop: 1 to loop, 0 for one-shot
// left_vol, right_vol: 0-255
// Returns 0 on success, -1 on failure.
int  XApuVoicePlay(int voice, const void *pcm, unsigned int bytes,
                   unsigned int rate, int bits, int channels, int loop,
                   int left_vol, int right_vol);

// Stop a playing voice immediately.
void XApuVoiceStop(int voice);

// Update volume on a playing voice. left/right: 0-255.
void XApuVoiceSetVolume(int voice, int left_vol, int right_vol);

// Update pitch (sample rate) on a playing voice.
void XApuVoiceSetPitch(int voice, unsigned int sample_rate);

// Returns 1 if voice is currently playing, 0 otherwise.
int  XApuVoiceIsPlaying(int voice);

// Read VP MIXBUF output and produce interleaved 16-bit stereo samples.
// output: destination buffer (num_samples * 2 shorts = num_samples * 4 bytes)
// num_samples: number of stereo sample frames to produce
void XApuPumpMixbuf(short *output, int num_samples);

// Diagnostic: fill buf with a human-readable status string (register values,
// MIXBUF sample peek, etc.). Returns number of chars written.
int  XApuDiagnostic(char *buf, int bufsize);

#if defined(__cplusplus)
}
#endif

#endif // HAL_APU_H
