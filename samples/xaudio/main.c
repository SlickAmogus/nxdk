#include <hal/debug.h>
#include <hal/video.h>
#include <hal/xbox.h>
#include <windows.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <hal/audio.h>
#include <xboxkrnl/xboxkrnl.h>
#include <assert.h>

/* xbox_log: output to kernel debug port (xemu console) */
void xbox_log(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    OutputDebugStringA(buf);
}

#define MIN(x,y) ((x)<(y)?(x):(y))

#include "nxdk_wav.h"

volatile unsigned int callback_count;

unsigned char *voice_data;
unsigned int voice_len;
unsigned int voice_pos;

#define NUM_BUFFERS 2
const unsigned short buffer_size = 48*1024;
unsigned char *buffers[NUM_BUFFERS];
unsigned int current_buf;

#define MAXRAM 0x03FFAFFF

static void provide_samples_callback(void *pac97Device, void *data)
{
    int is_final = (voice_pos+buffer_size) >= voice_len;
    int chunk_size = MIN(voice_len-voice_pos, buffer_size);

    memcpy(buffers[current_buf], voice_data+voice_pos, chunk_size);
    XAudioProvideSamples(buffers[current_buf], chunk_size, is_final);

    if (is_final) {
        voice_pos = 0;
    } else {
        voice_pos = voice_pos+chunk_size;
    }

    current_buf = (current_buf+1) % NUM_BUFFERS;
    callback_count++;
}

/* Generate a loud 440Hz square wave tone directly into the buffer */
static void fill_tone(unsigned char *buf, unsigned int len)
{
    short *samples = (short *)buf;
    unsigned int num_samples = len / 2; /* 2 bytes per sample */
    /* 48kHz stereo: period of 440Hz = 48000/440 = ~109 samples per channel */
    for (unsigned int i = 0; i < num_samples; i += 2) {
        /* i/2 = sample index for one channel at 48kHz */
        short val = ((i/2) % 109 < 55) ? 16000 : -16000;
        samples[i]   = val; /* left */
        samples[i+1] = val; /* right */
    }
}

int main(void)
{
    /* === PHASE 0: Allocate buffers (no video init yet) === */
    for (int i = 0; i < NUM_BUFFERS; i++) {
        buffers[i] = MmAllocateContiguousMemoryEx(buffer_size, 0, MAXRAM, 0,
            (PAGE_READWRITE | PAGE_WRITECOMBINE));
        assert(buffers[i] != NULL);
    }

    voice_data = nxdk_wav_h_bin;
    voice_len = nxdk_wav_h_bin_len;
    voice_pos = 0;
    current_buf = 0;
    callback_count = 0;

    /* === PHASE 1: Play audio BEFORE XVideoSetMode (10 seconds) === */
    /* If you hear sound here but not after, XVideoSetMode kills audio */
    XAudioInit(16, 2, &provide_samples_callback, NULL);

    for (int i = 0; i < NUM_BUFFERS; i++) {
        provide_samples_callback(NULL, NULL);
    }

    XAudioPlay();

    /* Let it play for 10 seconds (no screen output - dashboard framebuffer still active) */
    Sleep(10000);

    /* === PHASE 2: Now call XVideoSetMode === */
    XVideoSetMode(640, 480, 32, REFRESH_DEFAULT);

    /* Audio should still be playing via DPC callbacks.
     * If it stops here, XVideoSetMode killed the audio output path. */
    debugPrint("=== XAudio Diagnostic Test ===\n\n");
    debugPrint("PHASE 1 (before XVideoSetMode): audio played for 10 sec\n");
    debugPrint("  Did you hear sound? If yes, XVideoSetMode is the problem.\n\n");
    debugPrint("PHASE 2 (after XVideoSetMode): audio still playing...\n");
    debugPrint("  Callbacks so far: %d\n\n", callback_count);

    /* Let it play for another 10 seconds */
    for (int i = 0; i < 20; i++) {
        debugPrint("Phase 2 - callbacks: %d\n", callback_count);
        Sleep(500);
    }

    /* === PHASE 3: Stop and restart audio to test re-init === */
    XAudioPause();
    debugPrint("\nPHASE 3: Paused audio for 3 seconds...\n");
    Sleep(3000);

    /* Reset playback position and restart */
    voice_pos = 0;
    callback_count = 0;
    current_buf = 0;

    for (int i = 0; i < NUM_BUFFERS; i++) {
        provide_samples_callback(NULL, NULL);
    }

    XAudioPlay();
    debugPrint("PHASE 3: Restarted audio. Playing in loop...\n\n");

    /* === PHASE 4: Continuous loop with 3-second pauses === */
    unsigned int last_count = 0;
    int cycle = 0;
    while (1) {
        debugPrint("Cycle %d - callbacks: %d\n", cycle, callback_count);
        Sleep(500);

        /* Every ~30 seconds, pause briefly then restart */
        if (callback_count - last_count > 60) {
            cycle++;
            last_count = callback_count;
            XAudioPause();
            debugPrint("  -- 3 second pause --\n");
            Sleep(3000);

            voice_pos = 0;
            current_buf = 0;
            for (int i = 0; i < NUM_BUFFERS; i++) {
                provide_samples_callback(NULL, NULL);
            }
            XAudioPlay();
        }
    }

    return 0;
}
