About
===============
This is a fork of nxdk with modifications for the [**VibeDuke3D Xbox port**](https://github.com/SlickAmogus/VibeDuke3D/). Key changes:

### S/PDIF Audio Output Patch
- Enables digital audio output via S/PDIF on the original Xbox.

### Hardware Audio via RXDK DirectSound
This fork is used by jfduke3d-xbox which implements hardware audio through the Xbox APU's full VP→GP→EP→AC97 pipeline, including 5.1 surround sound via Dolby Digital AC3 encoding over optical S/PDIF.

**How it works:**
1. MultiVoc (the audio mixer) generates PCM into ring buffers
2. `driver_dsound_xbox.c` creates DirectSound looping buffers and pumps PCM into them
3. RXDK's DirectSound library programs the APU hardware:
   - **VP** (Voice Processor) plays the DirectSound buffers via hardware DMA
   - **GP** (Global Processor) handles effects and mixing via DSP code
   - **EP** (Encode Processor) performs Dolby AC3 encoding for optical output
4. `DirectSoundDoWork()` is called each frame to drive the hardware pipeline

**Prerequisites — RXDK dsound.lib:**
The Xbox DirectSound implementation requires pre-compiled object files from Microsoft's Xbox SDK (RXDK/XDK). These are **not included** in this repository due to licensing. You must provide them yourself:

1. Obtain `dsound.lib` from a legitimate Microsoft Xbox Development Kit (XDK)
2. Extract the individual .obj files from the .lib archive:
   ```
   lib /list dsound.lib              # list contents
   lib /extract:dsapi.obj dsound.lib # extract each .obj
   ```
   Or use a tool like 7-Zip or `ar` to extract all objects.
3. Strip debug sections (optional, reduces size — nxdk's linker may choke on MSVC debug info):
   ```
   llvm-objcopy --strip-debug *.obj
   ```
4. Place the extracted .obj files in `xbox_compat/dsound_objs/` in the root of your project
5. Place the RXDK headers (`dsound.h`, `dsfxparm.h`, `pshpack1.h`, `poppack.h`) in `xbox_compat/rxdk_include/`

**Required .obj files** (22 objects from dsound.lib):
```
ac97.obj  ac97xmo.obj  cipher.obj  dsapi.obj  dscommon.obj
dsmath.obj  dsoundi.obj  dspdma.obj  dsperf.obj  epdsp.obj
globals.obj  gpdsp.obj  heap.obj  hrtf.obj  i3dl2.obj
mcpapu.obj  mcpbuf.obj  mcpstrm.obj  mcpvoice.obj  mcpxcore.obj
mixbin.obj  xmotable.obj
```

**Key integration points:**
- `DirectSoundCreate()` initializes the APU and loads GP/EP DSP microcode
- **AC3 encoding quirk:** `DirectSoundOverrideSpeakerConfig(DSSPEAKER_ENABLE_AC3 | DSSPEAKER_SURROUND)` drops the AC3 flag (0x10000). You must also set the RXDK globals directly: `g_dwDirectSoundOverrideSpeakerConfig = 0x10002` and `g_dwDirectSoundSpeakerConfig = 0x10002` before calling `DirectSoundCreate()`. See "How to Use" section below for the full pattern.
- Surround buffers use `DSMIXBIN_*` routing (FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, LOW_FREQUENCY, BACK_LEFT, BACK_RIGHT)
- `g_DirectSoundCriticalSection` must be initialized before `DirectSoundCreate` when linking .obj files directly (no DLL init runs)
- APU registers at `0xFE800000` should be reset before DirectSound init to clear stale kernel state

**Build flags** (in your Makefile):
```makefile
DSOUND_CFLAGS = -I$(CURDIR)/xbox_compat/rxdk_include
DSOUND_OBJS = $(wildcard $(CURDIR)/xbox_compat/dsound_objs/*.obj)
# Apply RXDK headers only to the dsound driver file:
driver_dsound_xbox.obj: CFLAGS += $(DSOUND_CFLAGS)
# Link all dsound .obj files into the final executable
```


### How to Use Hardware Audio in Your nxdk Project

To add RXDK DirectSound hardware audio to your own nxdk project, you need the compatibility layer files from jfduke3d-xbox's `xbox_compat/` directory and proper build integration.

#### 1. Required Files

Copy these from [VibeDuke3D](https://github.com/SlickAmogus/VibeDuke3D/tree/main) `xbox_compat/` into your project:

| File | Purpose |
|------|---------|
| `dsound_bridge.c` | **Calling convention bridge** — RXDK's .obj files use MSVC `__stdcall` but nxdk uses `__cdecl`. This provides `__stdcall` wrappers for Win32 APIs (CreateEvent, WaitForSingleObject, CreateThread, etc.), Xbox API stubs (XMemAlloc/XMemFree for APU DMA memory, XGetAudioFlags), and section loading stubs. |
| `msvc_compat.c` | **MSVC runtime symbols** — `__ftol2` (float-to-int), `__CIpow` (intrinsic pow), `__CIsinh` (intrinsic sinh), `_fpclass` (IEEE 754 classification), `_except_handler3` (SEH stub). All implemented in pure x87 assembly with no C library dependency. |
| `xtl.h` | Xbox type definitions and audio flag constants (`XC_AUDIO_FLAGS_*`) |
| `xbox_defs.h` | Platform defines and type shims |

#### 2. Strip Debug Sections

**Critical:** lld 18.1.8 (nxdk's linker) crashes on MSVC `.debug$F` and `.debug$S` sections in the dsound .obj files. Strip them after extracting:

```bash
for f in dsound_objs/*.obj; do
    llvm-objcopy --remove-section='.debug$F' --remove-section='.debug$S' "$f"
done
```

#### 3. Makefile Integration

Add to your `Makefile.nxdk`:

```makefile
# RXDK DirectSound headers — apply ONLY to your dsound driver file
DSOUND_CFLAGS = -I$(CURDIR)/xbox_compat/rxdk_include
$(CURDIR)/your_dsound_driver.obj: CFLAGS += $(DSOUND_CFLAGS)

# Link the extracted dsound .obj files
DSOUND_OBJS_DIR = $(CURDIR)/xbox_compat/dsound_objs
DSOUND_OBJS = \
    $(DSOUND_OBJS_DIR)/dsapi.obj \
    $(DSOUND_OBJS_DIR)/globals.obj \
    $(DSOUND_OBJS_DIR)/mcpxcore.obj \
    $(DSOUND_OBJS_DIR)/mcpvoice.obj \
    $(DSOUND_OBJS_DIR)/mcpstrm.obj \
    $(DSOUND_OBJS_DIR)/mcpbuf.obj \
    $(DSOUND_OBJS_DIR)/mcpapu.obj \
    $(DSOUND_OBJS_DIR)/gpdsp.obj \
    $(DSOUND_OBJS_DIR)/epdsp.obj \
    $(DSOUND_OBJS_DIR)/dspdma.obj \
    $(DSOUND_OBJS_DIR)/dsmath.obj \
    $(DSOUND_OBJS_DIR)/dscommon.obj \
    $(DSOUND_OBJS_DIR)/heap.obj \
    $(DSOUND_OBJS_DIR)/ac97.obj \
    $(DSOUND_OBJS_DIR)/dsperf.obj \
    $(DSOUND_OBJS_DIR)/dsoundi.obj \
    $(DSOUND_OBJS_DIR)/hrtf.obj \
    $(DSOUND_OBJS_DIR)/i3dl2.obj \
    $(DSOUND_OBJS_DIR)/cipher.obj \
    $(DSOUND_OBJS_DIR)/ac97xmo.obj \
    $(DSOUND_OBJS_DIR)/wavexmo.obj \
    $(DSOUND_OBJS_DIR)/wavfileio.obj
NXDK_LDFLAGS += $(DSOUND_OBJS)

# Add your compat source files to SRCS
SRCS += $(CURDIR)/xbox_compat/dsound_bridge.c
SRCS += $(CURDIR)/xbox_compat/msvc_compat.c
```

#### 4. Initialization Code Pattern

```c
#include <dsound.h>

// RXDK globals (defined in globals.obj)
extern CRITICAL_SECTION g_DirectSoundCriticalSection;
extern DWORD g_dwDirectSoundOverrideSpeakerConfig;
extern DWORD g_dwDirectSoundSpeakerConfig;

void audio_init(void) {
    IDirectSound *pDS = NULL;

    // 1. Reset APU registers (clear stale kernel state)
    volatile unsigned long *apu = (volatile unsigned long *)0xFE800000u;
    apu[0x1004 / 4] = 0;           // Disable APU interrupts
    apu[0x2000 / 4] = 0;           // Stop setup engine
    apu[0x1100 / 4] = 0;           // Halt front-end
    apu[0x1000 / 4] = 0xFFFFFFFF;  // Clear pending interrupts

    // 2. Init DirectSound's critical section (no DLL init runs when linking .obj)
    RtlInitializeCriticalSection(&g_DirectSoundCriticalSection);

    // 3. Set speaker config for 5.1 AC3 BEFORE DirectSoundCreate.
    //    DirectSoundOverrideSpeakerConfig() drops the AC3 flag, so set
    //    the globals directly to ensure the EP loads the AC3 encoder.
    DWORD config = DSSPEAKER_ENABLE_AC3 | DSSPEAKER_SURROUND;  // 0x10002
    DirectSoundOverrideSpeakerConfig(config);
    g_dwDirectSoundOverrideSpeakerConfig = config;
    g_dwDirectSoundSpeakerConfig = config;

    // 4. Create DirectSound — reads speaker config, loads GP/EP microcode
    DirectSoundCreate(NULL, &pDS, NULL);

    // 5. Create sound buffers with explicit DSMIXBIN routing
    DSMIXBINVOLUMEPAIR pairs[2] = {
        { DSMIXBIN_FRONT_LEFT, 0 },
        { DSMIXBIN_FRONT_RIGHT, 0 }
    };
    DSMIXBINS bins = { 2, pairs };
    // ... set up DSBUFFERDESC with lpMixBins = &bins, create buffer ...

    // 6. Start playback, then call DirectSoundDoWork() every frame
}

void audio_pump(void) {
    // Copy PCM into DS buffer via Lock/Unlock, then:
    DirectSoundDoWork();  // Drives VP→GP→EP hardware pipeline
}
```

#### 5. Stereo Compatibility

With `DSSPEAKER_SURROUND`, the GP automatically generates a stereo downmix for the analog output. Players using composite/component without optical will hear all channels (including center and surround) folded into stereo. No extra code is needed — the hardware handles this.

#### 6. Reference Implementation

See `jfaudiolib/src/driver_dsound_xbox.c` in jfduke3d-xbox for a complete working implementation with:
- 5.1 surround via separate front/center/surround DirectSound buffers
- Per-channel DSMIXBIN routing
- Ring buffer pumping with Lock/Unlock
- Volume amplification, bass boost, and LFE feed

**Support**

Join The Chill Lounge discord for questions regarding any of my mods\projects (or feel free to message me):\
https://discord.gg/U29t39WR73

If you like what I do:\
[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/F1F3K8V3B)

---

nxdk - *the new open source xdk*
================================
nxdk is a software development kit for the original Xbox. nxdk is a revitalization of [OpenXDK](https://web.archive.org/web/20170624051336/http://openxdk.sourceforge.net:80/).
It is maintained by the [XboxDev](https://github.com/XboxDev/XboxDev) community.

Notable features:
- Portable toolchain that works on modern versions of Windows, macOS and Linux.
- No complicated cross-compiling or big library dependencies! Builds with `make` and just needs standard tools and llvm.
- Modern C / C++ standards and compiler features.
- Supports popular APIs like Windows API and BSD sockets.
- SDL2 support for input, audio and 2D graphics.
- Custom API for 3D graphics using NVIDIA-designed shader-languages (with additional Xbox extensions).
- Open-Source drivers which can be modified to get the most out of the hardware.
- Modifiable startup code, for as much system control as necessary.
- Supported by an active community that can help with problems and responds to bug reports.

Build Status
------
![CI Status for "Build Samples"](https://github.com/XboxDev/nxdk/actions/workflows/build_samples.yml/badge.svg)

Status
------
While nxdk still is in early stages of development, it can already be used in many projects.
Take a look at the [list of projects](https://github.com/XboxDev/nxdk/wiki/Projects-using-nxdk) that are build on top of nxdk. Additionally, the [provided samples](https://github.com/XboxDev/nxdk/tree/master/samples) show how to use common features.

Getting Started
---------------
### Prerequisites
You will need the following tools:
- GNU make
- [clang](http://clang.llvm.org/)
- [GNU bison](https://www.gnu.org/software/bison/) and [flex](http://flex.sourceforge.net/)
- [lld](http://lld.llvm.org/)
- [Git](http://git-scm.com/)
- [CMake](https://cmake.org/)

OS-specific instructions for installing these prerequisites can be found in the [Wiki](https://github.com/XboxDev/nxdk/wiki/Install-the-Prerequisites)

### Download nxdk
    git clone --recursive https://github.com/XboxDev/nxdk.git

### Build Samples
Samples are easily built by running the activation script `bin/activate`, which will spawn a shell that is fully set up to start using nxdk, and then running the Makefile in one of the sample directories. Details can be found in the [Wiki](https://github.com/XboxDev/nxdk/wiki/Build-a-Sample). nxdk also supports automatic [creation of ISO files](https://github.com/XboxDev/nxdk/wiki/Create-an-XISO).

Next Steps
----------
Copy one of the sample directories to get started. You can copy it anywhere you like. Run nxdk's activation script `bin/activate`, then, in the directory of your program, you can simply run `make`.

Credits
-------
- [OpenXDK](https://web.archive.org/web/20170624051336/http://openxdk.sourceforge.net:80/) is the inspiration for nxdk, and large parts of it have been reused. (License: MIT)
- Large parts of [pbkit](https://web.archive.org/web/20141024145308/http://forums.xbox-scene.com/index.php?/topic/573524-pbkit/), by openxdkman, are included, with modifications. (License: MIT)
- A network stack is included based on [lwIP](http://savannah.nongnu.org/projects/lwip/) (License: Modified BSD)
- A libc is included based on [PDCLib](https://github.com/DevSolar/pdclib) (License: CC0)
- Large parts of the runtime library are derived from LLVM's [compiler-rt](https://compiler-rt.llvm.org/) library (License: MIT)
- vp20compiler is based on nvvertparse.c from [Mesa](http://www.mesa3d.org/) (License: MIT)
- fp20compiler is based on nvparse from the [NVIDIA SDK 9.52](https://www.nvidia.com/object/sdk-9.html).
- The [NVIDIA Cg compiler](https://developer.nvidia.com/cg-toolkit) is bundled.
- extract-xiso developed by in et al. (License: BSD)

Code Overview
-------------
* `lib/hal` - Barebones Hardware Abstraction Layer for the Xbox, from OpenXDK.
* `lib/net` - Network stack for the Xbox based on lwIP.
* `lib/pdclib` - Xbox port of PDCLib, a CC0-licensed C standard library.
* `lib/pbkit` - A low level library for interfacing with the Xbox GPU.
* `lib/sdl` - Xbox ports of SDL2 and SDL_ttf.
* `lib/usb` - USB support from OpenXDK. Hacked together parts of an old Linux OHCI stack.
* `lib/winapi` - Xbox specific implementations of common useful WinAPI-functions.
* `lib/xboxkrnl` - Header and import library for interfacing with the Xbox kernel.
* `lib/xboxrt` - Miscellaneous functionality for debugging etc.
* `tools/cxbe` - Simple converter for PE executables to the Xbox executable format, from OpenXDK.
* `tools/fp20compiler` - Translates register combiner descriptions to Xbox pushbuffer commands.
* `tools/vp20compiler` - Translates vertex program assembly to Xbox microcode.
* `tools/extract-xiso` - Generates and extracts ISO images compatible with the Xbox (and XQEMU).
* `samples/` - Sample applications to get started.

