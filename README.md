About
===============
This is a fork of nxdk with modifications for the **jfduke3d Xbox port**. Key changes:

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
4. Place the extracted .obj files in `xbox_compat/dsound_objs/` in the jfduke3d-xbox tree
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
- `DirectSoundOverrideSpeakerConfig(DSSPEAKER_ENABLE_AC3 | DSSPEAKER_SURROUND)` enables 5.1 AC3 output
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

