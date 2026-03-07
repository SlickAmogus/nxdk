About This Fork
===============
This is a fork of nxdk with modifications for the **jfduke3d Xbox port**. Key changes:

### S/PDIF Audio Output Patch
- Enables digital audio output via S/PDIF on the original Xbox.

### APU Hardware Audio Driver (`lib/hal/apu.c`, `lib/hal/apu.h`)
A from-scratch Xbox APU (Audio Processing Unit) hardware driver for the MCPX chip, bypassing SDL/AC97 software mixing to use the hardware audio pipeline directly. This is research-grade code developed through extensive reverse-engineering on real Xbox hardware.

**Architecture:** The Xbox MCPX APU contains three subsystems:
- **VP (Voice Processor):** 256 hardware voices with per-voice pitch, volume, envelope, and filters. Processes PCM samples from RAM via DMA.
- **GP (Global Processor):** Programmable DSP (Motorola 56301-like) for effects (reverb, EQ, HRTF). Reads from VP's MIXBUF output.
- **EP (Encode Processor):** Programmable DSP for Dolby Digital 5.1 encoding. Outputs to AC97 DAC.

**What works:**
- VP voice processing confirmed (PAR_STATE advances through envelope stages, CBO increments through PCM buffer)
- GP DSP code execution confirmed (preamble marker written to XMEM)
- Two-phase GPRST initialization (DMA bootstrap then DSP release)
- Proper 16KB-aligned SGE table allocation (hardware masks bits [13:0])
- EP/GP DSP release after SE start (so SE catches initial idle edge)

**What's in progress:**
- GP DSP START_FRAME interrupt delivery — the SE (Setup Engine) never sends START_FRAME to the GP despite correct SECTL/SECONFIG/GPRST configuration. This prevents the GP from processing MIXBUF data into audio output. Active investigation ongoing.

**Key hardware findings documented in the driver:**
- PMEM MMIO access during GPRST=0 crashes the Xbox
- CPU cache coherency requires `wbinvd` after writing to DMA-visible memory
- PIO commands are not processed by the FE microcontroller (works in xemu only)
- SECONFIG bit 5 is auto-enabled by hardware when GP DSP properly signals idle
- Hardware zeros XMEM on DSP core reset (GPRST 1->3 transition)

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
