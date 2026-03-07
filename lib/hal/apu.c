// SPDX-License-Identifier: MIT
// Xbox APU Voice Processor driver
//
// Drives the MCPX APU's Voice Processor to hardware-mix up to 256 voices
// at zero CPU cost. The VP mixes into MIXBUF bins; we read those bins and
// feed the result to the AC97 controller for DAC output.

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <hal/apu.h>
#include <xboxkrnl/xboxkrnl.h>

// External logging function (from xbox_startup.c)
extern void xbox_log(const char *fmt, ...);

// ---------------------------------------------------------------------------
// APU MMIO base addresses (physical, identity-mapped on Xbox)
// ---------------------------------------------------------------------------
#define APU_BASE      0xFE800000u
#define APU_VP_BASE   0xFE820000u  // Voice Processor PIO
#define APU_GP_BASE   0xFE830000u  // Global Processor
#define APU_EP_BASE   0xFE850000u  // Encode Processor

static volatile uint32_t *const apu_reg  = (volatile uint32_t *)APU_BASE;
static volatile uint32_t *const vp_reg   = (volatile uint32_t *)APU_VP_BASE;
static volatile uint32_t *const gp_reg   = (volatile uint32_t *)APU_GP_BASE;
static volatile uint32_t *const ep_reg   = (volatile uint32_t *)APU_EP_BASE;

// Register helper — byte offset to uint32 index
#define APU(off)  apu_reg[(off) / 4]
#define VP(off)   vp_reg[(off) / 4]
#define GP(off)   gp_reg[(off) / 4]
#define EP(off)   ep_reg[(off) / 4]

// ---------------------------------------------------------------------------
// APU global registers (offsets from APU_BASE)
// ---------------------------------------------------------------------------
#define NV_PAPU_ISTS        0x1000   // Interrupt status
#define NV_PAPU_IEN         0x1004   // Interrupt enable
#define NV_PAPU_FECTL       0x1100   // Front-end control (FEMETHMODE at bits [7:5])
#define NV_PAPU_FECV        0x1110   // FE current voice (set by SET_CURRENT_VOICE)
#define NV_PAPU_FEAV        0x1118   // FE antecedent voice (set by SET_ANTECEDENT_VOICE)
#define NV_PAPU_FEDECMETH   0x1300   // FE decoded method (last PIO command)
#define NV_PAPU_FEDECPARAM  0x1304   // FE decoded parameter (last PIO argument)
#define NV_PAPU_FETFORCE0   0x1500   // FE trap force 0
#define NV_PAPU_FETFORCE1   0x1504   // FE trap force 1
#define NV_PAPU_SECTL       0x2000   // Section enable (XCNTMODE at bits [4:3])
#define NV_PAPU_SECONFIG    0x2004   // SE/VP config — bit 5 enables voice processing
#define NV_PAPU_XGSCNT      0x200C   // Global sample counter (ticks at 48kHz)
#define NV_PAPU_VPVADDR     0x202C   // Voice array physical address
#define NV_PAPU_VPSGEADDR   0x2030   // SGE table physical address
#define NV_PAPU_MAXVOICES   0x2050   // Max voice index for SE iteration
#define NV_PAPU_TVL2D       0x2054   // 2D voice list tail
#define NV_PAPU_CVL2D       0x2058   // 2D voice list current (VP iteration pointer)
#define NV_PAPU_NVL2D       0x205C   // 2D voice list next
#define NV_PAPU_TVL3D       0x2060   // 3D voice list tail
#define NV_PAPU_CVL3D       0x2064   // 3D voice list current
#define NV_PAPU_NVL3D       0x2068   // 3D voice list next
#define NV_PAPU_TVLmp       0x206C   // Multipass voice list tail
#define NV_PAPU_CVLmp       0x2070   // Multipass voice list current
#define NV_PAPU_NVLmp       0x2074   // Multipass voice list next
#define NV_PAPU_VPSSLADDR   0x2034   // VP SSL table base address
#define NV_PAPU_GPSADDR     0x2040   // GP scratch SGE base
#define NV_PAPU_GPFADDR     0x2044   // GP FIFO SGE base
#define NV_PAPU_EPSADDR     0x2048   // EP scratch SGE base
#define NV_PAPU_EPFADDR     0x204C   // EP FIFO SGE base
#define NV_PAPU_SUBMIX0     0x208C   // Submix voice count registers (8 regs × 4 bytes = 32 groups)
#define NV_PAPU_GPSMAXSGE   0x20D4   // GP scratch max SGE index
#define NV_PAPU_GPFMAXSGE   0x20D8   // GP FIFO max SGE index
#define NV_PAPU_EPSMAXSGE   0x20DC   // EP scratch max SGE index
#define NV_PAPU_EPFMAXSGE   0x20E0   // EP FIFO max SGE index
#define NV_PAPU_FENADDR     0x115C   // FE notifier base address

// FECTL register bits
#define FECTL_FEMETHMODE_MASK      0xE0  // bits [7:5]
#define FECTL_FEMETHMODE_FREE      0x00  // Free-running
#define FECTL_FEMETHMODE_TRAPPED   0xE0  // Trapped (halted)

// FETFORCE1 bits — must set to prevent xemu assert on idle voice
#define FETFORCE1_SE2FE_IDLE_VOICE (1 << 15)

// GP memory offsets (from APU_GP_BASE = 0xFE830000)
#define NV_PAPU_GPXMEM      0x0000   // GP X-memory (4096 × 24-bit words)
#define NV_PAPU_GPMIXBUF    0x5000   // GP MIXBUF (VP output, 1024 words)
#define NV_PAPU_GPYMEM      0x6000   // GP Y-memory (2048 words)
#define NV_PAPU_GPPMEM      0xA000   // GP P-memory (program, 4096 words)
#define NV_PAPU_GPRST       0xFFFC   // GP reset control (in GP MMIO space)

// EP memory offsets (from APU_EP_BASE = 0xFE850000)
#define NV_PAPU_EPXMEM      0x0000   // EP X-memory
#define NV_PAPU_EPYMEM      0x6000   // EP Y-memory
#define NV_PAPU_EPPMEM      0xA000   // EP P-memory (program, 4096 words)
#define NV_PAPU_EPRST       0xFFFC   // EP reset control

// GP reset register bits
#define GPRST_GPRST         (1 << 0) // GP subsystem reset (1=reset)
#define GPRST_GPDSPRST      (1 << 1) // GP DSP core reset (1=reset)

// DSP56300 instruction opcodes (24-bit, stored in low 24 bits of uint32)
#define DSP_OP_NOP           0x000000
#define DSP_OP_WAIT          0x000086
#define DSP_OP_JMP_ABS       0x0AF080 // JMP absolute (word 1); word 2 = target
// JCLR #1, X:<<$05, addr — Jump if bit 1 of peripheral 0xFFFFC5 is clear
// Peripheral 0xFFFFC5 is the DSP interrupt status register.
// Bit 1 = INTERRUPT_START_FRAME. Polls until frame interrupt fires.
#define DSP_OP_JCLR_FRAME    0x0A8581
// BSET #0, X:<<$04 — Set bit 0 of peripheral X:$FFFFC4 (idle/control register)
// Writing bit 0 signals "frame done / DSP idle" to the APU frame engine.
// DSP56300 BSET peripheral encoding: 0000101010pppppp0S1bbbbb (24-bit)
//   pppppp=000100 ($04), S=0 (X-mem), bbbbb=00000 (bit 0)
//   = 00001010_10000100_00100000 = 0x0A8420
#define DSP_OP_BSET_IDLE     0x0A8420
// BCLR #1, X:<<$05 — Clear bit 1 of peripheral X:$FFFFC5 (interrupt status)
// Acknowledges INTERRUPT_START_FRAME after reading it.
// DSP56300 BCLR peripheral encoding: 0000101010pppppp0S0bbbbb (24-bit)
//   pppppp=000101 ($05), S=0 (X-mem), bbbbb=00001 (bit 1)
//   = 00001010_10000101_00000001 = 0x0A8501
#define DSP_OP_BCLR_FRAME    0x0A8501

// Bootstrap loads 0x800 words from scratch into PMEM
#define GP_BOOTSTRAP_WORDS   0x800

// VP PIO command offsets (from APU_VP_BASE)
#define VP_PIO_FREE              0x010   // Read: FIFO free count (xemu returns 0x80)
#define VP_SET_ANTECEDENT_VOICE  0x120
#define VP_VOICE_ON              0x124
#define VP_VOICE_OFF             0x128
#define VP_VOICE_PAUSE           0x140
#define VP_SET_CURRENT_VOICE     0x2F8
#define VP_SET_VOICE_CFG_VBIN    0x300
#define VP_SET_VOICE_CFG_FMT     0x304
#define VP_SET_VOICE_TAR_VOLA    0x360
#define VP_SET_VOICE_TAR_VOLB    0x364
#define VP_SET_VOICE_TAR_VOLC    0x368
#define VP_SET_VOICE_TAR_PITCH   0x37C
#define VP_SET_VOICE_CFG_BUF_BASE 0x3A0
#define VP_SET_VOICE_CFG_BUF_LBO 0x3A4
#define VP_SET_VOICE_BUF_CBO    0x3D8
#define VP_SET_VOICE_CFG_BUF_EBO 0x3DC

// Voice list identifiers for SET_ANTECEDENT_VOICE
#define VOICE_LIST_2D_TOP   1   // Insert at head of 2D list (0=inherit, 1=2D, 2=3D, 3=MP)

// VP voice structure size
#define VP_VOICE_SIZE   128     // 0x80 bytes per voice

// Voice structure field offsets (128 bytes per voice, in RAM at VPVADDR)
#define VOICE_CFG_VBIN       0x00  // Volume bin routing (5 bits × 6 bins)
#define VOICE_CFG_FMT        0x04  // Format: sample size, container, stereo, loop
#define VOICE_CUR_PSL_START  0x20  // Buffer base address (byte offset in SGE space)
#define VOICE_CUR_PSH_SAMPLE 0x24  // Loop back offset (LBO, 24-bit)
#define VOICE_CUR_ECNT       0x34  // Envelope counters
#define VOICE_PAR_STATE      0x54  // State: active, paused, new, envelope states
#define VOICE_PAR_OFFSET     0x58  // Current buffer offset (CBO, 24-bit) + EA level
#define VOICE_PAR_NEXT       0x5C  // End buffer offset (EBO, 24-bit) + EF level
#define VOICE_TAR_VOLA       0x60  // Volumes 0,1 (12-bit attenuation each)
#define VOICE_TAR_VOLB       0x64  // Volumes 2,3
#define VOICE_TAR_VOLC       0x68  // Volumes 4,5
#define VOICE_TAR_PITCH_LINK 0x7C  // Pitch [31:16] + next voice handle [15:0]

// PAR_STATE bit positions (from xemu NV_PAVS_VOICE_PAR_STATE)
#define VOICE_PAR_ACTIVE     (1u << 21)  // ACTIVE_VOICE
#define VOICE_PAR_NEW        (1u << 20)  // NEW_VOICE (needs init by VP)
#define VOICE_PAR_PAUSED     (1u << 18)  // PAUSED (bit 18, not 19)
#define VOICE_PAR_EACUR_SHIFT 28         // Amplitude envelope state [31:28]
#define VOICE_PAR_EFCUR_SHIFT 24         // Filter envelope state [27:24]
// Envelope states: 0=OFF, 1=DELAY, 2=ATTACK, 3=HOLD, 4=DECAY, 5=SUSTAIN, 6=RELEASE

// SGE entry: 8 bytes
#define SGE_ENTRY_SIZE   8
#define SGE_MAX_ENTRIES  4096   // 32KB allocation / 8 bytes per entry

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define MAX_VOICES      256
#define MAX_HW_VOICES   8       // PCM pool slots (matches Duke3D's 8 voices)
#define PCM_POOL_SLOT   (64 * 1024)  // 64KB per slot
#define PAGE_SIZE       4096

// MIXBUF: 32 bins × 32 samples per frame, each sample is int32 (from VP)
#define MIXBUF_BINS           32
#define MIXBUF_SAMPLES_PER_BIN 32
#define MIXBUF_OFFSET         0x5000  // Within GP address space

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static int apu_initialized = 0;

// Contiguous memory allocations
static void     *voice_array_virt = NULL;   // 256 × 128 = 32KB
static uint32_t  voice_array_phys = 0;
static void     *sge_table_virt = NULL;     // 4096 × 8 = 32KB
static uint32_t  sge_table_phys = 0;

// PCM pool: MAX_HW_VOICES slots of PCM_POOL_SLOT bytes each
static void     *pcm_pool_virt[MAX_HW_VOICES];
static uint32_t  pcm_pool_phys[MAX_HW_VOICES];

// Voice allocation bitmap
static uint8_t   voice_allocated[MAX_VOICES];
// Map voice handle → PCM pool slot (-1 = none)
static int       voice_pcm_slot[MAX_VOICES];
// Track which PCM pool slots are in use
static uint8_t   pcm_slot_used[MAX_HW_VOICES];

// SGE allocation: next free SGE index
static int       sge_next_free = 0;
// Per-voice SGE base index
static int       voice_sge_base[MAX_VOICES];
static int       voice_sge_count[MAX_VOICES];

// Per-voice playing state (tracked in software for IsPlaying queries)
static uint8_t   voice_playing[MAX_VOICES];

// Diagnostic counters
static int diag_play_calls = 0;
static int diag_play_ok = 0;
static int diag_play_fail = 0;
static int diag_play_pool_fail = 0;
static int diag_play_sge_fail = 0;
static int diag_stop_calls = 0;
static int diag_alloc_calls = 0;
static int diag_alloc_ok = 0;
// Store first failure info
static int diag_first_fail_reason = 0;
static int diag_first_fail_voice = -1;
static unsigned int diag_first_fail_bytes = 0;
static const void *diag_first_fail_pcm = NULL;
static int diag_first_fail_bits = 0;
static int diag_first_fail_channels = 0;

// GP (Global Processor) state
static int       gp_initialized = 0;

// EP (Encode Processor) state
static int       ep_initialized = 0;

// Additional memory allocations for VP/FE
static void     *notifier_virt = NULL;    // FE notifier area
static uint32_t  notifier_phys = 0;
static void     *ssl_table_virt = NULL;   // VP SSL table
static uint32_t  ssl_table_phys = 0;

// Self-allocated GP/EP scratch SGE tables and pages
static void     *gp_scratch_virt = NULL;   // GP scratch SGE table (8 entries × 8 bytes)
static uint32_t  gp_scratch_phys = 0;
static void     *gp_scratch_pages_virt = NULL;  // GP scratch PMEM pages (2 × 4KB)
static uint32_t  gp_scratch_pages_phys = 0;
static void     *gp_fifo_virt = NULL;      // GP FIFO SGE table
static uint32_t  gp_fifo_phys = 0;
static void     *ep_scratch_virt = NULL;
static uint32_t  ep_scratch_phys = 0;
static void     *ep_scratch_pages_virt = NULL;
static uint32_t  ep_scratch_pages_phys = 0;
static void     *ep_fifo_virt = NULL;
static uint32_t  ep_fifo_phys = 0;


// ---------------------------------------------------------------------------
// Volume/pitch conversion helpers
// ---------------------------------------------------------------------------

// Convert Duke3D 0-255 linear volume to VP 12-bit attenuation (0=full, 0xFFF=mute)
static inline uint32_t vol_to_atten(int vol)
{
    if (vol <= 0) return 0xFFF;
    if (vol >= 255) return 0;
    return (uint32_t)((255 - vol) * 0xFFF / 255);
}

// Pack two 12-bit attenuation values into TAR_VOLA format (from xemu apu_regs.h):
// bits [15:4] = volume0 (left), bits [31:20] = volume1 (right)
static inline uint32_t pack_vola(uint32_t left_atten, uint32_t right_atten)
{
    return ((left_atten & 0xFFF) << 4) | ((right_atten & 0xFFF) << 20);
}

// Convert sample rate to VP pitch value.
// VP pitch = 4096 * log2(48000 / rate), stored as 16-bit unsigned.
// We use a fixed lookup for common rates, linear interpolation otherwise.
static uint16_t rate_to_pitch(unsigned int rate)
{
    // Common rates — pre-computed
    if (rate >= 48000) return 0x0000;
    if (rate == 44100) return 0x01F5;
    if (rate == 22050) return 0x11F5;
    if (rate == 11025) return 0x21F5;
    if (rate == 8000)  return 0x2960;

    // General case: approximate log2(48000/rate) * 4096
    // Use integer bit-scan + linear fractional part
    if (rate == 0) return 0xFFFF;

    // ratio = 48000 / rate in 16.16 fixed point
    uint32_t ratio_fp = (48000u << 16) / rate;
    // log2 of the integer part
    int int_part = 0;
    uint32_t tmp = ratio_fp >> 16;
    while (tmp > 1) { tmp >>= 1; int_part++; }
    // Fractional part: linear approximation between powers of 2
    uint32_t base = 1u << int_part;
    uint32_t next = base << 1;
    uint32_t frac = 0;
    if (next > base) {
        frac = ((ratio_fp >> 16) - base) * 4096 / (next - base);
    }
    uint32_t pitch = (uint32_t)int_part * 4096 + frac;
    if (pitch > 0xFFFF) pitch = 0xFFFF;
    return (uint16_t)pitch;
}

// ---------------------------------------------------------------------------
// SGE table management
// ---------------------------------------------------------------------------

// Set up SGE entries for a contiguous physical buffer.
// Returns the SGE base index, or -1 on failure.
static int sge_setup(uint32_t phys_addr, unsigned int bytes)
{
    int pages = (bytes + PAGE_SIZE - 1) / PAGE_SIZE;
    if (sge_next_free + pages > SGE_MAX_ENTRIES) return -1;

    int base = sge_next_free;
    uint8_t *sge = (uint8_t *)sge_table_virt;

    for (int i = 0; i < pages; i++) {
        int idx = base + i;
        uint32_t page_phys = phys_addr + (uint32_t)(i * PAGE_SIZE);
        // SGE entry: dword 0 = physical page address (page-aligned)
        //            dword 1 = control (0 for simple buffer)
        uint32_t *entry = (uint32_t *)(sge + idx * SGE_ENTRY_SIZE);
        entry[0] = page_phys & ~0xFFFu;  // Page-aligned address
        entry[1] = 0;                     // No special control
    }

    sge_next_free += pages;
    return base;
}

// Free SGE entries for a voice (just mark them reusable).
// For simplicity we don't compact — SGE table is large enough for our needs.
static void sge_free(int base, int count)
{
    // If these are the last entries allocated, reclaim them
    if (base + count == sge_next_free) {
        sge_next_free = base;
    }
    // Otherwise we just leak them — with 4096 entries and 8 voices this is fine
}

// ---------------------------------------------------------------------------
// PCM pool management
// ---------------------------------------------------------------------------

static int pcm_pool_alloc(void)
{
    for (int i = 0; i < MAX_HW_VOICES; i++) {
        if (!pcm_slot_used[i]) {
            pcm_slot_used[i] = 1;
            return i;
        }
    }
    return -1;
}

static void pcm_pool_free(int slot)
{
    if (slot >= 0 && slot < MAX_HW_VOICES) {
        pcm_slot_used[slot] = 0;
    }
}

// ---------------------------------------------------------------------------
// Allocate physically contiguous memory (wrapper)
// ---------------------------------------------------------------------------
// Allocate contiguous memory with specific alignment (0 = default page alignment)
static void *alloc_contiguous_aligned(unsigned int size, unsigned int align,
                                       uint32_t *out_phys)
{
    void *virt = MmAllocateContiguousMemoryEx(
        size,
        0,              // lowest address
        0x7FFFFFFF,     // highest (first 2GB)
        align,          // alignment (0 = page default)
        PAGE_READWRITE  // protection
    );
    if (!virt) return NULL;
    memset(virt, 0, size);
    *out_phys = (uint32_t)MmGetPhysicalAddress(virt);
    return virt;
}

static void *alloc_contiguous(unsigned int size, uint32_t *out_phys)
{
    return alloc_contiguous_aligned(size, 0, out_phys);
}

// ---------------------------------------------------------------------------
// GP DSP program — DirectSound's GP program extracted from XDK dump test
// ---------------------------------------------------------------------------
// The SE requires a real GP mixing program to advance the frame pipeline.
// Our previous 6-word idle loop (JCLR→BCLR→BSET→JMP) handled the frame
// protocol correctly but the SE never walked the voice list — CVL2D stayed
// FFFF. Loading the actual DirectSound GP program (355 non-zero words)
// matches the exact working state observed via the apu_dump test app.
//
// This program was extracted from a real Xbox running DirectSoundCreate().
// It handles MIXBUF processing, effect chain mixing, and frame handshaking.

static const uint32_t ds_gp_pmem[376] = {
    0x0BF080,0x000155,0x300600,0x310000,0x62F400,0x000800,0x330600,0x0BF080, // [000]
    0x0000DB,0x301800,0x310000,0x62F400,0x000800,0x330600,0x0BF080,0x0000EB, // [008]
    0x0BF080,0x00009E,0x301E00,0x61F400,0x001560,0x62F400,0x00B000,0x63F400, // [010]
    0x000280,0x0BF080,0x0000EB,0x44F400,0x0049E2,0x0200C4,0x240000,0x447000, // [018]
    0x00007F,0x447000,0x00007E,0x07F432,0xFFFFFF,0x07F430,0x000001,0x07F431, // [020]
    0x000001,0x08F484,0x000001,0x0BF080,0x000160,0x44F000,0xFFFFB3,0x447000, // [028]
    0x00007F,0x0BF080,0x00005C,0x44F000,0xFFFFB3,0x447000,0x00007D,0x0BF080, // [030]
    0x000171,0x200013,0x54F000,0xFFFFB3,0x44F000,0x00007D,0x0C1FF8,0x059405, // [038]
    0x218E00,0x200044,0x547000,0x00007C,0x56F400,0x000025,0x0BF080,0x000126, // [040]
    0x56F400,0x00001E,0x0BF080,0x000126,0x200013,0x54F000,0xFFFFB3,0x44F000, // [048]
    0x00007F,0x0C1FF8,0x059797,0x218E00,0x200044,0x547000,0x00007E,0x44F400, // [050]
    0x019E10,0x0C1FF8,0x05978F,0x050F8E,0x56F400,0x000006,0x0BF080,0x000126, // [058]
    0x448400,0x56F400,0x000000,0x200045,0x0D104A,0x000039,0x0A0480,0x000084, // [060]
    0x60F400,0x000080,0x44F400,0x000F80,0x0BF080,0x000165,0x0BF080,0x000169, // [068]
    0x05F420,0xFFFFFF,0x0461A0,0x0462A0,0x0463A0,0x0464A0,0x0465A0,0x60F400, // [070]
    0x000012,0x61F400,0x002971,0x62F400,0x000818,0x638100,0x0BF080,0x0000DB, // [078]
    0x56F400,0x000012,0x0BF080,0x000126,0x0A0481,0x00009D,0x60F400,0x00000C, // [080]
    0x56F400,0x000080,0x448000,0x200040,0x21D100,0x448200,0x209200,0x638300, // [088]
    0x0BF080,0x0000DB,0x56F400,0x00000C,0x0BF080,0x000126,0x44F400,0x000000, // [090]
    0x440400,0x56F400,0x000018,0x0BF080,0x000126,0x00000C,0x64F400,0x000025, // [098]
    0x44F400,0x008000,0x61F400,0x000024,0x446100,0x56F400,0x00002C,0x62F400, // [0A0]
    0x0000CC,0x060690,0x000017,0x565C00,0x0140C0,0x000007,0x44F400,0x0059D2, // [0A8]
    0x445C00,0x44F400,0x000020,0x445C00,0x07DA84,0x445C00,0x44F400,0x000000, // [0B0]
    0x445C00,0x57E100,0x575C00,0x0140C8,0x000800,0x576100,0x44F400,0x0007FF, // [0B8]
    0x445C00,0x56F400,0x000025,0x0BF080,0x0000D2,0x0140C0,0x000023,0x21D400, // [0C0]
    0x44F400,0x004000,0x445C00,0x00000C,0x001400,0x001440,0x001420,0x001480, // [0C8]
    0x0014A0,0x001460,0x0D1080,0x000038,0x08F497,0x000000,0x0140C6,0x003FFF, // [0D0]
    0x08CE15,0x08CE14,0x00000C,0x220E00,0x0140C6,0x003FFF,0x0140C2,0x004000, // [0D8]
    0x02008E,0x56F400,0x0059E0,0x0200CE,0x0A7093,0x000002,0x0A7091,0x000003, // [0E0]
    0x0A7092,0x000004,0x00000C,0x220E00,0x0140C6,0x003FFF,0x0140C2,0x004000, // [0E8]
    0x02008E,0x56F400,0x0059E2,0x0200CE,0x0A7093,0x000002,0x0A7091,0x000003, // [0F0]
    0x0A7092,0x000004,0x00000C,0x08F496,0x000001,0x0CD604,0x000000,0x00000C, // [0F8]
    0x08F496,0x000002,0x0CD624,0x000000,0x00000C,0x08F496,0x000003,0x0CD603, // [100]
    0x000000,0x00000C,0x08F496,0x000002,0x0CD624,0x000000,0x08F496,0x000003, // [108]
    0x0CD603,0x000000,0x00000C,0x08F496,0x000004,0x0CD623,0x000000,0x00000C, // [110]
    0x08F496,0x000004,0x0CD623,0x000000,0x08F496,0x000001,0x0CD604,0x000000, // [118]
    0x00000C,0x0CC507,0x000000,0x08F485,0x000080,0x00000C,0x050BC4,0x0140C6, // [120]
    0xFFDFFF,0x08CE14,0x050BCE,0x050BD6,0x00000C,0x050B9D,0x0140C6,0xFFDFFF, // [128]
    0x08CE14,0x050BC7,0x00000C,0x220E00,0x0140C6,0x003FFF,0x0140C2,0x004000, // [130]
    0x02008E,0x56F400,0x005BE2,0x0200CE,0x56F400,0x000020,0x02088E,0x0A7091, // [138]
    0x000003,0x0A7092,0x000004,0x00000C,0x220E00,0x0140C6,0x003FFF,0x0140C2, // [140]
    0x004000,0x02008E,0x56F400,0x005BE0,0x0200CE,0x56F400,0x000020,0x02088E, // [148]
    0x0A7091,0x000003,0x0A7092,0x000004,0x00000C,0x08F485,0x000FFF,0x60F400, // [150]
    0x000000,0x44F400,0x001000,0x0BF080,0x000165,0x0BF080,0x000169,0x00000C, // [158]
    0x0A8581,0x000160,0x08F485,0x000002,0x00000C,0x200013,0x06C420,0x565800, // [160]
    0x00000C,0x60F400,0x000000,0x44F400,0x000800,0x200013,0x06C420,0x5E5800, // [168]
    0x00000C,0x000000,0x00000C,0x000000,0x000000,0x000000,0x000000,0x000000, // [170]
};

// GP XMEM configuration from DirectSound — mixbin routing and effect chain params
static const struct { uint16_t addr; uint32_t val; } ds_gp_xmem[] = {
    {0x0006,0x004006},{0x0007,0x0059E0},{0x0008,0x000006},{0x000A,0x000800},
    {0x0018,0x004018},{0x0019,0x0059E2},{0x001A,0x000006},{0x001C,0x000800},
    {0x001E,0x00401E},{0x001F,0x0049E2},{0x0020,0x000280},{0x0021,0x001560},
    {0x0022,0x00B000},{0x0024,0x00B000},{0x0025,0x00002C},{0x0026,0x0059D2},
    {0x0027,0x000020},{0x0028,0x001400},{0x0029,0x000480},{0x002A,0x008000},
    {0x002B,0x0007FF},{0x002C,0x000033},{0x002D,0x0059D2},{0x002E,0x000020},
    {0x002F,0x001440},{0x0030,0x000780},{0x0031,0x008800},{0x0032,0x0007FF},
    {0x0033,0x00003A},{0x0034,0x0059D2},{0x0035,0x000020},{0x0036,0x001420},
    {0x0037,0x000280},{0x0038,0x009000},{0x0039,0x0007FF},{0x003A,0x000041},
    {0x003B,0x0059D2},{0x003C,0x000020},{0x003D,0x001480},{0x003E,0x000580},
    {0x003F,0x009800},{0x0040,0x0007FF},{0x0041,0x000048},{0x0042,0x0059D2},
    {0x0043,0x000020},{0x0044,0x0014A0},{0x0045,0x000180},{0x0046,0x00A000},
    {0x0047,0x0007FF},{0x0048,0x004000},{0x0049,0x0059D2},{0x004A,0x000020},
    {0x004B,0x001460},{0x004C,0x000480},{0x004D,0x00A800},{0x004E,0x0007FF},
    {0x007C,0x00000D},{0x007D,0x284164},{0x007E,0x0014A0},{0x007F,0x2AF61B},
};

// ---------------------------------------------------------------------------
// Custom GP program — copies MIXBUF bins 0+1 to CPU-readable XMEM each frame
// ---------------------------------------------------------------------------
// 20 words. Replaces the DirectSound GP program.
// Uses R7=0 (reset default) as base for XMEM reads with offset addressing.
// XMEM[0] = MIXBUF source base (0x001400 in DSP address space)
// XMEM[1] = XMEM destination base (0x000080)
// XMEM[4] = frame counter (incremented each frame, for CPU diagnostics)
//
// The frame protocol matches DirectSound's: poll INTERRUPT_START_FRAME (bit 1
// of peripheral $05), acknowledge with MOVEP, do work, signal idle via BSET.
// This keeps the SE frame pipeline running and voice lists walking.
static const uint32_t gp_mixbuf_copy[] = {
    /* $000 */ 0x0A8581,  // JCLR  #1, X:<<$05, $0000  ; poll frame start (bit 1)
    /* $001 */ 0x000000,  //   jump target = $0000
    /* $002 */ 0x08F485,  // MOVEP #imm, X:<<$05        ; acknowledge frame (write bit 1)
    /* $003 */ 0x000002,  //   immediate = 0x000002
    /* $004 */ 0x0A77D0,  // MOVE  X:(R7+off),R0        ; R0 = MIXBUF base from XMEM[0]
    /* $005 */ 0x000000,  //   offset = 0
    /* $006 */ 0x0A77D4,  // MOVE  X:(R7+off),R4        ; R4 = dest base from XMEM[1]
    /* $007 */ 0x000001,  //   offset = 1
    /* $008 */ 0x064080,  // DO    #64, $000B            ; copy 64 words (bins 0+1)
    /* $009 */ 0x00000B,  //   loop end address
    /* $00A */ 0x56D800,  // MOVE  X:(R0)+,A             ; read from MIXBUF
    /* $00B */ 0x565C00,  // MOVE  A,X:(R4)+             ; write to XMEM dest
    /* $00C */ 0x0A77CE,  // MOVE  X:(R7+off),A          ; read frame counter
    /* $00D */ 0x000004,  //   offset = 4 -> XMEM[4]
    /* $00E */ 0x000008,  // INC   A                     ; increment
    /* $00F */ 0x0A778E,  // MOVE  A,X:(R7+off)          ; write frame counter back
    /* $010 */ 0x000004,  //   offset = 4 -> XMEM[4]
    /* $011 */ 0x0A8420,  // BSET  #0, X:<<$04           ; signal idle
    /* $012 */ 0x0AF080,  // JMP   $0000                 ; loop back
    /* $013 */ 0x000000,  //   jump target
};

// XMEM destination for copied MIXBUF data (word offset in GP XMEM)
// Bin 0 (left):  XMEM[0x0080..0x009F] = 32 samples
// Bin 1 (right): XMEM[0x00A0..0x00BF] = 32 samples
// CPU reads at: APU_GP_BASE + word_offset * 4 = 0xFE830200..0xFE8302FC
#define GP_XMEM_DST_BASE    0x0080
#define GP_XMEM_DST_WORDS   64       // 2 bins × 32 samples
#define GP_XMEM_FRAME_CTR   4        // XMEM word for frame counter

static int gp_init(void)
{
    // --- GP init: allocate OWN scratch SGE + FIFO SGE (like DirectSound) ---
    // Previous approach reused kernel's GPSADDR — may point to stale/freed pages.
    // DirectSound always allocates its own GPSADDR and GPFADDR.

    // 1. Hold GP in full reset
    GP(NV_PAPU_GPRST) = 0;
    for (volatile int d = 0; d < 1000; d++) {}

    // 2. Allocate our own scratch pages (2 × 4KB = 8KB for 2048 PMEM words)
    gp_scratch_pages_virt = alloc_contiguous(PAGE_SIZE * 2, &gp_scratch_pages_phys);
    if (!gp_scratch_pages_virt) {
        xbox_log("APU: GP scratch pages alloc FAILED\n");
        return -1;
    }

    // 3. Allocate our own scratch SGE table — MUST be 16KB aligned!
    //    GPSADDR register masks bits [13:0], so address must be 0x4000-aligned.
    gp_scratch_virt = alloc_contiguous_aligned(PAGE_SIZE, 0x4000, &gp_scratch_phys);
    if (!gp_scratch_virt) {
        xbox_log("APU: GP scratch SGE alloc FAILED\n");
        return -1;
    }

    // 4. Allocate GP FIFO SGE table — also 16KB aligned (GPFADDR masks same way)
    gp_fifo_virt = alloc_contiguous_aligned(PAGE_SIZE, 0x4000, &gp_fifo_phys);
    if (!gp_fifo_virt) {
        xbox_log("APU: GP fifo SGE alloc FAILED\n");
        return -1;
    }

    xbox_log("APU: GP alloc scratch_pages=%08X sge=%08X fifo=%08X\n",
             (unsigned)gp_scratch_pages_phys, (unsigned)gp_scratch_phys,
             (unsigned)gp_fifo_phys);

    // 5. Set up scratch SGE table: 2 entries pointing to our pages
    volatile uint32_t *sge = (volatile uint32_t *)gp_scratch_virt;
    memset((void *)gp_scratch_virt, 0, PAGE_SIZE);
    sge[0] = gp_scratch_pages_phys;              // entry 0: page 0 address
    sge[1] = 0;                                   // entry 0: ctrl (not EOL)
    sge[2] = gp_scratch_pages_phys + PAGE_SIZE;   // entry 1: page 1 address
    sge[3] = (1u << 14);                           // entry 1: ctrl (EOL)

    // 6. Zero FIFO SGE table
    memset((void *)gp_fifo_virt, 0, PAGE_SIZE);

    // 7. Write our program to scratch page 0
    volatile uint32_t *p0 = (volatile uint32_t *)gp_scratch_pages_virt;
    for (int i = 0; i < 1024; i++) p0[i] = 0;

    // ALIVE-TEST v5: NO JCLR — free-running counter.
    // Previous versions waited for START_FRAME via JCLR but never got it.
    // This version: tight loop incrementing XMEM[4], signaling idle each iteration.
    // If XMEM[4] increments: DSP runs fine, problem is START_FRAME delivery.
    // XMEM[5] = 0xCAFE (execution marker, written once).
    // XMEM[6] = copy of counter (confirms XMEM writes work).
    {
        int a = 0;
        // --- Preamble: write 0xCAFE to XMEM[5] ---
        p0[a++] = 0x60F400;            // $000: MOVE #5, R0
        p0[a++] = 0x000005;            // $001:   immediate
        p0[a++] = 0x56F400;            // $002: MOVE #$00CAFE, A
        p0[a++] = 0x00CAFE;            // $003:   immediate
        p0[a++] = 0x566000;            // $004: MOVE A, X:(R0)

        // $005: BSET idle (initial edge for SE)
        p0[a++] = DSP_OP_BSET_IDLE;    // $005: BSET #0, X:<<$04

        // --- Main loop at $006: increment XMEM[4] then BSET idle, loop ---
        p0[a++] = 0x60F400;            // $006: MOVE #4, R0
        p0[a++] = 0x000004;            // $007:   immediate
        p0[a++] = 0x56E000;            // $008: MOVE X:(R0), A
        p0[a++] = 0x000008;            // $009: INC A
        p0[a++] = 0x566000;            // $00A: MOVE A, X:(R0) — XMEM[4]++

        // Store iteration count to XMEM[6] too (confirms writes work)
        p0[a++] = 0x60F400;            // $00B: MOVE #6, R0
        p0[a++] = 0x000006;            // $00C:   immediate
        p0[a++] = 0x566000;            // $00D: MOVE A, X:(R0) — XMEM[6] = counter

        // BSET idle (complete the frame handshake)
        p0[a++] = DSP_OP_BSET_IDLE;    // $00E: BSET #0, X:<<$04
        // JMP back to $006
        p0[a++] = DSP_OP_JMP_ABS;      // $00F: JMP
        p0[a++] = 0x000006;            // $010:   target = $006

        xbox_log("APU: GP ALIVE-TEST v5 (free-run counter, %d words)\n", a);
    }

    // Clear page 1 (PMEM words 1024-2047)
    volatile uint32_t *p1 = (volatile uint32_t *)((uint8_t *)gp_scratch_pages_virt + PAGE_SIZE);
    for (int i = 0; i < 1024; i++) p1[i] = 0;

    // 8. Flush CPU cache so DMA sees all our writes
    __asm__ volatile("wbinvd" ::: "memory");

    // 9. Write GPSADDR and GPFADDR to point to OUR SGE tables
    uint32_t old_gps = APU(NV_PAPU_GPSADDR);
    uint32_t old_gpf = APU(NV_PAPU_GPFADDR);
    APU(NV_PAPU_GPSADDR) = gp_scratch_phys;
    APU(NV_PAPU_GPFADDR) = gp_fifo_phys;
    // Set GPSMAXSGE to match our 2-entry scratch table (entries 0 and 1)
    APU(NV_PAPU_GPSMAXSGE) = 1;  // max index = 1 (2 entries: 0 and 1)
    xbox_log("APU: GP GPSADDR %08X->%08X GPFADDR %08X->%08X MAXSGE=%X\n",
             (unsigned)old_gps, (unsigned)APU(NV_PAPU_GPSADDR),
             (unsigned)old_gpf, (unsigned)APU(NV_PAPU_GPFADDR),
             (unsigned)APU(NV_PAPU_GPSMAXSGE));

    // 10. TWO-PHASE GPRST release:
    //     Phase A: GPRST=1 (release subsystem, trigger DMA bootstrap, DSP stays reset)
    //     Phase B: GPRST=3 (release DSP, starts executing from PMEM[0])
    //     Single-step 0→3 may start DSP before DMA completes on real hardware.
    xbox_log("APU: GP GPRST=%X, phase A: setting to 1 (DMA bootstrap)\n", (unsigned)GP(NV_PAPU_GPRST));
    GP(NV_PAPU_GPRST) = GPRST_GPRST;  // = 1 (subsystem only)

    // Wait for DMA bootstrap to complete
    for (volatile int d = 0; d < 5000000; d++) {}

    // Verify PMEM loaded correctly via MMIO (safe: GPRST≥1)
    volatile uint32_t *pmem = (volatile uint32_t *)(APU_GP_BASE + NV_PAPU_GPPMEM);
    xbox_log("APU: GP PMEM after DMA: %06X %06X %06X %06X %06X %06X %06X\n",
             (unsigned)(pmem[0] & 0xFFFFFF), (unsigned)(pmem[1] & 0xFFFFFF),
             (unsigned)(pmem[2] & 0xFFFFFF), (unsigned)(pmem[3] & 0xFFFFFF),
             (unsigned)(pmem[4] & 0xFFFFFF), (unsigned)(pmem[5] & 0xFFFFFF),
             (unsigned)(pmem[6] & 0xFFFFFF));
    xbox_log("APU: GP PMEM $005: %06X %06X %06X %06X %06X %06X\n",
             (unsigned)(pmem[5] & 0xFFFFFF), (unsigned)(pmem[6] & 0xFFFFFF),
             (unsigned)(pmem[7] & 0xFFFFFF), (unsigned)(pmem[8] & 0xFFFFFF),
             (unsigned)(pmem[9] & 0xFFFFFF), (unsigned)(pmem[10] & 0xFFFFFF));

    // Set up XMEM BEFORE releasing DSP (DSP not running yet at GPRST=1)
    // XMEM[4]=1 (frame counter), XMEM[5]=0 (preamble marker)
    // DSP preamble will write 0xCAFE to XMEM[5] when it starts.
    gp_reg[GP_XMEM_FRAME_CTR] = 1;
    gp_reg[5] = 0;
    xbox_log("APU: XMEM pre-DSP: [4]=%06X [5]=%06X\n",
             (unsigned)(gp_reg[GP_XMEM_FRAME_CTR] & 0xFFFFFF),
             (unsigned)(gp_reg[5] & 0xFFFFFF));

    // Phase B deferred: GPRST=3 happens AFTER SE starts (SECTL=0x0F).
    // The DSP's initial BSET idle must occur while SE is running,
    // so SE can detect the non-idle→idle edge and deliver START_FRAME.
    // See gp_release_dsp() called from XApuInit after SECTL=0x0F.

    gp_initialized = 1;
    return 0;
}

// Release GP DSP (phase B of init). Must be called AFTER SE is started.
static void gp_release_dsp(void)
{
    xbox_log("APU: GP phase B: setting GPRST=3 (release DSP, SE already running)\n");
    GP(NV_PAPU_GPRST) = GPRST_GPRST | GPRST_GPDSPRST;  // = 3

    // Wait for DSP to execute preamble + BSET idle
    for (volatile int d = 0; d < 1000000; d++) {}

    // Check if DSP executed
    {
        uint32_t pre_marker = gp_reg[5] & 0xFFFFFF;
        uint32_t fctr = gp_reg[GP_XMEM_FRAME_CTR] & 0xFFFFFF;
        xbox_log("APU: post-DSP: XMEM[5]=%06X(%s) XMEM[4]=%06X GPRST=%X\n",
                 (unsigned)pre_marker,
                 (pre_marker == 0x00CAFE) ? "DSP_ALIVE!" : "no_exec",
                 (unsigned)fctr,
                 (unsigned)GP(NV_PAPU_GPRST));
    }
}

static void gp_shutdown(void)
{
    if (!gp_initialized) return;

    // Put GP back in reset
    GP(NV_PAPU_GPRST) = 0;

    gp_initialized = 0;
}

// ---------------------------------------------------------------------------
// EP DSP bootstrap — frame-aware idle program (same protocol as GP)
// ---------------------------------------------------------------------------

static int ep_init(void)
{
    // --- EP init: allocate OWN scratch SGE + FIFO SGE (like DirectSound) ---

    // 1. Hold EP in full reset
    EP(NV_PAPU_EPRST) = 0;
    for (volatile int d = 0; d < 1000; d++) {}

    // 2. Allocate scratch pages, scratch SGE, FIFO SGE
    ep_scratch_pages_virt = alloc_contiguous(PAGE_SIZE * 2, &ep_scratch_pages_phys);
    if (!ep_scratch_pages_virt) { xbox_log("APU: EP scratch pages FAILED\n"); return -1; }

    ep_scratch_virt = alloc_contiguous_aligned(PAGE_SIZE, 0x4000, &ep_scratch_phys);
    if (!ep_scratch_virt) { xbox_log("APU: EP scratch SGE FAILED\n"); return -1; }

    ep_fifo_virt = alloc_contiguous_aligned(PAGE_SIZE, 0x4000, &ep_fifo_phys);
    if (!ep_fifo_virt) { xbox_log("APU: EP fifo SGE FAILED\n"); return -1; }

    xbox_log("APU: EP alloc pages=%08X sge=%08X fifo=%08X\n",
             (unsigned)ep_scratch_pages_phys, (unsigned)ep_scratch_phys,
             (unsigned)ep_fifo_phys);

    // 3. Set up scratch SGE
    volatile uint32_t *sge = (volatile uint32_t *)ep_scratch_virt;
    memset((void *)ep_scratch_virt, 0, PAGE_SIZE);
    sge[0] = ep_scratch_pages_phys;
    sge[1] = 0;
    sge[2] = ep_scratch_pages_phys + PAGE_SIZE;
    sge[3] = (1u << 14);  // EOL

    memset((void *)ep_fifo_virt, 0, PAGE_SIZE);

    // 4. Write frame-aware idle program to scratch page 0
    volatile uint32_t *p0 = (volatile uint32_t *)ep_scratch_pages_virt;
    for (int i = 0; i < 1024; i++) p0[i] = 0;
    p0[0] = DSP_OP_JCLR_FRAME;     // JCLR #1, X:<<$05, $0000
    p0[1] = 0x000000;
    p0[2] = DSP_OP_BCLR_FRAME;     // BCLR #1, X:<<$05 (ack)
    p0[3] = DSP_OP_BSET_IDLE;      // BSET #0, X:<<$04 (idle)
    p0[4] = DSP_OP_JMP_ABS;        // JMP $0000
    p0[5] = 0x000000;

    // Clear page 1
    volatile uint32_t *p1 = (volatile uint32_t *)((uint8_t *)ep_scratch_pages_virt + PAGE_SIZE);
    for (int i = 0; i < 1024; i++) p1[i] = 0;

    // 5. Flush cache
    __asm__ volatile("wbinvd" ::: "memory");

    // 6. Write EPSADDR and EPFADDR
    uint32_t old_eps = APU(NV_PAPU_EPSADDR);
    uint32_t old_epf = APU(NV_PAPU_EPFADDR);
    APU(NV_PAPU_EPSADDR) = ep_scratch_phys;
    APU(NV_PAPU_EPFADDR) = ep_fifo_phys;
    APU(NV_PAPU_EPSMAXSGE) = 1;
    xbox_log("APU: EP EPSADDR %08X->%08X EPFADDR %08X->%08X MAXSGE=%X\n",
             (unsigned)old_eps, (unsigned)APU(NV_PAPU_EPSADDR),
             (unsigned)old_epf, (unsigned)APU(NV_PAPU_EPFADDR),
             (unsigned)APU(NV_PAPU_EPSMAXSGE));

    // 7. Single-step release (same as GP)
    EP(NV_PAPU_EPRST) = GPRST_GPRST | GPRST_GPDSPRST;  // = 3
    for (volatile int d = 0; d < 5000000; d++) {}

    // Diagnostic
    volatile uint32_t *pmem = (volatile uint32_t *)(APU_EP_BASE + NV_PAPU_EPPMEM);
    xbox_log("APU: EP PMEM: %06X %06X %06X %06X %06X %06X\n",
             (unsigned)(pmem[0] & 0xFFFFFF), (unsigned)(pmem[1] & 0xFFFFFF),
             (unsigned)(pmem[2] & 0xFFFFFF), (unsigned)(pmem[3] & 0xFFFFFF),
             (unsigned)(pmem[4] & 0xFFFFFF), (unsigned)(pmem[5] & 0xFFFFFF));

    ep_initialized = 1;
    return 0;
}

static void ep_shutdown(void)
{
    if (!ep_initialized) return;

    EP(NV_PAPU_EPRST) = 0;

    ep_initialized = 0;
}

// ---------------------------------------------------------------------------
// XApuInit — Phase 0 + Phase 1
// ---------------------------------------------------------------------------
int XApuInit(void)
{
    if (apu_initialized) return 0;

    // -----------------------------------------------------------------------
    // Phase 0: PCI probe & enable
    // -----------------------------------------------------------------------
    // APU is PCI bus 0, device 0, function 5 on REAL Xbox hardware.
    // xemu uses PCI_DEVFN(5,0) = dev 5 func 0, but real MCPX puts all
    // southbridge functions under device 0 (func 0-6).
    // Confirmed by PCI probe: slot 5 has VID=01B010DE BAR=FE800000.
    ULONG apu_slot = (0 << 3) | 5;  // bus 0, dev 0, func 5

    // Read BAR0 to verify APU base address
    ULONG bar0 = 0;
    HalReadWritePCISpace(0, apu_slot, 0x10, &bar0, sizeof(bar0), FALSE);
    bar0 &= ~0xFu;  // Mask type bits

    // Enable bus master + memory space in PCI command register
    USHORT pci_cmd = 0;
    HalReadWritePCISpace(0, apu_slot, 0x04, &pci_cmd, sizeof(pci_cmd), FALSE);
    pci_cmd |= 0x06;  // Memory space + bus master
    HalReadWritePCISpace(0, apu_slot, 0x04, &pci_cmd, sizeof(pci_cmd), TRUE);

    // -----------------------------------------------------------------------
    // Allocate contiguous memory — VP structures + PCM pool + notifier
    // -----------------------------------------------------------------------
    // DirectSound proves VPVADDR/VPSGEADDR/VPSSLADDR ARE writable — it
    // reallocates all VP structures during DirectSoundCreate. We do the same:
    // allocate our own voice array, SGE table, and SSL table.

    // VP address registers mask bits [13:12] — require 16KB (0x4000) alignment.
    // Kernel uses 0x034B8000, DirectSound uses 0x03FC8000 (both 32KB-aligned).
    #define VP_ADDR_ALIGN  0x4000  // 16KB minimum alignment

    // Voice array: 256 voices × 128 bytes = 32KB (already >= 16KB, naturally aligned)
    voice_array_virt = alloc_contiguous_aligned(MAX_VOICES * VP_VOICE_SIZE,
                                                 VP_ADDR_ALIGN, &voice_array_phys);
    if (!voice_array_virt) { xbox_log("APU: voice array alloc FAILED\n"); goto fail; }

    // SGE table: 4096 entries × 8 bytes = 32KB
    sge_table_virt = alloc_contiguous_aligned(4096 * SGE_ENTRY_SIZE,
                                               VP_ADDR_ALIGN, &sge_table_phys);
    if (!sge_table_virt) { xbox_log("APU: SGE table alloc FAILED\n"); goto fail; }

    // SSL table: 1 page (4KB) — not used for buffer mode, but needs valid address
    // Must also be 16KB-aligned for the VPSSLADDR register
    ssl_table_virt = alloc_contiguous_aligned(PAGE_SIZE, VP_ADDR_ALIGN, &ssl_table_phys);
    if (!ssl_table_virt) { xbox_log("APU: SSL table alloc FAILED\n"); goto fail; }

    // PCM pool: 8 slots × 64KB = 512KB (our own audio sample buffers)
    for (int i = 0; i < MAX_HW_VOICES; i++) {
        pcm_pool_virt[i] = alloc_contiguous(PCM_POOL_SLOT, &pcm_pool_phys[i]);
        if (!pcm_pool_virt[i]) goto fail;
    }

    // Notifier area: 1 page (FENADDR IS writable)
    notifier_virt = alloc_contiguous(PAGE_SIZE, &notifier_phys);
    if (!notifier_virt) goto fail;

    xbox_log("APU: alloc VA=%08X SGE=%08X SSL=%08X\n",
             (unsigned)voice_array_phys, (unsigned)sge_table_phys,
             (unsigned)ssl_table_phys);

    // -----------------------------------------------------------------------
    // Initialize tracking state
    // -----------------------------------------------------------------------
    memset(voice_allocated, 0, sizeof(voice_allocated));
    memset(voice_pcm_slot, 0xFF, sizeof(voice_pcm_slot));  // -1
    memset(pcm_slot_used, 0, sizeof(pcm_slot_used));
    memset(voice_playing, 0, sizeof(voice_playing));
    memset(voice_sge_base, 0xFF, sizeof(voice_sge_base));
    memset(voice_sge_count, 0, sizeof(voice_sge_count));
    sge_next_free = 0;

    // -----------------------------------------------------------------------
    // Phase 1: Full APU init with our own VP structures
    // -----------------------------------------------------------------------
    // The apu_dump test proved DirectSound successfully writes VPVADDR,
    // VPSGEADDR, etc. — these registers ARE writable during init.
    // We replicate DirectSound's approach: stop SE, write all VP structure
    // addresses, init GP/EP, configure SE, then start.
    // -----------------------------------------------------------------------

    // Save kernel's state for logging
    uint32_t kern_sectl = APU(NV_PAPU_SECTL);
    uint32_t kern_fectl = APU(NV_PAPU_FECTL);
    uint32_t kern_gprst = GP(NV_PAPU_GPRST);
    uint32_t kern_eprst = EP(NV_PAPU_EPRST);
    uint32_t kern_vpva  = APU(NV_PAPU_VPVADDR);
    xbox_log("APU: kern SECTL=%08X FECTL=%08X GPRST=%X EPRST=%X\n",
             (unsigned)kern_sectl, (unsigned)kern_fectl,
             (unsigned)kern_gprst, (unsigned)kern_eprst);
    xbox_log("APU: kern VPVA=%08X (ours=%08X)\n",
             (unsigned)kern_vpva, (unsigned)voice_array_phys);

    // -----------------------------------------------------------------------
    // STEP 1: Stop SE, reset GP only. KEEP kernel EP intact (EPRST=1).
    // DirectSound does EPRST 1→3 (keeps kernel's EP program loaded).
    // Our previous approach (EPRST=0→3 with tiny idle program) may have
    // broken the SE frame pipeline.
    // -----------------------------------------------------------------------
    uint32_t kern_seconfig = APU(NV_PAPU_SECONFIG);
    xbox_log("APU: kern SECONFIG=%08X before any changes\n", (unsigned)kern_seconfig);
    APU(NV_PAPU_IEN) = 0;                // Disable APU interrupts
    APU(NV_PAPU_ISTS) = 0xFFFFFFFF;      // Clear all pending
    APU(NV_PAPU_SECTL) = 0;              // Stop frame engine
    GP(NV_PAPU_GPRST) = 0;              // Full GP reset
    // DO NOT reset EP! Keep kernel's EPRST=1 and EP program/scratch intact.
    // EP(NV_PAPU_EPRST) = 0;           // REMOVED — was killing kernel EP!
    for (volatile int d = 0; d < 10000; d++) {}

    // -----------------------------------------------------------------------
    // STEP 2: Write VP structure addresses (while SECTL=0, GP/EP in reset)
    // -----------------------------------------------------------------------
    // DirectSound writes these during init — they ARE writable when SE is stopped.
    APU(NV_PAPU_VPVADDR)   = voice_array_phys;
    APU(NV_PAPU_VPSGEADDR) = sge_table_phys;
    APU(NV_PAPU_VPSSLADDR) = ssl_table_phys;

    // Verify writes stuck
    {
        uint32_t rb_va  = APU(NV_PAPU_VPVADDR);
        uint32_t rb_sge = APU(NV_PAPU_VPSGEADDR);
        uint32_t rb_ssl = APU(NV_PAPU_VPSSLADDR);
        xbox_log("APU: wrote VPVA=%08X rb=%08X %s\n",
                 (unsigned)voice_array_phys, (unsigned)rb_va,
                 (rb_va == voice_array_phys) ? "OK" : "FAILED");
        xbox_log("APU: wrote SGE=%08X rb=%08X %s\n",
                 (unsigned)sge_table_phys, (unsigned)rb_sge,
                 (rb_sge == sge_table_phys) ? "OK" : "FAILED");
        xbox_log("APU: wrote SSL=%08X rb=%08X %s\n",
                 (unsigned)ssl_table_phys, (unsigned)rb_ssl,
                 (rb_ssl == ssl_table_phys) ? "OK" : "FAILED");
    }

    // -----------------------------------------------------------------------
    // STEP 3: Initialize GP/EP with two-phase reset
    // -----------------------------------------------------------------------

    // Init GP with two-phase reset
    if (gp_init() < 0) {
        xbox_log("APU: gp_init FAILED\n");
        goto fail;
    }
    xbox_log("APU: gp_init OK\n");

    // DON'T call ep_init() — keep kernel's EP program and scratch.
    // EP DSP release is DEFERRED to after SECTL=0x0F (same reason as GP:
    // EP must signal idle while SE is running, or SE misses the edge).
    {
        xbox_log("APU: EP kern EPSADDR=%08X EPFADDR=%08X MAXSGE=%X EPRST=%X\n",
                 (unsigned)APU(NV_PAPU_EPSADDR), (unsigned)APU(NV_PAPU_EPFADDR),
                 (unsigned)APU(NV_PAPU_EPSMAXSGE), (unsigned)EP(NV_PAPU_EPRST));
    }
    ep_initialized = 1;  // Mark as initialized for shutdown
    xbox_log("APU: ep_init SKIPPED (using kernel EP, DSP release deferred)\n");

    // Configure FE and voice lists
    APU(NV_PAPU_FENADDR) = notifier_phys;
    APU(NV_PAPU_FETFORCE0) = 0;          // No forced traps
    APU(NV_PAPU_FETFORCE1) = 0;          // Don't trap on idle voices!
    // NOTE: Setting FETFORCE1 bit 15 (SE2FE_IDLE_VOICE) causes the FE to
    // trap itself when ANY voice goes idle. Without an ISR to untrap it,
    // PIO processing stalls permanently. Set to 0 for free-running operation.
    APU(NV_PAPU_ISTS) = 0xFFFFFFFF;
    APU(NV_PAPU_TVL2D)  = 0xFFFF;
    APU(NV_PAPU_TVL3D)  = 0xFFFF;
    APU(NV_PAPU_TVLmp)  = 0xFFFF;

    // Log SECONFIG before writing — does it still have kernel value?
    {
        uint32_t secfg_before = APU(NV_PAPU_SECONFIG);
        xbox_log("APU: SECONFIG before write: %08X\n", (unsigned)secfg_before);
    }
    // Write SECONFIG — try 0x2A first, check if bit 5 sticks
    APU(NV_PAPU_SECONFIG) = 0x2A;
    {
        uint32_t secfg_after = APU(NV_PAPU_SECONFIG);
        xbox_log("APU: SECONFIG after 0x2A write: %08X (bit5=%s)\n",
                 (unsigned)secfg_after,
                 (secfg_after & 0x20) ? "STUCK" : "REJECTED");
    }

    // Initialize submix registers (0x208C-0x20A8) — voice-to-mixbin routing.
    // DirectSound sets all bytes to 0x01 (1 voice per submix group for 31 groups).
    // Without this, the SE may not route voice output to MIXBUF bins.
    for (int i = 0; i < 7; i++) {
        APU(NV_PAPU_SUBMIX0 + i * 4) = 0x01010101;
    }
    APU(NV_PAPU_SUBMIX0 + 7 * 4) = 0x00010101;  // Last reg: only 3 groups
    xbox_log("APU: submix registers initialized (208C-20A8)\n");

    // Preserve kernel FECTL config bits, only clear FEMETHMODE to FREE_RUNNING.
    // DirectSound sets FECTL to 0x0007130F (bits [7:5]=000 = FREE_RUNNING).
    // Match DirectSound's exact low 16 bits: 0x130F.
    APU(NV_PAPU_FECTL) = (kern_fectl & 0xFFFF0000u) | 0x130F;
    APU(NV_PAPU_SECTL) = 0x0F;           // All subsystems + XCNTMODE=running
    xbox_log("APU: FECTL=%08X SECTL=0x0F\n", (unsigned)APU(NV_PAPU_FECTL));

    // Try writing SECONFIG=0x2A now that SECTL=0x0F (SE running).
    APU(NV_PAPU_SECONFIG) = 0x2A;
    {
        uint32_t secfg_post = APU(NV_PAPU_SECONFIG);
        xbox_log("APU: SECONFIG after SECTL: %08X (bit5=%s)\n",
                 (unsigned)secfg_post,
                 (secfg_post & 0x20) ? "STUCK" : "REJECTED");
    }

    // NOW release EP DSP — SE is running, so initial idle edge will be detected
    {
        uint32_t cur_eprst = EP(NV_PAPU_EPRST);
        xbox_log("APU: EP phase B: releasing DSP (EPRST %X->3, SE already running)\n", (unsigned)cur_eprst);
        EP(NV_PAPU_EPRST) = GPRST_GPRST | GPRST_GPDSPRST;  // = 3
        for (volatile int d = 0; d < 1000000; d++) {}
        xbox_log("APU: EP EPRST=%X after release\n", (unsigned)EP(NV_PAPU_EPRST));
    }

    // NOW release GP DSP — SE is running, EP is running, so BSET idle edge will be detected
    gp_release_dsp();

    // Wait for SE to process a few frames with GP active
    for (volatile int d = 0; d < 2000000; d++) {}
    {
        uint32_t fctr = gp_reg[GP_XMEM_FRAME_CTR] & 0xFFFFFF;
        uint32_t secfg = APU(NV_PAPU_SECONFIG);
        xbox_log("APU: post-release: fctr=%u SECONFIG=%08X\n",
                 (unsigned)fctr, (unsigned)secfg);
    }

    // Verify our VPVADDR/VPSGEADDR survived GP/EP init
    {
        uint32_t post_vpva = APU(NV_PAPU_VPVADDR);
        uint32_t post_sge  = APU(NV_PAPU_VPSGEADDR);
        uint32_t post_ssl  = APU(NV_PAPU_VPSSLADDR);
        xbox_log("APU: post-init VPVA=%08X SGE=%08X SSL=%08X\n",
                 (unsigned)post_vpva, (unsigned)post_sge, (unsigned)post_ssl);
        if (post_vpva != voice_array_phys) {
            xbox_log("APU: WARNING VPVADDR changed! was %08X now %08X — re-writing\n",
                     (unsigned)voice_array_phys, (unsigned)post_vpva);
            APU(NV_PAPU_VPVADDR) = voice_array_phys;
        }
        if (post_sge != sge_table_phys) {
            xbox_log("APU: WARNING VPSGEADDR changed! re-writing\n");
            APU(NV_PAPU_VPSGEADDR) = sge_table_phys;
        }
        if (post_ssl != ssl_table_phys) {
            xbox_log("APU: WARNING VPSSLADDR changed! re-writing\n");
            APU(NV_PAPU_VPSSLADDR) = ssl_table_phys;
        }
    }

    // Monitor FECTL — it gets HALTED shortly after init. Log + re-clear.
    {
        uint32_t fectl_imm = APU(NV_PAPU_FECTL);
        xbox_log("APU: FECTL immediate after set: %08X\n", (unsigned)fectl_imm);
    }

    // Let frame engine settle
    for (volatile int d = 0; d < 50000; d++) {}

    {
        uint32_t fectl_post = APU(NV_PAPU_FECTL);
        xbox_log("APU: FECTL after settle: %08X\n", (unsigned)fectl_post);
        if (fectl_post & 0x80) {
            xbox_log("APU: FE HALTED! Trap reason=%X — re-clearing\n",
                     (unsigned)((fectl_post >> 8) & 0xF));
            // Re-assert FREE_RUNNING
            APU(NV_PAPU_FECTL) = (fectl_post & 0xFFFF0000u) | 0x130F;
            xbox_log("APU: FECTL re-cleared: %08X\n", (unsigned)APU(NV_PAPU_FECTL));
        }
    }

    // Verify XGSCNT is ticking
    {
        uint32_t cnt1 = APU(NV_PAPU_XGSCNT);
        for (volatile int d = 0; d < 10000; d++) {}
        uint32_t cnt2 = APU(NV_PAPU_XGSCNT);
        xbox_log("APU: XGSCNT %u->%u (+%d)\n",
                 (unsigned)cnt1, (unsigned)cnt2, (int)(cnt2 - cnt1));
        if (cnt2 == cnt1) {
            xbox_log("APU: WARNING — XGSCNT not ticking!\n");
        }
    }

    // -----------------------------------------------------------------------
    // STEP 3: Zero voice array and SGE (word-by-word, not memset)
    // -----------------------------------------------------------------------
    xbox_log("APU: zeroing voice array (32KB)...\n");
    {
        volatile uint32_t *va = (volatile uint32_t *)voice_array_virt;
        for (int i = 0; i < (MAX_VOICES * VP_VOICE_SIZE) / 4; i++) {
            va[i] = 0;
        }
    }
    xbox_log("APU: zeroing SGE table (32KB)...\n");
    {
        volatile uint32_t *sge = (volatile uint32_t *)sge_table_virt;
        for (int i = 0; i < (SGE_MAX_ENTRIES * SGE_ENTRY_SIZE) / 4; i++) {
            sge[i] = 0;
        }
    }
    xbox_log("APU: zeroed OK\n");

    // Flush cache so VP sees zeros in physical RAM
    __asm__ volatile("wbinvd" ::: "memory");

    // (SECTL/FECTL/etc. already set in STEP 2b above)

    // Verify XGSCNT is ticking (frame engine running)
    {
        uint32_t cnt1 = APU(NV_PAPU_XGSCNT);
        for (volatile int d = 0; d < 10000; d++) {}
        uint32_t cnt2 = APU(NV_PAPU_XGSCNT);
        xbox_log("APU: SECTL=%08X FECTL=%08X XGSCNT %u->%u (+%d)\n",
                 (unsigned)APU(NV_PAPU_SECTL), (unsigned)APU(NV_PAPU_FECTL),
                 (unsigned)cnt1, (unsigned)cnt2, (int)(cnt2 - cnt1));
        xbox_log("APU: GPRST=%X EPRST=%X PIO_FREE=%X\n",
                 (unsigned)GP(NV_PAPU_GPRST), (unsigned)EP(NV_PAPU_EPRST),
                 (unsigned)VP(VP_PIO_FREE));

        if (cnt2 == cnt1) {
            xbox_log("APU: WARNING — XGSCNT not ticking! Frame engine may not be running.\n");
        }
    }

    apu_initialized = 1;
    return 0;

fail:
    XApuShutdown();
    return -1;
}

// ---------------------------------------------------------------------------
// XApuShutdown
// ---------------------------------------------------------------------------
void XApuShutdown(void)
{
    if (!apu_initialized) return;

    // Stop all voices
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voice_playing[i]) {
            XApuVoiceStop(i);
        }
    }

    // Stop frame processing
    APU(NV_PAPU_SECTL) = 0;

    // Shut down GP and EP
    gp_shutdown();
    ep_shutdown();

    // Free all contiguous memory allocations
    for (int i = 0; i < MAX_HW_VOICES; i++) {
        if (pcm_pool_virt[i]) {
            MmFreeContiguousMemory(pcm_pool_virt[i]);
            pcm_pool_virt[i] = NULL;
        }
    }
    if (notifier_virt) {
        MmFreeContiguousMemory(notifier_virt);
        notifier_virt = NULL;
    }
    if (gp_scratch_virt) {
        MmFreeContiguousMemory(gp_scratch_virt);
        gp_scratch_virt = NULL;
    }
    if (gp_scratch_pages_virt) {
        MmFreeContiguousMemory(gp_scratch_pages_virt);
        gp_scratch_pages_virt = NULL;
    }
    if (gp_fifo_virt) {
        MmFreeContiguousMemory(gp_fifo_virt);
        gp_fifo_virt = NULL;
    }
    if (ep_scratch_virt) {
        MmFreeContiguousMemory(ep_scratch_virt);
        ep_scratch_virt = NULL;
    }
    if (ep_scratch_pages_virt) {
        MmFreeContiguousMemory(ep_scratch_pages_virt);
        ep_scratch_pages_virt = NULL;
    }
    if (ep_fifo_virt) {
        MmFreeContiguousMemory(ep_fifo_virt);
        ep_fifo_virt = NULL;
    }
    if (voice_array_virt) {
        MmFreeContiguousMemory(voice_array_virt);
        voice_array_virt = NULL;
    }
    if (sge_table_virt) {
        MmFreeContiguousMemory(sge_table_virt);
        sge_table_virt = NULL;
    }
    if (ssl_table_virt) {
        MmFreeContiguousMemory(ssl_table_virt);
        ssl_table_virt = NULL;
    }

    apu_initialized = 0;
}

// ---------------------------------------------------------------------------
// Voice allocation
// ---------------------------------------------------------------------------
int XApuVoiceAlloc(void)
{
    diag_alloc_calls++;
    if (!apu_initialized) return -1;

    // Full 32KB voice array allocated — all 256 voices available
    // Use voices 0-31 for Duke3D (only needs 8, but 32 gives headroom)
    for (int i = 0; i < 32; i++) {
        if (!voice_allocated[i]) {
            voice_allocated[i] = 1;
            voice_playing[i] = 0;
            voice_pcm_slot[i] = -1;
            diag_alloc_ok++;
            return i;
        }
    }
    return -1;
}

void XApuVoiceFree(int voice)
{
    if (voice < 0 || voice >= MAX_VOICES) return;
    if (!voice_allocated[voice]) return;

    if (voice_playing[voice]) {
        XApuVoiceStop(voice);
    }

    voice_allocated[voice] = 0;
}

// ---------------------------------------------------------------------------
// XApuVoicePlay — configure and start a VP voice
// ---------------------------------------------------------------------------
int XApuVoicePlay(int voice, const void *pcm, unsigned int bytes,
                  unsigned int rate, int bits, int channels, int loop,
                  int left_vol, int right_vol)
{
    diag_play_calls++;
    if (!apu_initialized) {
        diag_play_fail++;
        if (!diag_first_fail_reason) { diag_first_fail_reason = 1; diag_first_fail_voice = voice; }
        return -1;
    }
    if (voice < 0 || voice >= MAX_VOICES) {
        diag_play_fail++;
        if (!diag_first_fail_reason) { diag_first_fail_reason = 2; diag_first_fail_voice = voice; }
        return -1;
    }
    if (!voice_allocated[voice]) {
        diag_play_fail++;
        if (!diag_first_fail_reason) { diag_first_fail_reason = 3; diag_first_fail_voice = voice; }
        return -1;
    }
    if (!pcm || bytes == 0) {
        diag_play_fail++;
        if (!diag_first_fail_reason) {
            diag_first_fail_reason = 4;
            diag_first_fail_voice = voice;
            diag_first_fail_pcm = pcm;
            diag_first_fail_bytes = bytes;
            diag_first_fail_bits = bits;
            diag_first_fail_channels = channels;
        }
        return -1;
    }

    // Stop if already playing
    if (voice_playing[voice]) {
        XApuVoiceStop(voice);
    }

    // Allocate a PCM pool slot
    int slot = pcm_pool_alloc();
    if (slot < 0) {
        diag_play_pool_fail++;
        if (!diag_first_fail_reason) { diag_first_fail_reason = 5; diag_first_fail_voice = voice; }
        return -1;
    }

    // Clamp to slot size
    unsigned int copy_bytes = bytes;
    if (copy_bytes > PCM_POOL_SLOT) copy_bytes = PCM_POOL_SLOT;

    // Copy PCM data to contiguous memory
    memcpy(pcm_pool_virt[slot], pcm, copy_bytes);
    voice_pcm_slot[voice] = slot;

    // Set up SGE entries for this buffer
    int sge_base = sge_setup(pcm_pool_phys[slot], copy_bytes);
    if (sge_base < 0) {
        pcm_pool_free(slot);
        voice_pcm_slot[voice] = -1;
        diag_play_sge_fail++;
        if (!diag_first_fail_reason) { diag_first_fail_reason = 6; diag_first_fail_voice = voice; }
        return -1;
    }
    voice_sge_base[voice] = sge_base;
    voice_sge_count[voice] = (copy_bytes + PAGE_SIZE - 1) / PAGE_SIZE;

    // Calculate sample count
    int sample_bytes = (bits == 16) ? 2 : 1;
    if (channels == 2) sample_bytes *= 2;
    unsigned int num_samples = copy_bytes / sample_bytes;
    if (num_samples == 0) {
        pcm_pool_free(slot);
        voice_pcm_slot[voice] = -1;
        return -1;
    }

    // -----------------------------------------------------------------------
    // Configure VP voice via DIRECT voice array writes (bypasses PIO/FE)
    // -----------------------------------------------------------------------
    // The FE microcontroller doesn't process PIO commands on real hardware
    // (xemu bypasses FE entirely, dispatching PIO directly to fe_method).
    // Instead, write voice configuration directly to the voice array in RAM,
    // flush CPU cache (so VP's DMA sees our writes), then link into TVL2D.
    // -----------------------------------------------------------------------

    // Pointer to this voice's 128-byte entry in the voice array
    volatile uint32_t *v = (volatile uint32_t *)
        ((uint8_t *)voice_array_virt + voice * VP_VOICE_SIZE);

    // Clear the entire voice entry first
    for (int i = 0; i < VP_VOICE_SIZE / 4; i++) v[i] = 0;

    // CFG_FMT (0x04): sample format + routing + headroom
    // [31:30]=container (0=B8, 1=B16), [29:28]=sample size (0=U8, 1=S16)
    // [27]=stereo, [25]=loop, [24]=data_type (0=buffer)
    // [15:13]=HEADROOM (3 bits, 7=max), [9:5]=V7BIN, [4:0]=V6BIN
    // DirectSound always sets HEADROOM=7. Without it, VP may not mix.
    uint32_t fmt = (7u << 13);  // HEADROOM = 7 (critical for VP mixing)
    if (bits == 16) {
        fmt |= (1u << 28);  // S16 sample
        fmt |= (1u << 30);  // B16 container
    }
    if (channels == 2) fmt |= (1u << 27);
    if (loop)          fmt |= (1u << 25);
    // V6BIN/V7BIN: route to unused mixbins (31 = null sink)
    fmt |= (31u << 0);   // V6BIN → bin 31
    fmt |= (31u << 5);   // V7BIN → bin 31
    v[VOICE_CFG_FMT / 4] = fmt;

    // CFG_VBIN (0x00): vol0→mixbin0 (L), vol1→mixbin1 (R), rest→bin31 (null)
    // From xemu: V0BIN [4:0], V1BIN [9:5], V2BIN [14:10], V3BIN [20:16], V4BIN [25:21], V5BIN [30:26]
    // Note: bit 15 is a gap between V2 and V3
    v[VOICE_CFG_VBIN / 4] = (0u)            // vol0 → bin 0
                           | (1u << 5)       // vol1 → bin 1
                           | (31u << 10)     // vol2 → bin 31
                           | (31u << 16)     // vol3 → bin 31
                           | (31u << 21)     // vol4 → bin 31
                           | (31u << 26);    // vol5 → bin 31

    // CUR_PSL_START (0x20): buffer base = sge_base * PAGE_SIZE (byte offset in SGE space)
    v[VOICE_CUR_PSL_START / 4] = (uint32_t)sge_base * PAGE_SIZE;

    // CUR_PSH_SAMPLE (0x24): loop back offset = 0 (loop to start)
    v[VOICE_CUR_PSH_SAMPLE / 4] = 0;

    // PAR_OFFSET (0x58): current buffer offset = 0 (start from beginning)
    v[VOICE_PAR_OFFSET / 4] = 0;

    // PAR_NEXT (0x5C): end buffer offset = num_samples - 1
    v[VOICE_PAR_NEXT / 4] = num_samples - 1;

    // TAR_VOLA (0x60): volumes 0,1 as 12-bit attenuation
    uint32_t left_atten  = vol_to_atten(left_vol);
    uint32_t right_atten = vol_to_atten(right_vol);
    v[VOICE_TAR_VOLA / 4] = pack_vola(left_atten, right_atten);

    // TAR_VOLB/C (0x64, 0x68): muted (unused bins)
    v[VOICE_TAR_VOLB / 4] = (0xFFFu << 4) | (0xFFFu << 20);
    v[VOICE_TAR_VOLC / 4] = (0xFFFu << 4) | (0xFFFu << 20);

    // TAR_PITCH_LINK (0x7C): pitch [31:16] + next voice [15:0]
    // Prepend to TVL2D: our next = current head
    uint16_t pitch = rate_to_pitch(rate);
    uint32_t old_head = APU(NV_PAPU_TVL2D) & 0xFFFF;
    v[VOICE_TAR_PITCH_LINK / 4] = ((uint32_t)pitch << 16) | (old_head & 0xFFFF);

    // PAR_STATE (0x54): ACTIVE + NEW + ENVA=ATTACK(2) for amplitude ramp-up
    // Without ENVA=ATTACK, ea_level=0 and voice is completely muted.
    v[VOICE_PAR_STATE / 4] = VOICE_PAR_ACTIVE | VOICE_PAR_NEW
                            | (2u << VOICE_PAR_EACUR_SHIFT);

    // Flush CPU write-back cache so VP's DMA reads see our data in physical RAM.
    // This covers: PCM data (memcpy above), SGE entries, and voice array writes.
    // Without this, VP reads stale/zero data from RAM.
    __asm__ volatile("wbinvd" ::: "memory");

    // Link voice into TVL2D (MMIO write — immediately visible to VP hardware).
    // Must happen AFTER wbinvd so voice config is in RAM when VP processes it.
    APU(NV_PAPU_TVL2D) = (uint32_t)voice;

    voice_playing[voice] = 1;
    diag_play_ok++;
    return 0;
}

// ---------------------------------------------------------------------------
// XApuVoiceStop
// ---------------------------------------------------------------------------
void XApuVoiceStop(int voice)
{
    diag_stop_calls++;
    if (!apu_initialized) return;
    if (voice < 0 || voice >= MAX_VOICES) return;

    // Clear ACTIVE bit in voice array (direct RAM write, bypasses PIO)
    if (voice < 32 && voice_array_virt) {
        volatile uint32_t *v = (volatile uint32_t *)
            ((uint8_t *)voice_array_virt + voice * VP_VOICE_SIZE);
        v[VOICE_PAR_STATE / 4] = 0;  // Clear all state (ACTIVE=0)
        __asm__ volatile("wbinvd" ::: "memory");
    }

    voice_playing[voice] = 0;

    // Free PCM pool slot
    int slot = voice_pcm_slot[voice];
    if (slot >= 0) {
        pcm_pool_free(slot);
        voice_pcm_slot[voice] = -1;
    }

    // Free SGE entries
    if (voice_sge_base[voice] >= 0) {
        sge_free(voice_sge_base[voice], voice_sge_count[voice]);
        voice_sge_base[voice] = -1;
        voice_sge_count[voice] = 0;
    }
}

// ---------------------------------------------------------------------------
// XApuVoiceSetVolume
// ---------------------------------------------------------------------------
void XApuVoiceSetVolume(int voice, int left_vol, int right_vol)
{
    if (!apu_initialized) return;
    if (voice < 0 || voice >= MAX_VOICES) return;
    if (voice >= 32 || !voice_array_virt) return;

    volatile uint32_t *v = (volatile uint32_t *)
        ((uint8_t *)voice_array_virt + voice * VP_VOICE_SIZE);

    uint32_t left_atten  = vol_to_atten(left_vol);
    uint32_t right_atten = vol_to_atten(right_vol);
    v[VOICE_TAR_VOLA / 4] = pack_vola(left_atten, right_atten);
    __asm__ volatile("wbinvd" ::: "memory");
}

// ---------------------------------------------------------------------------
// XApuVoiceSetPitch
// ---------------------------------------------------------------------------
void XApuVoiceSetPitch(int voice, unsigned int sample_rate)
{
    if (!apu_initialized) return;
    if (voice < 0 || voice >= MAX_VOICES) return;
    if (voice >= 32 || !voice_array_virt) return;

    volatile uint32_t *v = (volatile uint32_t *)
        ((uint8_t *)voice_array_virt + voice * VP_VOICE_SIZE);

    // Preserve next-voice link in lower 16 bits
    uint32_t old_val = v[VOICE_TAR_PITCH_LINK / 4];
    uint16_t next_voice = old_val & 0xFFFF;
    v[VOICE_TAR_PITCH_LINK / 4] = ((uint32_t)rate_to_pitch(sample_rate) << 16)
                                 | next_voice;
    __asm__ volatile("wbinvd" ::: "memory");
}

// ---------------------------------------------------------------------------
// XApuVoiceIsPlaying
// ---------------------------------------------------------------------------
int XApuVoiceIsPlaying(int voice)
{
    if (!apu_initialized) return 0;
    if (voice < 0 || voice >= MAX_VOICES) return 0;
    if (!voice_playing[voice]) return 0;

    // Check the voice's ACTIVE_VOICE bit in PAR_STATE (bit 21)
    if (voice >= 32 || !voice_array_virt) return voice_playing[voice];
    volatile uint32_t *v = (volatile uint32_t *)
        ((uint8_t *)voice_array_virt + voice * VP_VOICE_SIZE);
    uint32_t state = v[VOICE_PAR_STATE / 4];

    if (!(state & VOICE_PAR_ACTIVE)) {
        // Voice finished — clean up
        voice_playing[voice] = 0;
        int slot = voice_pcm_slot[voice];
        if (slot >= 0) {
            pcm_pool_free(slot);
            voice_pcm_slot[voice] = -1;
        }
        if (voice_sge_base[voice] >= 0) {
            sge_free(voice_sge_base[voice], voice_sge_count[voice]);
            voice_sge_base[voice] = -1;
            voice_sge_count[voice] = 0;
        }
        return 0;
    }

    return 1;
}

// ---------------------------------------------------------------------------
// XApuPumpMixbuf — Read GP-copied MIXBUF data from XMEM, produce stereo PCM
// ---------------------------------------------------------------------------
//
// Our custom GP program copies MIXBUF bins 0+1 to XMEM each frame:
//   XMEM[0x0080..0x009F] = bin 0 (left),  32 × 24-bit samples
//   XMEM[0x00A0..0x00BF] = bin 1 (right), 32 × 24-bit samples
// CPU accesses via MMIO at APU_GP_BASE + word_offset * 4.
//
// MIXBUF MMIO (0xFE835000) is NOT CPU-readable on real hardware — always
// returns zero.  XMEM MMIO (0xFE830000+) IS CPU-readable (confirmed).
// ---------------------------------------------------------------------------

// Old MIXBUF MMIO (kept for diagnostic reference, reads zero on real hw)
#define GP_MIXBUF_BASE  (APU_GP_BASE + MIXBUF_OFFSET)

static int pump_log_count = 0;

void XApuPumpMixbuf(short *output, int num_samples)
{
    if (!apu_initialized || !output || num_samples <= 0) {
        if (output && num_samples > 0) {
            memset(output, 0, num_samples * 2 * sizeof(short));
        }
        return;
    }

    // Read from XMEM destination where our GP program copies MIXBUF data
    // Layout: bin 0 (left) at words 0-31, bin 1 (right) at words 32-63
    volatile int32_t *xmem_dst = (volatile int32_t *)&gp_reg[GP_XMEM_DST_BASE];

    // Watchdog: re-clear FECTL if halted, re-assert SECONFIG
    {
        uint32_t fectl = APU(NV_PAPU_FECTL);
        if (fectl & 0xE0) {
            APU(NV_PAPU_FECTL) = (fectl & 0xFFFF0000u) | 0x130F;
        }
        // Continuously re-assert SECONFIG=0x2A — the SE may keep clearing bit 5
        APU(NV_PAPU_SECONFIG) = 0x2A;
    }
    if (pump_log_count < 10) {
        uint32_t frame_ctr = gp_reg[GP_XMEM_FRAME_CTR];
        uint32_t preamble = gp_reg[5] & 0xFFFFFF;
        int32_t s0 = xmem_dst[0], s1 = xmem_dst[1];
        int32_t r0 = xmem_dst[32], r1 = xmem_dst[33];
        uint32_t fectl = APU(NV_PAPU_FECTL);
        uint32_t sectl = APU(NV_PAPU_SECTL);
        uint32_t seconfig = APU(NV_PAPU_SECONFIG);
        xbox_log("APU: PUMP#%d fctr=%u pre=%06X FECTL=%08X SECTL=%08X SECFG=%08X\n",
                 pump_log_count, (unsigned)frame_ctr, (unsigned)preamble,
                 (unsigned)fectl, (unsigned)sectl, (unsigned)seconfig);
        xbox_log("APU: PUMP#%d L:%06X,%06X R:%06X,%06X\n",
                 pump_log_count,
                 (unsigned)(s0 & 0xFFFFFF), (unsigned)(s1 & 0xFFFFFF),
                 (unsigned)(r0 & 0xFFFFFF), (unsigned)(r1 & 0xFFFFFF));
        pump_log_count++;
    }

    // VP produces 32 samples per frame per bin.
    // We read 32 samples at a time from each bin and interleave to stereo.
    int samples_written = 0;

    while (samples_written < num_samples) {
        int chunk = num_samples - samples_written;
        if (chunk > MIXBUF_SAMPLES_PER_BIN) chunk = MIXBUF_SAMPLES_PER_BIN;

        for (int i = 0; i < chunk; i++) {
            // Read from XMEM: bin 0 (left) and bin 1 (right)
            // Values are 24-bit signed stored in low 24 bits of 32-bit words.
            int32_t raw_l = xmem_dst[i];
            int32_t raw_r = xmem_dst[32 + i];

            // Sign-extend 24-bit to 32-bit
            int32_t left  = (raw_l & 0xFFFFFF);
            if (left  & 0x800000) left  |= (int32_t)0xFF000000;
            int32_t right = (raw_r & 0xFFFFFF);
            if (right & 0x800000) right |= (int32_t)0xFF000000;

            // Scale from 24-bit range (±8388607) to 16-bit range (±32767)
            left  >>= 8;
            right >>= 8;

            // Clamp to 16-bit range
            if (left > 32767) left = 32767;
            else if (left < -32768) left = -32768;
            if (right > 32767) right = 32767;
            else if (right < -32768) right = -32768;

            output[(samples_written + i) * 2 + 0] = (short)left;
            output[(samples_written + i) * 2 + 1] = (short)right;
        }

        samples_written += chunk;
    }
}

// ---------------------------------------------------------------------------
// XApuDiagnostic — dump APU state for debugging
// ---------------------------------------------------------------------------
int XApuDiagnostic(char *buf, int bufsize)
{
    if (!buf || bufsize < 1) return 0;

    int n = 0;
    // Safe snprintf helper: clamp n to prevent overflow
    #define DPRINTF(...) do { \
        if (n < bufsize - 1) { \
            int _r = snprintf(buf + n, bufsize - n, __VA_ARGS__); \
            if (_r > 0) n += _r; \
            if (n >= bufsize) n = bufsize - 1; \
        } \
    } while(0)

    DPRINTF("APU: init=%d gp=%d ep=%d (three-phase)\n",
            apu_initialized, gp_initialized, ep_initialized);

    if (!apu_initialized) return n;

    // VP PIO_FREE read — xemu returns 0x80 if VP subregion is reachable
    uint32_t pio_free = VP(VP_PIO_FREE);
    DPRINTF("VP PIO_FREE=%08X (expect 0x80)\n", (unsigned)pio_free);

    // Direct register write test: write pattern to TVL3D, read back, restore
    {
        uint32_t tvl3d_orig = APU(NV_PAPU_TVL3D);
        APU(NV_PAPU_TVL3D) = 0x1234;
        uint32_t tvl3d_test = APU(NV_PAPU_TVL3D);
        APU(NV_PAPU_TVL3D) = tvl3d_orig;
        DPRINTF("REG W/R: TVL3D %04X->%04X (wrote 0x1234)\n",
                (unsigned)(tvl3d_orig & 0xFFFF), (unsigned)(tvl3d_test & 0xFFFF));
    }

    // APU global registers — FECTL is at 0x1100
    uint32_t fectl_cur = APU(NV_PAPU_FECTL);
    if ((fectl_cur & 0xFFFF) != 0x130F) {
        // Reset FECTL to DirectSound-matching value (FREE_RUNNING mode)
        APU(NV_PAPU_FECTL) = (fectl_cur & 0xFFFF0000u) | 0x130F;
    }
    DPRINTF("SECTL=%08X FECTL=%08X TVL2D=%08X\n",
            (unsigned)APU(NV_PAPU_SECTL), (unsigned)fectl_cur,
            (unsigned)APU(NV_PAPU_TVL2D));
    DPRINTF("FECV=%08X FEAV=%08X FF1=%08X ISTS=%08X\n",
            (unsigned)APU(NV_PAPU_FECV), (unsigned)APU(NV_PAPU_FEAV),
            (unsigned)APU(NV_PAPU_FETFORCE1), (unsigned)APU(NV_PAPU_ISTS));

    // GP frame counter — verify GP DSP is running
    // XMEM[4] = frame counter (incremented each SE frame by DSP)
    // XMEM[5] = preamble marker (0xCAFE if DSP executed preamble at boot)
    {
        uint32_t fc1 = gp_reg[GP_XMEM_FRAME_CTR];
        uint32_t preamble = gp_reg[5] & 0xFFFFFF;
        for (volatile int d = 0; d < 10000; d++) {}
        uint32_t fc2 = gp_reg[GP_XMEM_FRAME_CTR];
        DPRINTF("GP FCTR: %u->%u (+%d) PREAMBLE=%06X(%s)\n",
                (unsigned)fc1, (unsigned)fc2, (int)(fc2 - fc1),
                (unsigned)preamble,
                (preamble == 0x00CAFE) ? "DSP_RAN!" : "no_exec");
    }

    // XMEM destination check — our GP program copies MIXBUF here
    volatile int32_t *xmem_dst = (volatile int32_t *)&gp_reg[GP_XMEM_DST_BASE];
    int xmem_nz = 0;
    for (int i = 0; i < GP_XMEM_DST_WORDS; i++) {
        if (xmem_dst[i] != 0) xmem_nz++;
    }
    DPRINTF("XMEM nz=%d/%d\n", xmem_nz, GP_XMEM_DST_WORDS);
    DPRINTF("XMEM L: %06X %06X %06X %06X\n",
            (unsigned)(xmem_dst[0] & 0xFFFFFF), (unsigned)(xmem_dst[1] & 0xFFFFFF),
            (unsigned)(xmem_dst[2] & 0xFFFFFF), (unsigned)(xmem_dst[3] & 0xFFFFFF));
    DPRINTF("XMEM R: %06X %06X %06X %06X\n",
            (unsigned)(xmem_dst[32] & 0xFFFFFF), (unsigned)(xmem_dst[33] & 0xFFFFFF),
            (unsigned)(xmem_dst[34] & 0xFFFFFF), (unsigned)(xmem_dst[35] & 0xFFFFFF));

    // Also check raw MIXBUF MMIO (expected: all zeros on real hw)
    volatile int32_t *mixbuf = (volatile int32_t *)GP_MIXBUF_BASE;
    int mixbuf_nz = 0;
    for (int i = 0; i < 64; i++) {
        if (mixbuf[i] != 0) mixbuf_nz++;
    }
    DPRINTF("MMIO_MIX nz=%d/64\n", mixbuf_nz);

    // Voice tracking
    int active = 0;
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voice_playing[i]) active++;
    }
    DPRINTF("VP act=%d play=%d/%d fail=%d pool=%d\n",
            active, diag_play_ok, diag_play_calls,
            diag_play_fail, diag_play_pool_fail);

    // Check voice array in RAM for first playing voice
    if (active > 0 && voice_array_virt) {
        for (int i = 0; i < 32; i++) {
            if (voice_playing[i]) {
                volatile uint32_t *v = (volatile uint32_t *)
                    ((uint8_t *)voice_array_virt + i * VP_VOICE_SIZE);
                DPRINTF("  v%d: PAR=%08X FMT=%08X PITCH=%08X BUF=%08X EBO=%08X sge=%d\n",
                        i, (unsigned)v[VOICE_PAR_STATE / 4],
                        (unsigned)v[VOICE_CFG_FMT / 4],
                        (unsigned)v[VOICE_TAR_PITCH_LINK / 4],
                        (unsigned)v[VOICE_CUR_PSL_START / 4],
                        (unsigned)v[VOICE_PAR_NEXT / 4],
                        voice_sge_base[i]);
                break;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Clock test + PMEM readback
    // -----------------------------------------------------------------------
    {
        uint32_t cnt1 = APU(NV_PAPU_XGSCNT);
        for (volatile int d = 0; d < 10000; d++) {}
        uint32_t cnt2 = APU(NV_PAPU_XGSCNT);
        uint32_t gprst = GP(NV_PAPU_GPRST);
        uint32_t eprst = EP(NV_PAPU_EPRST);
        DPRINTF("CLK: XGSCNT %u->%u (+%d) GPRST=%X EPRST=%X\n",
                (unsigned)cnt1, (unsigned)cnt2, (int)(cnt2 - cnt1),
                (unsigned)gprst, (unsigned)eprst);

        // Read back GP PMEM to verify our program was loaded
        volatile uint32_t *gp_pmem = (volatile uint32_t *)(APU_GP_BASE + NV_PAPU_GPPMEM);
        DPRINTF("GP PMEM: %06X %06X %06X %06X %06X %06X %06X %06X\n",
                (unsigned)(gp_pmem[0] & 0xFFFFFF), (unsigned)(gp_pmem[1] & 0xFFFFFF),
                (unsigned)(gp_pmem[2] & 0xFFFFFF), (unsigned)(gp_pmem[3] & 0xFFFFFF),
                (unsigned)(gp_pmem[4] & 0xFFFFFF), (unsigned)(gp_pmem[5] & 0xFFFFFF),
                (unsigned)(gp_pmem[6] & 0xFFFFFF), (unsigned)(gp_pmem[7] & 0xFFFFFF));
        DPRINTF("PMEM $008: %06X %06X %06X %06X %06X %06X\n",
                (unsigned)(gp_pmem[0x008] & 0xFFFFFF), (unsigned)(gp_pmem[0x009] & 0xFFFFFF),
                (unsigned)(gp_pmem[0x00A] & 0xFFFFFF), (unsigned)(gp_pmem[0x00B] & 0xFFFFFF),
                (unsigned)(gp_pmem[0x010] & 0xFFFFFF), (unsigned)(gp_pmem[0x011] & 0xFFFFFF));
    }

    // PIO FIFO state
    DPRINTF("PIO FREE=%X\n", (unsigned)VP(VP_PIO_FREE));

    // -----------------------------------------------------------------------
    // Wide register dump — find unknown registers blocking FE
    // -----------------------------------------------------------------------

    // FE area: 0x1000-0x13FF only — SKIP 0x1400-0x14FF (PIO FIFO, reading drains it!)
    DPRINTF("FE_REGS:");
    for (uint32_t off = 0x1000; off <= 0x13FF; off += 4) {
        uint32_t val = APU(off);
        if (val != 0) {
            DPRINTF(" %04X=%08X", (unsigned)off, (unsigned)val);
        }
    }
    // Also read FETFORCE0/1 at 0x1500/0x1504 (safe to read)
    {
        uint32_t ff0 = APU(NV_PAPU_FETFORCE0);
        uint32_t ff1 = APU(NV_PAPU_FETFORCE1);
        if (ff0) DPRINTF(" 1500=%08X", (unsigned)ff0);
        if (ff1) DPRINTF(" 1504=%08X", (unsigned)ff1);
    }
    DPRINTF("\n");

    // SECTL area + VP config: 0x2000-0x20F0
    DPRINTF("SE_REGS:");
    for (uint32_t off = 0x2000; off <= 0x20E0; off += 4) {
        uint32_t val = APU(off);
        if (val != 0) {
            DPRINTF(" %04X=%08X", (unsigned)off, (unsigned)val);
        }
    }
    DPRINTF("\n");

    // Address register readback — verify our writes took effect
    DPRINTF("ADDR: VPVA=%08X VPSGE=%08X SSL=%08X FEN=%08X\n",
            (unsigned)APU(NV_PAPU_VPVADDR), (unsigned)APU(NV_PAPU_VPSGEADDR),
            (unsigned)APU(NV_PAPU_VPSSLADDR), (unsigned)APU(NV_PAPU_FENADDR));
    DPRINTF("ADDR: GPS=%08X GPF=%08X EPS=%08X EPF=%08X\n",
            (unsigned)APU(NV_PAPU_GPSADDR), (unsigned)APU(NV_PAPU_GPFADDR),
            (unsigned)APU(NV_PAPU_EPSADDR), (unsigned)APU(NV_PAPU_EPFADDR));

    // GP FIFO registers (0x3000-0x307C) — never dumped before!
    // These control GP DSP's DMA FIFOs for audio data transfer.
    DPRINTF("GP_FIFO:");
    for (uint32_t off = 0x3000; off <= 0x307C; off += 4) {
        uint32_t val = APU(off);
        if (val != 0) {
            DPRINTF(" %04X=%08X", (unsigned)off, (unsigned)val);
        }
    }
    DPRINTF("\n");
    // EP FIFO registers (0x4000-0x407C)
    DPRINTF("EP_FIFO:");
    for (uint32_t off = 0x4000; off <= 0x407C; off += 4) {
        uint32_t val = APU(off);
        if (val != 0) {
            DPRINTF(" %04X=%08X", (unsigned)off, (unsigned)val);
        }
    }
    DPRINTF("\n");

    // Voice lists: TVL/CVL/NVL for 2D, 3D, MP (0x2054-0x2070)
    // Sample CVL2D twice to detect SE movement
    {
        uint32_t cvl1 = APU(NV_PAPU_CVL2D) & 0xFFFF;
        for (volatile int d = 0; d < 1000; d++) {}
        uint32_t cvl2 = APU(NV_PAPU_CVL2D) & 0xFFFF;
        DPRINTF("VLISTS: TVL2D=%04X CVL2D=%04X->%04X NVL2D=%04X\n",
                (unsigned)(APU(NV_PAPU_TVL2D) & 0xFFFF),
                (unsigned)cvl1, (unsigned)cvl2,
                (unsigned)(APU(NV_PAPU_NVL2D) & 0xFFFF));
    }
    DPRINTF("SECONFIG=%08X MAXVOICES=%08X\n",
            (unsigned)APU(NV_PAPU_SECONFIG),
            (unsigned)APU(NV_PAPU_MAXVOICES));

    // FE decoded method/param — last PIO command the FE actually processed
    DPRINTF("FEDEC: METH=%08X PARAM=%08X\n",
            (unsigned)APU(NV_PAPU_FEDECMETH),
            (unsigned)APU(NV_PAPU_FEDECPARAM));

    // EP PMEM readback
    {
        volatile uint32_t *ep_pmem = (volatile uint32_t *)(APU_EP_BASE + NV_PAPU_EPPMEM);
        DPRINTF("EP PMEM: %06X %06X %06X\n",
                (unsigned)(ep_pmem[0] & 0xFFFFFF), (unsigned)(ep_pmem[1] & 0xFFFFFF),
                (unsigned)(ep_pmem[2] & 0xFFFFFF));
    }

    // GP DSP peripheral area — check if there are status regs
    // Try reading some GP MMIO addresses outside the known XMEM/YMEM/PMEM ranges
    {
        // GP control regs near GPRST (0xFFFC)
        uint32_t gp_fff0 = GP(0xFFF0);
        uint32_t gp_fff4 = GP(0xFFF4);
        uint32_t gp_fff8 = GP(0xFFF8);
        DPRINTF("GP_CTL: FFF0=%08X FFF4=%08X FFF8=%08X FFFC=%08X\n",
                (unsigned)gp_fff0, (unsigned)gp_fff4,
                (unsigned)gp_fff8, (unsigned)GP(NV_PAPU_GPRST));
    }

    // VP PIO registers — scan the low range for unknown config regs
    DPRINTF("VP_LOW:");
    for (uint32_t off = 0x000; off <= 0x020; off += 4) {
        uint32_t val = VP(off);
        if (val != 0 || off == 0x010) {
            DPRINTF(" %03X=%08X", (unsigned)off, (unsigned)val);
        }
    }
    DPRINTF("\n");

    // -----------------------------------------------------------------------
    // Direct voice test — set up voice, check if GP copies MIXBUF to XMEM
    // -----------------------------------------------------------------------
    // Use voice 31 (test voice, not used by game's 8-voice pool)
    {
        // STEP 1: Clear XMEM destination (where GP copies MIXBUF)
        for (int i = 0; i < GP_XMEM_DST_WORDS; i++) {
            gp_reg[GP_XMEM_DST_BASE + i] = 0;
        }
        int pre_nz = 0;
        for (int i = 0; i < GP_XMEM_DST_WORDS; i++) {
            if (gp_reg[GP_XMEM_DST_BASE + i] != 0) pre_nz++;
        }
        DPRINTF("XMEMCLR: pre_nz=%d (should be 0)\n", pre_nz);

        // Save frame counter before test
        uint32_t fc_before = gp_reg[GP_XMEM_FRAME_CTR];

        // STEP 2: Set up voice 31 with test tone
        volatile uint32_t *v = (volatile uint32_t *)
            ((uint8_t *)voice_array_virt + 31 * VP_VOICE_SIZE);

        for (int i = 0; i < VP_VOICE_SIZE / 4; i++) v[i] = 0;

        // S16 mono + HEADROOM=7 + V6BIN/V7BIN=31 (null)
        v[VOICE_CFG_FMT / 4] = (1u << 28) | (1u << 30) | (7u << 13)
                               | (31u << 0) | (31u << 5);
        v[VOICE_CFG_VBIN / 4] = (0u) | (1u << 5)
                               | (31u << 10) | (31u << 16) | (31u << 21) | (31u << 26);
        v[VOICE_CUR_PSL_START / 4] = 0;  // SGE offset 0 (test sine buffer)
        v[VOICE_PAR_OFFSET / 4] = 0;     // CBO = 0
        v[VOICE_PAR_NEXT / 4] = 31;      // EBO = 31 (32 samples)
        v[VOICE_TAR_VOLA / 4] = 0;       // Full volume
        v[VOICE_TAR_PITCH_LINK / 4] = 0x0000FFFF;  // Native rate, no next voice
        v[VOICE_PAR_STATE / 4] = VOICE_PAR_ACTIVE | VOICE_PAR_NEW
                                | (2u << VOICE_PAR_EACUR_SHIFT);

        __asm__ volatile("wbinvd" ::: "memory");

        // Link into TVL2D
        APU(NV_PAPU_TVL2D) = 31;

        DPRINTF("DIRECT: TVL=%04X PAR=%08X FMT=%08X\n",
                (unsigned)(APU(NV_PAPU_TVL2D) & 0xFFFF),
                (unsigned)v[VOICE_PAR_STATE / 4],
                (unsigned)v[VOICE_CFG_FMT / 4]);

        // STEP 3: Wait for multiple VP frames (VP frame rate = 1500Hz)
        for (volatile int d = 0; d < 100000; d++) {}

        // STEP 4: Check XMEM destination — did GP copy MIXBUF data?
        uint32_t fc_after = gp_reg[GP_XMEM_FRAME_CTR];
        int post_nz = 0;
        for (int i = 0; i < GP_XMEM_DST_WORDS; i++) {
            if (gp_reg[GP_XMEM_DST_BASE + i] != 0) post_nz++;
        }
        DPRINTF("DIRECT2: CVL=%04X NVL=%04X PAR=%08X\n",
                (unsigned)(APU(NV_PAPU_CVL2D) & 0xFFFF),
                (unsigned)(APU(NV_PAPU_NVL2D) & 0xFFFF),
                (unsigned)v[VOICE_PAR_STATE / 4]);
        DPRINTF("XMEMCHK: nz=%d/%d fctr=%u->%u (+%d)\n",
                post_nz, GP_XMEM_DST_WORDS,
                (unsigned)fc_before, (unsigned)fc_after,
                (int)(fc_after - fc_before));
        if (post_nz > 0) {
            DPRINTF("XMEM_L: %06X %06X %06X %06X\n",
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 0] & 0xFFFFFF),
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 1] & 0xFFFFFF),
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 2] & 0xFFFFFF),
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 3] & 0xFFFFFF));
            DPRINTF("XMEM_R: %06X %06X %06X %06X\n",
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 32] & 0xFFFFFF),
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 33] & 0xFFFFFF),
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 34] & 0xFFFFFF),
                    (unsigned)(gp_reg[GP_XMEM_DST_BASE + 35] & 0xFFFFFF));
        }

        // STEP 5: Check voice 31's PAR_STATE — did VP modify it?
        DPRINTF("V31_POST: PAR=%08X CBO=%08X ECNT=%08X\n",
                (unsigned)v[VOICE_PAR_STATE / 4],
                (unsigned)v[VOICE_PAR_OFFSET / 4],
                (unsigned)v[VOICE_CUR_ECNT / 4]);

        // Cleanup: remove from list, clear voice
        APU(NV_PAPU_TVL2D) = 0xFFFF;
        for (int i = 0; i < VP_VOICE_SIZE / 4; i++) v[i] = 0;
        __asm__ volatile("wbinvd" ::: "memory");
    }

    #undef DPRINTF
    return n;
}
