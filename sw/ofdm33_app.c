/* ============================================================================
 *  ofdm33_app.c -- bare-metal (standalone, A53 #0) control app for the 33 kHz
 *  OFDM receiver on ZCU104.  Register-level access only (no driver IDs), so it
 *  builds the same with classic Vitis and the Vitis Unified (SDT) flow.
 *
 *  UART 115200 8N1, menu:
 *    h  help                    s  status (counters, CFO, settings)
 *    n  noise floor -> sets rmin (play NOTHING from the Focusrite)
 *    l  level meter (play a 33 kHz TONE: run_tx.m S.mode='tone')
 *    e  FEC on/off: rate-1/2 K=7 convolutional code (run_tx.m S.fec = true), Viterbi on PS
 *    k  test mode on/off: known payload from run_tx.m S.mode='test', count bit errors
 *    m  monitor a test tone (tone_forever.m): CSV line per 0.5 s, any key stops
 *    d  capture raw baseband, dump over UART (for MATLAB read_uart_dump.m)
 *    g  run receiver: print every decoded frame (any key stops)
 *    a  auto level on/off: AGC picks gain_shift and rmin from the measured level
 *    + / -  gain_shift up / down      t  threshold   o  timing offset
 *    b  bb_shift                      c  ADC channel L/R     r  rmin
 *
 *  Rev 3 (12/11/2026):
 *    * auto level ('a', on after reset): while 'g' runs, the raw tap is captured
 *      back to back (4096 samples = 0.512 s). From every packet:
 *        - AGC: >= 8 samples near clipping -> gain_shift - 2 at once; >= 8 samples
 *          above 0.40 of full scale (or a frame crest factor below 2.2) in 2
 *          packets -> - 1; frame present and peak below 0.15 in 2 packets -> + 1
 *          or + 2 (max AGC_GAIN_MAX); silence -> hold. Applied right after a
 *          frame (in the gap).
 *        - rmin: Rn = quietest 128-sample window, Rs = 75th percentile window.
 *          frame present (Rs >= 2 Rn): rmin = max(sqrt(Rn Rs), Rs / 32, min(16 Rn, Rs / 6))
 *          silence                   : rmin = 6 Rs;  never below RMIN_FLOOR.
 *      A manual '+', '-', 'r' or 'n' switches auto level off (old scripts work).
 *    * FEC is detected per frame: the CRC is tried the way 'e' says first, then
 *      the other way. 'e' only sets what is tried first / counted in test mode.
 *    * defaults after reset: gain_shift 2, thr 180, FEC expected, auto level on.
 *    * lines starting with '#' are information (ignored by rx_log.m).
 *
 *  Rev 2 (30/10/2026):
 *    * raw capture (n/l/d) no longer returns a stale packet from the previous
 *      capture: the packet the tap has already started is drained afterwards.
 *    * the frame detector (sync_en) only runs inside 'g'; on entry the bits FIFO
 *      is flushed, so 'g' never prints old frames and the sync buffer can not
 *      overrun while the menu is idle.
 *    * rmin follows gain_shift (x4 / /4 per step) and is clamped to 32 bit.
 *    * level meter: OFDM peak = tone level (MATLAB normalises the peak to S.amp).
 *    * frames with a wrong DMA length are reported as BAD.
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xuartps_hw.h"
#include "sleep.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------- base addresses (from xparameters.h) ---------------------- */
#if defined(XPAR_I2S_RX_BASEADDR)
#  define I2S_BASE   XPAR_I2S_RX_BASEADDR
#elif defined(XPAR_I2S_RX_S_AXI_CTRL_BASEADDR)
#  define I2S_BASE   XPAR_I2S_RX_S_AXI_CTRL_BASEADDR
#else
#  error "I2S base address not found: set I2S_BASE from the Vivado Address Editor"
#endif
#ifndef XPAR_DMA_BITS_BASEADDR
#  error "XPAR_DMA_BITS_BASEADDR missing (cell dma_bits)"
#endif
#ifndef XPAR_DMA_RAW_BASEADDR
#  error "XPAR_DMA_RAW_BASEADDR missing (cell dma_raw)"
#endif
#ifndef XPAR_GPIO_CFG_BASEADDR
#  error "XPAR_GPIO_CFG_BASEADDR missing (cell gpio_cfg)"
#endif
#ifndef XPAR_GPIO_STAT_BASEADDR
#  error "XPAR_GPIO_STAT_BASEADDR missing (cell gpio_stat)"
#endif
#define DMA_BITS   XPAR_DMA_BITS_BASEADDR
#define DMA_RAW    XPAR_DMA_RAW_BASEADDR
#define GPIO_CFG   XPAR_GPIO_CFG_BASEADDR
#define GPIO_STAT  XPAR_GPIO_STAT_BASEADDR
#ifndef STDIN_BASEADDRESS
#  define STDIN_BASEADDRESS 0xFF000000u
#endif

/* ---------------- register offsets ---------------------------------------- */
#define I2S_VER   0x00
#define I2S_CFG   0x04
#define I2S_CTRL  0x08
#define I2S_ISR   0x14
#define I2S_TIM   0x20     /* SCLK divider = MCLK / (2*SCLK)                 */
#define I2S_CH01  0x30

#define S2MM_CR   0x30
#define S2MM_SR   0x34
#define S2MM_DA   0x48
#define S2MM_DAH  0x4C
#define S2MM_LEN  0x58
#define SR_IDLE   (1u << 1)
#define SR_ERR    (7u << 4)
#define SR_IOC    (1u << 12)

#define GPIO_DATA  0x00
#define GPIO2_DATA 0x08

/* ---------------- OFDM frame constants (match RTL / MATLAB) --------------- */
#define FRAME_BYTES   224
#define FRAME_BITS    1792
#define RAW_PKT       4096            /* samples per tap packet (axis_tap_pkt) */
#define RAW_NPKT_MAX  16
#define FSB           8000.0
#define DF            62.5            /* subcarrier spacing Hz                 */
#define FIR_GAIN      0.50            /* DC gain of FIR Compiler (approx.)     */

static u8  bits_buf[FRAME_BYTES * 4]          __attribute__((aligned(64)));
static u32 raw_buf[RAW_PKT * RAW_NPKT_MAX]    __attribute__((aligned(64)));
static u32 drain_buf[RAW_PKT]                 __attribute__((aligned(64)));   /* scratch for dma_drain */
static u8  whiten[FRAME_BYTES];

/* ---------------- settings (gpio_cfg) ------------------------------------- */
static struct {
    unsigned gain;      /* 0..15  */
    unsigned ch;        /* 0 L / 1 R */
    unsigned sync_en;
    unsigned tap_en;
    unsigned thr;       /* 1/256 */
    int      toffs;     /* -128..127 */
    unsigned bbsh;      /* 0..7 */
    u32      rmin;
    unsigned test;      /* 1 = frames carry the known test payload */
    unsigned fec;       /* 1 = FEC coded frames expected (tried first)  */
    unsigned agc;       /* 1 = auto level: gain_shift + rmin follow the signal */
} S = { 2, 0, 1, 0, 180, 0, 2, 4000, 0, 1, 1 };

static void cfg_write(void)
{
    u32 v = (S.gain & 15) | ((S.ch & 1) << 4) | ((S.sync_en & 1) << 5) |
            ((S.tap_en & 1) << 6) | ((S.thr & 255) << 8) |
            (((u32)S.toffs & 255) << 16) | ((S.bbsh & 7) << 24);
    Xil_Out32(GPIO_CFG + GPIO_DATA, v);
    Xil_Out32(GPIO_CFG + GPIO2_DATA, S.rmin);
}

/* ---------------- UART helpers -------------------------------------------- */
static int key_ready(void) { return XUartPs_IsReceiveData(STDIN_BASEADDRESS); }
static char key_get(void)  { return (char)XUartPs_RecvByte(STDIN_BASEADDRESS); }
static int read_int(const char *prompt, int def)
{
    char b[16]; int n = 0; char c;
    printf("%s [%d]: ", prompt, def); fflush(stdout);
    while (1) {
        c = key_get();
        if (c == '\r' || c == '\n') break;
        if ((c == 8 || c == 127) && n) { n--; printf("\b \b"); fflush(stdout); continue; }
        if (n < 15 && ((c >= '0' && c <= '9') || (c == '-' && n == 0))) {
            b[n++] = c; putchar(c); fflush(stdout);
        }
    }
    b[n] = 0; printf("\r\n");
    if (n == 0) return def;
    return (int)strtol(b, NULL, 10);
}

/* ---------------- AXI DMA (simple mode, S2MM) ----------------------------- */
static void dma_reset(UINTPTR base)
{
    Xil_Out32(base + S2MM_CR, 1u << 2);
    for (int i = 0; i < 100000 && (Xil_In32(base + S2MM_CR) & (1u << 2)); i++) ;
    Xil_Out32(base + S2MM_CR, 1u);                       /* run */
}
static void dma_arm(UINTPTR base, void *buf, u32 len)
{
    Xil_DCacheInvalidateRange((UINTPTR)buf, len);
    Xil_Out32(base + S2MM_SR, SR_IOC);                   /* clear IOC */
    Xil_Out32(base + S2MM_DA, (u32)(UINTPTR)buf);
    Xil_Out32(base + S2MM_DAH, (u32)((u64)(UINTPTR)buf >> 32));
    Xil_Out32(base + S2MM_LEN, len);                     /* starts transfer */
}
/* 1 = done, 0 = busy, -1 = error */
static int dma_done(UINTPTR base)
{
    u32 sr = Xil_In32(base + S2MM_SR);
    if (sr & SR_ERR) return -1;
    return (sr & SR_IOC) ? 1 : 0;
}
/* Swallow every packet still queued in the AXIS FIFO in front of a DMA: keep
 * accepting packets until none completes within wait_ms. Returns the count.
 * An AXI DMA reset does NOT empty the AXIS Data FIFO, so this is the only way
 * to get rid of old data. */
static int dma_drain(UINTPTR base, int wait_ms)
{
    int n = 0;
    for (int it = 0; it < 64; it++) {
        dma_reset(base);
        dma_arm(base, drain_buf, sizeof drain_buf);
        int t = 0, r;
        while ((r = dma_done(base)) == 0 && t < wait_ms) { usleep(1000); t++; }
        if (r != 1) break;                                /* nothing (more) queued */
        n++;
    }
    dma_reset(base);                                      /* cancel the pending transfer */
    return n;
}
static u32 rmin_sat(double r)
{
    if (r > 4294967295.0) return 0xFFFFFFFFu;
    if (r < 64.0) return 64u;
    return (u32)r;
}

/* ---------------- payload: de-whitening, CRC, text ------------------------ */
static void make_whiten(void)            /* PN9 x^9+x^5+1, seed 0x155, LSB first */
{
    unsigned s = 0x155;
    memset(whiten, 0, sizeof whiten);
    for (int k = 0; k < FRAME_BITS; k++) {
        unsigned fb = ((s >> 8) ^ (s >> 4)) & 1;
        if (s & 1) whiten[k >> 3] |= (u8)(1u << (k & 7));
        s = ((s << 1) | fb) & 0x1FF;
    }
}
static u16 crc16(const u8 *b, int n)     /* CRC-16/CCITT-FALSE */
{
    u16 c = 0xFFFF;
    for (int k = 0; k < n; k++) {
        c ^= (u16)b[k] << 8;
        for (int i = 0; i < 8; i++) c = (c & 0x8000) ? (u16)((c << 1) ^ 0x1021) : (u16)(c << 1);
    }
    return c;
}

/* ---------------- I2S Receiver init --------------------------------------- */
static void i2s_init(void)
{
    Xil_Out32(I2S_BASE + I2S_CTRL, 0);
    Xil_Out32(I2S_BASE + I2S_CH01, 1);                  /* channel 0/1 */
    Xil_Out32(I2S_BASE + I2S_TIM, 2);                   /* 24.576 MHz / (2*6.144 MHz) */
    Xil_Out32(I2S_BASE + I2S_CTRL, 1);                  /* enable */
    u32 ctrl = Xil_In32(I2S_BASE + I2S_CTRL);
    printf("I2S RX: ver 0x%08lx cfg 0x%08lx ctrl 0x%08lx (bit3 32-bit LRCLK = %lu) tim %lu\r\n",
           (unsigned long)Xil_In32(I2S_BASE + I2S_VER), (unsigned long)Xil_In32(I2S_BASE + I2S_CFG),
           (unsigned long)ctrl, (unsigned long)((ctrl >> 3) & 1),
           (unsigned long)Xil_In32(I2S_BASE + I2S_TIM));
    if (!((ctrl >> 3) & 1))
        printf("  !! 32-bit LRCLK mode OFF -> BCK = 48 fs, PCM1808 (256 fs) will NOT lock.\r\n"
               "     Set i2s_rx 'C_32BIT_LR' = 1 in the block design.\r\n");
}

/* ---------------- raw capture ---------------------------------------------- */
static int raw_capture(int npkt)
{
    if (npkt > RAW_NPKT_MAX) npkt = RAW_NPKT_MAX;
    S.tap_en = 0; cfg_write();
    dma_drain(DMA_RAW, 20);                     /* leftovers of an aborted capture */
    dma_reset(DMA_RAW);
    S.tap_en = 1; cfg_write();
    int r = 1; u32 sr = 0;
    for (int p = 0; p < npkt; p++) {
        dma_arm(DMA_RAW, &raw_buf[p * RAW_PKT], RAW_PKT * 4);
        int t = 0;
        while ((r = dma_done(DMA_RAW)) == 0 && t < 3000) { usleep(1000); t++; }
        if (r != 1) { sr = Xil_In32(DMA_RAW + S2MM_SR); break; }
    }
    S.tap_en = 0; cfg_write();
    /* The tap always finishes a packet it has started (4096 samples = 0.51 s)
     * and it has normally started the next one before tap_en drops. Swallow it
     * now, otherwise the NEXT capture would return these old samples. */
    dma_drain(DMA_RAW, 800);
    if (r != 1) {
        printf("raw capture: DMA %s (SR=0x%08lx) -- is the ADC clock running?\r\n",
               r < 0 ? "error" : "timeout", (unsigned long)sr);
        return 0;
    }
    Xil_DCacheInvalidateRange((UINTPTR)raw_buf, npkt * RAW_PKT * 4);
    return npkt * RAW_PKT;
}
static void raw_stats(int n, double *pwr, double *peak, double *mre, double *mim, double *df)
{
    double p = 0, pk = 0, sr = 0, si = 0, cr = 0, ci = 0;
    double pr = 0, pi = 0;
    for (int k = 0; k < n; k++) {
        double re = (s16)(raw_buf[k] & 0xFFFF), im = (s16)(raw_buf[k] >> 16);
        double m = re * re + im * im;
        p += m; if (m > pk) pk = m; sr += re; si += im;
        if (k) { cr += pr * re + pi * im; ci += pr * im - pi * re; }   /* conj(prev)*cur */
        pr = re; pi = im;
    }
    *pwr = p / n; *peak = sqrt(pk); *mre = sr / n; *mim = si / n;
    *df = atan2(ci, cr) * FSB / (2.0 * M_PI);
}

static void cmd_noise(void)
{
    int n = raw_capture(1); if (!n) return;
    double p, pk, mr, mi, df; raw_stats(n, &p, &pk, &mr, &mi, &df);
    double R = 128.0 * p;
    u32 rmin = rmin_sat(8.0 * R);
    S.rmin = rmin; cfg_write();
    printf("Noise: rms %.1f LSB, peak %.0f, R_noise %.0f -> rmin = %lu (8x)\r\n",
           sqrt(p), pk, R, (unsigned long)rmin);
    if (pk >= 32000) printf("  !! baseband clipping -- lower gain\r\n");
}

static void cmd_level(void)
{
    int n = raw_capture(1); if (!n) return;
    double p, pk, mr, mi, df; raw_stats(n, &p, &pk, &mr, &mi, &df);
    double a = sqrt(p);                                        /* tone -> |bb| const */
    double adc = a / (4096.0 * (1 << S.gain) * FIR_GAIN);      /* fraction of ADC FS */
    printf("Tone: |bb| %.0f LSB (peak %.0f), ADC ~ %.1f dBFS (+/-3 dB), offset %+.2f Hz (%+.0f ppm)\r\n",
           a, pk, 20 * log10(adc + 1e-12), df, df / 33000.0 * 1e6);
    double in16 = adc * (double)(1 << S.gain);                  /* 1.0 = clip after gain_shift */
    printf("  MATLAB scales the OFDM peak to S.amp exactly like the tone: the OFDM peak will\r\n"
           "  sit at this level. Use the SAME S.amp for 'text'. Aim: -10 .. -2 dBFS.\r\n");
    if (in16 > 0.8)
        printf("  -> too hot: clipping risk (ADC or gain_shift). Lower Focusrite level / S.amp or '-'\r\n");
    else if (a < 64.0)
        printf("  -> weak: raise Focusrite level / S.amp%s\r\n", S.gain < 15 ? " or '+' gain_shift" : "");
    else
        printf("  -> level OK\r\n");
    if (a * (double)(1 << S.bbsh) / 4.0 > 8191.0)
        printf("  !! OFDM peaks would saturate the CORDIC input: lower bb_shift ('b')\r\n");
    if (fabs(df) > 30) printf("  !! offset > 30 Hz: tone not at 33 kHz or sample-rate mismatch (Focusrite at 96 kHz?)\r\n");
}

/* Continuous tone monitor: for a test tone from tone_forever.m (fixed or step).
 * Every 4096-sample packet (0.512 s) prints one CSV line
 *     M,<k>,<f_Hz>,<dBFS>,<|bb|>,<peak>,<coh>
 * f_Hz = 33000 + measured offset (so no sync with MATLAB is needed), dBFS =
 * ADC level, coh = tone coherence 0..1 (1 = clean single tone, <0.5 = noise /
 * several tones / sweep in progress). Packets are taken back to back (no
 * drain between them), so there is one line per 0.512 s. Any key stops. */
static void tone_coh(int n, double *coh)
{
    double cr = 0, ci = 0, p = 0, pr = 0, pi = 0;
    for (int k = 0; k < n; k++) {
        double re = (s16)(raw_buf[k] & 0xFFFF), im = (s16)(raw_buf[k] >> 16);
        p += re * re + im * im;
        if (k) { cr += pr * re + pi * im; ci += pr * im - pi * re; }
        pr = re; pi = im;
    }
    *coh = (p > 0) ? sqrt(cr * cr + ci * ci) / p : 0.0;
}

static void cmd_monitor(void)
{
    printf("Monitor tone (any key stops). Columns: M,k,f_Hz,dBFS,|bb|,peak,coh\r\n"
           "Save the terminal log, then run read_tone_log.m in MATLAB.\r\n");
    S.sync_en = 0; S.tap_en = 0; cfg_write();
    dma_drain(DMA_RAW, 800);                    /* old packets out */
    dma_reset(DMA_RAW);
    S.tap_en = 1; cfg_write();
    int k = 0, err = 0;
    while (!key_ready()) {
        dma_arm(DMA_RAW, raw_buf, RAW_PKT * 4);
        int t = 0, r;
        while ((r = dma_done(DMA_RAW)) == 0 && t < 3000 && !key_ready()) { usleep(1000); t++; }
        if (r != 1) { err = (r < 0) ? -1 : (t >= 3000); break; }
        Xil_DCacheInvalidateRange((UINTPTR)raw_buf, RAW_PKT * 4);
        double p, pk, mr, mi, df, coh;
        raw_stats(RAW_PKT, &p, &pk, &mr, &mi, &df);
        tone_coh(RAW_PKT, &coh);
        double a = sqrt(p);
        double adc = a / (4096.0 * (1 << S.gain) * FIR_GAIN);
        printf("M,%d,%.1f,%.1f,%.0f,%.0f,%.2f\r\n", k++, 33000.0 + df,
               20 * log10(adc + 1e-12), a, pk, coh);
    }
    S.tap_en = 0; cfg_write();
    if (key_ready()) (void)key_get();           /* swallow the stop key */
    dma_drain(DMA_RAW, 800);
    if (err) printf("monitor: DMA %s -- is the ADC clock running?\r\n", err < 0 ? "error" : "timeout");
    printf("monitor stopped (%d lines)\r\n", k);
}

static void cmd_dump(void)
{
    int np = read_int("packets of 4096 samples (0.512 s each)", 4);
    if (np < 1) np = 1;
    int n = raw_capture(np); if (!n) return;
    printf("XSCT alternative: mrd -bin -file raw.bin 0x%08lx %d\r\n",
           (unsigned long)(UINTPTR)raw_buf, n);
    printf("RAW_BEGIN %d\r\n", n);
    for (int k = 0; k < n; k++)
        printf("%d,%d\r\n", (s16)(raw_buf[k] & 0xFFFF), (s16)(raw_buf[k] >> 16));
    printf("RAW_END\r\n");
}

static void cmd_status(void)
{
    u32 s1 = Xil_In32(GPIO_STAT + GPIO_DATA), s2 = Xil_In32(GPIO_STAT + GPIO2_DATA);
    s16 ph = (s16)(s1 >> 16);
    printf("sync frames %lu, demap frames %lu, I2S drops %lu, last CFO %+.2f Hz\r\n",
           (unsigned long)(s1 & 0xFFFF), (unsigned long)(s2 & 0xFFFF),
           (unsigned long)(s2 >> 16), ph / 8192.0 * DF);
    printf("gain_shift %u  ch %c  sync_en %u  thr %u (%.2f)  toffs %d  bb_shift %u  rmin %lu  test %u  fec %u  agc %u\r\n",
           S.gain, S.ch ? 'R' : 'L', S.sync_en, S.thr, S.thr / 256.0, S.toffs, S.bbsh,
           (unsigned long)S.rmin, S.test, S.fec, S.agc);
}

/* ---------------- auto level: AGC + rmin from the raw tap ------------------
 * |bb| scale: a passband signal at full scale after gain_shift (16 bit) gives
 * |bb| = 4096 * FIR_GAIN = 2048 (same relation as in cmd_level). */
#define LVL_FS        (4096.0 * FIR_GAIN)   /* |bb| at 16-bit full scale        */
#define LVL_CLIP      (0.88 * LVL_FS)       /* ~1800: samples this high = clipping */
#define LVL_HI        (0.40 * LVL_FS)       /* ~820 : less than 8 dB headroom     */
#define LVL_LO        (0.15 * LVL_FS)       /* ~307 : raise gain below this peak  */
#define LVL_CREST_MIN 2.2                   /* peak/rms of |bb|: OFDM ~2.7-2.9, clipped lower.
                                               Second clipping test, independent of LVL_FS */
#define LVL_PRESENT   2.0                   /* Rs / Rn above this = a frame is in the packet
                                               (noise alone gives ~1.3) */
#define LVL_CREST_HARD 1.6                  /* crest this low = heavily saturated */
#define LVL_CREST_PK  (0.25 * LVL_FS)       /* crest test only above this peak    */
#define LVL_NSAMP     8                     /* samples needed above CLIP / HI     */
#define AGC_GAIN_MAX  6u
#define RMIN_FLOOR    1500.0                /* above the fixed-point noise of the DDC */
#define LVL_WIN       128                   /* = 2L of the frame detector         */
#define LVL_HOP       64
#define LVL_NWIN      ((RAW_PKT - LVL_WIN) / LVL_HOP + 1)
typedef struct {
    double pk;            /* max |bb|                                   */
    int    n_hi, n_clip;  /* samples above LVL_HI / LVL_CLIP            */
    double rn, rs;        /* R of the quietest window / 75th percentile */
    double crest;         /* pk / rms of the frame (rms from rs)        */
    int    present;       /* a frame is in the packet                   */
} lvl_t;
static void level_analyze(const u32 *buf, lvl_t *L)
{
    static double R[LVL_NWIN];
    static double m2[RAW_PKT];
    double pk2 = 0; int nh = 0, nc = 0;
    const double hi2 = LVL_HI * LVL_HI, cl2 = LVL_CLIP * LVL_CLIP;
    for (int k = 0; k < RAW_PKT; k++) {
        double re = (s16)(buf[k] & 0xFFFF), im = (s16)(buf[k] >> 16);
        double m = re * re + im * im;
        m2[k] = m;
        if (m > pk2) pk2 = m;
        if (m > hi2) nh++;
        if (m >= cl2) nc++;
    }
    for (int w = 0; w < LVL_NWIN; w++) {
        double a = 0;
        for (int k = 0; k < LVL_WIN; k++) a += m2[w * LVL_HOP + k];
        int j = w;                                   /* insertion sort, ascending */
        while (j > 0 && R[j - 1] > a) { R[j] = R[j - 1]; j--; }
        R[j] = a;
    }
    L->pk = sqrt(pk2); L->n_hi = nh; L->n_clip = nc;
    L->rn = R[1] < 1.0 ? 1.0 : R[1];
    L->rs = R[(3 * LVL_NWIN) / 4];
    L->present = (L->rs >= LVL_PRESENT * L->rn);
    L->crest = (L->rs > 0) ? L->pk / sqrt(L->rs / LVL_WIN) : 0.0;
}
static u32 rmin_target(const lvl_t *L)
{
    double t;
    if (L->present) {
        /* weak signal: stay 16x above the quietest window (false triggers were seen
         * at 8..10x), but never closer than 1/6 of the frame level */
        double lo = 16.0 * L->rn < L->rs / 6.0 ? 16.0 * L->rn : L->rs / 6.0;
        t = sqrt(L->rn * L->rs);
        if (t < L->rs / 32.0) t = L->rs / 32.0;
        if (t < lo) t = lo;
    } else t = 6.0 * L->rs;
    if (t < RMIN_FLOOR) t = RMIN_FLOOR;
    return rmin_sat(t);
}
/* AGC decision for one packet. c[0..2] = run-length counters (high, low, quiet)
 * kept by the caller. Returns the gain_shift change: -4 .. +2.
 *   clipping (>= 8 samples near full scale)        : -2 at once (-4 when crest < 1.6)
 *   >= 8 samples above LVL_HI in 2 packets         : -1
 *   crest factor of the frame below LVL_CREST_MIN  : -1 (same counter)
 *   frame present, peak < LVL_LO in 2 packets      : +1 (+2 when peak < LVL_LO / 4)
 *   silence: gain is held (the next transmission is probably as loud as the last).
 *            Only when the gain is below AGC_GAIN_IDLE it goes up 1 step per
 *            AGC_QUIET_PKTS silent packets (10 s), so that a signal that is below
 *            1 LSB at a very low gain is found again. */
#define AGC_QUIET_PKTS 20
#define AGC_GAIN_IDLE  2u
static int agc_step(const lvl_t *L, unsigned gain, int *c)
{
    if (L->n_clip >= LVL_NSAMP) {                 /* crest < 1.6: far into saturation -> 4 steps */
        int d = (L->present && L->crest < LVL_CREST_HARD) ? 4 : 2;
        c[0] = c[1] = c[2] = 0;
        if (d > (int)gain) d = (int)gain;
        return -d;
    }
    if (L->n_hi >= LVL_NSAMP ||
        (L->present && L->pk > LVL_CREST_PK && L->crest < LVL_CREST_MIN)) {
        c[1] = c[2] = 0;
        if (++c[0] >= 2) { c[0] = 0; return gain >= 1 ? -1 : 0; }
        return 0;
    }
    c[0] = 0;
    if (!L->present) {
        c[1] = 0;
        if (++c[2] >= AGC_QUIET_PKTS) { c[2] = 0; return gain < AGC_GAIN_IDLE ? 1 : 0; }
        return 0;
    }
    c[2] = 0;
    if (L->pk < LVL_LO) {
        if (++c[1] >= 2) {
            c[1] = 0;
            if (gain >= AGC_GAIN_MAX) return 0;
            return (L->pk < LVL_LO / 4.0 && gain + 2 <= AGC_GAIN_MAX) ? 2 : 1;
        }
        return 0;
    }
    c[1] = 0;
    return 0;
}

/* ---------------- measurement helpers ------------------------------------ */
static unsigned long now_ms(void)          /* ARMv8 generic timer, no BSP header needed */
{
    unsigned long long cnt, frq;
    __asm__ volatile ("mrs %0, cntpct_el0" : "=r" (cnt));
    __asm__ volatile ("mrs %0, cntfrq_el0" : "=r" (frq));
    if (frq < 1000ULL) frq = 100000000ULL;   /* CNTFRQ not set: assume 100 MHz */
    return (unsigned long)(cnt / (frq / 1000ULL));
}
/* Known test payload (MATLAB ofdm_test_payloads.m, run_tx.m S.mode = 'test'):
 *   byte 0 = seq, 1 = 0xE5, 2 = 0, 3..221 = (29*k + 7) mod 256, 222..223 = CRC.
 * Bit errors are counted over bytes 1..221 = 1768 bits (seq and CRC excluded). */
#define TEST_MARK  0xE5
#define TEST_BITS  1768
static int popcount8(u8 v) { int n = 0; while (v) { n += v & 1; v >>= 1; } return n; }
static int test_biterr(const u8 *p)
{
    int e = popcount8((u8)(p[1] ^ TEST_MARK)) + popcount8(p[2]);
    for (int k = 3; k < 222; k++) e += popcount8((u8)(p[k] ^ (u8)(29 * k + 7)));
    return e;
}

/* FEC_BEGIN ------------------------------------------------------------------
 * Rate-1/2 K=7 convolutional code, generators 133/171 (octal), tail-terminated.
 * Must match MATLAB ofdm_fec_frame.m / ofdm_fec_unframe.m:
 *   111 info bytes -> 888 bits (LSB first) + 2 pad + 6 tail = 896 bits
 *   encoder: reg = (b << 6) | state, out = parity(reg & 0133), parity(reg & 0171),
 *            state = reg >> 1
 *   block interleaver: coded bit k -> transmitted position (k % 112) * 16 + k / 112
 * Hard-decision Viterbi; the final path metric = number of channel bits corrected. */
#define FEC_INFO_BYTES  111
#define FEC_TEXT_BYTES  106
#define FEC_STEPS       896
#define FEC_CODED_BITS  1792
#define FEC_TEST_BITS   864          /* test payload bytes 1..108 */
static u8 fec_o0[128], fec_o1[128];
static unsigned long long fec_surv[FEC_STEPS];
static u8 fec_rx[FEC_CODED_BITS], fec_c[FEC_CODED_BITS], fec_u[FEC_STEPS];
static int fec_par7(unsigned v) { v &= 0x7Fu; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1; return (int)(v & 1u); }
static void fec_init(void)
{
    for (unsigned r = 0; r < 128; r++) { fec_o0[r] = (u8)fec_par7(r & 0x5Bu); fec_o1[r] = (u8)fec_par7(r & 0x79u); }
}
/* in: 224 de-whitened bytes; out: 111 info bytes; returns channel bits corrected */
static int fec_decode(const u8 *in, u8 *out)
{
    unsigned pm[64], npm[64];
    for (int j = 0; j < FEC_CODED_BITS; j++) fec_rx[j] = (u8)((in[j >> 3] >> (j & 7)) & 1u);
    for (int k = 0; k < FEC_CODED_BITS; k++) fec_c[k] = fec_rx[(k % 112) * 16 + k / 112];
    for (int st = 0; st < 64; st++) pm[st] = st ? 100000u : 0u;
    for (int t = 0; t < FEC_STEPS; t++) {
        u8 a = fec_c[2 * t], b = fec_c[2 * t + 1];
        unsigned long long sv = 0;
        for (unsigned ns = 0; ns < 64; ns++) {
            unsigned bin = ns >> 5, s0 = (ns & 31u) << 1, s1 = s0 | 1u;
            unsigned r0 = (bin << 6) | s0, r1 = (bin << 6) | s1;
            unsigned m0 = pm[s0] + (unsigned)(fec_o0[r0] ^ a) + (unsigned)(fec_o1[r0] ^ b);
            unsigned m1 = pm[s1] + (unsigned)(fec_o0[r1] ^ a) + (unsigned)(fec_o1[r1] ^ b);
            if (m1 < m0) { npm[ns] = m1; sv |= 1ULL << ns; } else npm[ns] = m0;
        }
        fec_surv[t] = sv;
        for (int st = 0; st < 64; st++) pm[st] = npm[st];
    }
    unsigned st = 0;
    for (int t = FEC_STEPS - 1; t >= 0; t--) {
        fec_u[t] = (u8)(st >> 5);
        st = ((st & 31u) << 1) | (unsigned)((fec_surv[t] >> st) & 1ULL);
    }
    for (int i = 0; i < FEC_INFO_BYTES; i++) {
        u8 v = 0;
        for (int bt = 0; bt < 8; bt++) v |= (u8)(fec_u[8 * i + bt] << bt);
        out[i] = v;
    }
    return (int)pm[0];
}
static int fec_test_biterr(const u8 *q)
{
    int e = popcount8((u8)(q[1] ^ TEST_MARK)) + popcount8(q[2]);
    for (int k = 3; k < 109; k++) e += popcount8((u8)(q[k] ^ (u8)(29 * k + 7)));
    return e;
}
/* FEC_END -------------------------------------------------------------------- */

/* Decode one de-whitened 224-byte frame. The CRC is tried the way 'expect_fec'
 * says first, then the other way. p (224 bytes) receives the frame (FEC frame:
 * 111 info bytes, rest 0). *is_fec = type that passed the CRC, or the expected
 * type when neither passed. *nraw = channel bits corrected (FEC) or -1.
 * Returns 1 when a CRC matched. */
static int frame_decode(const u8 *d, int expect_fec, u8 *p, int *is_fec, int *nraw)
{
    u8 q[FEC_INFO_BYTES];
    int nr = -1;
    int ok_u = (crc16(d, 222) == (u16)((d[222] << 8) | d[223]));
    if (!expect_fec && ok_u) { memcpy(p, d, FRAME_BYTES); *is_fec = 0; *nraw = -1; return 1; }
    nr = fec_decode(d, q);
    int ok_f = (crc16(q, 109) == (u16)((q[109] << 8) | q[110]));
    if (ok_f || (expect_fec && !ok_u)) {
        memset(p, 0, FRAME_BYTES); memcpy(p, q, FEC_INFO_BYTES);
        *is_fec = 1; *nraw = nr; return ok_f;
    }
    memcpy(p, d, FRAME_BYTES); *is_fec = 0; *nraw = -1;
    return ok_u;
}

static void cmd_run(void)
{
    static u8 seg[256][220]; static u8 seglen[256]; static u8 have[256];
    int nok = 0, nbad = 0, nfr_msg = 0;
    unsigned long berr = 0, bits = 0, rawerr = 0, rawbits = 0, t0 = now_ms();
    /* level monitor / auto level state */
    lvl_t L;
    int tap_ok = 1, tap_err = 0, skip = 0, pend = 0, npk = 0, nchg = 0;
    int agc_cnt[3] = { 0, 0, 0 };
    unsigned long pend_t = 0, clip_t = 0, ok_t = 0;     /* ok_t: time of the last good frame */
    double pend_pk = 0, pend_cr = 0, pk_max = 0, pk_last = 0, cr_last = 0;

    memset(have, 0, sizeof have);
    S.sync_en = 0; S.tap_en = 0; cfg_write();
    int stale = dma_drain(DMA_BITS, 20);                /* frames left in fifo_bits */
    if (stale) printf("(discarded %d old frame(s) left in the FIFO)\r\n", stale);
    dma_reset(DMA_BITS);
    dma_drain(DMA_RAW, 20);                             /* leftovers of an aborted capture */
    dma_reset(DMA_RAW);
    u32 st1 = Xil_In32(GPIO_STAT + GPIO_DATA), st2 = Xil_In32(GPIO_STAT + GPIO2_DATA);
    printf("Receiving... (any key = stop)  rmin %lu  gain_shift %u%s\r\n",
           (unsigned long)S.rmin, S.gain, S.test ? "  TEST MODE" : "");
    if (S.fec) printf("FEC ON (rate 1/2, Viterbi)\r\n");
    if (S.agc) printf("# auto level ON: gain_shift and rmin follow the signal ('a' = off)\r\n");
    if (S.test) printf("F,t_ms,seq,crc_ok,cfo_hz,bit_err,bytes,raw_err\r\n");
    dma_arm(DMA_BITS, bits_buf, FRAME_BYTES);
    dma_arm(DMA_RAW, raw_buf, RAW_PKT * 4);             /* arm before enabling the tap */
    S.sync_en = 1; S.tap_en = 1; cfg_write();
    while (!key_ready()) {
        /* ---- raw tap: one packet per 0.512 s -> level, AGC, rmin ---------- */
        if (tap_ok) {
            int rr = dma_done(DMA_RAW);
            if (rr < 0) {
                dma_reset(DMA_RAW);
                if (++tap_err > 3) { tap_ok = 0; printf("# level monitor off (raw DMA errors)\r\n"); }
                else dma_arm(DMA_RAW, raw_buf, RAW_PKT * 4);
            } else if (rr == 1) {
                Xil_DCacheInvalidateRange((UINTPTR)raw_buf, RAW_PKT * 4);
                level_analyze(raw_buf, &L);
                dma_arm(DMA_RAW, raw_buf, RAW_PKT * 4);
                if (skip) skip--;                        /* packet spans a gain change */
                else if (!pend) {
                    unsigned long tn = now_ms();
                    npk++; pk_last = L.pk; if (L.pk > pk_max) pk_max = L.pk;
                    if (L.present) cr_last = L.crest;
                    if (L.n_clip >= LVL_NSAMP && (!S.agc || S.gain == 0) && (!clip_t || tn - clip_t > 5000)) {
                        clip_t = tn ? tn : 1;
                        printf("# CLIP: input saturates (peak %.0f, limit %.0f): %s\r\n", L.pk, LVL_FS,
                               S.agc ? "lower S.amp or the amplifier" : "lower gain_shift ('-') or S.amp, or 'a'");
                    }
                    if (S.agc) {
                        int dg = agc_step(&L, S.gain, agc_cnt);
                        if (dg) { pend = dg; pend_t = tn; pend_pk = L.pk; pend_cr = L.crest; }
                        else if (L.present || !ok_t || tn - ok_t > 2000) {
                            /* "silence" is only believed when no good frame came for 2 s:
                             * at low SNR a packet with a frame can look like silence */
                            u32 t = rmin_target(&L);
                            /* up only on a 2x change, down already on 1.3x: a too high rmin loses frames */
                            if ((double)t > 2.0 * (double)S.rmin || 1.3 * (double)t < (double)S.rmin) {
                                printf("# rmin %lu to %lu (%s)\r\n", (unsigned long)S.rmin, (unsigned long)t,
                                       L.present ? "frame" : "silence");
                                S.rmin = t; cfg_write();
                            }
                        }
                    }
                }
            }
        }
        int r = dma_done(DMA_BITS);
        if (r < 0) { printf("DMA error SR=0x%08lx\r\n", (unsigned long)Xil_In32(DMA_BITS + S2MM_SR));
                     dma_reset(DMA_BITS); dma_arm(DMA_BITS, bits_buf, FRAME_BYTES); continue; }
        if (r == 0) {
            if (!pend || now_ms() - pend_t <= 600) continue;
            /* a gain change is pending and no frame came for 0.6 s: change now */
        } else {
            Xil_DCacheInvalidateRange((UINTPTR)bits_buf, FRAME_BYTES);
            u32 got = Xil_In32(DMA_BITS + S2MM_LEN);
            u8 d[FRAME_BYTES], p[FRAME_BYTES];
            for (int k = 0; k < FRAME_BYTES; k++) d[k] = bits_buf[k] ^ whiten[k];
            dma_arm(DMA_BITS, bits_buf, FRAME_BYTES);       /* re-arm early */

            int is_fec = 0, nraw = -1;
            int ok = frame_decode(d, (int)S.fec, p, &is_fec, &nraw) && (got == FRAME_BYTES);
            int maxlen = is_fec ? FEC_TEXT_BYTES : 219;
            s16 ph = (s16)(Xil_In32(GPIO_STAT + GPIO_DATA) >> 16);
            int len = p[2] > maxlen ? maxlen : p[2];
            char txt[220]; memcpy(txt, &p[3], len); txt[len] = 0;
            if (ok) nok++; else nbad++;
            unsigned long tf = now_ms() - t0;
            if (ok) ok_t = tf + t0 ? tf + t0 : 1;
            if (S.test) {                                    /* machine-readable line */
                int e = is_fec ? fec_test_biterr(p) : test_biterr(p);
                berr += (unsigned long)e; bits += is_fec ? FEC_TEST_BITS : TEST_BITS;
                if (nraw >= 0) { rawerr += (unsigned long)nraw; rawbits += FEC_CODED_BITS; }
                printf("F,%lu,%u,%d,%+.2f,%d,%lu,%d\r\n", tf, p[0], ok, ph / 8192.0 * DF, e,
                       (unsigned long)got, nraw >= 0 ? nraw : e);
            } else {
                printf("[%s] t %7lu ms seq %3u/%u len %3d CFO %+6.2f Hz  bytes %lu  \"%s\"",
                       ok ? "OK " : "BAD", tf, p[0], p[1], len, ph / 8192.0 * DF, (unsigned long)got,
                       ok ? txt : "");
                if (nraw >= 0) printf("  fixed %d", nraw);
                if (ok && is_fec != (int)S.fec) printf("  (%s frame)", is_fec ? "FEC" : "uncoded");
                printf("\r\n");
                if (ok && p[1] > 0 && p[0] < p[1]) {        /* assemble multi-frame text */
                    if (!have[p[0]]) { have[p[0]] = 1; memcpy(seg[p[0]], &p[3], len); seglen[p[0]] = (u8)len; nfr_msg++; }
                    if (nfr_msg >= p[1]) {
                        printf(">>> message complete (%d frames): ", p[1]);
                        for (int f = 0; f < p[1]; f++) { seg[f][seglen[f]] = 0; printf("%s", (char *)seg[f]); }
                        printf("\r\n");
                        memset(have, 0, sizeof have); nfr_msg = 0;
                    }
                }
            }
        }
        if (pend) {                                         /* a frame just ended (gap), or timeout */
            int g = (int)S.gain + pend;
            if (g < 0) g = 0;
            if (g > (int)AGC_GAIN_MAX) g = (int)AGC_GAIN_MAX;
            double f = (g >= (int)S.gain) ? (double)(1u << (2 * (g - (int)S.gain)))
                                          : 1.0 / (double)(1u << (2 * ((int)S.gain - g)));
            printf("# AGC gain_shift %u to %d (peak %.0f of %.0f, crest %.1f)\r\n", S.gain, g, pend_pk, LVL_FS, pend_cr);
            f *= (double)S.rmin;
            S.rmin = rmin_sat(f < RMIN_FLOOR ? RMIN_FLOOR : f);
            S.gain = (unsigned)g; cfg_write();
            pend = 0; skip = 1; agc_cnt[0] = agc_cnt[1] = agc_cnt[2] = 0; nchg++;
        }
    }
    key_get();
    S.sync_en = 0; S.tap_en = 0; cfg_write();           /* detector idle in the menu */
    u32 e1 = Xil_In32(GPIO_STAT + GPIO_DATA), e2 = Xil_In32(GPIO_STAT + GPIO2_DATA);
    printf("stopped: %d OK, %d CRC errors | detected %u, demapped %u, I2S drops %u\r\n",
           nok, nbad, (unsigned)((e1 - st1) & 0xFFFF), (unsigned)((e2 - st2) & 0xFFFF),
           (unsigned)(((e2 >> 16) - (st2 >> 16)) & 0xFFFF));
    if (S.test && bits)
        printf("test: %lu bit errors / %lu bits, BER %.2e, frames with errors %d of %d\r\n",
               berr, bits, (double)berr / (double)bits, nbad, nok + nbad);
    if (rawbits)
        printf("fec: %lu channel bit errors corrected / %lu coded bits, channel BER %.2e\r\n",
               rawerr, rawbits, (double)rawerr / (double)rawbits);
    if (npk)
        printf("level: last peak %.0f, max %.0f of %.0f, crest %.1f | gain_shift %u, rmin %lu, auto %s, %d gain change(s)\r\n",
               pk_last, pk_max, LVL_FS, cr_last, S.gain, (unsigned long)S.rmin, S.agc ? "on" : "off", nchg);
    dma_drain(DMA_RAW, 800);                            /* the tap finishes its last packet */
}

static void help(void)
{
    printf("\r\n--- OFDM 33 kHz RX (ZCU104) ---\r\n"
           " h help   s status   n noise->rmin   l level (33 kHz tone)   d raw dump\r\n"
           " m monitor tone (level + frequency, one CSV line per 0.5 s)\r\n"
           " k test mode (known payload, bit-error count in 'g')   e FEC on/off\r\n"
           " a auto level (AGC: gain_shift + rmin follow the signal; '+' '-' 'r' 'n' switch it off)\r\n"
           " g receive frames    +/- gain_shift   t thr   r rmin   o toffs   b bb_shift   c L/R\r\n"
           " (frame detector runs only inside 'g'; counters are printed when 'g' stops)\r\n");
}

/* a manual gain / rmin command means the user (or a script) wants fixed settings */
static void agc_manual(const char *cmd)
{
    if (!S.agc) return;
    S.agc = 0;
    printf("auto level off (manual '%s'); 'a' turns it on again\r\n", cmd);
}

int main(void)
{
    Xil_DCacheEnable();
    printf("\r\n\r\nOFDM33 RX app rev 3.2 (" __DATE__ " " __TIME__ ")  (I2S 0x%08lx, DMA bits 0x%08lx raw 0x%08lx, GPIO 0x%08lx/0x%08lx)\r\n",
           (unsigned long)I2S_BASE, (unsigned long)DMA_BITS, (unsigned long)DMA_RAW,
           (unsigned long)GPIO_CFG, (unsigned long)GPIO_STAT);
    make_whiten();
    fec_init();
    S.sync_en = 0; cfg_write();
    i2s_init();
    dma_reset(DMA_BITS); dma_reset(DMA_RAW);
    usleep(100000);
    S.sync_en = 0; cfg_write();          /* frame detector runs only inside 'g' */
    help(); cmd_status();
    while (1) {
        printf("> "); fflush(stdout);
        char c = key_get(); printf("%c\r\n", c);
        switch (c) {
            case 'h': help(); break;
            case 's': cmd_status(); break;
            case 'n': cmd_noise(); agc_manual("n"); break;
            case 'l': cmd_level(); break;
            case 'd': cmd_dump(); break;
            case 'm': cmd_monitor(); break;
            case 'g': cmd_run(); break;
            case '+': if (S.gain < 15) { S.gain++; S.rmin = rmin_sat(4.0 * S.rmin); }
                      cfg_write(); printf("gain_shift %u  (rmin -> %lu)\r\n", S.gain, (unsigned long)S.rmin);
                      agc_manual("+"); break;
            case '-': if (S.gain > 0)  { S.gain--; S.rmin = rmin_sat(S.rmin / 4.0); }
                      cfg_write(); printf("gain_shift %u  (rmin -> %lu)\r\n", S.gain, (unsigned long)S.rmin);
                      agc_manual("-"); break;
            case 'a': S.agc ^= 1u;
                      printf("auto level %s (AGC + rmin from the measured level while 'g' runs)\r\n", S.agc ? "ON" : "off"); break;
            case 't': S.thr = (unsigned)read_int("thr (1/256, e.g. 150)", S.thr) & 255; cfg_write(); break;
            case 'o': S.toffs = read_int("toffs (samples, signed)", S.toffs); cfg_write(); break;
            case 'b': S.bbsh = (unsigned)read_int("bb_shift 0..7", S.bbsh) & 7; cfg_write(); break;
            case 'c': S.ch ^= 1; cfg_write(); printf("ADC channel %c\r\n", S.ch ? 'R' : 'L'); break;
            case 'k': S.test ^= 1u;
                      printf("test mode %s (MATLAB run_tx.m: S.mode = 'test')\r\n", S.test ? "ON" : "off"); break;
            case 'e': S.fec ^= 1u;
                      printf("FEC %s (MATLAB run_tx.m: S.fec = true)\r\n", S.fec ? "ON" : "off"); break;
            case 'r': S.rmin = (u32)read_int("rmin", (int)S.rmin); cfg_write(); agc_manual("r"); break;
            default: break;
        }
    }
    return 0;
}
