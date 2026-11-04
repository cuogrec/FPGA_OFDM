/* ============================================================================
 *  ofdm33_app.c -- bare-metal (standalone, A53 #0) control app for the 33 kHz
 *  OFDM receiver on ZCU104.  Register-level access only (no driver IDs), so it
 *  builds the same with classic Vitis and the Vitis Unified (SDT) flow.
 *
 *  UART 115200 8N1, menu:
 *    h  help                    s  status (counters, CFO, settings)
 *    n  noise floor -> sets rmin (play NOTHING from the Focusrite)
 *    l  level meter (play a 33 kHz TONE: run_tx.m S.mode='tone')
 *    d  capture raw baseband, dump over UART (for MATLAB read_uart_dump.m)
 *    g  run receiver: print every decoded frame (any key stops)
 *    + / -  gain_shift up / down      t  threshold   o  timing offset
 *    b  bb_shift                      c  ADC channel L/R
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
} S = { 0, 0, 1, 0, 150, 0, 2, 1000 };

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
    printf("gain_shift %u  ch %c  sync_en %u  thr %u (%.2f)  toffs %d  bb_shift %u  rmin %lu\r\n",
           S.gain, S.ch ? 'R' : 'L', S.sync_en, S.thr, S.thr / 256.0, S.toffs, S.bbsh,
           (unsigned long)S.rmin);
}

static void cmd_run(void)
{
    static u8 seg[256][220]; static u8 seglen[256]; static u8 have[256];
    int nok = 0, nbad = 0, nfr_msg = 0;
    memset(have, 0, sizeof have);
    S.sync_en = 0; cfg_write();
    int stale = dma_drain(DMA_BITS, 20);                /* frames left in fifo_bits */
    if (stale) printf("(discarded %d old frame(s) left in the FIFO)\r\n", stale);
    dma_reset(DMA_BITS);
    u32 st1 = Xil_In32(GPIO_STAT + GPIO_DATA), st2 = Xil_In32(GPIO_STAT + GPIO2_DATA);
    S.sync_en = 1; cfg_write();
    printf("Receiving... (any key = stop)  rmin %lu  gain_shift %u\r\n",
           (unsigned long)S.rmin, S.gain);
    dma_arm(DMA_BITS, bits_buf, FRAME_BYTES);
    while (!key_ready()) {
        int r = dma_done(DMA_BITS);
        if (r == 0) continue;
        if (r < 0) { printf("DMA error SR=0x%08lx\r\n", (unsigned long)Xil_In32(DMA_BITS + S2MM_SR));
                     dma_reset(DMA_BITS); dma_arm(DMA_BITS, bits_buf, FRAME_BYTES); continue; }
        Xil_DCacheInvalidateRange((UINTPTR)bits_buf, FRAME_BYTES);
        u32 got = Xil_In32(DMA_BITS + S2MM_LEN);
        u8 p[FRAME_BYTES];
        for (int k = 0; k < FRAME_BYTES; k++) p[k] = bits_buf[k] ^ whiten[k];
        dma_arm(DMA_BITS, bits_buf, FRAME_BYTES);       /* re-arm early */

        int ok = (got == FRAME_BYTES) && (crc16(p, 222) == (u16)((p[222] << 8) | p[223]));
        s16 ph = (s16)(Xil_In32(GPIO_STAT + GPIO_DATA) >> 16);
        int len = p[2] > 219 ? 219 : p[2];
        char txt[220]; memcpy(txt, &p[3], len); txt[len] = 0;
        if (ok) nok++; else nbad++;
        printf("[%s] seq %3u/%u len %3d CFO %+6.2f Hz  bytes %lu  \"%s\"\r\n",
               ok ? "OK " : "BAD", p[0], p[1], len, ph / 8192.0 * DF, (unsigned long)got,
               ok ? txt : "");
        if (ok && p[1] > 0 && p[0] < p[1]) {            /* assemble multi-frame text */
            if (!have[p[0]]) { have[p[0]] = 1; memcpy(seg[p[0]], &p[3], len); seglen[p[0]] = (u8)len; nfr_msg++; }
            if (nfr_msg >= p[1]) {
                printf(">>> message complete (%d frames): ", p[1]);
                for (int f = 0; f < p[1]; f++) { seg[f][seglen[f]] = 0; printf("%s", (char *)seg[f]); }
                printf("\r\n");
                memset(have, 0, sizeof have); nfr_msg = 0;
            }
        }
    }
    key_get();
    S.sync_en = 0; cfg_write();                         /* detector idle in the menu */
    u32 e1 = Xil_In32(GPIO_STAT + GPIO_DATA), e2 = Xil_In32(GPIO_STAT + GPIO2_DATA);
    printf("stopped: %d OK, %d CRC errors | detected %u, demapped %u, I2S drops %u\r\n",
           nok, nbad, (unsigned)((e1 - st1) & 0xFFFF), (unsigned)((e2 - st2) & 0xFFFF),
           (unsigned)(((e2 >> 16) - (st2 >> 16)) & 0xFFFF));
}

static void help(void)
{
    printf("\r\n--- OFDM 33 kHz RX (ZCU104) ---\r\n"
           " h help   s status   n noise->rmin   l level (33 kHz tone)   d raw dump\r\n"
           " g receive frames    +/- gain_shift   t thr   o toffs   b bb_shift   c L/R\r\n"
           " (frame detector runs only inside 'g'; counters are printed when 'g' stops)\r\n");
}

int main(void)
{
    Xil_DCacheEnable();
    printf("\r\n\r\nOFDM33 RX app  (I2S 0x%08lx, DMA bits 0x%08lx raw 0x%08lx, GPIO 0x%08lx/0x%08lx)\r\n",
           (unsigned long)I2S_BASE, (unsigned long)DMA_BITS, (unsigned long)DMA_RAW,
           (unsigned long)GPIO_CFG, (unsigned long)GPIO_STAT);
    make_whiten();
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
            case 'n': cmd_noise(); break;
            case 'l': cmd_level(); break;
            case 'd': cmd_dump(); break;
            case 'g': cmd_run(); break;
            case '+': if (S.gain < 15) { S.gain++; S.rmin = rmin_sat(4.0 * S.rmin); }
                      cfg_write(); printf("gain_shift %u  (rmin -> %lu)\r\n", S.gain, (unsigned long)S.rmin); break;
            case '-': if (S.gain > 0)  { S.gain--; S.rmin = rmin_sat(S.rmin / 4.0); }
                      cfg_write(); printf("gain_shift %u  (rmin -> %lu)\r\n", S.gain, (unsigned long)S.rmin); break;
            case 't': S.thr = (unsigned)read_int("thr (1/256, e.g. 150)", S.thr) & 255; cfg_write(); break;
            case 'o': S.toffs = read_int("toffs (samples, signed)", S.toffs); cfg_write(); break;
            case 'b': S.bbsh = (unsigned)read_int("bb_shift 0..7", S.bbsh) & 7; cfg_write(); break;
            case 'c': S.ch ^= 1; cfg_write(); printf("ADC channel %c\r\n", S.ch ? 'R' : 'L'); break;
            case 'r': S.rmin = (u32)read_int("rmin", (int)S.rmin); cfg_write(); break;
            default: break;
        }
    }
    return 0;
}