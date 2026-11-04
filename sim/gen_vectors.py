"""Generate RX test vectors: TX frame -> 33 kHz passband -> channel -> PCM1808 -> DDC model
   (DDS PINC=43008, cmpy 16-bit, FIR decim 12) -> 8 kHz 16-bit IQ  (bb_in.hex)."""
import sys, numpy as np
sys.path.insert(0, '../model'); import ofdm_ref as M
rng = np.random.default_rng(int(sys.argv[1]) if len(sys.argv) > 1 else 1)
snr_db = float(sys.argv[2]) if len(sys.argv) > 2 else 25
ppm    = float(sys.argv[3]) if len(sys.argv) > 3 else 40     # TX/RX clock mismatch
nfr = 3
frames, bits_all = [], []
for f in range(nfr):
    b = rng.integers(0, 2, M.BITS_PER_FRAME); bits_all.append(b)
    frames.append(np.concatenate([M.build_frame(b), np.zeros(400)]))   # 50 ms gap
bb = np.concatenate([np.zeros(777)] + frames)
tx = M.upconvert(bb)
# clock mismatch: resample by (1+ppm)
import scipy.signal as ss
tx = ss.resample(tx, int(round(len(tx) / (1 + ppm * 1e-6))))     # band-limited time scaling
# bucket channel: direct + reflections (ms-scale)
h = np.zeros(200); h[0] = 1; h[37] = 0.5; h[90] = -0.3; h[150] = 0.15   # up to 1.56 ms
rx = np.convolve(tx, h)[:len(tx)]
p = np.mean(rx[np.abs(rx) > 0] ** 2)
rx = rx + rng.normal(0, np.sqrt(p / 10 ** (snr_db / 10) * (M.FS / 2) / 2000), len(rx))  # SNR in 4 kHz band
adc = np.round(rx / np.max(np.abs(rx)) * 0.5 * 2**23).astype(np.int64)        # 24-bit, -6 dBFS
# i2s_ch_sel (gain 0): top 16 bits
x16 = adc >> 8
n = np.arange(len(x16))
ph = (n * 43008) & 0xFFFF
cs = np.round(np.cos(2*np.pi*ph/65536) * 32767).astype(np.int64)
sn = np.round(np.sin(2*np.pi*ph/65536) * 32767).astype(np.int64)
mr = (x16 * cs) >> 17; mi = (x16 * sn) >> 17                 # cmpy out 16 of 33 bits
hq = np.round(M.fir_coeffs() / np.max(np.abs(M.fir_coeffs())) * 32767).astype(np.int64)
yr = np.convolve(mr, hq)[::12]; yi = np.convolve(mi, hq)[::12]
sh = int(np.ceil(np.log2(np.sum(np.abs(hq)) * 32768))) + 1 - 16
yr >>= sh; yi >>= sh
print("baseband peak", np.max(np.abs(yr)), np.max(np.abs(yi)), "samples", len(yr))
with open('bb_in.hex', 'w') as fh:
    for a, b in zip(yr, yi): fh.write("%08x\n" % (((int(b) & 0xFFFF) << 16) | (int(a) & 0xFFFF)))
np.save('bits_ref.npy', np.concatenate(bits_all))
