"""MATLAB TX WAV -> (kenh xo nuoc + nhieu) -> PCM1808 24 bit -> mo hinh DDC bit-true -> bb_in.hex
   python3 wav_to_bb.py ofdm_tx.wav [snr_db] [ppm]"""
import sys, numpy as np, scipy.signal as ss
from scipy.io import wavfile
sys.path.insert(0, '../model'); import ofdm_ref as M
fs, w = wavfile.read(sys.argv[1]); assert fs == 96000
tx = w.astype(float) / np.max(np.abs(w))
snr_db = float(sys.argv[2]) if len(sys.argv) > 2 else 30
ppm    = float(sys.argv[3]) if len(sys.argv) > 3 else 50
tx = ss.resample(tx, int(round(len(tx) / (1 + ppm * 1e-6))))
h = np.zeros(200); h[0] = 1; h[37] = 0.5; h[90] = -0.3; h[150] = 0.15
rx = np.convolve(tx, h)[:len(tx)]
p = np.mean(rx[np.abs(rx) > 1e-6] ** 2)
rx += np.random.default_rng(7).normal(0, np.sqrt(p / 10**(snr_db/10) * (fs/2) / 2000), len(rx))
adc = np.round(rx / np.max(np.abs(rx)) * 0.5 * 2**23).astype(np.int64)
x16 = adc >> 8; n = np.arange(len(x16)); ph = (n * 43008) & 0xFFFF
cs = np.round(np.cos(2*np.pi*ph/65536)*32767).astype(np.int64)
sn = np.round(np.sin(2*np.pi*ph/65536)*32767).astype(np.int64)
mr = (x16*cs) >> 17; mi = (x16*sn) >> 17
hq = np.round(M.fir_coeffs()/np.max(np.abs(M.fir_coeffs()))*32767).astype(np.int64)
sh = int(np.ceil(np.log2(np.sum(np.abs(hq))*32768))) + 1 - 16
yr = np.convolve(mr, hq)[::12] >> sh; yi = np.convolve(mi, hq)[::12] >> sh
with open('bb_in.hex', 'w') as fh:
    for a, b in zip(yr, yi): fh.write("%08x\n" % (((int(b)&0xFFFF) << 16) | (int(a)&0xFFFF)))
print("baseband samples", len(yr), "peak", np.abs(yr).max())
