"""Reference model (Python) for the 33 kHz OFDM link -- same parameters as the RTL.
   Used to generate FIR coefficients, the LTF table and test vectors."""
import numpy as np, scipy.signal as ss

FS      = 96000          # PCM1808 sample rate
FC      = 33000          # carrier (BII-7511H resonance)
DEC     = 12             # decimation -> 8 kHz complex baseband
FSB     = FS // DEC
N, CP   = 128, 32        # FFT size / cyclic prefix (4 ms)
NSYM    = 16             # data symbols per frame
ACTIVE  = [f for f in range(-32, 33) if f != 0]          # 64 subcarriers, 62.5 Hz spacing
PILOTS  = [-28, -20, -12, -4, 4, 12, 20, 28]
DATA    = [f for f in ACTIVE if f not in PILOTS]          # 56 data subcarriers
BITS_PER_FRAME = NSYM * len(DATA) * 2                     # 1792 bits = 224 bytes

def pn9(n, seed=0x1FF):
    s, out = seed, []
    for _ in range(n):
        b = ((s >> 8) ^ (s >> 4)) & 1          # x^9 + x^5 + 1
        out.append(s & 1)
        s = ((s << 1) | b) & 0x1FF
    return np.array(out)

PN      = pn9(64 + 64)
LTF     = 1 - 2 * PN[:64]                                  # BPSK, index j: f = ACTIVE[j]
SC_QPSK = PN[64:128]                                      # bits for SC preamble (even bins)

WHITEN = pn9(BITS_PER_FRAME, 0x155)                      # data whitening (MATLAB ofdm_params.whiten)

def crc16_ccitt(b):
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8): c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c

def payload_to_bits(p):        # 224 bytes -> LSB-first bits -> whitened
    return np.array([(x >> i) & 1 for x in p for i in range(8)]) ^ WHITEN

def fir_coeffs(ntaps=72):
    return ss.remez(ntaps, [0, 2200, 6000, FS / 2], [1, 0], weight=[1, 4], fs=FS)

def fbin(f):                    # frequency index -> FFT bin
    return f % N

def ofdm_symbol(X):             # X: dict f -> value
    v = np.zeros(N, complex)
    for f, x in X.items(): v[fbin(f)] = x
    t = np.fft.ifft(v) * np.sqrt(N)
    return np.concatenate([t[-CP:], t])

def qpsk_map(b):                # Gray: b0 -> I sign, b1 -> Q sign
    return ((1 - 2 * b[0::2]) + 1j * (1 - 2 * b[1::2])) / np.sqrt(2)

def build_frame(bits):
    assert len(bits) == BITS_PER_FRAME
    even = [f for f in ACTIVE if f % 2 == 0]
    sc = qpsk_map(SC_QPSK[:2 * len(even)]) * np.sqrt(2)
    syms = [ofdm_symbol(dict(zip(even, sc))), ofdm_symbol(dict(zip(ACTIVE, LTF)))]
    q = qpsk_map(bits).reshape(NSYM, len(DATA))
    for s in range(NSYM):
        X = dict(zip(DATA, q[s])); X.update({p: 1.0 for p in PILOTS})
        syms.append(ofdm_symbol(X))
    return np.concatenate(syms)          # complex baseband @ 8 kHz

def upconvert(bb, fc=FC, fs=FS):
    up = ss.resample_poly(bb, DEC, 1)
    n = np.arange(len(up))
    return np.real(up * np.exp(2j * np.pi * fc / fs * n))

def bits_from_words(words):     # LSB-first packing, as the RTL does
    return np.array([(w >> i) & 1 for w in words for i in range(32)])

if __name__ == "__main__":
    h = fir_coeffs()
    print("FIR taps:", len(h))
    print("LTF bits (j=0..63, f=-32..32):", "".join(map(str, PN[:64])))
    lb = sum(int(b) << j for j, b in enumerate(PN[:64]))
    print("LTF_BITS = 64'h%016X" % lb)
    print("CoefficientVector:", ",".join("%.10f" % c for c in h))
