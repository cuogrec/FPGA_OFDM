import sys, numpy as np
sys.path.insert(0,'../model'); import ofdm_ref as M
ref = np.load('bits_ref.npy').reshape(-1, M.BITS_PER_FRAME)
L = [l.split() for l in open('words_out.txt')]
w = [int(a,16) for a,_ in L]; last = [int(b) for _,b in L]
nfw = M.BITS_PER_FRAME // 32
print("words", len(w), "tlast idx", [i for i,x in enumerate(last) if x])
for f in range(len(w)//nfw):
    b = M.bits_from_words(w[f*nfw:(f+1)*nfw])
    errs = [np.sum(b != r) for r in ref]
    print("frame %d: bit errors vs ref frames %s" % (f, errs))
