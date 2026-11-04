import sys, numpy as np
sys.path.insert(0,'../model'); import ofdm_ref as M
ref = np.load('bits_ref.npy').reshape(-1, M.BITS_PER_FRAME)
w = [int(l.split()[0],16) for l in open('words_out.txt')]
nfw=56
for f in range(len(w)//nfw):
    e = (M.bits_from_words(w[f*nfw:(f+1)*nfw]) != ref[f]).reshape(M.NSYM, 56, 2).any(2)
    print("per symbol:", e.sum(1)); print("per subc :", "".join(str(min(9,x)) for x in e.sum(0)))
