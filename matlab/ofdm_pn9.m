function b = ofdm_pn9(n, seed)
% OFDM_PN9  LFSR x^9 + x^5 + 1, xuat bit LSB truoc (giong RTL / ofdm_ref.py).
s = seed; b = zeros(n,1);
for k = 1:n
    fb   = bitand(bitxor(bitshift(s,-8), bitshift(s,-4)), 1);
    b(k) = bitand(s, 1);
    s    = bitand(bitor(bitshift(s,1), fb), 511);
end
end
