function pl = ofdm_text_to_payloads(txt, P, seq0)
% OFDM_TEXT_TO_PAYLOADS  Chia van ban (UTF-8) thanh cac payload 224 byte.
%   pl : [nfr x 224] uint8 ; seq0 : so thu tu khung dau tien (mac dinh 0)
if nargin < 3, seq0 = 0; end
u   = double(unicode2native(txt, 'UTF-8'));
nfr = max(1, ceil(numel(u) / P.text_bytes));
assert(nfr <= 255, 'Van ban qua dai (toi da 255 khung)');
pl  = zeros(nfr, P.bytes_per_frame, 'uint8');
for f = 1:nfr
    seg = u((f-1)*P.text_bytes+1 : min(f*P.text_bytes, numel(u)));
    b = zeros(1, P.bytes_per_frame - 2);
    b(1) = mod(seq0 + f - 1, 256);  b(2) = nfr;  b(3) = numel(seg);
    b(4:3+numel(seg)) = seg;
    c = crc16_ccitt(b);
    pl(f,:) = uint8([b, floor(c/256), mod(c,256)]);
end
end
