function pl = ofdm_test_payloads(n, P, seq0)
% OFDM_TEST_PAYLOADS  n khung thu nghiem co noi dung biet truoc, de dem bit loi.
%   pl = ofdm_test_payloads(n, P)      -> [n x 224] uint8, seq = 0..n-1
%   Byte 0 = seq, 1 = 0xE5 (dau hieu), 2 = 0, 3..221 = mod(29*k + 7, 256),
%   222..223 = CRC-16. PHAI khop voi test_biterr() trong sw/ofdm33_app.c
%   (lenh 'k' tren board). Bit loi dem tren byte 1..221 = 1768 bit.
if nargin < 3, seq0 = 0; end
assert(n >= 1 && n <= 256, 'n phai trong 1..256 (seq 1 byte)');
k  = 3:221;                                   % chi so byte (bat dau tu 0)
pl = zeros(n, P.bytes_per_frame, 'uint8');
for f = 1:n
    b = zeros(1, P.bytes_per_frame - 2);
    b(1) = mod(seq0 + f - 1, 256);  b(2) = 229;  b(3) = 0;
    b(4:222) = mod(29*k + 7, 256);
    c = crc16_ccitt(b);
    pl(f, :) = uint8([b, floor(c/256), mod(c, 256)]);
end
end
