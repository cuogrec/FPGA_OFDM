function [info, crc_ok, nraw] = ofdm_fec_unframe(coded)
% OFDM_FEC_UNFRAME  224 byte nhan duoc (da bo lam trang) -> 111 byte tin (Viterbi cung).
%   info   : 111 byte (uint8)
%   crc_ok : CRC-16 cua byte 0..108 so voi byte 109..110
%   nraw   : so bit kenh da duoc sua (khoang cach Hamming toi tu ma tot nhat)
coded = double(coded(:));
assert(numel(coded) == 224, 'Can dung 224 byte');
x = reshape(mod(floor(coded * 2.^(-(0:7))), 2).', [], 1);  % 1792 bit theo thu tu phat
k = (0:1791).';
c = x(mod(k, 112) * 16 + floor(k / 112) + 1);               % giai xen ke

% bang dau ra cho moi reg = b*64 + state (0..127)
o0 = zeros(128, 1);  o1 = zeros(128, 1);
for r = 0:127
    o0(r + 1) = mod(sum(bitget(bitand(r, 91), 1:7)), 2);
    o1(r + 1) = mod(sum(bitget(bitand(r, 121), 1:7)), 2);
end
ns  = (0:63).';
bin = floor(ns / 32);                     % bit vao tao ra trang thai ns
s0  = mod(ns, 32) * 2;  s1 = s0 + 1;      % hai trang thai truoc
r0  = bin * 64 + s0;    r1 = bin * 64 + s1;
pm  = [0; 1e6 * ones(63, 1)];
surv = false(64, 896);
for t = 1:896
    a = c(2*t - 1);  b = c(2*t);
    m0 = pm(s0 + 1) + (o0(r0 + 1) ~= a) + (o1(r0 + 1) ~= b);
    m1 = pm(s1 + 1) + (o0(r1 + 1) ~= a) + (o1(r1 + 1) ~= b);
    pick = m1 < m0;
    pm = m0;  pm(pick) = m1(pick);
    surv(:, t) = pick;
end
nraw = pm(1);
u = zeros(896, 1);  s = 0;
for t = 896:-1:1
    u(t) = floor(s / 32);
    s = mod(s, 32) * 2 + surv(s + 1, t);
end
info = uint8(reshape(u(1:888), 8, []).' * (2.^(0:7)).').';
crc_ok = crc16_ccitt(double(info(1:109)).') == double(info(110)) * 256 + double(info(111));
end
