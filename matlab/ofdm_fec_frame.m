function coded = ofdm_fec_frame(info)
% OFDM_FEC_FRAME  111 byte tin -> 224 byte da ma hoa (FEC) cho mot khung OFDM.
%   1) 111 byte -> 888 bit (LSB truoc) + 2 bit dem + 6 bit duoi 0 = 896 bit
%   2) Ma chap K = 7, ti le 1/2, da thuc 133/171 (bat phan) -> 1792 bit
%      thanh ghi: reg = b*64 + state, ra [parity(reg & 133o), parity(reg & 171o)],
%      state = floor(reg/2)   (PHAI khop voi fec_viterbi() trong sw/ofdm33_app.c)
%   3) Xen ke khoi 16 x 112: bit ma k -> vi tri mod(k,112)*16 + floor(k/112)
%      (bit ma lien nhau cach nhau 8 song mang; cung song mang -> cach nhau 112 bit ma)
%   4) Dong goi lai 224 byte (LSB truoc). Lam trang PN9 van do ofdm_payload_to_bits lam.
info = double(info(:));
assert(numel(info) == 111, 'Can dung 111 byte tin');
u = reshape(mod(floor(info * 2.^(-(0:7))), 2).', [], 1);   % 888 bit, LSB truoc
u = [u; 0; 0; zeros(6, 1)];                                 % 896 bit
c = zeros(1792, 1);  st = 0;
for t = 1:896
    reg = u(t) * 64 + st;
    c(2*t - 1) = par(bitand(reg, 91));                      % 133 bat phan
    c(2*t)     = par(bitand(reg, 121));                     % 171 bat phan
    st = floor(reg / 2);
end
k  = (0:1791).';
x  = zeros(1792, 1);
x(mod(k, 112) * 16 + floor(k / 112) + 1) = c;               % xen ke
coded = uint8(reshape(x, 8, []).' * (2.^(0:7)).').';        % 224 byte
end

function p = par(v)
p = mod(sum(bitget(v, 1:7)), 2);
end
