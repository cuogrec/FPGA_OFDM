function [coded, info] = ofdm_fec_payloads(kind, arg, P)
% OFDM_FEC_PAYLOADS  Tao khung co FEC (224 byte da ma hoa moi khung).
%   [coded, info] = ofdm_fec_payloads('text', 'Xin chao', P)  -> van ban, 106 byte/khung
%   [coded, info] = ofdm_fec_payloads('test', 50, P)          -> 50 khung thu (dem bit loi)
%   info  : [nfr x 111] byte tin truoc ma hoa
%     van ban: [seq, nfr, len, 106 byte text, CRC-16 (big-endian)]
%     khung thu: [seq, 0xE5, 0, mod(29k+7,256) voi k = 3..108, CRC-16]
%   coded : [nfr x 224] -> dua thang vao ofdm_tx_signal nhu payload thuong.
%   App tren board: bat lenh 'e' (FEC) va 'k' (neu la khung thu).
TXT = 106;
switch kind
    case 'text'
        u   = double(unicode2native(arg, 'UTF-8'));
        nfr = max(1, ceil(numel(u) / TXT));
        assert(nfr <= 255, 'Van ban qua dai');
        info = zeros(nfr, 111);
        for f = 1:nfr
            seg = u((f-1)*TXT + 1 : min(f*TXT, numel(u)));
            b = zeros(1, 109);
            b(1) = mod(f - 1, 256);  b(2) = nfr;  b(3) = numel(seg);
            b(4:3 + numel(seg)) = seg;
            info(f, :) = [b, crc_bytes(b)];
        end
    case 'test'
        nfr = arg;  assert(nfr >= 1 && nfr <= 256);
        info = zeros(nfr, 111);
        k = 3:108;
        for f = 1:nfr
            b = zeros(1, 109);
            b(1) = mod(f - 1, 256);  b(2) = 229;  b(3) = 0;
            b(4:109) = mod(29*k + 7, 256);
            info(f, :) = [b, crc_bytes(b)];
        end
    otherwise
        error('kind = text | test');
end
info  = uint8(info);
coded = zeros(size(info, 1), 224, 'uint8');
for f = 1:size(info, 1)
    coded(f, :) = ofdm_fec_frame(info(f, :));
end
end

function c = crc_bytes(b)
v = crc16_ccitt(b);
c = [floor(v / 256), mod(v, 256)];
end
