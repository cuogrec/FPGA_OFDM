function [y, info] = ofdm_tx_signal(payloads, P, S)
% OFDM_TX_SIGNAL  Ghep cac khung thanh tin hieu am thanh 96 kHz.
%   payloads : [nfr x 224] uint8
%   S        : thiet lap tu run_tx.m (fc, amp, gap_ms, preroll_ms, repeat, papr_clip_db)
nfr = size(payloads, 1);
gap = zeros(round(S.gap_ms * 1e-3 * P.fsb), 1);
bb  = zeros(round(S.preroll_ms * 1e-3 * P.fsb), 1);
for r = 1:S.repeat
    for f = 1:nfr
        x = ofdm_build_frame(ofdm_payload_to_bits(payloads(f,:), P), P);
        if isfinite(S.papr_clip_db)                     % cat dinh baseband (tuy chon)
            a = sqrt(mean(abs(x).^2)) * 10^(S.papr_clip_db/20);
            k = abs(x) > a;  x(k) = a * x(k) ./ abs(x(k));
        end
        bb = [bb; x; gap]; %#ok<AGROW>
    end
end
% duoi im lang: card am thanh/driver con giu mot doan dem khi dung phat, neu duoi qua ngan
% thi phan cuoi khung cuoi bi cat (khung cuoi BAD, fixed ~120). Mac dinh 600 ms.
if isfield(S, 'postroll_ms'), tail_ms = S.postroll_ms; else, tail_ms = 600; end
bb = [bb; zeros(round(tail_ms * 1e-3 * P.fsb), 1)];
y  = ofdm_upconvert(bb, P, S.fc);
y  = S.amp * y / max(abs(y));                           % chuan hoa dinh
info.nframes   = nfr * S.repeat;
info.duration  = numel(y) / P.fs;
info.frame_ms  = (P.N + P.CP) * P.frame_syms / P.fsb * 1e3;
info.papr_db   = 20*log10(max(abs(y)) / sqrt(mean(y(y ~= 0).^2)));
end
