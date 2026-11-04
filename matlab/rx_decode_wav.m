%% rx_decode_wav.m -- giai mot file WAV (file TX, hoac ban ghi tu hydrophone/Focusrite input)
%  hoac dump baseband tu dma_raw cua FPGA. Dung de so sanh voi ket qua FPGA.
clear; P = ofdm_params();
file = 'ofdm_tx.wav';          % WAV 96 kHz (kenh 1 duoc dung)
opt.thr  = 150/256;            % nguong metric (giong gpio_cfg thr)
opt.plot = true;

[y, fs] = audioread(file);
assert(fs == P.fs, 'WAV phai la 96 kHz');
res = ofdm_rx_ref(y, P, 'passband', opt);
for k = 1:numel(res)
    fprintf('khung %2d @%6d: seq %3d CRC %d  CFO %+5.2f Hz  SNR %4.1f dB  "%s"\n', k, res(k).start, ...
        res(k).seq, res(k).crc_ok, res(k).cfo_hz, res(k).snr_db, res(k).text);
end
fprintf('Van ban: "%s"\n', ofdm_join_text(res));

% --- dump baseband cua dma_raw (int16 re,im xen ke, 8 kHz):
% fid = fopen('raw.bin','rb'); v = fread(fid, inf, 'int16'); fclose(fid);
% res = ofdm_rx_ref(complex(v(1:2:end), v(2:2:end)), P, 'baseband', opt);
% --- ket qua bit cua FPGA (dma_bits):   ofdm_decode_dma('bits.bin');
