function [r, res] = read_uart_dump(file)
% READ_UART_DUMP  Doc baseband tho tu FPGA (lenh 'd' cua ofdm33_app) va giai thu.
%   file : log UART (.txt, co RAW_BEGIN ... RAW_END)  hoac  raw.bin (XSCT mrd -bin)
%   Ve pho baseband 8 kHz va chay bo thu tham chieu.
P = ofdm_params();
[~, ~, ext] = fileparts(file);
if strcmpi(ext, '.bin')
    fid = fopen(file, 'rb'); v = fread(fid, inf, 'int16'); fclose(fid);
    r = complex(v(1:2:end), v(2:2:end));
else
    t  = fileread(file);
    i0 = strfind(t, 'RAW_BEGIN'); i1 = strfind(t, 'RAW_END');
    assert(~isempty(i0) && ~isempty(i1), 'Khong thay RAW_BEGIN / RAW_END trong log');
    body = t(i0(end):i1(end)-1);
    body = body(find(body == newline, 1)+1:end);           % bo dong RAW_BEGIN
    v = sscanf(strrep(body, ',', ' '), '%d');
    r = complex(v(1:2:end), v(2:2:end));
end
fprintf('%d mau (%.2f s), rms %.1f, peak %.0f LSB\n', numel(r), numel(r)/P.fsb, rms_(r), max(abs(r)));

figure('Name', 'FPGA baseband');
subplot(2,1,1); plot((0:numel(r)-1)/P.fsb, abs(r)); grid on; xlabel('s'); ylabel('|bb|');
title('Bien do baseband (8 kHz)');
nf = 4096; w = 0.5 - 0.5*cos(2*pi*(0:nf-1)'/nf);
S = zeros(nf,1); k = 0;
for i = 1:nf/2:numel(r)-nf+1, S = S + abs(fft(r(i:i+nf-1).*w)).^2; k = k + 1; end
f = ((0:nf-1)' - nf/2) / nf * P.fsb;
subplot(2,1,2); plot((f + P.fc)/1e3, 10*log10(fftshift(S)/max(k,1) + eps)); grid on;
xlabel('tan so tuong duong (kHz)'); ylabel('dB'); title('Pho (OFDM phai nam 31-35 kHz)');

res = ofdm_rx_ref(r, P, 'baseband', struct('plot', true));
for i = 1:numel(res)
    fprintf('khung %d: seq %d CRC %d CFO %+.2f Hz SNR %.1f dB "%s"\n', i, res(i).seq, ...
        res(i).crc_ok, res(i).cfo_hz, res(i).snr_db, res(i).text);
end
end
function v = rms_(x), v = sqrt(mean(abs(x).^2)); end
