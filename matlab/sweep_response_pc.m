%% ======================================================================
%  sweep_response_pc.m -- QUET DAP UNG TAN SO: PHAT VA THU CUNG LUC BANG MAY TINH
%  MATLAB -> Focusrite Output -> (amp) -> transducer phat ... nuoc ...
%  hydrophone/dau do thu -> (tien khuech dai) -> Focusrite Input -> MATLAB
%  Moi tan so trong F.freqs (TUY Y, vd 20-45 kHz) phat F.dwell_s giay, do bien do
%  thu duoc o dung tan so do, ve dap ung va tinh be rong -3 dB.
%  Dung audioPlayerRecorder (Audio Toolbox): phat + thu cung xung nhip.
%  Khong can FPGA. Dung de tim tan so cong huong thuc va dai thong cua cap dau do
%  (OFDM dang can dai 31-35 kHz co phang khong).
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP (sua o day) ----------------
F.fs       = 96000;               % Hz (card phai dat dung toc do nay)
F.freqs    = 20000:500:45000;     % Hz, danh sach tan so TUY Y (< fs/2)
F.dwell_s  = 0.8;                 % s moi tan so
F.settle_s = 0.4;                 % s dau bo di (dau do con "rung", tre am thanh)
F.amp      = 0.1;                 % bien do phat 0..1. Bat dau NHO khi co bo khuech dai!
F.tx_ch    = 1;                   % Output cua card
F.rx_ch    = 1;                   % Input cua card
F.blk      = 2048;                % mau moi khoi
F.device   = 'Scarlett';         % chuoi tim trong ten thiet bi (regexp)
F.csv_file = 'sweep_response.csv';

%% >>> DEVICE (khoi nay tao ham duplex = @(x) -> y: phat x, tra ve phan thu duoc)
assert(max(F.freqs) < F.fs/2, 'Moi tan so phai < fs/2 = %d Hz', F.fs/2);
apr = audioPlayerRecorder('SampleRate', F.fs, ...
        'PlayerChannelMapping', F.tx_ch, 'RecorderChannelMapping', F.rx_ch);
dev = getAudioDevices(apr);
k   = find(~cellfun(@isempty, regexpi(dev, F.device)), 1);
if isempty(k), error('Khong tim thay thiet bi "%s". Co: %s', F.device, strjoin(dev, ' | ')); end
apr.Device = dev{k};
fprintf('Phat+thu qua: %s, %d Hz, Output %d -> Input %d\n', dev{k}, F.fs, F.tx_ch, F.rx_ch);
cleanupObj = onCleanup(@() release(apr));
duplex = @(x) apr(x);
%% <<< DEVICE

%% ---------------- QUET ----------------
nb_d  = ceil(F.dwell_s * F.fs / F.blk);          % khoi moi tan so
nb_s  = ceil(F.settle_s * F.fs / F.blk);         % khoi bo di
nf    = numel(F.freqs);
lvl   = nan(nf, 1);  fmeas = nan(nf, 1);  pnr = nan(nf, 1);  clipf = false(nf, 1);
n     = (0:F.blk-1).';
ph    = 0;

% 1) nen nhieu: khong phat gi
nz = [];
for b = 1:nb_d
    y = duplex(zeros(F.blk, 1));  if b > nb_s, nz = [nz; y(:, 1)]; end %#ok<AGROW>
end
noise_rms_db = 20 * log10(sqrt(mean(nz.^2)) + eps);
fprintf('Nen nhieu thu: %.1f dBFS (rms, toan bang)\n', noise_rms_db);

% 2) tung tan so (pha lien tuc, bien do tang dan o khoi dau cua moi buoc)
fprintf('Quet %d tan so, ~%.0f s ... (Ctrl+C de dung)\n', nf, nf * nb_d * F.blk / F.fs);
for q = 1:nf
    f0 = F.freqs(q);  rec = [];
    for b = 1:nb_d
        x = F.amp * sin(ph + 2*pi*f0/F.fs * n);
        ph = mod(ph + 2*pi*f0/F.fs * F.blk, 2*pi);
        if b == 1, x = x .* linspace(0, 1, F.blk).'; end
        y = duplex(x);
        if b > nb_s, rec = [rec; y(:, 1)]; end %#ok<AGROW>
    end
    [fm, a, p] = tone_estimate(rec, F.fs, [f0 - 150, f0 + 150]);
    fmeas(q) = fm;  lvl(q) = a;  pnr(q) = p;  clipf(q) = max(abs(rec)) > 0.98;
    fprintf('  %6.0f Hz -> thu %8.1f Hz, %6.1f dBFS, PNR %4.0f dB%s\n', f0, fm, a, p, ...
            iff(clipf(q), '  CLIP!', ''));
end
release(apr);

%% ---------------- KET QUA ----------------
gain_db = lvl - 20 * log10(F.amp);                % thu/phat (dB), gom ca kenh nuoc
good    = pnr > 10 & ~clipf;
if ~any(good), error('Khong co diem nao tin cay (PNR > 10 dB, khong clip). Tang F.amp hoac GAIN.'); end
gtmp = gain_db;  gtmp(~good) = -Inf;                % chi xet diem tin cay
[mx, im] = max(gtmp);
fpk = F.freqs(im);
fprintf('\nDinh: %.0f Hz (%.1f dB so voi phat)\n', fpk, mx);
% be rong -3 dB quanh dinh (lien tuc, chi diem tin cay)
lo = im;  while lo > 1  && good(lo-1) && gain_db(lo-1) >= mx - 3, lo = lo - 1; end
hi = im;  while hi < nf && good(hi+1) && gain_db(hi+1) >= mx - 3, hi = hi + 1; end
fprintf('Be rong -3 dB: %.0f .. %.0f Hz (%.0f Hz)\n', F.freqs(lo), F.freqs(hi), F.freqs(hi) - F.freqs(lo));
for fx = [31000 33000 35000]
    [d, j] = min(abs(F.freqs - fx));
    if d <= 250 && good(j)
        fprintf('  tai %.0f Hz: %+.1f dB so voi dinh\n', F.freqs(j), gain_db(j) - mx);
    end
end

fid = fopen(F.csv_file, 'w');
fprintf(fid, 'f_Hz,f_meas_Hz,level_dBFS,gain_dB,pnr_dB,clip\n');
fprintf(fid, '%d,%.2f,%.2f,%.2f,%.1f,%d\n', [F.freqs(:), fmeas, lvl, gain_db, pnr, clipf].');
fclose(fid);
fprintf('Da luu %s\n', F.csv_file);

figure('Name', 'Dap ung tan so', 'Color', 'w');
plot(F.freqs / 1e3, gain_db, 'o-', 'LineWidth', 1.3); hold on;
plot(F.freqs(~good) / 1e3, gain_db(~good), 'rx', 'MarkerSize', 10);
xline(fpk / 1e3, '--r');
grid on; xlabel('Tan so (kHz)'); ylabel('Muc thu - muc phat (dB)');
title(sprintf('Dap ung tan so (dinh %.1f kHz); x do = khong tin cay / clip', fpk / 1e3));

function r = iff(c, a, b)
if c, r = a; else, r = b; end
end
