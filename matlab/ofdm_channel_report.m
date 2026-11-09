function R = ofdm_channel_report(files, opt)
% OFDM_CHANNEL_REPORT  Phan tich kenh nuoc tu file dump bang goc cua FPGA.
%   R = ofdm_channel_report('data/20261108_d_2m_dump.txt')
%   R = ofdm_channel_report({'a.txt', 'b.txt'}, struct('tag', '2m'))
%
%   File dump = log UART co RAW_BEGIN ... RAW_END (lenh 'd' cua app, hoac rx_log('d')).
%   Nen ghi khi MATLAB dang phat khung, tot nhat la S.mode = 'test' (dem duoc bit loi).
%
%   Ket qua cho moi khung: CRC, CFO, SNR (tu EVM), bit loi (neu la khung test).
%   Ket qua gop:
%     - dap ung tan so |H| trong bang 31-35 kHz (do phang, suy hao o mep)
%     - SNR theo tung song mang (tim song mang yeu / bi nhieu hep)
%     - dap ung xung va tre da duong (so voi CP 4 ms)
%     - muc do kenh thay doi giua cac khung va trong 1 khung (pilot)
%   Ghi 1 dong tom tat vao opt.csv (mac dinh 'channel_summary.csv' canh file dump).
%
%   opt: .tag ('')  .csv (auto, '' = khong ghi)  .plot (true)  .thr (150/256)
if nargin < 2, opt = struct(); end
if ischar(files), files = {files}; end
tag    = getopt(opt, 'tag', '');
doplot = getopt(opt, 'plot', true);
thr    = getopt(opt, 'thr', 150/256);
csvf   = getopt(opt, 'csv', fullfile(fileparts(files{1}), 'channel_summary.csv'));
P  = ofdm_params();
N  = P.N;  SL = P.N + P.CP;

%% ---- giai thu tung file ----
res = [];
for i = 1:numel(files)
    r = load_dump(files{i});
    fprintf('%s: %d mau (%.1f s), rms %.0f LSB, dinh %.0f LSB\n', files{i}, numel(r), ...
        numel(r) / P.fsb, sqrt(mean(abs(r).^2)), max(abs(r)));
    ri = ofdm_rx_ref(r, P, 'baseband', struct('thr', thr));
    if isempty(res), res = ri; else, res = [res, ri]; end %#ok<AGROW>
end
nf = numel(res);
assert(nf > 0, ['Khong tim thay khung nao. Kiem tra MATLAB dang phat khi chup dump, ' ...
                'muc thu (lenh l) va gain_shift.']);
% Loai khung hong han (SNR thap hon trung vi > 6 dB): thuong do du lieu dump bi dut
% giua cac goi hoac phat hien nham, khong dai dien cho kenh.
snr_all = [res.snr_db];
bad = snr_all < median(snr_all) - 6;
if any(bad)
    fprintf(['Loai %d khung hong (SNR %s dB, seq %s): nhieu kha nang do dump bi dut, ' ...
             'khong tinh vao thong ke kenh.\n'], nnz(bad), mat2str(round(10*snr_all(bad))/10), ...
             mat2str([res(bad).seq]));
    res = res(~bad);  nf = numel(res);
end

%% ---- tung khung ----
crc = [res.crc_ok];  cfo = [res.cfo_hz];  snr = [res.snr_db];  seq = [res.seq];
ref = ofdm_test_payloads(1, P);
istest = mean(arrayfun(@(x) double(x.bytes(2)) == 229, res)) > 0.5;
berr = nan(1, nf);  raw = nan(1, nf);  isfec = false;
if ~istest                                            % thu giai FEC (khung tu run_tx S.fec = true)
    q = cell(1, nf);  okf = false(1, nf);
    for k = 1:nf, [q{k}, okf(k), raw(k)] = ofdm_fec_unframe(res(k).bytes); end
    isfec = mean(cellfun(@(x) double(x(2)) == 229, q)) > 0.5 || mean(okf) > 0.5;
    if isfec
        crc = okf;  seq = cellfun(@(x) double(x(1)), q);
        [~, finfo] = ofdm_fec_payloads('test', 1, P);
        for k = 1:nf
            if double(q{k}(2)) == 229
                berr(k) = sum(de2bits(q{k}(2:109)) ~= de2bits(finfo(1, 2:109)));
            end
        end
    else
        raw(:) = NaN;
    end
end
if istest
    for k = 1:nf
        a = de2bits(res(k).bytes(2:222));  b = de2bits(ref(1, 2:222));
        berr(k) = sum(a ~= b);
    end
end

%% ---- dap ung tan so ----
Hm  = [res.H];                                        % 64 x nf, thu tu P.active
Hn  = Hm ./ sqrt(mean(abs(Hm).^2, 1));                % chuan hoa muc tung khung
Hdb = 10*log10(mean(abs(Hn).^2, 2));
Hdb = Hdb - max(Hdb);
fk  = P.fc + P.active(:) * P.fsb / N;                 % Hz
ctr = abs(P.active(:)) <= 8;  edg = abs(P.active(:)) >= 28;
edge_drop = mean(Hdb(ctr)) - mean(Hdb(edg));          % dB, mep thap hon tam
ripple    = max(Hdb) - min(Hdb);

%% ---- SNR theo song mang ----
E = zeros(numel(P.data), 1);  cnt = 0;
for k = 1:nf
    z = res(k).zd;  z = z / sqrt(mean(abs(z(:)).^2));
    q = (sign(real(z)) + 1j*sign(imag(z))) / sqrt(2);
    E = E + sum(abs(z - q).^2, 2);  cnt = cnt + size(z, 2);
end
snr_sc = 10*log10(1 ./ (E / cnt));                    % dB, moi song mang du lieu
fd = P.fc + P.data(:) * P.fsb / N;
[snr_worst, iw] = min(snr_sc);

%% ---- dap ung xung / tre da duong ----
fa  = -32:32;                                         % luoi tan so lien tuc, co DC
w   = 0.5 + 0.5*cos(pi * fa / 33);                    % cua so Hann tren bang 4 kHz
pdp = zeros(N, 1);
for k = 1:nf
    h0 = zeros(numel(fa), 1);
    h0(fa ~= 0) = Hn(:, k);
    h0(fa == 0) = (Hn(P.active == -1, k) + Hn(P.active == 1, k)) / 2;   % noi suy DC
    V = zeros(N, 1);  V(mod(fa, N) + 1) = h0 .* w(:);
    h = ifft(V);
    [~, ip] = max(abs(h));
    pdp = pdp + circshift(abs(h).^2, 1 - ip);         % dong chinh dinh ve tre 0
end
pdp = pdp / max(pdp);
lag = [0:95, -32:-1]';                                % mau (8 kHz): 0..11.9 ms va -4..-0.1 ms
tau = lag / P.fsb * 1e3;                              % ms
pdb = 10*log10(pdp + 1e-12);
pos = lag >= 0;
iex = find(pos & pdb >= -20, 1, 'last');  excess20 = tau(iex);
use = pdb >= -25;
tm  = sum(pdp(use) .* tau(use)) / sum(pdp(use));
trms = sqrt(sum(pdp(use) .* (tau(use) - tm).^2) / sum(pdp(use)));
inCP = lag >= -4 & lag <= P.CP;
sir_cp = 10*log10(sum(pdp(inCP)) / max(sum(pdp(~inCP)), 1e-12));

%% ---- kenh thay doi theo thoi gian ----
rho = nan(1, max(nf - 1, 0));
fa64 = P.active(:);
for k = 1:nf - 1
    a = Hm(:, k);  b = Hm(:, k + 1);
    d = a .* conj(b);                                 % bo doc pha do lech dinh thoi 1-2 mau
    sl = angle(sum(d(2:end) .* conj(d(1:end-1))));
    b = b .* exp(1j * sl * fa64);
    rho(k) = abs(a' * b) / (norm(a) * norm(b));
end
amp_fl = nan(1, nf);  ph_jit = nan(1, nf);  ph_slope = nan(1, nf);
Ts = SL / P.fsb;                                      % 20 ms moi symbol
for k = 1:nf
    pl = res(k).pil;                                  % 8 x 16
    amp_fl(k) = mean(std(20*log10(abs(pl) + eps), 0, 2));
    phs = unwrap(angle(mean(pl, 1)));
    c = polyfit(0:P.NSYM-1, phs, 1);
    ph_slope(k) = c(1) / (2*pi*Ts);                   % Hz: CFO con sot
    ph_jit(k) = std(phs - polyval(c, 0:P.NSYM-1)) * 180/pi;
end

%% ---- tom tat ----
R = struct('tag', tag, 'files', {files}, 'n_frames', nf, 'crc_ok', sum(crc), ...
    'fer', 1 - mean(crc), 'ber', sum(berr) / ((1768 - 904*isfec) * nf), 'fec', isfec, ...
    'ch_ber', sum(raw) / (1792 * nf), 'snr_mean_db', mean(snr), ...
    'snr_min_db', min(snr), 'cfo_mean_hz', mean(cfo), 'cfo_std_hz', std(cfo), ...
    'edge_drop_db', edge_drop, 'ripple_db', ripple, 'snr_sc_worst_db', snr_worst, ...
    'snr_sc_worst_hz', fd(iw), 'tau_rms_ms', trms, 'excess20_ms', excess20, ...
    'sir_cp_db', sir_cp, 'rho_mean', mean(rho), 'amp_fluct_db', mean(amp_fl), ...
    'phase_jitter_deg', mean(ph_jit), 'resid_cfo_hz', mean(ph_slope));
R.frames = struct('seq', seq, 'crc_ok', crc, 'cfo_hz', cfo, 'snr_db', snr, 'bit_err', berr, 'raw_err', raw);
R.f_hz = fk;  R.H_db = Hdb;  R.fd_hz = fd;  R.snr_sc_db = snr_sc;  R.tau_ms = tau;  R.pdp_db = pdb;

fprintf('\n===== Phan tich kenh %s (%d khung) =====\n', tag, nf);
fprintf('Khung dung CRC      : %d / %d (FER %.1f%%)\n', sum(crc), nf, 100*R.fer);
if istest, fprintf('Bit loi (khung test): %d / %d bit, BER %.2e\n', sum(berr), 1768*nf, R.ber); end
if isfec
    fprintf('FEC                 : da sua %d bit kenh (BER kenh %.2e), con lai %d bit loi (BER %.2e)\n', ...
        sum(raw), R.ch_ber, sum(berr(~isnan(berr))), R.ber);
end
fprintf('SNR (EVM)           : TB %.1f dB, thap nhat %.1f dB\n', R.snr_mean_db, R.snr_min_db);
fprintf('CFO                 : %+.2f Hz (do lech %.2f Hz), con sot sau bu %+.2f Hz\n', ...
    R.cfo_mean_hz, R.cfo_std_hz, R.resid_cfo_hz);
fprintf('Dap ung tan so      : mep thap hon tam %.1f dB, nhap nho %.1f dB\n', edge_drop, ripple);
fprintf('Song mang yeu nhat  : %.0f Hz, SNR %.1f dB\n', fd(iw), snr_worst);
fprintf('Tre da duong        : RMS %.2f ms, tre vuot -20 dB %.2f ms (CP = 4 ms)\n', trms, excess20);
fprintf('Nang luong ngoai CP : SIR %.1f dB\n', sir_cp);
fprintf('Kenh giua cac khung : tuong quan TB %.3f (1 = khong doi)\n', R.rho_mean);
fprintf('Trong 1 khung       : bien do pilot dao dong %.2f dB, pha lech %.1f do\n', ...
    R.amp_fluct_db, R.phase_jitter_deg);
fprintf('Goi y:\n');
if excess20 > 3,      fprintf('  - Tre da duong gan/vuot CP: can nhac CP dai hon.\n'); end
if edge_drop > 6,     fprintf('  - Mep bang yeu > 6 dB: can nhac doi song mang ve dinh dap ung.\n'); end
if R.amp_fluct_db > 1 || R.phase_jitter_deg > 10
    fprintf('  - Kenh doi trong 1 khung: nen bam kenh theo pilot.\n'); end
if snr_worst < mean(snr_sc) - 6
    fprintf('  - Song mang %.0f Hz yeu bat thuong: kiem tra nhieu hep / ho pha-dinh.\n', fd(iw)); end
fprintf('  (do phan giai tre ~0.5 ms vi bang thong 4 kHz)\n');

if ~isempty(csvf), append_csv(csvf, R); fprintf('Da ghi tom tat vao %s\n', csvf); end

%% ---- do thi ----
if doplot
    figure('Name', ['Kenh nuoc ' tag], 'Color', 'w');
    subplot(2, 2, 1);
    plot(fk/1e3, 10*log10(abs(Hn).^2), 'Color', [.8 .8 .8]); hold on;
    plot(fk/1e3, Hdb, 'b', 'LineWidth', 1.5); grid on;
    xlabel('Tan so (kHz)'); ylabel('|H| (dB)'); title('Dap ung tan so (xam: tung khung)');
    subplot(2, 2, 2);
    plot(fd/1e3, snr_sc, 'o-'); grid on; xlabel('Tan so (kHz)'); ylabel('SNR (dB)');
    title('SNR theo song mang du lieu');
    subplot(2, 2, 3);
    [ts, is] = sort(tau);
    plot(ts, pdb(is), 'b'); hold on; grid on;
    plot([4 4], [-40 0], 'r--'); plot([min(ts) max(ts)], [-20 -20], 'k:');
    ylim([-40 1]); xlabel('Tre so voi dinh (ms)'); ylabel('dB');
    title(sprintf('Dap ung xung: RMS %.2f ms, -20 dB tai %.2f ms (do: CP)', trms, excess20));
    subplot(2, 2, 4);
    plot(snr, 'o-'); hold on; grid on;
    bad = find(~crc); plot(bad, snr(bad), 'rx', 'MarkerSize', 10);
    xlabel('Khung'); ylabel('SNR (dB)');
    title(sprintf('SNR tung khung (x do = loi CRC); CFO TB %+.2f Hz', R.cfo_mean_hz));
end
end

%% ======================================================================
function r = load_dump(file)
t  = fileread(file);
i0 = strfind(t, 'RAW_BEGIN');  i1 = strfind(t, 'RAW_END');
assert(~isempty(i0) && ~isempty(i1), 'Khong thay RAW_BEGIN / RAW_END trong %s', file);
body = t(i0(end):i1(end) - 1);
body = body(find(body == newline, 1) + 1:end);
v = sscanf(strrep(body, ',', ' '), '%d');
v = v(1:2*floor(numel(v)/2));
r = complex(v(1:2:end), v(2:2:end));
end

function b = de2bits(bytes)
b = mod(floor(double(bytes(:)) * 2.^(-(0:7))), 2);
b = b(:);
end

function append_csv(f, R)
cols = {'tag','n_frames','crc_ok','fer','ber','snr_mean_db','snr_min_db','cfo_mean_hz', ...
        'cfo_std_hz','edge_drop_db','ripple_db','snr_sc_worst_db','snr_sc_worst_hz', ...
        'tau_rms_ms','excess20_ms','sir_cp_db','rho_mean','amp_fluct_db', ...
        'phase_jitter_deg','resid_cfo_hz'};
newf = ~exist(f, 'file');
fid = fopen(f, 'a');
if newf, fprintf(fid, 'time,%s,files\n', strjoin(cols, ',')); end
fprintf(fid, '%s', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
for i = 1:numel(cols)
    v = R.(cols{i});
    if ischar(v), fprintf(fid, ',%s', v); else, fprintf(fid, ',%.4g', v); end
end
fprintf(fid, ',%s\n', strjoin(R.files, ';'));
fclose(fid);
end

function v = getopt(o, f, d), if isfield(o, f), v = o.(f); else, v = d; end, end