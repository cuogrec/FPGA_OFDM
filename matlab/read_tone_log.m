function T = read_tone_log(logfile)
% READ_TONE_LOG  Doc log lenh 'm' (monitor tone) cua app ZCU104 va ve dap ung tan so.
%   Cach dung:
%     1) MATLAB: chay tone_forever.m (F.mode = 'step' de quet, hoac 'fixed')
%     2) Tera Term: File > Log... (bat ghi log), go 'm' tren app, doi vai vong quet,
%        bam phim bat ky de dung, tat log
%     3) MATLAB: T = read_tone_log('teraterm.log');   % hoac khong doi so -> chon file
%
%   Dong log:  M,k,f_Hz,dBFS,|bb|,peak,coh
%   Tra ve T (bang): moi dong la mot tan so da on dinh (trung vi cac khoi sach).
%   Luu y:
%     * dBFS trong app gia dinh he so FIR = 0.5 (chua kiem chung) -> tin vao
%       CHENH LECH giua cac tan so, khong tin gia tri tuyet doi qua +-3 dB.
%     * FIR giam dan o ngoai +-2.5 kHz quanh 33 kHz (-1.3 dB o 30/36 kHz,
%       -3.3 dB o 29.5/36.5 kHz): script da bu lai (cot level_corr).
%     * Chi do duoc 29-37 kHz; ngoai dai nay tan so bi gap pho (aliasing).
if nargin < 1 || isempty(logfile)
    [f, d] = uigetfile({'*.log;*.txt', 'Log (*.log, *.txt)'; '*.*', 'All'}, 'Chon log Tera Term');
    if isequal(f, 0), return; end
    logfile = fullfile(d, f);
end

txt  = fileread(logfile);
rows = regexp(txt, '(?m)^M,(\d+),([-\d.]+),([-\d.]+),(\d+),(\d+),([\d.]+)', 'tokens');
assert(~isempty(rows), 'Khong co dong "M,..." nao trong %s (da go lenh m chua?)', logfile);
A = cellfun(@(c) str2double(c), rows, 'UniformOutput', false);
A = vertcat(A{:});          % [k f dBFS |bb| peak coh]
k = A(:,1); f = A(:,2); lv = A(:,3); coh = A(:,6);
fprintf('Doc %d dong, %.1f s\n', size(A,1), size(A,1) * 4096 / 8000);

%% Bu dap ung FIR (do trong app: file ofdm33_rx_bd.tcl, 72 tap, /12)
fir_off = [0 500 1000 1500 2000 2500 3000 3500 4000];
fir_db  = [0 -0.03 -0.07 -0.03 -0.01 -0.29 -1.26 -3.28 -6.7];
h_db = interp1(fir_off, fir_db, min(abs(f - 33000), 4000), 'pchip');
lvc  = lv - h_db;

%% Chi giu khoi sach (don tan), gom theo tan so
good = coh >= 0.8;
fprintf('Khoi sach (coh >= 0.8): %d / %d\n', nnz(good), numel(good));
idx = find(good);
[fs_sorted, o] = sort(f(idx)); idx = idx(o);
cl = [1; 1 + cumsum(diff(fs_sorted) > 30)];      % cum moi khi nhay > 30 Hz
n  = max(cl);
fr = zeros(n,1); lr = fr; lcr = fr; cn = fr; cr = fr;
for c = 1:n
    m = idx(cl == c);
    fr(c)  = median(f(m));  lr(c) = median(lv(m));  lcr(c) = median(lvc(m));
    cn(c)  = numel(m);      cr(c) = median(coh(m));
end
% Bo cac cum qua it khoi (thuong la chuyen tiep giua hai tan so)
keep = cn >= 2;
T = table(fr(keep), lr(keep), lcr(keep), cn(keep), cr(keep), ...
    'VariableNames', {'f_Hz', 'level_dBFS', 'level_corr_dBFS', 'n_blocks', 'coh'});
disp(T);

%% Ve
figure('Name', 'Tone monitor', 'Color', 'w');
subplot(2,1,1);
plot(k * 4096 / 8000, f, '.-'); grid on; ylabel('Tan so do duoc (Hz)');
title('Theo thoi gian'); xlabel('Thoi gian (s)');
subplot(2,1,2);
if height(T) > 1
    plot(T.f_Hz / 1e3, T.level_corr_dBFS, 'o-', 'LineWidth', 1.4); hold on;
    plot(T.f_Hz / 1e3, T.level_dBFS, 'x:', 'Color', [.6 .6 .6]);
    legend('da bu FIR', 'tho', 'Location', 'best');
    xlabel('Tan so (kHz)'); ylabel('Muc thu (dBFS)'); grid on;
    title('Dap ung tan so (muc phat khong doi)');
    [mx, im] = max(T.level_corr_dBFS);
    xline(T.f_Hz(im) / 1e3, '--r', sprintf('%.2f kHz', T.f_Hz(im) / 1e3));
    fprintf('Dinh: %.0f Hz (%.1f dBFS). Muc tai 31 / 35 kHz so voi dinh: ', T.f_Hz(im), mx);
    for fx = [31000 35000]
        [~, j] = min(abs(T.f_Hz - fx));
        fprintf('%.0f Hz: %+.1f dB  ', T.f_Hz(j), T.level_corr_dBFS(j) - mx);
    end
    fprintf('\n');
else
    plot(k * 4096 / 8000, lv, '.-'); grid on; xlabel('Thoi gian (s)'); ylabel('dBFS');
    title('Che do fixed: muc thu theo thoi gian');
end
end
