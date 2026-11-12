%% ======================================================================
%  test_agc.m -- KIEM TRA TU DONG MUC (app rev 3: lenh a)
%  Quet bien do phat ma KHONG gui lenh gain_shift hay rmin nao: app phai tu chon.
%  Dat khi: moi muc tu 0,9 xuong 0,005 co >= 97 % khung dung (FEC), tinh ca luc
%  AGC dang chinh; muc 0,003 la gioi han, chi ghi nhan.
%  Truoc khi chay: DONG TERA TERM; board da nap app rev 3 (dong chao co "rev 3").
%  Thoi gian: ~1 phut moi muc, 11 muc ~ 11 phut.
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP ----------------
T.amps   = [0.5 0.9 0.25 0.12 0.05 0.02 0.008 0.005 0.003 0.9 0.05];   % 2 muc cuoi: nhay lon
T.secs   = 51;
T.thr    = 180;
T.device = 'Scarlett';
T.port   = 'COM6';
T.note   = 'xo';
T.pass   = 97;               % % khung dung can dat
T.limit  = 0.004;            % bien do nho hon muc nay chi ghi nhan

P  = ofdm_params();
Tf = (P.N + P.CP) * (P.NSYM + 2) / P.fsb + 0.150;
expct = floor(T.secs / Tf);

%% ---------------- CHUAN BI BOARD ----------------
st = board_status(T.port);
if isnan(st.agc), error('App tren board chua co tu dong muc (lenh a): nap app rev 3.'); end
fprintf('Board: gain_shift %d, rmin %d, test %d, fec %d, agc %d\n', st.gain, st.rmin, st.test, st.fec, st.agc);
if st.test == 0, rx_log('k', 'port', T.port); end
if st.fec  == 0, rx_log('e', 'port', T.port); end
rx_log('t', 'value', T.thr, 'port', T.port);          % lenh t khong tat tu dong muc
st = board_status(T.port);
if st.agc == 0, rx_log('a', 'port', T.port); end      % bat SAU CUNG (+ - r n se tat no)
devid = find_device(T.device);

%% ---------------- CHAY ----------------
R = [];
for i = 1:numel(T.amps)
    amp = T.amps(i);
    S = struct('fc', P.fc, 'amp', amp, 'gap_ms', 150, 'preroll_ms', 300, ...
               'papr_clip_db', Inf, 'repeat', ceil((T.secs / Tf + 20) / 50));
    y  = ofdm_tx_signal(ofdm_fec_payloads('test', 50, P), P, S);
    tag = sprintf('AGC_%02d_amp%04d', i, round(amp * 1000));
    fprintf('\n===== [%d/%d] amp %.3f (%+.1f dB) =====\n', i, numel(T.amps), amp, 20*log10(amp/0.9));
    po = audioplayer(y, P.fs, 24, devid);
    play(po);  pause(0.8);
    out = rx_log('g', 'N', 1000, 'secs', T.secs, 'idle', 20, 'tag', tag, 'port', T.port, 'note', T.note);
    stop(po);  pause(1.0);
    s2 = board_status(T.port);                         % gain_shift, rmin ma app da chon
    if s2.agc == 0, error('Tu dong muc da bi tat giua chung (co lenh + - r n nao do).'); end
    r = classify(out.rows, expct);
    r.amp = amp;  r.amp_db = 20*log10(amp/0.9);  r.gain = s2.gain;  r.rmin = s2.rmin;  r.file = out.files{1};
    R = [R; r]; %#ok<AGROW>
    fprintf('-> gain_shift %d, rmin %d | %d khung that (%d bat nham), %d dung = %.0f%% | bit sua TB %.1f\n', ...
        r.gain, r.rmin, r.real, r.junk, r.ok, r.ok_pct, r.raw_mean);
end

%% ---------------- KET QUA ----------------
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('AGC_results_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
fprintf(fid, 'step,amp,amp_db,gain,rmin,expected,detected,junk,real,ok,ok_pct,ch_ber,raw_mean,raw_max,first_ok_ms,file\n');
for k = 1:numel(R)
    q = R(k);
    fprintf(fid, '%d,%.4f,%.1f,%d,%d,%d,%d,%d,%d,%d,%.1f,%.3g,%.2f,%g,%g,%s\n', k, q.amp, q.amp_db, q.gain, ...
        q.rmin, expct, q.detected, q.junk, q.real, q.ok, q.ok_pct, q.ch_ber, q.raw_mean, q.raw_max, q.first_ok, q.file);
end
fclose(fid);

fprintf('\n================ TOM TAT (tu dong muc) ================\n');
fprintf(' buoc   amp      dB   gain    rmin   dung%%  bat nham  bit sua TB  khung dung dau (ms)\n');
allok = true;
for k = 1:numel(R)
    q = R(k);  need = q.amp >= T.limit;  pass = q.ok_pct >= T.pass;
    if need && ~pass, allok = false; end
    if need, verdict = 'DAT';  if ~pass, verdict = 'CHUA DAT'; end
    else,    verdict = '(gioi han)'; end
    fprintf(' %2d   %6.3f  %+6.1f   %d   %7d   %4.0f   %4d     %7.1f    %6g      %s\n', ...
        k, q.amp, q.amp_db, q.gain, q.rmin, q.ok_pct, q.junk, q.raw_mean, q.first_ok, verdict);
end
if allok, fprintf('KET LUAN: DAT cong chang 1 (khong can chinh tay tu 0,9 xuong 0,005).\n');
else,     fprintf('KET LUAN: CHUA DAT. Gui thu muc data de phan tich (cac dong "# AGC", "level:" trong file .log).\n'); end
fprintf('Da luu %s\n', f);

figure('Name', 'Tu dong muc', 'Color', 'w');
subplot(2, 1, 1);  bar([R.ok_pct]);  grid on;  ylim([0 105]);  ylabel('Khung dung (%)');
set(gca, 'XTick', 1:numel(R), 'XTickLabel', arrayfun(@(a) sprintf('%.3g', a), [R.amp], 'UniformOutput', false));
title('Tu dong muc: ti le khung dung theo bien do phat (theo thu tu chay)');
subplot(2, 1, 2);  stairs([[R.gain], R(end).gain], 'LineWidth', 1.5);  grid on;  ylim([-0.5 6.5]);
set(gca, 'XTick', (1:numel(R)) + 0.5, 'XTickLabel', arrayfun(@(a) sprintf('%.3g', a), [R.amp], 'UniformOutput', false));
xlabel('Bien do phat');  ylabel('gain\_shift app chon');

%% ======================================================================
function r = classify(rows, expct)
% khung bat nham: KHONG dung CRC VA (lech nhip 510 ms > 3 ms, hoac > 200 bit kenh sai)
r = struct('detected', size(rows, 1), 'junk', 0, 'real', 0, 'ok', 0, 'ok_pct', 0, ...
           'ch_ber', NaN, 'raw_mean', NaN, 'raw_max', NaN, 'first_ok', NaN);
if isempty(rows), return; end
t = rows(:, 1);  raw = rows(:, 7);  ok = rows(:, 3) == 1;
ref = t(find(ok, 1));  if isempty(ref), ref = median(mod(t, 510)); end
ph  = mod(t - ref + 255, 510) - 255;
bad = ~ok & (abs(ph) > 3 | raw > 200);
g = rows(~bad, :);
r.junk = sum(bad);  r.real = size(g, 1);  r.ok = sum(g(:, 3));
r.ok_pct = 100 * min(r.ok / expct, 1);
if any(ok), r.first_ok = t(find(ok, 1)); end
if r.real
    r.ch_ber = sum(g(:, 7)) / (1792 * r.real);  r.raw_mean = mean(g(:, 7));  r.raw_max = max(g(:, 7));
end
end

function st = board_status(port)
o = rx_log('s', 'port', port);
s = strjoin(o.lines, ' ');
st.gain = tok(s, 'gain_shift (\d+)');  st.rmin = tok(s, 'rmin (\d+)');
st.test = tok(s, 'test (\d)');  st.fec = tok(s, 'fec (\d)');  st.agc = tok(s, 'agc (\d)');
end

function v = tok(s, pat)
t = regexp(s, pat, 'tokens', 'once');
if isempty(t), v = NaN; else, v = str2double(t{1}); end
end

function id = find_device(name)
info = audiodevinfo;
names = {info.output.Name};
k = find(~cellfun(@isempty, regexpi(names, name)), 1);
if isempty(k), error('Khong thay thiet bi "%s". Co: %s', name, strjoin(names, ' | ')); end
id = info.output(k).ID;
fprintf('Phat qua: %s\n', names{k});
end
