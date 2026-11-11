%% ======================================================================
%  test_a1b.m -- PHEP THU A1b (bo sung cho A1)
%  Phan 1 "gain": o bien do lon (0.9, 0.5) giam gain_shift 4 -> 1.
%     - Neu so bit kenh sai GIAM manh khi giam gain_shift  -> bao hoa SO o phia thu
%       (khau i2s_ch_sel / bo tron 16 bit), khong phai do bo khuech dai.
%     - Neu KHONG doi -> meo nam o phia phat (Focusrite / bo khuech dai / dau do)
%       hoac ADC PCM1808 (truoc gain_shift).
%  Phan 2 "low": keo dai xuong bien do rat nho (0.05 -> 0.003) de tim diem gay
%     that su do nhieu (A1 chua cham toi: FEC van 99/99 o 0.05).
%  Khung bat nham (CFO lech hoac > 200 bit kenh sai) duoc dem rieng, khong tinh BER.
%
%  Truoc khi chay: DONG TERA TERM; giu nguyen volume bo khuech dai, vi tri dau do
%  NHU LUC CHAY A1. Thoi gian ~ 1 phut / muc (mac dinh 22 muc ~ 23 phut).
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP ----------------
T.parts      = {'gain', 'low'};              % bo bot 1 phan neu muon chay nhanh
T.gain_amps  = [0.9 0.5];                    % phan 1
T.gain_list  = [4 3 2 1];
T.low_amps   = [0.05 0.03 0.02 0.012 0.008 0.005 0.003];  % phan 2 (~4 dB/buoc)
T.low_modes  = {'fec', 'nofec'};
T.secs       = 51;
T.gain_ref   = 4;
T.rmin_ref   = 200000;   % rmin o amp 0.9, gain 4 (nhu A1)
T.rmin_floor = 3000;     % san rmin (thap qua -> bat nham nhieu)
T.thr        = 180;
T.device     = 'Scarlett';
T.out_ch     = 1;
T.port       = 'COM6';
T.note       = 'xo';

P  = ofdm_params();
Tf = (P.N + P.CP) * (P.NSYM + 2) / P.fsb + 0.150;
expct = floor(T.secs / Tf);                  % so khung toi da thu duoc trong T.secs

%% ---------------- CHUAN BI BOARD ----------------
st = board_status(T.port);
fprintf('Board: gain_shift %d, test %d, fec %d\n', st.gain, st.test, st.fec);
if st.test == 0, rx_log('k', 'port', T.port); end
rx_log('t', 'value', T.thr, 'port', T.port);
gain_now = st.gain;  fec_now = st.fec;
devid = find_device(T.device);

% danh sach muc can chay: {phan, mode, amp, gain}
L = {};
if any(strcmp(T.parts, 'gain'))
    for a = T.gain_amps, for g = T.gain_list, L(end+1, :) = {'gain', 'fec', a, g}; end, end %#ok<SAGROW>
end
if any(strcmp(T.parts, 'low'))
    for m = T.low_modes, for a = T.low_amps, L(end+1, :) = {'low', m{1}, a, T.gain_ref}; end, end %#ok<SAGROW>
end
fprintf('%d muc, khoang %.0f phut\n', size(L, 1), size(L, 1) * (T.secs + 4) / 60);

%% ---------------- CHAY ----------------
R = [];
for i = 1:size(L, 1)
    [part, m, amp, g] = L{i, :};
    want = strcmp(m, 'fec');
    if fec_now ~= want, rx_log('e', 'port', T.port); fec_now = want; end
    while gain_now < g, rx_log('+', 'port', T.port); gain_now = gain_now + 1; end
    while gain_now > g, rx_log('-', 'port', T.port); gain_now = gain_now - 1; end
    % R cua bo dong bo ~ (bien do)^2, moi buoc gain_shift = x2 bien do = x4 cong suat
    rmin = round(max(T.rmin_ref * (amp / 0.9)^2 * 4^(g - T.gain_ref), T.rmin_floor));
    rx_log('r', 'value', rmin, 'port', T.port);

    S = struct('fc', P.fc, 'amp', amp, 'gap_ms', 150, 'preroll_ms', 300, ...
               'papr_clip_db', Inf, 'repeat', ceil((T.secs / Tf + 20) / 50));
    if want, pl = ofdm_fec_payloads('test', 50, P); else, pl = ofdm_test_payloads(50, P); end
    y  = ofdm_tx_signal(pl, P, S);
    y2 = zeros(numel(y), 2);  y2(:, T.out_ch) = y;
    tag = sprintf('A1b_%s_%s_amp%04d_g%d', part, m, round(amp * 1000), g);
    fprintf('\n===== [%d/%d] %s %s | amp %.3f (%+.1f dB) | gain_shift %d | rmin %d =====\n', ...
        i, size(L, 1), part, m, amp, 20*log10(amp/0.9), g, rmin);

    po = audioplayer(y2, P.fs, 24, devid);
    play(po);  pause(0.8);
    out = rx_log('g', 'N', 1000, 'secs', T.secs, 'idle', 20, 'tag', tag, 'port', T.port, 'note', T.note);
    stop(po);  pause(1.0);

    r = classify(out.rows, want, expct);
    r.part = part;  r.mode = m;  r.amp = amp;  r.amp_db = 20*log10(amp/0.9);  r.gain = g;
    r.rmin = rmin;  r.file = out.files{1};
    R = [R; r]; %#ok<AGROW>
    fprintf('-> %d khung that (%d bat nham), %d dung = %.0f%% | BER kenh %.2e | sai TB %.1f, max %g\n', ...
        r.real, r.junk, r.ok, r.ok_pct, r.ch_ber, r.raw_mean, r.raw_max);
end

% tra board ve gain_shift chuan
while gain_now < T.gain_ref, rx_log('+', 'port', T.port); gain_now = gain_now + 1; end
while gain_now > T.gain_ref, rx_log('-', 'port', T.port); gain_now = gain_now - 1; end

%% ---------------- LUU + VE ----------------
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('A1b_results_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
fprintf(fid, 'part,mode,amp,amp_db,gain,rmin,expected,detected,junk,real,ok,ok_pct,ch_ber,raw_mean,raw_max,file\n');
for k = 1:numel(R)
    q = R(k);
    fprintf(fid, '%s,%s,%.4f,%.1f,%d,%d,%d,%d,%d,%d,%d,%.1f,%.3g,%.2f,%g,%s\n', q.part, q.mode, q.amp, ...
        q.amp_db, q.gain, q.rmin, expct, q.detected, q.junk, q.real, q.ok, q.ok_pct, q.ch_ber, ...
        q.raw_mean, q.raw_max, q.file);
end
fclose(fid);
fprintf('\nDa luu %s\n', f);

figure('Name', 'A1b', 'Color', 'w');
ig = strcmp({R.part}, 'gain');
if any(ig)
    subplot(1, 2, 1); hold on; grid on;
    for a = T.gain_amps
        s = ig & [R.amp] == a;
        plot([R(s).gain], [R(s).raw_mean], 'o-', 'LineWidth', 1.5, 'DisplayName', sprintf('amp %.2f', a));
    end
    set(gca, 'XDir', 'reverse');  xlabel('gain\_shift');  ylabel('Bit kenh sai TB / khung (tren 1792)');
    title('Phan 1: giam gain\_shift o bien do lon');  legend('Location', 'best');
end
il = strcmp({R.part}, 'low');
if any(il)
    subplot(1, 2, 2); hold on; grid on;
    for m = T.low_modes
        s = il & strcmp({R.mode}, m{1});
        plot([R(s).amp_db], [R(s).ok_pct], 'o-', 'LineWidth', 1.5, 'DisplayName', m{1});
    end
    xlabel('Bien do so voi 0,9 (dB)');  ylabel('Khung dung (% so khung phat)');  ylim([0 105]);
    title('Phan 2: bien do nho');  legend('Location', 'best');
end

%% ======================================================================
function r = classify(rows, want, expct)
% khung bat nham: KHONG dung CRC VA (lech nhip 510 ms > 3 ms, hoac > 200 bit kenh sai).
% (Khong dung CFO: o SNR thap CFO cua khung that cung dao dong +-1 Hz.)
r.detected = size(rows, 1);
if isempty(rows)
    r.junk = 0; r.real = 0; r.ok = 0; r.ok_pct = 0; r.ch_ber = NaN; r.raw_mean = NaN; r.raw_max = NaN; return;
end
t = rows(:, 1);  raw = rows(:, 7);  ok = rows(:, 3) == 1;
ref = t(find(ok, 1));  if isempty(ref), ref = median(mod(t, 510)); end
ph  = mod(t - ref + 255, 510) - 255;                  % lech so voi nhip khung (ms)
bad = ~ok & (abs(ph) > 3 | raw > 200);
g = rows(~bad, :);
nbit = 1792 * want + 1768 * ~want;
r.junk = sum(bad);  r.real = size(g, 1);  r.ok = sum(g(:, 3));
r.ok_pct = 100 * min(r.ok / expct, 1);
if r.real
    r.ch_ber = sum(g(:, 7)) / (nbit * r.real);  r.raw_mean = mean(g(:, 7));  r.raw_max = max(g(:, 7));
else
    r.ch_ber = NaN;  r.raw_mean = NaN;  r.raw_max = NaN;
end
end

function st = board_status(port)
o = rx_log('s', 'port', port);
s = strjoin(o.lines, ' ');
st.gain = tok(s, 'gain_shift (\d+)');  st.test = tok(s, 'test (\d)');  st.fec = tok(s, 'fec (\d)');
if isnan(st.fec), error('App tren board chua co lenh FEC (e): nap ban moi.'); end
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
