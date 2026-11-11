%% ======================================================================
%  test_a1.m -- PHEP THU A1: DU TRU CONG SUAT (CO FEC VA KHONG FEC)
%  Chay trong MOT cua so MATLAB: phat bang audioplayer (chay nen) va thu bang
%  rx_log, lan luot qua tung muc bien do. Ket qua: data/A1_results.csv + do thi.
%
%  Truoc khi chay:
%    - DONG TERA TERM (cong COM chi mo duoc boi 1 chuong trinh)
%    - board da nap app (co lenh k, e), dau do trong xo nhu lan do truoc
%    - bo khuech dai bat, volume nhu lan do truoc (KHONG doi trong suot phep thu)
%  Thoi gian: khoang 1 phut moi muc -> 9 muc FEC + 5 muc khong FEC ~ 14 phut.
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP ----------------
T.amps          = [0.9 0.7 0.5 0.35 0.25 0.18 0.12 0.08 0.05];  % ~3 dB moi buoc
T.nofec_min_amp = 0.25;      % khong FEC: chi do tu muc nay tro len (thap hon chac chan hong)
T.secs          = 51;        % thoi gian thu moi muc (~100 khung, chu ky 0,51 s)
T.gain          = 4;         % gain_shift giu co dinh
T.rmin_ref      = 200000;    % rmin o bien do 0,9 (da chay tot)
T.rmin_min      = 20000;     % san cua rmin (tranh bat nham do nhieu bo khuech dai)
T.thr           = 180;       % 0,70
T.device        = 'Scarlett';% chuoi tim ten thiet bi phat (audiodevinfo)
T.out_ch        = 1;         % Output 1 cua Focusrite
T.port          = 'COM6';
T.note          = 'xo';      % ghi chu vao runs.csv

P  = ofdm_params();
Tf = (P.N + P.CP) * (P.NSYM + 2) / P.fsb + 0.150;      % chu ky khung (s), gap 150 ms

%% ---------------- CHUAN BI BOARD ----------------
st = board_status(T.port);
fprintf('Board: gain_shift %d, test %d, fec %d\n', st.gain, st.test, st.fec);
if st.test == 0, rx_log('k', 'port', T.port); end
while st.gain < T.gain, rx_log('+', 'port', T.port); st.gain = st.gain + 1; end
while st.gain > T.gain, rx_log('-', 'port', T.port); st.gain = st.gain - 1; end
rx_log('t', 'value', T.thr, 'port', T.port);
fec_now = st.fec;

devid = find_device(T.device);

%% ---------------- CHAY ----------------
R = struct('mode', {}, 'amp', {}, 'amp_db', {}, 'rmin', {}, 'sent', {}, 'detected', {}, ...
           'ok', {}, 'ok_pct', {}, 'ber', {}, 'ch_ber', {}, 'raw_mean', {}, 'raw_max', {}, 'file', {});
for mode = {'fec', 'nofec'}
    m = mode{1};  want = strcmp(m, 'fec');
    if fec_now ~= want, rx_log('e', 'port', T.port); fec_now = want; end
    for amp = T.amps
        if ~want && amp < T.nofec_min_amp, continue; end
        rmin = round(max(T.rmin_ref * (amp / 0.9)^2, T.rmin_min));
        rx_log('r', 'value', rmin, 'port', T.port);

        % --- tin hieu phat: khung thu, du dai cho T.secs + du phong
        S = struct('fc', P.fc, 'amp', amp, 'gap_ms', 150, 'preroll_ms', 300, ...
                   'papr_clip_db', Inf, 'repeat', ceil((T.secs / Tf + 20) / 50));
        if want, pl = ofdm_fec_payloads('test', 50, P); else, pl = ofdm_test_payloads(50, P); end
        y  = ofdm_tx_signal(pl, P, S);
        y2 = zeros(numel(y), 2);  y2(:, T.out_ch) = y;
        tag = sprintf('A1_%s_amp%03d', m, round(amp * 100));
        fprintf('\n===== %s | bien do %.2f (%+.1f dB) | rmin %d =====\n', m, amp, 20*log10(amp/0.9), rmin);

        pl_obj = audioplayer(y2, P.fs, 24, devid);
        play(pl_obj);  pause(0.8);                                   % qua preroll
        out = rx_log('g', 'N', 1000, 'secs', T.secs, 'idle', 20, 'tag', tag, ...
                     'port', T.port, 'note', T.note);
        stop(pl_obj);  pause(1.0);                                   % cho tieng vang tat

        rows = out.rows;  n = size(rows, 1);
        sent = round(T.secs / Tf);
        k = numel(R) + 1;
        R(k).mode = m;  R(k).amp = amp;  R(k).amp_db = 20*log10(amp/0.9);  R(k).rmin = rmin;
        R(k).sent = sent;
        R(k).detected = n;
        R(k).ok = sum(rows(:, 3));
        R(k).ok_pct = 100 * min(R(k).ok / sent, 1);
        R(k).ber = out.summary.ber;
        if n
            nbit = 1792 * want + 1768 * ~want;
            R(k).ch_ber = sum(rows(:, 7)) / (nbit * n);
            R(k).raw_mean = mean(rows(:, 7));  R(k).raw_max = max(rows(:, 7));
        else
            R(k).ch_ber = NaN;  R(k).raw_mean = NaN;  R(k).raw_max = NaN;
        end
        R(k).file = out.files{1};
        fprintf('-> %s %.2f: %d/%d khung dung (%.0f%%), BER kenh %.2e, bit kenh sai TB %.1f, max %g\n', ...
            m, amp, R(k).ok, sent, R(k).ok_pct, R(k).ch_ber, R(k).raw_mean, R(k).raw_max);
    end
end

%% ---------------- LUU + VE ----------------
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('A1_results_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
fprintf(fid, 'mode,amp,amp_db,rmin,sent,detected,ok,ok_pct,ber,ch_ber,raw_mean,raw_max,file\n');
for k = 1:numel(R)
    fprintf(fid, '%s,%.3f,%.1f,%d,%d,%d,%d,%.1f,%.3g,%.3g,%.2f,%g,%s\n', R(k).mode, R(k).amp, ...
        R(k).amp_db, R(k).rmin, R(k).sent, R(k).detected, R(k).ok, R(k).ok_pct, R(k).ber, ...
        R(k).ch_ber, R(k).raw_mean, R(k).raw_max, R(k).file);
end
fclose(fid);
fprintf('\nDa luu %s\n', f);

fe = strcmp({R.mode}, 'fec');  nf = ~fe;
figure('Name', 'A1 - du tru cong suat', 'Color', 'w');
subplot(2, 1, 1);
plot([R(fe).amp_db], [R(fe).ok_pct], 'o-', 'LineWidth', 1.5); hold on;
plot([R(nf).amp_db], [R(nf).ok_pct], 's--', 'LineWidth', 1.2); grid on;
xlabel('Bien do phat so voi 0,9 (dB)'); ylabel('Khung dung (%)'); ylim([0 105]);
legend('Co FEC', 'Khong FEC', 'Location', 'southeast');
title('Ti le khung dung theo bien do phat');
subplot(2, 1, 2);
semilogy([R(fe).amp_db], [R(fe).ch_ber], 'o-', 'LineWidth', 1.5); hold on;
semilogy([R(nf).amp_db], [R(nf).ch_ber], 's--', 'LineWidth', 1.2); grid on;
xlabel('Bien do phat so voi 0,9 (dB)'); ylabel('BER kenh (truoc FEC)');
title('BER kenh theo bien do phat');

%% ======================================================================
function st = board_status(port)
o = rx_log('s', 'port', port);
L = strjoin(o.lines, ' ');
st.gain = tok(L, 'gain_shift (\d+)');  st.test = tok(L, 'test (\d)');  st.fec = tok(L, 'fec (\d)');
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
