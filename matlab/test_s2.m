%% ======================================================================
%  test_s2.m -- BUOI DO 2 (mot cua so MATLAB, khong can sua file nao khac)
%    A2  bao dong gia : khong phat, 5 phut            -> dat khi 0 khung
%    A3  chay dai     : 1000 khung FEC, amp 0,5       -> dat khi >= 999 khung dung
%    A1c day o gain 2 : amp 0,05 -> 0,003, co/khong FEC (so voi A1b o gain 4)
%  Truoc khi chay: DONG TERA TERM, bat bo khuech dai, dau do trong xo nhu cac lan truoc.
%  Tong thoi gian ~ 30 phut. Script tu dat gain_shift, thr, rmin, test, fec.
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP ----------------
T.parts      = {'A2', 'A3', 'A1c'};          % bo bot neu chi muon chay mot phan
T.gain       = 2;                            % gain_shift cho ca buoi
T.amp        = 0.5;                          % bien do cho A3 (va de tinh rmin cua A2)
T.a2_secs    = 300;
T.a3_frames  = 1000;
T.low_amps   = [0.05 0.03 0.02 0.012 0.008 0.005 0.003];
T.low_modes  = {'fec', 'nofec'};
T.secs       = 51;                           % moi muc cua A1c
T.rmin_ref   = 200000;                       % rmin o amp 0,9, gain 4
T.rmin_floor = 200;                          % san rmin o gain 2 (= 3000 o gain 4)
T.thr        = 180;
T.device     = 'Scarlett';
T.port       = 'COM6';
T.note       = 'xo';

P  = ofdm_params();
Tf = (P.N + P.CP) * (P.NSYM + 2) / P.fsb + 0.150;
rmin_of = @(amp) round(max(T.rmin_ref * (amp / 0.9)^2 * 4^(T.gain - 4), T.rmin_floor));

%% ---------------- CHUAN BI BOARD ----------------
st = board_status(T.port);
fprintf('Board: gain_shift %d, test %d, fec %d\n', st.gain, st.test, st.fec);
if st.test == 0, rx_log('k', 'port', T.port); end
while st.gain < T.gain, rx_log('+', 'port', T.port); st.gain = st.gain + 1; end
while st.gain > T.gain, rx_log('-', 'port', T.port); st.gain = st.gain - 1; end
rx_log('t', 'value', T.thr, 'port', T.port);
fec_now = st.fec;
if fec_now == 0, rx_log('e', 'port', T.port); fec_now = 1; end
devid = find_device(T.device);
R = [];  summary = {};

%% ---------------- A2: bao dong gia ----------------
if any(strcmp(T.parts, 'A2'))
    rx_log('r', 'value', rmin_of(T.amp), 'port', T.port);
    fprintf('\n===== A2: KHONG PHAT, cho %d s (rmin %d) =====\n', T.a2_secs, rmin_of(T.amp));
    out = rx_log('g', 'N', 1000, 'secs', T.a2_secs, 'idle', T.a2_secs + 60, 'tag', 'A2_baodonggia', ...
                 'port', T.port, 'note', T.note);
    summary{end+1} = sprintf('A2: %d khung bat nham trong %d s (dat khi 0)', size(out.rows, 1), T.a2_secs);
    fprintf('-> %s\n', summary{end});
end

%% ---------------- A3: chay dai ----------------
if any(strcmp(T.parts, 'A3'))
    rx_log('r', 'value', rmin_of(T.amp), 'port', T.port);
    nfr = T.a3_frames + 30;
    S = struct('fc', P.fc, 'amp', T.amp, 'gap_ms', 150, 'preroll_ms', 300, ...
               'papr_clip_db', Inf, 'repeat', 5);                      % 1 doan = 250 khung
    fprintf('\n===== A3: %d khung FEC, amp %.2f, gain_shift %d (~%.0f phut) =====\n', ...
        T.a3_frames, T.amp, T.gain, T.a3_frames * Tf / 60);
    y  = single(ofdm_tx_signal(ofdm_fec_payloads('test', 50, P), P, S));
    y  = repmat(y, ceil(nfr / 250), 1);                                 % lap lai doan 250 khung (~250 MB)
    po = audioplayer(y, P.fs, 24, devid);
    play(po);  pause(0.8);
    out = rx_log('g', 'N', T.a3_frames, 'secs', nfr * Tf + 20, 'idle', 20, 'tag', 'A3_dai', ...
                 'port', T.port, 'note', T.note);
    stop(po);  clear po y;  pause(1.0);
    r = classify(out.rows, true, T.a3_frames);
    cfo = out.rows(:, 4);  if isempty(cfo), cfo = NaN; end
    summary{end+1} = sprintf(['A3: %d/%d khung dung (%d bat nham), BER kenh %.2e, bit sua TB %.1f, max %g, ' ...
        'CFO %.2f..%.2f Hz (dat khi >= %d)'], r.ok, T.a3_frames, r.junk, r.ch_ber, r.raw_mean, r.raw_max, ...
        min(cfo), max(cfo), T.a3_frames - 1);
    fprintf('-> %s\n', summary{end});
end

%% ---------------- A1c: day o gain 2 ----------------
if any(strcmp(T.parts, 'A1c'))
    expct = floor(T.secs / Tf);
    for mode = T.low_modes
        m = mode{1};  want = strcmp(m, 'fec');
        if fec_now ~= want, rx_log('e', 'port', T.port); fec_now = want; end
        for amp = T.low_amps
            rmin = rmin_of(amp);
            rx_log('r', 'value', rmin, 'port', T.port);
            S = struct('fc', P.fc, 'amp', amp, 'gap_ms', 150, 'preroll_ms', 300, ...
                       'papr_clip_db', Inf, 'repeat', ceil((T.secs / Tf + 20) / 50));
            if want, pl = ofdm_fec_payloads('test', 50, P); else, pl = ofdm_test_payloads(50, P); end
            y = ofdm_tx_signal(pl, P, S);
            tag = sprintf('A1c_%s_amp%04d_g%d', m, round(amp * 1000), T.gain);
            fprintf('\n===== A1c %s | amp %.3f (%+.1f dB) | gain_shift %d | rmin %d =====\n', ...
                m, amp, 20*log10(amp/0.9), T.gain, rmin);
            po = audioplayer(y, P.fs, 24, devid);
            play(po);  pause(0.8);
            out = rx_log('g', 'N', 1000, 'secs', T.secs, 'idle', 20, 'tag', tag, 'port', T.port, 'note', T.note);
            stop(po);  pause(1.0);
            r = classify(out.rows, want, expct);
            r.mode = m;  r.amp = amp;  r.amp_db = 20*log10(amp/0.9);  r.gain = T.gain;  r.rmin = rmin;
            r.file = out.files{1};
            R = [R; r]; %#ok<AGROW>
            fprintf('-> %d khung that (%d bat nham), %d dung = %.0f%% | BER kenh %.2e\n', ...
                r.real, r.junk, r.ok, r.ok_pct, r.ch_ber);
        end
    end
    if fec_now == 0, rx_log('e', 'port', T.port); end                 % tra ve FEC bat
    f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('A1c_results_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
    fid = fopen(f, 'w');
    fprintf(fid, 'mode,amp,amp_db,gain,rmin,expected,detected,junk,real,ok,ok_pct,ch_ber,raw_mean,raw_max,file\n');
    for k = 1:numel(R)
        q = R(k);
        fprintf(fid, '%s,%.4f,%.1f,%d,%d,%d,%d,%d,%d,%d,%.1f,%.3g,%.2f,%g,%s\n', q.mode, q.amp, q.amp_db, ...
            q.gain, q.rmin, expct, q.detected, q.junk, q.real, q.ok, q.ok_pct, q.ch_ber, q.raw_mean, q.raw_max, q.file);
    end
    fclose(fid);
    fprintf('\nDa luu %s\n', f);
    figure('Name', 'A1c - day o gain 2', 'Color', 'w');  hold on;  grid on;
    for mode = T.low_modes
        s = strcmp({R.mode}, mode{1});
        plot([R(s).amp_db], [R(s).ok_pct], 'o-', 'LineWidth', 1.5, 'DisplayName', [mode{1} ', gain 2']);
    end
    % A1b o gain 4 (10/11/2026) de so sanh
    db4 = [-25.1 -29.5 -33.1 -37.5 -41.0 -45.1 -49.5];
    plot(db4, [99 100 100 100 100 100 95], 'k--', 'DisplayName', 'fec, gain 4 (A1b)');
    plot(db4, [98 99 99 97 79 19 0], 'k:', 'DisplayName', 'nofec, gain 4 (A1b)');
    xlabel('Bien do so voi 0,9 (dB)');  ylabel('Khung dung (% so khung phat)');  ylim([0 105]);
    legend('Location', 'southwest');  title('A1c: gioi han duoi o gain\_shift 2');
end

fprintf('\n================ TOM TAT ================\n');
for k = 1:numel(summary), fprintf('%s\n', summary{k}); end
for k = 1:numel(R)
    fprintf('A1c %-5s amp %.3f (%+.1f dB): %3.0f%% (%d bat nham)\n', R(k).mode, R(k).amp, R(k).amp_db, R(k).ok_pct, R(k).junk);
end
fprintf('Board dang o: gain_shift %d, test 1, fec 1. Muon phat van ban: go k de tat che do thu.\n', T.gain);

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
