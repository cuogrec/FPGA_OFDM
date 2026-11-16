%% ======================================================================
%  test_nb.m -- CHANG 3, VIEC 1: KIEM TRA CHONG NHIEU BANG HEP (bitstream rev 2 + app rev 3.4)
%    Phan A (kiem bitstream): tu dong muc TAT, gain 2, rmin RAT THAP (1500), khong tone.
%           Khoang nghi co thanh phan hep (DC/u) nen: chong nhieu TAT -> bat nham lien tuc,
%           BAT -> binh thuong. Neu hai lan giong nhau: bitstream CHUA duoc nap ban moi.
%    Phan B: tu dong muc BAT, OFDM + tone 32 kHz 0 / +6 / +10 dB, chong nhieu BAT roi TAT.
%    Phan C: tu dong muc BAT, chong nhieu BAT: im lang 30 s, roi CHI tone +6 dB 30 s -> dem bat nham.
%  Truoc khi chay: DONG TERA TERM; da nap bitstream moi va app rev 3.4. ~15 phut.
%  ======================================================================
clear; clc;

T.port   = 'COM6';
T.device = 'Scarlett';
T.note   = 'xo';
T.thr    = 180;
T.amp    = 0.3;
T.tones  = [0 6 10];            % dB so voi rms OFDM (phan B)
T.frames = 60;
T.c_secs = 30;                  % phan C
T.c_db   = 6;
T.preroll_s = 2.0;
T.parts  = {'A', 'B', 'C'};

P   = ofdm_params();
PER = (P.N + P.CP) * P.frame_syms / P.fsb + 0.150;

st = board_status(T.port);
if isnan(st.nb), error('App chua co lenh p (chong nhieu bang hep): nap app rev 3.4.'); end
rx_log('t', 'value', T.thr, 'port', T.port);
ensure(T.port, 'fec',  'e', 1);
ensure(T.port, 'test', 'k', 1);
devid = find_device(T.device);
pl = ofdm_fec_payloads('test', 50, P);
S  = struct('fc', P.fc, 'amp', T.amp, 'gap_ms', 150, 'preroll_ms', T.preroll_s * 1000, ...
            'papr_clip_db', Inf, 'repeat', ceil((T.frames + 10) / 50));
y0 = ofdm_tx_signal(pl, P, S);
on = abs(y0) > 1e-6 * max(abs(y0));  rms_s = sqrt(mean(y0(on) .^ 2));
R = {};

%% ---------------- A: kiem bitstream ----------------
if any(strcmp(T.parts, 'A'))
    st = board_status(T.port);
    set_gain(T.port, st.gain, 2);                    % +/- tat tu dong muc
    ensure(T.port, 'agc', 'a', 0);
    rx_log('r', 'value', 1500, 'port', T.port);
    for nb = [0 1]
        ensure(T.port, 'nb', 'p', nb);
        r = measure(sprintf('NBA%d_rminthap', nb), y0, T, P, PER, devid);
        R{end+1} = sprintf('A rmin 1500, chong nhieu %s: %3d/%3d khung dung (%5.1f%%), bat nham %d', ...
            onoff(nb), r.ok, r.expected, r.ok_pct, r.junk); %#ok<SAGROW>
        fprintf('-> %s\n', R{end});
        resA(nb + 1) = r; %#ok<SAGROW>
    end
    if resA(1).junk == 0 && abs(resA(1).ok_pct - resA(2).ok_pct) < 5
        R{end+1} = 'A: ca 2 che do deu khong bat nham (be yen) -> A chua du ket luan; xem phan B: tone +6 dB BAT >> TAT nghia la bitstream moi dang chay.';
    else
        R{end+1} = 'A: co khac biet BAT/TAT -> bitstream moi dang chay.';
    end
    fprintf('-> %s\n', R{end});
end
ensure(T.port, 'agc', 'a', 1);

%% ---------------- B: tone manh ----------------
if any(strcmp(T.parts, 'B'))
    for nb = [1 0]
        ensure(T.port, 'nb', 'p', nb);
        for db = T.tones
            y = add_tone(y0, 32000, db, T.amp, P);
            r = measure(sprintf('NBB%d_tone%+03d', nb, db), y, T, P, PER, devid);
            R{end+1} = sprintf('B tone %+3d dB, chong nhieu %s: %3d/%3d khung dung (%5.1f%%), bat nham %d, bit sua TB %.1f', ...
                db, onoff(nb), r.ok, r.expected, r.ok_pct, r.junk, r.raw_mean); %#ok<SAGROW>
            fprintf('-> %s\n', R{end});
        end
    end
end

%% ---------------- C: im lang va chi tone ----------------
if any(strcmp(T.parts, 'C'))
    ensure(T.port, 'nb', 'p', 1);
    fprintf('\n===== C1: IM LANG %d s (bo khuech dai bat, khong phat) =====\n', T.c_secs);
    out = rx_log('g', 'N', 1000, 'secs', T.c_secs, 'idle', T.c_secs + 10, 'tag', 'NBC_imlang', ...
                 'port', T.port, 'note', T.note);
    R{end+1} = sprintf('C im lang %d s, chong nhieu BAT: %d khung bat nham', T.c_secs, size(out.rows, 1));
    fprintf('-> %s\n', R{end});
    t  = (0:round(T.c_secs * P.fs) - 1).' / P.fs;
    yt = sqrt(2) * rms_s * 10^(T.c_db / 20) * sin(2 * pi * 32000 * t);
    yt = min(max(yt, -0.99), 0.99);
    fprintf('\n===== C2: CHI TONE %+d dB, %d s =====\n', T.c_db, T.c_secs);
    po = audioplayer(yt, P.fs, 24, devid);  play(po);  pause(0.3);
    out = rx_log('g', 'N', 1000, 'secs', T.c_secs - 1, 'idle', T.c_secs + 10, 'tag', 'NBC_chitone', ...
                 'port', T.port, 'note', T.note);
    stop(po);  pause(1.0);
    R{end+1} = sprintf('C chi tone %+d dB %d s, chong nhieu BAT: %d khung bat nham', T.c_db, T.c_secs, size(out.rows, 1));
    fprintf('-> %s\n', R{end});
end
ensure(T.port, 'nb', 'p', 1);

fprintf('\n================ TOM TAT CHONG NHIEU BANG HEP ================\n');
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('NB_results_%s.txt', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
for k = 1:numel(R), fprintf('%s\n', R{k}); if fid > 0, fprintf(fid, '%s\n', R{k}); end, end
if fid > 0, fclose(fid); fprintf('Da luu %s\n', f); end

%% ======================================================================
function r = measure(tag, y, T, P, PER, devid)
fprintf('\n===== %s =====\n', tag);
po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.3);
out = rx_log('g', 'N', T.frames, 'secs', (T.frames + 8) * PER + T.preroll_s + 5, 'idle', 12, ...
             'tag', tag, 'port', T.port, 'note', T.note);
stop(po);  pause(1.0);
r = classify_p(out.rows, PER * 1000);
end

function s = onoff(nb)
if nb, s = 'BAT'; else, s = 'TAT'; end
end

function y = add_tone(y, f0, rel_db, amp, P)
% tron tone f0 (Hz) vao tin hieu phat; muc tone (rms) = rms OFDM + rel_db; chuan hoa lai dinh = amp
on  = abs(y) > 1e-6 * max(abs(y));
rms_s = sqrt(mean(y(on) .^ 2));
t   = (0:numel(y) - 1).' / P.fs;
y   = y + sqrt(2) * rms_s * 10^(rel_db / 20) * sin(2 * pi * f0 * t);
y   = amp * y / max(abs(y));
end

function r = classify_p(rows, per_ms)
% khung bat nham: KHONG dung CRC VA (lech nhip > 5 ms, hoac > 200 bit kenh sai)
r = struct('ok', 0, 'real', 0, 'junk', 0, 'expected', 0, 'missed', 0, 'ok_pct', 0, ...
           'raw_mean', NaN, 'raw_max', NaN);
if isempty(rows), return; end
t = rows(:, 1);  raw = rows(:, 7);  ok = rows(:, 3) == 1;
ref = t(find(ok, 1));  if isempty(ref), ref = t(1); end
ph  = mod(t - ref + per_ms / 2, per_ms) - per_ms / 2;
bad = ~ok & (abs(ph) > 5 | raw > 200);
if ~any(ok), bad = false(size(ok)); end         % khong khung nao dung: tinh het la khung that (0 %)
g = rows(~bad, :);
r.junk = sum(bad);  r.real = size(g, 1);  r.ok = sum(g(:, 3));
if r.real
    r.expected = round((max(g(:, 1)) - min(g(:, 1))) / per_ms) + 1;
    r.missed   = max(r.expected - r.real, 0);
    r.ok_pct   = 100 * r.ok / r.expected;
    r.raw_mean = mean(g(:, 7));  r.raw_max = max(g(:, 7));
end
end

function ensure(port, field, key, want)
st = board_status(port);
if isnan(st.(field)), error('Khong doc duoc "%s" tu lenh s.', field); end
if st.(field) ~= want, rx_log(key, 'port', port); end
end

function set_gain(port, g0, g1)
while g0 < g1, rx_log('+', 'port', port); g0 = g0 + 1; end
while g0 > g1, rx_log('-', 'port', port); g0 = g0 - 1; end
end

function st = board_status(port)
o = rx_log('s', 'port', port);
s = strjoin(o.lines, ' ');
st.gain = tok(s, 'gain_shift (\d+)');  st.rmin = tok(s, 'rmin (\d+)');
st.test = tok(s, 'test (\d)');  st.fec = tok(s, 'fec (\d)');  st.agc = tok(s, 'agc (\d)');
st.nb = tok(s, 'nb (\d)');
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
