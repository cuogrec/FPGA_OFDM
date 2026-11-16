%% ======================================================================
%  test_wire.m -- CHANG 2, THU TREN DAY (khong can nuoc), ~10 phut
%    C1  hoi quy : 200 khung FEC qua day noi -> phai dung 100 %
%    C3  do nhay : giam bien do den khi FEC hong -> moc so sanh voi day A1b/AGC trong xo
%  Noi day: Focusrite Output 1 -> MACH CHIA AP -> PCM1808 (nhu TEST_LOOPBACK.md),
%  KHONG qua bo khuech dai / dau do. Dong Tera Term. App rev 3.x (tu dong muc).
%  ======================================================================
clear; clc;

T.port   = 'COM6';
T.device = 'Scarlett';
T.note   = 'day';
T.thr    = 180;
T.c1_amp = 0.3;
T.c1_frames = 200;
T.c3_amps = [0.1 0.03 0.01 0.003 0.001 0.0005 0.0003 0.0002];
T.c3_frames = 60;
T.preroll_s = 2.0;

P   = ofdm_params();
PER = (P.N + P.CP) * P.frame_syms / P.fsb + 0.150;
fprintf(['Kiem tra: Focusrite Output 1 da noi qua mach chia ap vao PCM1808, KHONG qua bo khuech dai?\n' ...
         '(Tin hieu truc tiep tu card co the qua lon voi ADC neu thieu mach chia ap.)\n']);
input('Enter de bat dau: ', 's');

st = board_status(T.port);
if isnan(st.agc), error('App tren board chua co tu dong muc: nap app rev 3.x.'); end
rx_log('t', 'value', T.thr, 'port', T.port);
ensure(T.port, 'fec',  'e', 1);
ensure(T.port, 'test', 'k', 1);
ensure(T.port, 'agc',  'a', 1);
devid = find_device(T.device);
pl = ofdm_fec_payloads('test', 50, P);

levels = [T.c1_amp, T.c3_amps];  R = [];
for i = 1:numel(levels)
    amp = levels(i);  nfr = T.c3_frames;  tag = sprintf('C3_amp%05d', round(amp * 1e4));
    if i == 1, nfr = T.c1_frames; tag = 'C1_hoiquy'; end
    S = struct('fc', P.fc, 'amp', amp, 'gap_ms', 150, 'preroll_ms', T.preroll_s * 1000, ...
               'papr_clip_db', Inf, 'repeat', ceil((nfr + 10) / 50));
    y = ofdm_tx_signal(pl, P, S);
    fprintf('\n===== %s | amp %.4f (%+.1f dB so voi 0,3) =====\n', tag, amp, 20 * log10(amp / 0.3));
    po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.3);
    out = rx_log('g', 'N', nfr, 'secs', (nfr + 8) * PER + T.preroll_s + 5, 'idle', 12, ...
                 'tag', tag, 'port', T.port, 'note', T.note);
    stop(po);  pause(1.0);
    r = classify_p(out.rows, PER * 1000);
    r.amp = amp;  r.tag = tag;
    s2 = board_status(T.port);  r.gain = s2.gain;
    R = [R; r]; %#ok<AGROW>
    fprintf('-> %s: %d/%d khung dung (%.0f%%), bit sua TB %.1f, gain_shift %d\n', tag, r.ok, r.expected, ...
        r.ok_pct, r.raw_mean, r.gain);
    if i > 1 && r.ok_pct < 50, fprintf('FEC da hong o muc nay -> dung quet.\n'); break; end
end

fprintf('\n================ TOM TAT TREN DAY ================\n');
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('WIRE_results_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');  fprintf(fid, 'tag,amp,gain,ok,expected,ok_pct,junk,raw_mean,raw_max\n');
for k = 1:numel(R)
    q = R(k);
    fprintf('%-12s amp %.4f  gain %d  %3d/%-3d  %5.1f%%  bat nham %d  bit sua TB %.1f\n', q.tag, q.amp, q.gain, ...
        q.ok, q.expected, q.ok_pct, q.junk, q.raw_mean);
    fprintf(fid, '%s,%.5f,%d,%d,%d,%.1f,%d,%.2f,%g\n', q.tag, q.amp, q.gain, q.ok, q.expected, q.ok_pct, q.junk, q.raw_mean, q.raw_max);
end
fclose(fid);
c1 = R(1);
fprintf('C1 hoi quy: %s (%d/%d)\n', ifelse(c1.ok_pct >= 100, 'DAT', 'CHUA DAT'), c1.ok, c1.expected);
good = R(2:end);  good = good([good.ok_pct] >= 95);
if ~isempty(good), fprintf('C3 do nhay: muc nho nhat con >= 95%% khung dung: amp %.4f\n', min([good.amp])); end
fprintf('Da luu %s\n', f);

%% ======================================================================
function s = ifelse(c, a, b)
if c, s = a; else, s = b; end
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
