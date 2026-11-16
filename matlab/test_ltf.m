%% ======================================================================
%  test_ltf.m -- CHANG 3, VIEC 2: lap LTF trong khung (bitstream rev 3, app rev 3.5)
%  So sanh 3 kieu khung:  ltf 0 (khung cu, 360 ms) | ltf 8 (380 ms) | ltf 4 (420 ms)
%  trong 2 dieu kien:     nuoc YEN  va  KHUAY nuoc lien tuc.
%  Moi lan do 60 khung thu co FEC (~40 s). Tong ~ 5 phut.
%
%  Truoc khi chay: DONG TERA TERM. Da nap bitstream rev 3 + app rev 3.5.
%  Khi script hoi "KHUAY": khuay nuoc deu tay, lien tuc trong ca 3 lan do (~2 phut).
%  ======================================================================
clear; clc;
T.port   = 'COM6';
T.device = 'Scarlett';
T.out_ch = 1;
T.note   = 'xo';
T.thr    = 180;
T.amp    = 0.3;
T.frames = 60;
T.preroll_s = 2.0;
T.modes  = [0 8 4];                 % gia tri ltf_rep
T.conds  = {'yen', 'khuay'};

st = board_status(T.port);
if isnan(st.ltf), error('App chua co lenh f (lap LTF): nap app rev 3.5.'); end
rx_log('t', 'value', T.thr, 'port', T.port);
ensure(T.port, 'fec',  'e', 1);
ensure(T.port, 'test', 'k', 1);
ensure(T.port, 'agc',  'a', 1);
ensure(T.port, 'nb',   'p', 1);
devid = find_device(T.device);
R = {};  res = [];

for c = 1:numel(T.conds)
    if strcmp(T.conds{c}, 'yen')
        input('De nuoc YEN, khong cham vao xo. Bam Enter de bat dau... ', 's');
    else
        input('Bat dau KHUAY nuoc lien tuc (giu deu ~2 phut), roi bam Enter... ', 's');
    end
    for M = T.modes
        P   = ofdm_params(M);
        PER = (P.N + P.CP) * P.frame_syms / P.fsb + 0.150;
        pl  = ofdm_fec_payloads('test', 50, P);
        S   = struct('fc', P.fc, 'amp', T.amp, 'gap_ms', 150, 'preroll_ms', T.preroll_s * 1000, ...
                     'papr_clip_db', Inf, 'repeat', ceil((T.frames + 10) / 50));
        y   = ofdm_tx_signal(pl, P, S);
        y2  = zeros(numel(y), 2);  y2(:, T.out_ch) = y;
        rx_log('f', 'value', M, 'port', T.port);
        tag = sprintf('LTF%d_%s', M, T.conds{c});
        fprintf('\n===== %s =====\n', tag);
        po = audioplayer(y2, P.fs, 24, devid);  play(po);  pause(0.3);
        out = rx_log('g', 'N', T.frames, 'secs', (T.frames + 8) * PER + T.preroll_s + 5, 'idle', 12, ...
                     'tag', tag, 'port', T.port, 'note', T.note);
        stop(po);  pause(1.0);
        r = classify_p(out.rows, PER * 1000);
        r.M = M;  r.cond = T.conds{c};
        res = [res; r]; %#ok<AGROW>
        R{end+1} = sprintf('%-5s ltf %d: %3d/%3d khung dung (%5.1f%%), bat nham %d, bit kenh sai TB %.1f, max %g', ...
            T.conds{c}, M, r.ok, r.expected, r.ok_pct, r.junk, r.raw_mean, r.raw_max); %#ok<SAGROW>
        fprintf('-> %s\n', R{end});
    end
    if strcmp(T.conds{c}, 'khuay'), fprintf('\nDUNG KHUAY duoc roi.\n'); end
end
rx_log('f', 'value', 0, 'port', T.port);         % tra board ve khung cu

% kiem bitstream: nuoc yen, ltf 4 ma hong het trong khi ltf 0 tot -> bitstream chua co lap LTF
iy0 = find(strcmp({res.cond}, 'yen') & [res.M] == 0, 1);
iy4 = find(strcmp({res.cond}, 'yen') & [res.M] > 0);
if ~isempty(iy0) && ~isempty(iy4) && res(iy0).ok_pct > 80 && all([res(iy4).ok_pct] < 20)
    R{end+1} = 'CANH BAO: nuoc yen, ltf 0 tot nhung ltf 4/8 hong het -> bitstream rev 3 CHUA duoc nap (build.tcl impl, cap nhat XSA).';
end

fprintf('\n================ TOM TAT LAP LTF ================\n');
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('LTF_results_%s.txt', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
for k = 1:numel(R), fprintf('%s\n', R{k}); if fid > 0, fprintf(fid, '%s\n', R{k}); end, end
if fid > 0, fclose(fid); fprintf('Da luu %s\n', f); end

%% ======================================================================
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
st.nb = tok(s, 'nb (\d)');  st.ltf = tok(s, 'ltf (\d+)');
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
