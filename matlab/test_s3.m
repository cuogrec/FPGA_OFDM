%% ======================================================================
%  test_s3.m -- CHANG 2, BUOI 1 (khong can dung vao dau do), ~25 phut
%    A4  van ban dai tieng Viet co dau (1159 byte UTF-8, 11 khung FEC, phat 3 lan)
%        -> dat khi moi ban tin '>>> message complete' trung tung byte voi ban goc
%    A8  khoang nghi giua khung 150 -> 50 -> 20 ms, moi muc 100 khung
%        -> biet khoang nghi ngan nhat con ~100 % khung dung
%    A9  dap ung tan so: tone quet 30-36 kHz buoc 250 Hz, lenh m cua board
%        -> dinh cong huong that, bang -3 dB, muc tai 31 / 35 kHz
%  Truoc khi chay: DONG TERA TERM; board da nap app rev 3.x (tu dong muc);
%  bo khuech dai bat, dau do trong xo nhu cac lan truoc.
%  Script tu bat/tat k, e, a tren board. Cuoi buoi: test 1, fec 1, agc 1.
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP ----------------
T.parts     = {'A4', 'A8', 'A9'};   % bo bot neu chi muon chay mot phan
T.port      = 'COM6';
T.device    = 'Scarlett';
T.note      = 'xo';
T.thr       = 180;
T.preroll_s = 2.0;                  % im lang dau tin hieu: board kip vao 'g' truoc khung dau
T.a4_amp    = 0.3;
T.a4_repeat = 3;
T.a8_amp    = 0.3;
T.a8_gaps   = [150 50 20];          % ms
T.a8_frames = 100;
T.a9_amp    = 0.3;                  % bien do tone (dinh), nho hon nguong cat o gain 2
T.a9_gain   = 2;                    % gain_shift co dinh cho lenh m (AGC chi chay trong g)
T.a9_f      = 30000:250:36000;      % Hz
T.a9_step_s = 3;                    % giay moi tan so (~6 goi do)

% Van ban A4 (UTF-8, de o dang hex de file .m chi co ky tu ASCII):
A4_HEX = [ ...
    '5468e1bbad206e676869e1bb876d2074727579e1bb816e2076c4836e2062e1baa36e207469e1babf6e67205669e1bb87' ...
    '742063c3b32064e1baa57520717561206bc3aa6e68207468e1bba77920c3a26d203333206b487a2e2042e1bb99207068' ...
    'c3a174204d41544c41422074e1baa16f2074c3ad6e206869e1bb8775204f46444d2067e1bb936d2036342073c3b36e67' ...
    '206d616e6720636f6e2c20c49169e1bb8175206368e1babf205150534b2c206dc3a3206368e1baad702074e1bb89206c' ...
    'e1bb8720312f322076c3a02078656e206be1babd206269742e2054c3ad6e206869e1bb877520c4916920717561206361' ...
    '726420c3a26d207468616e6820466f637573726974652c2062e1bb99206b6875e1babf636820c491e1baa1692063c3b4' ...
    '6e67207375e1baa5742076c3a020c491e1baa7752064c3b2204249492d37353131482076c3a06f206ec6b0e1bb9b632e' ...
    '20c490e1baa7752064c3b2207468e1bba92068616920746875206ce1baa1692c2062e1bb992063687579e1bb836e20c4' ...
    '91e1bb95692050434d31383038206ce1baa579206de1baab75203936206b487a2c2046504741207472c3aa6e205a4355' ...
    '31303420c491e1bb936e672062e1bb992c2063c3a26e2062e1bab16e67206bc3aa6e682076c3a0206769e1baa36920c4' ...
    '9169e1bb8175206368e1babf207468656f207468e1bb9d69206769616e207468e1bbb1632e204cc3b5692041524d2067' ...
    '69e1baa369206dc3a320566974657262692c206b69e1bb836d20747261204352432072e1bb9369206768c3a9702063c3' ...
    'a163206b68756e67207468c3a06e682062e1baa36e2074696e20686fc3a06e206368e1bb896e682e2054726f6e672078' ...
    'c3b4206ec6b0e1bb9b632c2068e1bb87207468e1bb916e6720c491c3a32074727579e1bb816e2031303030206b68756e' ...
    '67206c69c3aa6e207469e1babf70206b68c3b46e672073616920626974206ec3a06f2c2076c3a02062e1bb992074e1bb' ...
    'b120c491e1bb996e67206de1bba963206769e1bbaf20c491c6b0e1bba3632074e1bb89206ce1bb87206b68756e6720c4' ...
    '91c3ba6e67207472c3aa6e203935207068e1baa76e207472c4836d2074726f6e672064e1baa369203435206442206dc3' ...
    'a0206b68c3b46e672063e1baa76e206368e1bb896e68207461792e2043c3a163206bc3bd2074e1bbb120c491e1bab763' ...
    '206269e1bb87742063e1baa76e206869e1bb876e20c491c3ba6e673a20c48320c3a220c49120c3aa20c3b420c6a120c6' ...
    'b02c20c48220c38220c49020c38a20c39420c6a020c6af2c20687579e1bb816e2073e1baaf632068e1bb8f69206e67c3' ...
    'a3206ee1bab76e673a20c3a020c3a120e1baa320c3a320e1baa12c20e1bb8120e1babf20e1bb8320e1bb8520e1bb872c' ...
    '20e1bb9320e1bb9120e1bb9520e1bb9720e1bb992c20e1bbab20e1bba920e1bbad20e1bbaf20e1bbb12c20e1bbb320c3' ...
    'bd20e1bbb720e1bbb920e1bbb52e204ee1babf752064c3b26e67206ec3a079206869e1bb876e206e677579c3aa6e2076' ...
    'e1bab96e20e1bb9f206375e1bb9169207468c3ac207068c3a970207468e1bbad20413420c491c3a320c491e1baa1742e' ...
    '2048e1babf742e' ...
    ];

P  = ofdm_params();
TFR = (P.N + P.CP) * P.frame_syms / P.fsb;            % 0,36 s moi khung

%% ---------------- CHUAN BI BOARD ----------------
st = board_status(T.port);
if isnan(st.agc), error('App tren board chua co tu dong muc: nap app rev 3.x (dong chao co "rev 3").'); end
fprintf('Board: gain_shift %d, rmin %d, test %d, fec %d, agc %d\n', st.gain, st.rmin, st.test, st.fec, st.agc);
rx_log('t', 'value', T.thr, 'port', T.port);
ensure(T.port, 'fec', 'e', 1);
ensure(T.port, 'agc', 'a', 1);
devid = find_device(T.device);
res = {};                                            % cac dong tom tat

%% ---------------- A4: van ban dai co dau ----------------
if any(strcmp(T.parts, 'A4'))
    ensure(T.port, 'test', 'k', 0);                  % che do thuong: board in chu
    txt_b = uint8(sscanf(A4_HEX, '%2x')).';
    txt   = native2unicode(txt_b, 'UTF-8');
    pl    = ofdm_fec_payloads('text', txt, P);
    nfr   = size(pl, 1);
    S  = sig_struct(P, T.a4_amp, 150, T.preroll_s * 1000, T.a4_repeat);
    y  = ofdm_tx_signal(pl, P, S);
    fprintf('\n===== A4: %d byte UTF-8 -> %d khung FEC x %d lan (%.0f s) =====\n', numel(txt_b), nfr, T.a4_repeat, numel(y) / P.fs);
    po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.3);
    out = rx_log('g', 'N', nfr * T.a4_repeat, 'secs', numel(y) / P.fs + 5, 'idle', 15, ...
                 'tag', 'A4_vanban', 'port', T.port, 'note', T.note);
    stop(po);  pause(1.0);
    [nmsg, ngood, firstbad] = check_messages(out.raw, txt_b);
    nok = 0;  if ~isempty(out.rows), nok = sum(out.rows(:, 3)); end
    pass = nmsg >= 2 && ngood == nmsg;
    res{end+1} = sprintf('A4 van ban: %d/%d khung dung, %d ban tin ghep xong, %d trung tung byte -> %s', ...
        nok, nfr * T.a4_repeat, nmsg, ngood, verdict(pass));
    fprintf('-> %s\n', res{end});
    if ~isempty(firstbad)
        fprintf('   Ban tin sai dau tien (%d byte, ban goc %d byte), byte khac dau tien o vi tri %d\n', ...
            firstbad.len, numel(txt_b), firstbad.pos);
    end
    if ngood, fprintf('   Ban tin nhan duoc:\n   %s\n', txt); end
end

%% ---------------- A8: khoang nghi ngan hon ----------------
if any(strcmp(T.parts, 'A8'))
    ensure(T.port, 'test', 'k', 1);
    pl = ofdm_fec_payloads('test', 50, P);
    for gap = T.a8_gaps
        per = TFR + gap / 1000;
        S  = sig_struct(P, T.a8_amp, gap, T.preroll_s * 1000, ceil((T.a8_frames + 15) / 50));
        y  = ofdm_tx_signal(pl, P, S);
        fprintf('\n===== A8: khoang nghi %d ms (chu ky %.0f ms, %.2f kbit/s chu co FEC) =====\n', ...
            gap, per * 1000, 106 * 8 / per / 1000);
        po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.3);
        out = rx_log('g', 'N', T.a8_frames, 'secs', (T.a8_frames + 10) * per + T.preroll_s + 5, 'idle', 10, ...
                     'tag', sprintf('A8_gap%03d', gap), 'port', T.port, 'note', T.note);
        stop(po);  pause(1.0);
        r = classify_p(out.rows, per * 1000);
        res{end+1} = sprintf(['A8 nghi %3d ms: %3d/%3d khung dung (%.0f%%), mat %d, bat nham %d, ' ...
            'bit sua TB %.1f max %g -> %s'], gap, r.ok, r.expected, r.ok_pct, r.missed, r.junk, ...
            r.raw_mean, r.raw_max, verdict(r.ok_pct >= 99));
        fprintf('-> %s\n', res{end});
    end
end

%% ---------------- A9: dap ung tan so ----------------
if any(strcmp(T.parts, 'A9'))
    st = board_status(T.port);
    set_gain(T.port, st.gain, T.a9_gain);             % +/- tat tu dong muc (bat lai o duoi)
    y = step_tone(T.a9_f, T.a9_step_s, T.a9_amp, P.fs);
    fprintf('\n===== A9: tone %.1f-%.1f kHz, buoc %d Hz, %d s moi buoc (%.0f s), gain_shift %d =====\n', ...
        T.a9_f(1) / 1e3, T.a9_f(end) / 1e3, T.a9_f(2) - T.a9_f(1), T.a9_step_s, numel(y) / P.fs, T.a9_gain);
    po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.5);
    out = rx_log('m', 'secs', numel(y) / P.fs - 0.5, 'tag', 'A9_dapung', 'port', T.port, 'note', T.note);
    stop(po);  pause(0.5);
    ensure(T.port, 'agc', 'a', 1);
    if isfield(out, 'tone') && height(out.tone) > 2
        Tt = out.tone;  lv = Tt.level_corr_dBFS;  fz = Tt.f_Hz;
        [mx, im] = max(lv);
        k1 = im;  while k1 > 1 && lv(k1 - 1) >= mx - 3, k1 = k1 - 1; end
        k2 = im;  while k2 < numel(lv) && lv(k2 + 1) >= mx - 3, k2 = k2 + 1; end
        [~, j31] = min(abs(fz - 31000));  [~, j35] = min(abs(fz - 35000));
        clipwarn = '';
        if ~isempty(out.rows) && max(out.rows(:, 3)) > -6.02 * T.a9_gain - 2
            clipwarn = ' (CANH BAO: muc tone sat nguong cat, giam T.a9_amp)';
        end
        res{end+1} = sprintf(['A9 dap ung: dinh %.0f Hz (lech %+.0f Hz so voi 33 kHz), bang -3 dB %.0f-%.0f Hz, ' ...
            'muc 31 kHz %+.1f dB, 35 kHz %+.1f dB so voi dinh%s'], fz(im), fz(im) - 33000, fz(k1), fz(k2), ...
            lv(j31) - mx, lv(j35) - mx, clipwarn);
    else
        res{end+1} = 'A9 dap ung: khong du dong M sach de ve (kiem tra tone co phat ra khong, muc tone, file log)';
    end
    fprintf('-> %s\n', res{end});
end

%% ---------------- TOM TAT ----------------
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('S3_results_%s.txt', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
fprintf('\n================ TOM TAT BUOI 1 (chang 2) ================\n');
for k = 1:numel(res)
    fprintf('%s\n', res{k});
    if fid > 0, fprintf(fid, '%s\n', res{k}); end
end
if fid > 0, fclose(fid); fprintf('Da luu %s\n', f); end
st = board_status(T.port);
fprintf('Board dang o: gain_shift %d, test %d, fec %d, agc %d. Muon phat van ban: go k de tat che do thu.\n', ...
    st.gain, st.test, st.fec, st.agc);

%% ======================================================================
function S = sig_struct(P, amp, gap_ms, preroll_ms, rep)
S = struct('fc', P.fc, 'amp', amp, 'gap_ms', gap_ms, 'preroll_ms', preroll_ms, ...
           'papr_clip_db', Inf, 'repeat', rep);
end

function [nmsg, ngood, firstbad] = check_messages(raw, ref)
% tim cac dong '>>> message complete (N frames): <text>' va so tung byte voi ref
nmsg = 0;  ngood = 0;  firstbad = [];
if isempty(raw), return; end
b = uint8(double(raw));
key = '>>> message complete';
idx = strfind(raw, key);
for i = idx
    c = find(raw(i:end) == ':', 1);
    if isempty(c), continue; end
    a = i + c + 1;                                   % sau ': '
    e = find(b(a:end) == 13 | b(a:end) == 10, 1);
    if isempty(e), continue; end                     % dong chua in xong
    m = b(a:a + e - 2);
    nmsg = nmsg + 1;
    if isequal(m(:).', ref(:).')
        ngood = ngood + 1;
    elseif isempty(firstbad)
        n = min(numel(m), numel(ref));
        p = find(m(1:n) ~= ref(1:n), 1);  if isempty(p), p = n + 1; end
        firstbad = struct('len', numel(m), 'pos', p);
    end
end
% app rev 3.6 (hien thi gon): ban tin lap lai giong het ban truoc chi in '>>> message repeated'
nrep = numel(strfind(raw, '>>> message repeated'));
nmsg = nmsg + nrep;
if nrep && nmsg > nrep && ngood == nmsg - nrep, ngood = ngood + nrep; end
end

function y = step_tone(fl, step_s, amp, fs)
% tone quet bac thang, pha lien tuc, 0,5 s im lang dau, tang/giam bien do 50 ms
ns = round(step_s * fs);
f  = repelem(fl(:), ns);
ph = 2 * pi * cumsum(f) / fs;
y  = amp * sin(ph);
nr = round(0.05 * fs);  w = 0.5 - 0.5 * cos(pi * (0:nr - 1).' / nr);
y(1:nr) = y(1:nr) .* w;  y(end - nr + 1:end) = y(end - nr + 1:end) .* flipud(w);
y  = [zeros(round(0.5 * fs), 1); y];
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

function s = verdict(pass)
if pass, s = 'DAT'; else, s = 'CHUA DAT'; end
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
