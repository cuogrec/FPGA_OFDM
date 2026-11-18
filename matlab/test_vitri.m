%% ======================================================================
%  test_vitri.m -- DANH GIA LAI SAU KHI DOI VI TRI DAU DO (~6 phut)
%    V0  im lang 20 s       : nhieu nen, bat nham (khong phat gi)
%    V1  100 khung, amp 0,3 : cung muc voi lan do chuan A5_doidien (12/11) -> so sanh truc tiep
%    V2  dump bang goc      : SNR, do gon dap ung tan so, tre da duong so voi CP, kenh co dung yen khong
%    V3  100 khung, amp 0,9 : manh hon ~10 dB -> tach loi do tin hieu yeu hay do kenh (da duong, dong)
%    V4  van ban 3 lan      : kiem tra truyen text thuc te (tuy chon, T.text)
%  Cuoi cung in CHAN DOAN va so voi lan do chuan.
%
%  Truoc khi chay: DONG TERA TERM. Nuoc yen, khong cham vao xo trong luc do.
%  Board: app rev 3.x (tu dong muc). Neu da tung go 'f', script tu dua ltf ve 0.
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP ----------------
T.port     = 'COM6';
T.device   = 'Scarlett';
T.tag      = 'vitri_moi';       % doi ten neu do nhieu vi tri (vd 'vitri_B')
T.note     = 'be';
T.thr      = 180;
T.amp_lo   = 0.3;               % giong A5 (de so sanh)
T.amp_hi   = 0.9;
T.frames   = 100;
T.npkt     = 8;                 % goi dump (~11 khung, ~35 s qua UART)
T.silence_s= 20;
T.text     = true;
T.msg      = 'Kiem tra vi tri moi: Truyen thong thuy am OFDM 33 kHz tren ZCU104. 0123456789';
T.preroll_s= 2.0;

% Lan do chuan (12/11/2026, A5_doidien: hai dau do doi dien, giua do sau, amp 0,3)
REF = struct('ok_pct', 100, 'raw_mean', 2.1, 'peak_dbfs', -20.9, 'snr', 17.4, 'excess20', NaN, 'rho', 1.000);

P   = ofdm_params();
PER = (P.N + P.CP) * P.frame_syms / P.fsb + 0.150;      % 0,51 s (khoang nghi 150 ms)

%% ---------------- CHUAN BI BOARD ----------------
st = board_status(T.port);
if isnan(st.agc), error('App tren board chua co tu dong muc: nap app rev 3.x.'); end
if ~isnan(st.ltf) && st.ltf ~= 0, rx_log('f', 'value', 0, 'port', T.port); end
rx_log('t', 'value', T.thr, 'port', T.port);
ensure(T.port, 'fec',  'e', 1);
ensure(T.port, 'test', 'k', 1);
ensure(T.port, 'agc',  'a', 1);
devid = find_device(T.device);
pl = ofdm_fec_payloads('test', 50, P);
input('Nuoc YEN, dau do o vi tri moi, khong cham vao xo. Enter de bat dau... ', 's');

%% ---------------- V0: im lang ----------------
fprintf('\n===== V0: im lang %d s (khong phat gi) =====\n', T.silence_s);
o0 = rx_log('g', 'N', 1000, 'secs', T.silence_s, 'idle', T.silence_s + 10, ...
            'tag', [T.tag '_V0_imlang'], 'port', T.port, 'note', T.note);
V0.junk = size(o0.rows, 1);
[V0.peak_dbfs, V0.gain] = adc_peak(o0.files{2});
fprintf('-> %d lan bat nham trong %d s; dinh nhieu tai ADC %.1f dBFS\n', V0.junk, T.silence_s, V0.peak_dbfs);

%% ---------------- V1 + V2: amp thap, roi dump ----------------
[V1, V2] = do_frames('V1', T.amp_lo, true, T, P, PER, pl, devid);
%% ---------------- V3: amp cao ----------------
V3 = do_frames('V3', T.amp_hi, false, T, P, PER, pl, devid);

%% ---------------- V4: van ban ----------------
V4 = struct('done', NaN, 'ok', NaN, 'n', NaN);
if T.text
    fprintf('\n===== V4: van ban x 3 lan, amp %.2f =====\n', T.amp_lo);
    ensure(T.port, 'test', 'k', 0);
    cod = ofdm_fec_payloads('text', T.msg, P);
    S = struct('fc', P.fc, 'amp', T.amp_lo, 'gap_ms', 50, 'preroll_ms', T.preroll_s * 1000, ...
               'papr_clip_db', Inf, 'repeat', 3);
    y = ofdm_tx_signal(cod, P, S);
    po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.3);
    o4 = rx_log('g', 'N', 3 * size(cod, 1), 'secs', numel(y) / P.fs + 6, 'idle', 10, ...
                'tag', [T.tag '_V4_vanban'], 'port', T.port, 'note', T.note);
    stop(po);  pause(0.5);
    V4.done = numel(strfind(o4.raw, 'message complete')) + numel(strfind(o4.raw, 'message repeated'));
    V4.ok = 0;  if ~isempty(o4.rows), V4.ok = sum(o4.rows(:, 3)); end
    V4.n = 3 * size(cod, 1);
    fprintf('-> %d/%d khung OK, %d lan "message complete"\n', V4.ok, V4.n, V4.done);
    ensure(T.port, 'test', 'k', 1);
end

%% ---------------- TOM TAT + CHAN DOAN ----------------
L = {};
L{end+1} = sprintf('Vi tri: %s   (%s)', T.tag, datestr(now, 'yyyy-mm-dd HH:MM'));
L{end+1} = sprintf('V0 im lang %d s     : %d bat nham, nhieu nen dinh %.1f dBFS', T.silence_s, V0.junk, V0.peak_dbfs);
L{end+1} = line_frames('V1 amp 0,3', V1, REF);
L{end+1} = sprintf('V2 kenh (dump)      : SNR %.1f dB (chuan %.1f), song mang yeu nhat %.1f dB o %.2f kHz, gon |H| %.1f dB, tre -20 dB %.2f ms (CP 4 ms), rho %.3f', ...
                   V2.snr, REF.snr, V2.snr_worst, V2.f_worst / 1000, V2.ripple, V2.excess20, V2.rho);
REF3 = REF;  REF3.peak_dbfs = REF.peak_dbfs + 20 * log10(T.amp_hi / T.amp_lo);   % chuan quy doi len amp 0,9
L{end+1} = line_frames('V3 amp 0,9', V3, REF3);
if T.text, L{end+1} = sprintf('V4 van ban x3       : %d/%d khung OK, %d/3 ban tin ghep xong', V4.ok, V4.n, V4.done); end

D = {};
if V0.junk > 0, D{end+1} = sprintf('Co %d lan bat nham khi IM LANG: co nguon nhieu moi gan vi tri nay (day cap, nguon, may bom) -> doi day/xa nguon nhieu.', V0.junk); end
dpk = V1.peak_dbfs - REF.peak_dbfs;
if dpk < -10, D{end+1} = sprintf('Tin hieu thu yeu hon lan chuan %.0f dB -> dau do lech huong / bi che / xa hon. Dua ve doi dien hoac tang amp.', -dpk); end
if dpk > 3,   D{end+1} = sprintf('Tin hieu manh hon lan chuan %.0f dB (gan hon) - tot, AGC tu xu ly.', dpk); end
if ~isnan(V2.excess20) && V2.excess20 > 4, D{end+1} = sprintf('Tre da duong %.1f ms > CP 4 ms (vang tu mat nuoc / thanh xo) -> dua dau do xa mat nuoc va thanh xo, ve giua do sau.', V2.excess20); end
if ~isnan(V2.rho) && V2.rho < 0.99, D{end+1} = sprintf('Kenh thay doi giua cac khung (rho %.3f): nuoc chua yen hoac dau do dung dua -> co dinh dau do, cho nuoc lang.', V2.rho); end
if ~isnan(V2.snr_worst) && V2.snr_worst < -6 && V1.ok_pct < 95, D{end+1} = sprintf('Co ho fading sau o %.2f kHz (SNR song mang %.1f dB): binh thuong voi xo nho; neu nhieu khung hong thi dich dau do vai cm.', V2.f_worst / 1000, V2.snr_worst); end
if V1.ok_pct < 95 && V3.ok_pct >= 95, D{end+1} = 'amp 0,3 hong nhung amp 0,9 tot -> loi do tin hieu yeu: tang S.amp (run_tx) len 0,6-0,9.'; end
if V1.ok_pct < 95 && V3.ok_pct < 95, D{end+1} = 'Hong ca hai muc bien do -> loi do KENH (da duong / kenh dong), khong phai do yeu: xem tre da duong va rho o tren, doi vi tri.'; end
if V1.junk + V3.junk > 2, D{end+1} = 'Nhieu lan bat nham khi dang phat -> vang dai lam bo do kich hoat trong khoang nghi: tang S.gap_ms len 150.'; end
if isempty(D), D{end+1} = 'Khong thay van de: vi tri moi tuong duong lan do chuan.'; end

fprintf('\n================ KET QUA ================\n');
for k = 1:numel(L), fprintf('%s\n', L{k}); end
fprintf('\n---------------- CHAN DOAN ----------------\n');
for k = 1:numel(D), fprintf('* %s\n', D{k}); end
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('VT_results_%s_%s.txt', T.tag, datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
if fid > 0
    fprintf(fid, '%s\n', L{:});  fprintf(fid, '\nCHAN DOAN\n');  fprintf(fid, '* %s\n', D{:});  fclose(fid);
    fprintf('\nDa luu %s\n', f);
end

%% ======================================================================
function [r, ch] = do_frames(name, amp, dodump, T, P, PER, pl, devid)
fprintf('\n===== %s: %d khung thu, amp %.2f%s =====\n', name, T.frames, amp, repmat(' + dump', 1, dodump));
S = struct('fc', P.fc, 'amp', amp, 'gap_ms', 150, 'preroll_ms', T.preroll_s * 1000, ...
           'papr_clip_db', Inf, 'repeat', ceil((T.frames + 10 + 15 * dodump) / 50));
y = ofdm_tx_signal(pl, P, S);
po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.3);
out = rx_log('g', 'N', T.frames, 'secs', (T.frames + 8) * PER + T.preroll_s + 5, 'idle', 12, ...
             'tag', sprintf('%s_%s_amp%03d', T.tag, name, round(amp * 100)), 'port', T.port, 'note', T.note);
r = classify_p(out.rows, PER * 1000);
[r.peak_dbfs, r.gain] = adc_peak(out.files{2});
r.cfo_std = NaN;  if sum(out.rows(:, 3) == 1) > 2, r.cfo_std = std(out.rows(out.rows(:, 3) == 1, 4)); end
fprintf('-> %d/%d khung dung (%.0f%%), mat %d, bat nham %d, bit sua TB %.1f max %g, ADC %.1f dBFS (gain %d)\n', ...
        r.ok, r.expected, r.ok_pct, r.missed, r.junk, r.raw_mean, r.raw_max, r.peak_dbfs, r.gain);
ch = struct('snr', NaN, 'snr_worst', NaN, 'f_worst', NaN, 'ripple', NaN, 'excess20', NaN, 'rho', NaN);
if dodump
    try
        od = rx_log('d', 'npkt', T.npkt, 'tag', [T.tag '_V2_dump'], 'port', T.port, 'note', T.note);
        if isfield(od, 'report')
            q = od.report;
            ch.snr = q.snr_mean_db;  ch.ripple = q.ripple_db;  ch.excess20 = q.excess20_ms;  ch.rho = q.rho_mean;
            if isfield(q, 'snr_sc_worst_db'), ch.snr_worst = q.snr_sc_worst_db; ch.f_worst = q.snr_sc_worst_hz; end
        end
    catch err
        fprintf(2, 'Dump loi (%s) -> bo qua phan tich kenh.\n', err.message);
    end
end
stop(po);  pause(1.0);
end

function s = line_frames(name, r, REF)
s = sprintf('%-20s: %d/%d khung dung (%.0f%%, chuan %.0f%%), bat nham %d, bit sua TB %.1f (chuan %.1f) max %g, ADC %.1f dBFS (chuan %.1f), gain %d', ...
            name, r.ok, r.expected, r.ok_pct, REF.ok_pct, r.junk, r.raw_mean, REF.raw_mean, r.raw_max, ...
            r.peak_dbfs, REF.peak_dbfs, r.gain);
end

function [pk_db, g] = adc_peak(logf)
% muc dinh tai ADC tu dong 'level:' cua app: 20log10(peak/2048) - 6,02*gain_shift
pk_db = NaN;  g = NaN;
try
    s = fileread(logf);
    t = regexp(s, 'level: last peak (\d+), max \d+ of 2048, crest [\d.]+ \| gain_shift (\d+)', 'tokens', 'once');
    if ~isempty(t)
        pk = str2double(t{1});  g = str2double(t{2});
        pk_db = 20 * log10(max(pk, 1) / 2048) - 6.02 * g;
    end
catch
end
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
if ~any(ok), bad = false(size(ok)); end
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

function st = board_status(port)
% lan 's' dau co the chi dung lenh 'g' dang chay (board tra ve 'stopped: ...'), nen hoi lai
for k = 1:2
    o = rx_log('s', 'port', port);
    s = strjoin(o.lines, ' ');
    if ~isempty(regexp(s, 'agc \d', 'once')), break; end
    pause(1.0);
end
st.gain = tok(s, 'gain_shift (\d+)');  st.rmin = tok(s, 'rmin (\d+)');
st.test = tok(s, 'test (\d)');  st.fec = tok(s, 'fec (\d)');  st.agc = tok(s, 'agc (\d)');
st.ltf  = tok(s, 'ltf (\d+)');
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
