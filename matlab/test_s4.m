%% ======================================================================
%  test_s4.m -- CHANG 2, BUOI 2 (ban thao tac tay trong luc do), ~25-30 phut
%    A6  kenh bien doi : nuoc yen / khuay nuoc / di chuyen dau thu, moi buoc 150 khung + dump
%    A7  nhieu ngoai   : doi chung / go thanh xo / may bom-quat / tone 32 kHz tron vao tin hieu
%    A5  tu the dau do : 6 tu the, moi tu the 100 khung + dump
%  Truoc moi buoc script dung lai, in viec can lam; go Enter de do, 's' de bo qua buoc,
%  'q' de dung han (van luu ket qua cac buoc da do).
%  Truoc khi chay: DONG TERA TERM; board da nap app rev 3.x (tu dong muc).
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP ----------------
T.parts    = {'A6', 'A7', 'A5'};
T.port     = 'COM6';
T.device   = 'Scarlett';
T.note     = 'xo';
T.thr      = 180;
T.amp      = 0.3;               % bien do phat OFDM (dinh)
T.preroll_s= 2.0;
T.npkt     = 8;                 % goi dump (8 x 0,512 s ~ 11 khung; truyen UART ~35 s)
T.a6_frames= 150;
T.a7_frames= 100;
T.a5_frames= 100;
T.a5_dump  = true;
T.a5_tilt  = false;           % true = them 2 tu the dau thu nam ngang (dau do tru: xoay quanh truc khong doi gi)

P   = ofdm_params();
PER = (P.N + P.CP) * P.frame_syms / P.fsb + 0.150;    % 0,51 s

% {nhom, tag, viec can lam, so khung, dump?, tone [Hz dB] hoac []}
STEPS = {
 'A6', 'A6_yen',      'Nuoc YEN, dau do de nguyen nhu binh thuong. Khong cham vao xo.',                         T.a6_frames, true,  []
 'A6', 'A6_khuay',    'KHUAY NUOC lien tuc (tay hoac que, vong tron, vua phai) tu khi thay "Dang thu" den het buoc, ca luc dump.', T.a6_frames, true, []
 'A6', 'A6_dichuyen', 'DI CHUYEN CHAM dau THU qua lai ~10 cm/s (giu trong nuoc, cung do sau) trong suot buoc do.', T.a6_frames, true, []
 'A7', 'A7_doichung', 'Doi chung: nuoc yen, khong gay nhieu.',                                                  T.a7_frames, false, []
 'A7', 'A7_go',       'GO vao thanh xo (ngon tay hoac but) 1-2 lan moi giay trong suot buoc do.',                T.a7_frames, false, []
 'A7', 'A7_maybom',   'BAT may bom / quat / may khoan gan xo (cang gan cang tot), de chay suot buoc do. Khong co thi go s.', T.a7_frames, false, []
 'A7', 'A7_tone_m12', 'Khong can lam gi: script tron them tone 32 kHz yeu hon OFDM 12 dB vao tin hieu phat.',     T.a7_frames, false, [32000 -12]
 'A7', 'A7_tone_m6',  'Khong can lam gi: tone 32 kHz yeu hon OFDM 6 dB.',                                        T.a7_frames, false, [32000 -6]
 'A5', 'A5_doidien',  'Tu the CHUAN: hai dau do doi dien, giua do sau (nhu cac lan do truoc).',                 T.a5_frames, T.a5_dump, []
 'A5t','A5_ngang_ben', 'Dat dau THU NAM NGANG, truc vuong goc duong noi hai dau do (than tru van huong ve dau phat). Giu khoang cach, do sau.', T.a5_frames, T.a5_dump, []
 'A5t','A5_ngang_dau', 'Dat dau THU NAM NGANG, truc chia thang vao dau phat (dau tron huong ve dau phat). Giu khoang cach, do sau.', T.a5_frames, T.a5_dump, []
 'A5', 'A5_satday',   'Dua lai doi dien; ha CA HAI dau do xuong sat day xo (cach day ~2-3 cm).',                 T.a5_frames, T.a5_dump, []
 'A5', 'A5_giua',     'Ca hai dau do o GIUA do sau, doi dien (giong tu the chuan, de kiem tra lap lai).',        T.a5_frames, T.a5_dump, []
 'A5', 'A5_satmat',   'Nang CA HAI dau do len sat mat nuoc (ngap ~2-3 cm), doi dien.',                           T.a5_frames, T.a5_dump, []
};

%% ---------------- CHUAN BI BOARD ----------------
st = board_status(T.port);
if isnan(st.agc), error('App tren board chua co tu dong muc: nap app rev 3.x.'); end
rx_log('t', 'value', T.thr, 'port', T.port);
ensure(T.port, 'fec',  'e', 1);
ensure(T.port, 'test', 'k', 1);
ensure(T.port, 'agc',  'a', 1);
devid = find_device(T.device);
pl = ofdm_fec_payloads('test', 50, P);

%% ---------------- CHAY ----------------
R = [];  stop_all = false;
for i = 1:size(STEPS, 1)
    [grp, tag, todo, nfr, dodump, tone] = STEPS{i, :};
    if strcmp(grp, 'A5t'), grp = 'A5'; if ~T.a5_tilt, continue; end, end
    if ~any(strcmp(T.parts, grp)) || stop_all, continue; end
    fprintf('\n=========== [%d/%d] %s ===========\n%s\n', i, size(STEPS, 1), tag, todo);
    a = input('Enter = do, s = bo qua, q = dung: ', 's');
    if strcmpi(strtrim(a), 'q'), stop_all = true; continue; end
    if strcmpi(strtrim(a), 's'), continue; end

    extra = 15 * dodump;                               % phat them khung cho luc dump
    S = struct('fc', P.fc, 'amp', T.amp, 'gap_ms', 150, 'preroll_ms', T.preroll_s * 1000, ...
               'papr_clip_db', Inf, 'repeat', ceil((nfr + 10 + extra) / 50));
    y = ofdm_tx_signal(pl, P, S);
    if ~isempty(tone), y = add_tone(y, tone(1), tone(2), T.amp, P); end
    po = audioplayer(y, P.fs, 24, devid);  play(po);  pause(0.3);
    out = rx_log('g', 'N', nfr, 'secs', (nfr + 8) * PER + T.preroll_s + 5, 'idle', 12, ...
                 'tag', tag, 'port', T.port, 'note', T.note);
    r = classify_p(out.rows, PER * 1000);
    r.tag = tag;  r.grp = grp;
    r.cfo_std = NaN;  if size(out.rows, 1) > 2, r.cfo_std = std(out.rows(out.rows(:, 3) == 1, 4)); end
    [r.peak_dbfs, r.gain] = adc_peak(out.files{2});
    r.rho = NaN;  r.amp_fl = NaN;  r.ph_jit = NaN;  r.snr = NaN;  r.ripple = NaN;  r.excess20 = NaN;
    if dodump
        try
            od = rx_log('d', 'npkt', T.npkt, 'tag', tag, 'port', T.port, 'note', T.note);
            if isfield(od, 'report')
                q = od.report;
                r.rho = q.rho_mean;  r.amp_fl = q.amp_fluct_db;  r.ph_jit = q.phase_jitter_deg;
                r.snr = q.snr_mean_db;  r.ripple = q.ripple_db;  r.excess20 = q.excess20_ms;
            end
        catch err
            fprintf(2, 'Dump loi (%s) -> bo qua dump buoc nay.\n', err.message);
        end
    end
    stop(po);  pause(1.0);
    R = [R; r]; %#ok<AGROW>
    fprintf(['-> %s: %d/%d khung dung (%.0f%%), mat %d, bat nham %d, bit sua TB %.1f max %g, ' ...
             'CFO std %.2f Hz, dinh ADC %.1f dBFS\n'], tag, r.ok, r.expected, r.ok_pct, r.missed, r.junk, ...
             r.raw_mean, r.raw_max, r.cfo_std, r.peak_dbfs);
end

%% ---------------- TOM TAT ----------------
if isempty(R), fprintf('Khong co buoc nao duoc do.\n'); return; end
f = fullfile(fileparts(mfilename('fullpath')), 'data', sprintf('S4_results_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(f, 'w');
hdr = 'tag,ok,expected,ok_pct,missed,junk,raw_mean,raw_max,cfo_std_hz,peak_dbfs,gain,rho,amp_fluct_db,phase_jit_deg,snr_db,ripple_db,excess20_ms';
fprintf(fid, '%s\n', hdr);
fprintf('\n================ TOM TAT BUOI 2 (chang 2) ================\n');
fprintf('%-13s %8s %6s %5s %6s %7s %7s %6s %6s %6s %6s %6s\n', 'buoc', 'dung', '%', 'nham', 'sua TB', 'CFOstd', 'ADC dB', 'rho', 'pil dB', 'pil do', 'SNR', 'tre20');
for k = 1:numel(R)
    q = R(k);
    fprintf(fid, '%s,%d,%d,%.1f,%d,%d,%.2f,%g,%.3f,%.1f,%g,%.4f,%.2f,%.2f,%.1f,%.1f,%.2f\n', q.tag, q.ok, q.expected, ...
        q.ok_pct, q.missed, q.junk, q.raw_mean, q.raw_max, q.cfo_std, q.peak_dbfs, q.gain, q.rho, q.amp_fl, ...
        q.ph_jit, q.snr, q.ripple, q.excess20);
    fprintf('%-13s %4d/%-3d %6.1f %5d %6.1f %7.2f %7.1f %6.3f %6.2f %6.1f %6.1f %6.2f\n', q.tag, q.ok, q.expected, ...
        q.ok_pct, q.junk, q.raw_mean, q.cfo_std, q.peak_dbfs, q.rho, q.amp_fl, q.ph_jit, q.snr, q.excess20);
end
fclose(fid);
fprintf('Da luu %s\n', f);
fprintf(['Doc bang: "dung %%" < 95 -> buoc do lam hong link; "sua TB" = bit kenh Viterbi sua moi khung;\n' ...
         '"rho" < 0,99 hoac "pil dB" > 3 -> kenh doi trong khung (can bam kenh theo pilot);\n' ...
         '"ADC dB" = muc dinh tai ADC (so sanh cuong do thu giua cac tu the); "tre20" = tre da duong o -20 dB (ms, CP = 4 ms).\n']);

%% ======================================================================
function y = add_tone(y, f0, rel_db, amp, P)
% tron tone f0 (Hz) vao tin hieu phat; muc tone (rms) = rms OFDM + rel_db; chuan hoa lai dinh = amp
on  = abs(y) > 1e-6 * max(abs(y));
rms_s = sqrt(mean(y(on) .^ 2));
t   = (0:numel(y) - 1).' / P.fs;
y   = y + sqrt(2) * rms_s * 10^(rel_db / 20) * sin(2 * pi * f0 * t);
y   = amp * y / max(abs(y));
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
