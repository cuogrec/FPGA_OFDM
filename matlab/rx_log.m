function out = rx_log(cmd, varargin)
% RX_LOG  Ghi so lieu tu board ZCU104 qua UART bang MATLAB (thay cho chup man hinh Tera Term).
%   DONG TERA TERM TRUOC: cong COM chi mo duoc boi mot chuong trinh.
%
%   rx_log('g', 'N', 200, 'tag', '2m', 'dist_m', 2)  chay g, dung sau 200 khung -> CSV tung khung
%   rx_log('d', 'npkt', 16, 'tag', '2m')              chup bang goc 16 x 0,51 s -> file dump
%                                                     + ofdm_channel_report (phan tich kenh)
%   rx_log('m', 'secs', 120, 'tag', 'quet')           giam sat tone (dung voi tone_forever.m)
%   rx_log('k')  rx_log('n')  rx_log('l')  rx_log('s')  rx_log('+')  ...   gui 1 lenh, in phan hoi
%   rx_log('t', 'value', 120)                          lenh can nhap so (t, o, b, r)
%
%   Tuy chon: 'port' ('COM6')  'baud' (115200)  'dir' (thu muc data canh rx_log.m)  'tag' ('')  'dist_m'  'depth_m'
%     'note'  'N' (200 khung)  'secs' (Inf; 60 cho 'm')  'idle' (15 s khong co du lieu thi dung)
%     'npkt' (16)  'analyze' (true)  'value' ([])
%   Moi lan chay g / d / m them 1 dong vao <dir>/runs.csv (kem gain_shift, rmin doc bang lenh s).
%   Che do khung thu (dem bit loi): tren board bat lenh k, MATLAB phat run_tx.m voi S.mode = 'test'.
o = struct('port', 'COM6', 'baud', 115200, 'dir', '', 'tag', '', 'dist_m', NaN, ...
           'depth_m', NaN, 'note', '', 'N', 200, 'secs', Inf, 'idle', 15, 'npkt', 16, ...
           'analyze', true, 'value', []);
for i = 1:2:numel(varargin)
    assert(isfield(o, varargin{i}), 'Khong co tuy chon "%s"', varargin{i});
    o.(varargin{i}) = varargin{i + 1};
end
assert(ischar(cmd) && numel(cmd) == 1, 'cmd la 1 ky tu lenh cua app, vd ''g''');
if isempty(o.dir)                                   % mac dinh: thu muc data canh rx_log.m
    o.dir = fullfile(fileparts(mfilename('fullpath')), 'data');
end
if ~exist(o.dir, 'dir')
    [okd, msg] = mkdir(o.dir);
    if ~okd, error('Khong tao duoc thu muc %s: %s', o.dir, msg); end
end
tag = regexprep(o.tag, '[^\w\-]', '_');
base = fullfile(o.dir, sprintf('%s_%s', datestr(now, 'yyyymmdd_HHMMSS'), cmd));
if ~isempty(tag), base = [base '_' tag]; end

try
    s = serialport(o.port, o.baud);
catch err
    error('Khong mo duoc %s: %s\nDong Tera Term (hoac chuong trinh khac dang giu cong) roi chay lai.', ...
          o.port, err.message);
end
cleanupObj = onCleanup(@() close_port(s));
io = struct('s', s, 'buf', '', 'raw', '');
pause(0.2);  io = getlines(io);  io.buf = '';  io.raw = '';      % bo du lieu cu

out = struct('cmd', cmd, 'files', {{}});
if any(cmd == 'gdm')
    [io, st] = status(io);
    out.status = st;
    fprintf('Board: gain_shift %d, rmin %d, test %d\n', st.gain_shift, st.rmin, st.test);
end

switch cmd
    %% ------------------------------------------------------------------ g
    case 'g'
        if st.test == 0
            fprintf('(Che do test dang TAT: chi dem duoc khung OK/CRC. Muon dem bit loi: rx_log(''k''))\n');
        end
        io = send(io, 'g');
        rows = zeros(0, 7);  t0 = tic;  tl = tic;
        fprintf('Dang thu %d khung ...\n', o.N);
        while size(rows, 1) < o.N && toc(t0) < o.secs && toc(tl) < o.idle
            [io, L] = getlines(io);
            for i = 1:numel(L)
                v = parse_frame(L{i});
                if ~isempty(v)
                    rows(end + 1, :) = v; %#ok<AGROW>
                    tl = tic;
                    print_frame(size(rows, 1), v);
                end
            end
            pause(0.05);
        end
        if toc(tl) >= o.idle, fprintf('(%.0f s khong nhan duoc khung nao -> dung)\n', o.idle); end
        io = send(io, 'x');
        [io, L] = wait_prompt(io, 5);
        sm = parse_summary(L);
        out.rows = rows;  out.summary = sm;
        csvf = [base '.csv'];
        write_rows(csvf, 't_ms,seq,crc_ok,cfo_hz,bit_err,bytes,raw_err', rows, ...
                   '%.0f,%.0f,%.0f,%.2f,%.0f,%.0f,%.0f');
        save_text([base '.log'], io.raw);
        out.files = {csvf, [base '.log']};
        n = size(rows, 1);  nok = sum(rows(:, 3));
        ber = sm.ber;                                   % tu dong 'test:' cua app
        if isnan(ber) && n && all(~isnan(rows(:, 5))), ber = sum(rows(:, 5)) / (1768 * n); end
        fprintf('\n%d khung: %d OK, %d loi CRC (FER %.1f%%)', n, nok, n - nok, 100 * (1 - nok / max(n, 1)));
        if ~isnan(ber), fprintf(', BER sau giai ma %.2e', ber); end
        if ~isnan(sm.chber), fprintf(', BER kenh (truoc FEC) %.2e', sm.chber); end
        if n, fprintf(', CFO %+.2f +/- %.2f Hz', mean(rows(:, 4)), std(rows(:, 4))); end
        fprintf('\n');
        if ~isempty(sm.detected)
            fprintf('Board: detected %d, demapped %d, I2S drops %d\n', sm.detected, sm.demapped, sm.drops);
        end
        add_run(o, cmd, st, n, nok, sm, ber, mean_or_nan(rows, 4), csvf);

    %% ------------------------------------------------------------------ d
    case 'd'
        io = send(io, 'd');
        io = wait_text(io, 'packets', 3);
        io = send(io, sprintf('%d\r', o.npkt));
        i0 = numel(io.raw);
        fprintf('Dang chup %d goi (%.1f s) va nhan qua UART (khoang %.0f s) ...\n', ...
                o.npkt, 0.512 * o.npkt, 4.5 * o.npkt);
        tl = tic;  tp = tic;  nprev = 0;
        while isempty(strfind(io.raw(max(1, i0 - 10):end), 'RAW_END')) && toc(tl) < o.idle
            io = getlines(io);  io.buf = '';
            if numel(io.raw) > nprev, nprev = numel(io.raw); tl = tic; end
            if toc(tp) > 5, fprintf('  %.0f kB\n', numel(io.raw) / 1024); tp = tic; end
            pause(0.05);
        end
        assert(~isempty(strfind(io.raw, 'RAW_END')), 'Khong nhan du dump (khong thay RAW_END).');
        [io, ~] = wait_prompt(io, 3);
        dumpf = [base '_dump.txt'];
        save_text(dumpf, io.raw);
        out.files = {dumpf};
        fprintf('Da luu %s\n', dumpf);
        nfr = NaN; nok = NaN; ber = NaN; cfo = NaN;
        if o.analyze
            R = ofdm_channel_report(dumpf, struct('tag', o.tag));
            out.report = R;
            nfr = R.n_frames; nok = R.crc_ok; ber = R.ber; cfo = R.cfo_mean_hz;
        end
        add_run(o, cmd, st, nfr, nok, parse_summary({}), ber, cfo, dumpf);

    %% ------------------------------------------------------------------ m
    case 'm'
        secs = o.secs;  if isinf(secs), secs = 60; end
        io = send(io, 'm');
        rows = zeros(0, 6);  t0 = tic;
        fprintf('Giam sat tone %.0f s ...\n', secs);
        while toc(t0) < secs
            [io, L] = getlines(io);
            for i = 1:numel(L)
                if strncmp(L{i}, 'M,', 2)
                    v = sscanf(L{i}(3:end), '%f,%f,%f,%f,%f,%f');
                    if numel(v) == 6
                        rows(end + 1, :) = v.'; %#ok<AGROW>
                        fprintf('%4d  %9.1f Hz  %6.1f dBFS  coh %.2f\n', v(1), v(2), v(3), v(6));
                    end
                end
            end
            pause(0.05);
        end
        io = send(io, 'x');
        [io, ~] = wait_prompt(io, 3);
        logf = [base '.log'];  csvf = [base '.csv'];
        save_text(logf, io.raw);
        write_rows(csvf, 'k,f_hz,dbfs,bb,peak,coh', rows, '%.0f,%.1f,%.1f,%.0f,%.0f,%.2f');
        out.rows = rows;  out.files = {csvf, logf};
        if o.analyze && size(rows, 1) > 2 && exist('read_tone_log', 'file')
            out.tone = read_tone_log(logf);
        end
        add_run(o, cmd, st, size(rows, 1), NaN, parse_summary({}), NaN, NaN, csvf);

    %% ------------------------------------------------- lenh 1 ky tu khac
    otherwise
        io = send(io, cmd);
        if ~isempty(o.value)
            io = wait_text(io, ']:', 3);
            io = send(io, sprintf('%g\r', o.value));
        end
        tmax = 4;  if cmd == 'n' || cmd == 'l', tmax = 8; end
        [io, L] = wait_prompt(io, tmax);
        fprintf('%s\n', L{:});
        out.lines = L;
        fid = fopen(fullfile(o.dir, 'session.log'), 'a');
        if fid > 0                                      % ghi log phien, khong bat buoc
            fprintf(fid, '--- %s  lenh %s ---\n%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), cmd, ...
                    strjoin(L, newline));
            fclose(fid);
        end
end
end

%% ======================================================================
function close_port(s)
try, delete(s); catch, end
end

function io = send(io, txt)
write(io.s, uint8(txt), 'uint8');
end

function [io, L] = getlines(io)
L = {};
n = io.s.NumBytesAvailable;
if n > 0
    d = char(read(io.s, n, 'uint8'));
    d = d(:).';
    io.raw = [io.raw d];
    io.buf = [io.buf d];
end
k = find(io.buf == char(10));
if ~isempty(k)
    chunk = io.buf(1:k(end));
    io.buf = io.buf(k(end) + 1:end);
    L = regexp(chunk, '\n', 'split');
    L = strrep(L(1:end - 1), char(13), '');
end
end

function [io, L] = wait_prompt(io, tmax)
% doc den khi app in lai dau nhac "> " (cuoi bo dem, chua xuong dong)
L = {};  t = tic;
while toc(t) < tmax
    [io, Li] = getlines(io);
    L = [L, Li]; %#ok<AGROW>
    if numel(io.buf) >= 2 && strcmp(io.buf(end - 1:end), '> '), break; end
    pause(0.05);
end
end

function io = wait_text(io, txt, tmax)
t = tic;
while toc(t) < tmax
    io = getlines(io);
    if ~isempty(strfind(io.buf, txt)) || ~isempty(strfind(io.raw(max(1, end - 200):end), txt)), return; end
    pause(0.05);
end
warning('Khong thay "%s" tu board', txt);
end

function [io, st] = status(io)
io = send(io, 's');
[io, L] = wait_prompt(io, 3);
st = struct('gain_shift', NaN, 'rmin', NaN, 'test', 0, 'line', '');
for i = 1:numel(L)
    if ~isempty(strfind(L{i}, 'gain_shift'))
        st.line = L{i};
        t = regexp(L{i}, 'gain_shift (\d+)', 'tokens', 'once');  if ~isempty(t), st.gain_shift = str2double(t{1}); end
        t = regexp(L{i}, 'rmin (\d+)', 'tokens', 'once');        if ~isempty(t), st.rmin = str2double(t{1}); end
        t = regexp(L{i}, 'test (\d)', 'tokens', 'once');         if ~isempty(t), st.test = str2double(t{1}); end
    end
end
end

function v = parse_frame(ln)
% tra ve [t_ms seq crc_ok cfo_hz bit_err bytes] hoac []
v = [];
if strncmp(ln, 'F,', 2)
    x = sscanf(ln(3:end), '%f,%f,%f,%f,%f,%f,%f');
    if numel(x) == 6, v = [x.', x(5)]; elseif numel(x) == 7, v = x.'; end
    return;
end
t = regexp(ln, '^\[(OK |BAD)\] t\s+(\d+) ms seq\s+(\d+)/\d+ len\s+\d+ CFO\s+([-+.\d]+) Hz\s+bytes (\d+)', ...
           'tokens', 'once');
if ~isempty(t)
    v = [str2double(t{2}), str2double(t{3}), strcmp(t{1}, 'OK '), str2double(t{4}), NaN, str2double(t{5}), NaN];
end
end

function print_frame(n, v)
if isnan(v(5)), e = ''; else, e = sprintf('  bit loi %4d', v(5)); end
if ~isnan(v(7)) && v(7) ~= v(5), e = sprintf('%s  (kenh sai %d, da sua)', e, v(7)); end
ok = 'CRC';  if v(3), ok = 'OK '; end
fprintf('%4d  seq %3d  %s  CFO %+6.2f Hz%s\n', n, v(2), ok, v(4), e);
end

function sm = parse_summary(L)
sm = struct('ok', [], 'bad', [], 'detected', [], 'demapped', [], 'drops', [], 'ber', NaN, 'chber', NaN);
for i = 1:numel(L)
    t = regexp(L{i}, '(\d+) OK, (\d+) CRC errors \| detected (\d+), demapped (\d+), I2S drops (\d+)', ...
               'tokens', 'once');
    if ~isempty(t)
        x = str2double(t);
        sm.ok = x(1); sm.bad = x(2); sm.detected = x(3); sm.demapped = x(4); sm.drops = x(5);
    end
    t = regexp(L{i}, 'test: (\d+) bit errors / (\d+) bits', 'tokens', 'once');
    if ~isempty(t), sm.ber = str2double(t{1}) / str2double(t{2}); end
    t = regexp(L{i}, 'fec: (\d+) channel bit errors corrected / (\d+) coded bits', 'tokens', 'once');
    if ~isempty(t), sm.chber = str2double(t{1}) / str2double(t{2}); end
end
end

function write_rows(f, hdr, rows, fmt)
fid = fopen_chk(f, 'w');
fprintf(fid, '%s\n', hdr);
fprintf(fid, [fmt '\n'], rows.');
fclose(fid);
fprintf('Da luu %s (%d dong)\n', f, size(rows, 1));
end

function save_text(f, txt)
fid = fopen_chk(f, 'w');  fwrite(fid, txt, 'char');  fclose(fid);
end

function m = mean_or_nan(rows, c)
if isempty(rows), m = NaN; else, m = mean(rows(:, c)); end
end

function add_run(o, cmd, st, n, nok, sm, ber, cfo, file)
f = fullfile(o.dir, 'runs.csv');
newf = ~exist(f, 'file');
fid = fopen_chk(f, 'a');
if newf
    fprintf(fid, ['time,cmd,tag,dist_m,depth_m,gain_shift,rmin,test,n_frames,crc_ok,' ...
                  'detected,demapped,i2s_drops,ber,cfo_mean_hz,file,note\n']);
end
fprintf(fid, '%s,%s,%s,%g,%g,%g,%g,%g,%g,%g,%s,%s,%s,%.3g,%.3g,%s,"%s"\n', ...
    datestr(now, 'yyyy-mm-dd HH:MM:SS'), cmd, o.tag, o.dist_m, o.depth_m, st.gain_shift, ...
    st.rmin, st.test, n, nok, num2s(sm.detected), num2s(sm.demapped), num2s(sm.drops), ...
    ber, cfo, file, o.note);
fclose(fid);
end

function s = num2s(x)
if isempty(x), s = ''; else, s = sprintf('%d', x); end
end

function fid = fopen_chk(f, mode)
[fid, msg] = fopen(f, mode);
if fid < 0
    error('Khong ghi duoc file %s (%s). File dang mo trong Excel? Thu muc chi doc?', f, msg);
end
end