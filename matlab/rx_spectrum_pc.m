%% ======================================================================
%  rx_spectrum_pc.m -- THU SONG BANG MAY TINH, TAN SO TUY Y
%  Dau do thu / hydrophone -> (tien khuech dai) -> NGO VAO Focusrite -> MATLAB
%  Hien pho truc tiep, tu tim dinh manh nhat trong dai F.band va do:
%     tan so (Hz), bien do (dBFS), PNR (dB). Dong cua so de dung va luu log.
%  Phat bang tone_forever.m tren Output, thu o Input -> thu duoc moi tan so < fs/2.
%
%  Luu y Focusrite Solo Gen 3:
%    * Dat 96 kHz trong Focusrite Control (hoac Windows Sound > Advanced); co the
%      dat 192 kHz de thu toi 96 kHz (doi F.fs).
%    * Input 1 co preamp mic (XLR) / Input 2 co Line-Inst. Nut GAIN + che do
%      Air/Inst: chinh sao cho vach "clip" khong sang va muc ~ -20..-6 dBFS.
%    * Dau vao audio co the loc DC / thong cao ~ vai Hz: khong anh huong 20-45 kHz,
%      nhung dap ung phan cung cua card tren 30 kHz khong phang tuyet doi.
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP (sua o day) ----------------
F.fs       = 96000;           % Hz, phai trung voi toc do lay mau cua card
F.band     = [20000 45000];   % Hz, dai tim dinh (bat ky, < fs/2)
F.in_ch    = 1;               % kenh input cua card: 1 hoac 2
F.win_s    = 0.5;             % do dai cua so phan tich (s). Dai -> chinh xac tan so cao hon
F.hop_s    = 0.25;            % chu ky cap nhat man hinh (s)
F.device   = 'Focusrite';     % chuoi tim trong ten thiet bi (regexp)
F.driver   = '';              % '' = mac dinh; 'ASIO' neu co
F.blk      = 2048;            % so mau moi lan doc
F.hist_s   = 60;              % lich su hien thi (s)
F.log_file = 'rx_log.csv';    % luu t, f, dBFS, PNR khi dung ('' = khong luu)
F.min_pnr  = 10;              % dB: duoi muc nay coi la "khong co tone"

%% ---------------- MO THIET BI ----------------
assert(F.band(2) < F.fs/2, 'F.band phai nho hon fs/2 = %d Hz', F.fs/2);
args = {'SampleRate', F.fs, 'SamplesPerFrame', F.blk, ...
        'ChannelMappingSource', 'Property', 'ChannelMapping', F.in_ch};
if ~isempty(F.driver), args = [args, {'Driver', F.driver}]; end
rd  = audioDeviceReader(args{:});
dev = getAudioDevices(rd);
k   = find(~cellfun(@isempty, regexpi(dev, F.device)), 1);
if isempty(k), error('Khong tim thay thiet bi "%s". Co: %s', F.device, strjoin(dev, ' | ')); end
rd.Device = dev{k};
fprintf('Thu qua: %s (%s), %d Hz, input %d\n', dev{k}, rd.Driver, F.fs, F.in_ch);
cleanupObj = onCleanup(@() release(rd));     % Ctrl+C van giai phong card

%% ---------------- GIAO DIEN ----------------
fig = figure('Name', 'Thu song - may tinh', 'Color', 'w', 'NumberTitle', 'off', ...
             'Position', [100 100 900 650]);
setappdata(fig, 'stop', false);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Dung + luu', 'Position', [10 10 100 28], ...
          'Callback', @(~, ~) setappdata(fig, 'stop', true));
ax1 = subplot(2, 1, 1); hold(ax1, 'on'); grid(ax1, 'on');
hs  = plot(ax1, NaN, NaN, 'b');  hp = plot(ax1, NaN, NaN, 'rv', 'MarkerFaceColor', 'r');
xlabel(ax1, 'Tan so (kHz)'); ylabel(ax1, 'Bien do (dBFS)');
ax2 = subplot(2, 1, 2); hold(ax2, 'on'); grid(ax2, 'on');
yyaxis(ax2, 'left');  hf = plot(ax2, NaN, NaN, 'b.-'); ylabel(ax2, 'Tan so (kHz)');
yyaxis(ax2, 'right'); hl = plot(ax2, NaN, NaN, 'r.-'); ylabel(ax2, 'Muc (dBFS)');
xlabel(ax2, 'Thoi gian (s)');

%% ---------------- VONG THU ----------------
Nw    = round(F.win_s * F.fs);
Nh    = round(F.hop_s * F.fs);
buf   = zeros(Nw, 1);
nsince = 0;  filled = 0;  t = 0;
T = []; Fq = []; Lv = []; Pn = [];
for i = 1:3, rd(); end                        % bo vai khoi dau (card khoi dong)
fprintf('Dang thu... dong cua so hoac bam "Dung + luu" de dung.\n');

while ishandle(fig) && ~getappdata(fig, 'stop')
    [x, nover] = rd();
    if nover > 0, fprintf('  overrun %d mau\n', nover); end
    x     = x(:, 1);
    buf   = [buf(numel(x)+1:end); x];
    filled = min(Nw, filled + numel(x));
    nsince = nsince + numel(x);
    t      = t + numel(x) / F.fs;
    if filled < Nw || nsince < Nh, continue; end
    nsince = 0;

    [f, a, pnr, sp] = tone_estimate(buf, F.fs, F.band);
    clipped = max(abs(buf)) > 0.98;
    ok      = pnr >= F.min_pnr;
    if ~ok, f = NaN; end

    T(end+1) = t;  Fq(end+1) = f;  Lv(end+1) = a;  Pn(end+1) = pnr; %#ok<SAGROW>
    set(hs, 'XData', sp.f / 1e3, 'YData', sp.db);
    if ok, set(hp, 'XData', f / 1e3, 'YData', a); else, set(hp, 'XData', NaN, 'YData', NaN); end
    xlim(ax1, F.band / 1e3);
    m = T >= t - F.hist_s;
    set(hf, 'XData', T(m), 'YData', Fq(m) / 1e3);
    set(hl, 'XData', T(m), 'YData', Lv(m));
    if ok
        title(ax1, sprintf('f = %.1f Hz | %.1f dBFS | PNR %.0f dB%s', f, a, pnr, ...
              iff(clipped, '  !! CLIP: giam GAIN', '')));
    else
        title(ax1, sprintf('khong thay tone (PNR %.0f dB < %.0f)%s', pnr, F.min_pnr, ...
              iff(clipped, '  !! CLIP', '')));
    end
    drawnow limitrate;
end

%% ---------------- KET THUC ----------------
if ishandle(fig), close(fig); end
release(rd);
if ~isempty(F.log_file) && ~isempty(T)
    fid = fopen(F.log_file, 'w');
    fprintf(fid, 't_s,f_Hz,level_dBFS,pnr_dB\n');
    fprintf(fid, '%.3f,%.2f,%.2f,%.1f\n', [T; Fq; Lv; Pn]);
    fclose(fid);
    fprintf('Da luu %d dong vao %s\n', numel(T), F.log_file);
end

function r = iff(c, a, b)
if c, r = a; else, r = b; end
end
