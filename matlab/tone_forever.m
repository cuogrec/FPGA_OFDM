%% ======================================================================
%  tone_forever.m -- PHAT SIN LIEN TUC (VO HAN) DE THU DAU DO / TRANSDUCER
%  MATLAB -> Focusrite Scarlett Solo Gen 3 (96 kHz) -> (amp) -> BII-7511H
%  Dung bang Ctrl+C (da co onCleanup tu giai phong card am thanh).
%
%  Hai che do:
%    'fixed' : phat mot tan so co dinh F.freq (vd 33 kHz) mai mai
%    'step'  : cu moi F.step_s giay doi sang tan so ke tiep trong F.step_list,
%              lap vo han -> nghe/do bien do tung tan so de tim cong huong thuc te
%  Pha lien tuc giua cac khoi nen khong bi "tách tách" khi doi tan so.
%  ======================================================================
clear; clc;

%% ---------------- THIET LAP (sua o day) ----------------
F.mode      = 'fixed';              % 'fixed' | 'step'
F.freq      = 33000;                % Hz, dung cho che do 'fixed'
F.step_list = 29000:500:37000;      % Hz, dung cho che do 'step'
F.step_s    = 4;                    % giay moi buoc (che do 'step')
F.amp       = 1;                  % bien do dinh 0..1 (giong S.amp trong run_tx.m)
F.ramp_ms   = 50;                   % tang dan bien do luc bat dau, tranh tieng "bup"
F.out_ch    = 1;                    % 1 = Output 1, 2 = Output 2, 0 = ca hai
F.device    = 'Scarlett';          % chuoi tim trong ten thiet bi (regexp)
F.driver    = '';                   % '' = mac dinh; 'ASIO' neu co Audio Toolbox + ASIO
F.fs        = 96000;                % phai bang fs cua FPGA (96 kHz)
F.blk       = 4096;                 % so mau moi khoi (~42.7 ms)

%% ---------------- KIEM TRA ----------------
assert(F.amp > 0 && F.amp <= 1, 'F.amp phai trong (0, 1]');
if strcmp(F.mode, 'fixed'), fl = F.freq; else, fl = F.step_list; end
assert(all(fl > 0 & fl < F.fs/2), 'Tan so phai nam trong (0, fs/2 = %d Hz)', F.fs/2);
fprintf('Che do: %s | bien do %.2f (%.1f dBFS) | fs = %d Hz\n', ...
    F.mode, F.amp, 20*log10(F.amp), F.fs);

%% ---------------- MO THIET BI ----------------
args = {'SampleRate', F.fs, 'BitDepth', '24-bit integer'};
if ~isempty(F.driver), args = [args, {'Driver', F.driver}]; end
w   = audioDeviceWriter(args{:});
dev = getAudioDevices(w);
k   = find(~cellfun(@isempty, regexpi(dev, F.device)), 1);
if isempty(k)
    error('Khong tim thay thiet bi "%s". Co: %s', F.device, strjoin(dev, ' | '));
end
w.Device = dev{k};
fprintf('Phat qua: %s (%s)\n', dev{k}, w.Driver);
cleanupObj = onCleanup(@() release(w));   % Ctrl+C -> tu dong giai phong card

%% ---------------- VONG PHAT VO HAN ----------------
n      = (0:F.blk-1).';
ph     = 0;                               % pha tich luy (rad), lien tuc giua cac khoi
g      = 0;                               % he so tang dan dau phat
g_inc  = 1 / max(1, round(F.ramp_ms*1e-3*F.fs));
idx    = 1;                               % chi so tan so hien tai ('step')
f      = fl(1);
t_step = 0;                               % thoi gian da o tan so hien tai
nblk   = 0;
fprintf('Dang phat %.0f Hz ... (Ctrl+C de dung)\n', f);

while true
    % --- doi tan so o che do 'step'
    if strcmp(F.mode, 'step') && t_step >= F.step_s
        idx = mod(idx, numel(fl)) + 1;
        f = fl(idx);  t_step = 0;
        fprintf('  -> %.0f Hz\n', f);
    end

    % --- tao khoi sin lien pha
    x = sin(ph + 2*pi*f/F.fs * n);
    ph = mod(ph + 2*pi*f/F.fs * F.blk, 2*pi);

    % --- tang dan bien do (chi o luc dau)
    if g < 1
        gg = min(1, g + g_inc*(1:F.blk).');
        g  = gg(end);
        x  = x .* gg;
    end
    x = F.amp * x;

    if F.out_ch == 0, y2 = [x x];
    else, y2 = zeros(F.blk, 2); y2(:, F.out_ch) = x; end

    nu = w(y2);
    if nu > 0 && nblk > 2
        warning('Underrun %d mau (card am thanh bi doi)', nu);
    end
    t_step = t_step + F.blk / F.fs;
    nblk   = nblk + 1;
end
