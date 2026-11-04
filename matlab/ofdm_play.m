function ofdm_play(y, P, S)
% OFDM_PLAY  Phat qua Focusrite Solo (Gen 3) o 96 kHz.
%   Uu tien audioDeviceWriter (Audio Toolbox, ho tro ASIO); neu khong co thi
%   dung audioplayer cua MATLAB co ban, chon thiet bi theo ten S.device.
%   S.out_ch : 1 = Output 1 (L), 2 = Output 2 (R), 0 = ca hai
y2 = zeros(numel(y), 2);
if S.out_ch == 0, y2 = [y y]; else, y2(:, S.out_ch) = y; end

if S.use_toolbox && exist('audioDeviceWriter', 'class') == 8
    args = {'SampleRate', P.fs, 'BitDepth', '24-bit integer'};
    if ~isempty(S.driver), args = [args, {'Driver', S.driver}]; end
    w   = audioDeviceWriter(args{:});
    dev = getAudioDevices(w);
    k   = find(~cellfun(@isempty, regexpi(dev, S.device)), 1);
    if isempty(k), error('Khong tim thay thiet bi "%s". Co: %s', S.device, strjoin(dev, ' | ')); end
    w.Device = dev{k};
    fprintf('Phat qua audioDeviceWriter: %s (%s)\n', dev{k}, w.Driver);
    blk = 4096;
    for i = 1:blk:numel(y)
        u = y2(i:min(i+blk-1, end), :);
        if size(u,1) < blk, u(blk,:) = 0; end
        nu = w(u);
        if nu > 0, warning('Underrun %d mau', nu); end
    end
    pause(0.2); release(w);
else
    info = audiodevinfo;
    id = -1;
    for k = 1:numel(info.output)
        if ~isempty(regexpi(info.output(k).Name, S.device)), id = info.output(k).ID; break; end
    end
    if id < 0
        warning('Khong thay "%s" -> dung thiet bi mac dinh', S.device);
        p = audioplayer(y2, P.fs, 24);
    else
        fprintf('Phat qua audioplayer: %s\n', info.output(k).Name);
        p = audioplayer(y2, P.fs, 24, id);
    end
    playblocking(p);
end
end
