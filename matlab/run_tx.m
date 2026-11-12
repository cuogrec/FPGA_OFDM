%% ======================================================================
%  run_tx.m -- BO PHAT OFDM 33 kHz (MATLAB -> Focusrite Solo Gen 3 -> BII-7511H)
%  Sua cac dong trong muc THIET LAP roi bam Run. Moi lan chay:
%    1) tao khung tu van ban  2) ghi WAV  3) (tuy chon) phat qua Focusrite
%    4) (tuy chon) tu giai lai bang bo thu tham chieu de kiem tra
%  ======================================================================
clear; clc;
P = ofdm_params();

%% ---------------- THIET LAP (sua o day) ----------------
S.mode        = 'text';        % 'text' van ban | 'test' khung thu (dem bit loi) | 'tone' | 'sweep'
S.fec         = true;         % true = ma chap 1/2 + xen ke (board: lenh e). Van ban 106 byte/khung
S.test_frames = 50;            % so khung moi lan phat o che do 'test' (seq 0..49, toi da 256)
S.text        = 'Hello ZCU104! Thu nghiem OFDM thuy am 33 kHz.';
S.repeat      = 5;             % so lan lap lai toan bo ban tin
S.gap_ms      = 150;           % khoang lang giua cac khung (cho tieng vang tat dan)
S.preroll_ms  = 300;           % im lang dau file (card am thanh khoi dong)
S.amp         = 0.9;           % bien do dinh (0..1 full scale). Tang dan neu thu yeu
S.papr_clip_db= Inf;           % cat dinh baseband, vd 8 (dB tren RMS); Inf = tat
S.fc          = P.fc;          % song mang - FPGA co dinh 33 kHz, chi doi de thu nghiem
S.tone_s      = 10;            % thoi luong che do 'tone' / 'sweep' (s)
S.sweep_hz    = [25000 41000]; % dai quet cho 'sweep'

S.save_wav    = true;
S.wav_file    = 'ofdm_tx.wav';
S.play        = true;          % false = chi tao file
S.device      = 'Scarlett';   % chuoi tim trong ten thiet bi (regexp, khong phan biet hoa)
S.driver      = '';            % '' = mac dinh; 'ASIO' neu dung Audio Toolbox + Focusrite ASIO
S.use_toolbox = true;          % dung audioDeviceWriter neu co Audio Toolbox
S.out_ch      = 1;             % 1 = Output 1, 2 = Output 2, 0 = ca hai
S.loop        = 1;             % so lan phat lai file (Inf = lien tuc, Ctrl+C de dung)

S.selftest    = true;          % giai lai tin hieu vua tao (co nhieu + da duong) truoc khi phat
S.selftest_snr= 30;            % dB (kenh 4 tia khac nghiet; chua co FEC nen 1 bit loi = CRC loi)

%% ---------------- TAO TIN HIEU ----------------
switch S.mode
    case 'text'
        if S.fec, [pl, plinfo] = ofdm_fec_payloads('text', S.text, P);
        else,     pl = ofdm_text_to_payloads(S.text, P); end
        [y, info] = ofdm_tx_signal(pl, P, S);
        fprintf('Van ban %d byte -> %d khung x %d lan = %d khung, %.2f s, khung %.0f ms, PAPR %.1f dB\n', ...
            numel(unicode2native(S.text, 'UTF-8')), size(pl,1), S.repeat, info.nframes, ...
            info.duration, info.frame_ms, info.papr_db);
    case 'test'
        if S.fec, [pl, plinfo] = ofdm_fec_payloads('test', S.test_frames, P);
        else,     pl = ofdm_test_payloads(S.test_frames, P); end
        [y, info] = ofdm_tx_signal(pl, P, S);
        fprintf('Khung thu nghiem: %d khung x %d lan = %d khung, %.2f s (board: k%s roi g)\n', ...
            size(pl,1), S.repeat, info.nframes, info.duration, repmat(', e', 1, S.fec));
    case {'tone', 'sweep'}
        y = ofdm_tone(P, S);
        fprintf('%s, %.1f s, bien do %.2f\n', S.mode, S.tone_s, S.amp);
    otherwise
        error('S.mode = text | test | tone | sweep');
end

if S.save_wav
    audiowrite(S.wav_file, y, P.fs, 'BitsPerSample', 24);
    fprintf('Da ghi %s (96 kHz, 24 bit, mono)\n', S.wav_file);
end

%% ---------------- TU KIEM TRA ----------------
if S.selftest && strcmp(S.mode, 'text') && S.fec
    ch = zeros(round(2e-3*P.fs),1); ch([1 37 90 150]) = [1 0.5 -0.3 0.15];
    r  = conv(y, ch);  r = r(1:numel(y));
    pw = mean(r(r ~= 0).^2);
    r  = r + sqrt(pw / 10^(S.selftest_snr/10) * (P.fs/2) / 4000) * randn(size(r));
    res = ofdm_rx_ref(r, P, 'passband');
    nok = 0;  nfix = zeros(1, numel(res));  txt = '';
    for k = 1:numel(res)
        [q, ok, nfix(k)] = ofdm_fec_unframe(res(k).bytes);
        nok = nok + ok;
        if ok && k <= size(pl, 1), txt = [txt char(q(4:3+min(q(3),106)))]; end %#ok<AGROW>
    end
    fprintf('Tu kiem tra FEC (SNR %g dB, 4 tia): %d khung, CRC OK %d, bit kenh da sua/khung: %s\n', ...
        S.selftest_snr, numel(res), nok, mat2str(nfix));
    if ~isempty(txt), fprintf('  -> "%s"\n', txt); end
elseif S.selftest && strcmp(S.mode, 'text')
    ch = zeros(round(2e-3*P.fs),1); ch([1 37 90 150]) = [1 0.5 -0.3 0.15];   % kenh xo nuoc gia lap
    r  = conv(y, ch);  r = r(1:numel(y));
    pw = mean(r(r ~= 0).^2);
    r  = r + sqrt(pw / 10^(S.selftest_snr/10) * (P.fs/2) / 4000) * randn(size(r));
    res = ofdm_rx_ref(r, P, 'passband');
    ref = ofdm_payload_to_bits(pl(1,:), P);  ber = zeros(1, numel(res));
    for k = 1:numel(res)
        [~, j] = min(abs(double([pl(:,1)]) - double(res(k).seq)));   % khung goc theo seq
        ber(k) = sum(ofdm_payload_to_bits(res(k).bytes, P) ~= ofdm_payload_to_bits(pl(j,:), P));
    end
    fprintf('Tu kiem tra (SNR %g dB, 4 tia): %d/%d khung, CRC OK %d, bit loi/khung: %s, SNR do %.1f dB\n', ...
        S.selftest_snr, numel(res), info.nframes, sum([res.crc_ok]), mat2str(ber), mean([res.snr_db]));
    if ~isempty(res), fprintf('  -> "%s"\n', ofdm_join_text(res)); end
end

%% ---------------- PHAT ----------------
if S.play
    k = 0;
    while k < S.loop
        k = k + 1;
        fprintf('Phat lan %d ...\n', k);
        ofdm_play(y, P, S);
    end
end