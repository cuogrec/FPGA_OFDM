function P = ofdm_params()
% OFDM_PARAMS  Tham so co dinh cua khung OFDM 33 kHz.
%   PHAI khop voi bo thu FPGA (rtl/*.v) -- khong sua cac gia tri o day
%   tru khi sua ca RTL. Cac tuy chon thu nghiem nam trong run_tx.m.
P.fs      = 96000;            % Focusrite / PCM1808 sample rate
P.fc      = 33000;            % song mang (DDS FPGA co dinh 33 kHz)
P.dec     = 12;               % 96 kHz / 12 = 8 kHz baseband
P.fsb     = P.fs / P.dec;
P.N       = 128;              % FFT
P.CP      = 32;               % cyclic prefix (4 ms)
P.NSYM    = 16;               % symbol du lieu / khung
P.active  = [-32:-1, 1:32];   % 64 subcarrier, 62.5 Hz
P.pilots  = [-28 -20 -12 -4 4 12 20 28];
P.data    = setdiff(P.active, P.pilots);         % 56, tang dan theo f
P.bits_per_frame  = P.NSYM * numel(P.data) * 2;  % 1792
P.bytes_per_frame = P.bits_per_frame / 8;        % 224
% Dinh dang payload 224 byte:
%   [0] seq  [1] tong so khung cua ban tin  [2] so byte text L (<=219)
%   [3..221] text (UTF-8, phan du = 0)  [222..223] CRC-16/CCITT (big-endian)
P.hdr_bytes  = 3;
P.text_bytes = P.bytes_per_frame - P.hdr_bytes - 2;   % 219
pn = ofdm_pn9(128, 511);
P.ltf     = 1 - 2*pn(1:64);           % BPSK, theo thu tu P.active
P.sc_bits = pn(65:128);               % QPSK tren subcarrier chan (preamble S&C)
P.whiten  = ofdm_pn9(P.bits_per_frame, 341);  % chuoi lam trang du lieu (seed 0x155)
end
