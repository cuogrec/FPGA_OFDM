function res = ofdm_rx_ref(x, P, kind, opt)
% OFDM_RX_REF  Bo thu tham chieu (float) -- cung thuat toan voi FPGA.
%   x    : tin hieu thu
%   kind : 'passband' (thuc, 96 kHz: WAV / ghi am)  |  'baseband' (phuc, 8 kHz: dma_raw)
%   opt  : .thr (0.6) .toffs (0) .plot (false) .fc (P.fc)
%   res  : mang struct moi khung: seq nfrag len text crc_ok cfo_hz snr_db start
if nargin < 4, opt = struct(); end
thr   = getopt(opt, 'thr', 150/256);
toffs = getopt(opt, 'toffs', 0);
fc    = getopt(opt, 'fc', P.fc);
doplot= getopt(opt, 'plot', false);
N = P.N; CP = P.CP; L = N/2; SL = N + CP;

% ---- DDC: tron -fc, loc thong thap, chia 12 ----
if strcmp(kind, 'passband')
    x  = double(x(:, 1));
    n  = (0:numel(x)-1).';
    xm = x .* exp(-2j*pi*fc/P.fs*n);
    K  = 16; m = (-K*P.dec:K*P.dec).'; c = 3000/P.fs;
    h  = 2*c*sincf(2*c*m) .* (0.54 + 0.46*cos(pi*m/(K*P.dec)));  h = h/sum(h);
    y  = conv(xm, h);  y = y(K*P.dec+1 : K*P.dec+numel(xm));
    r  = y(1:P.dec:end);
else
    r  = x(:);
end

% ---- Schmidl & Cox: P(d), R(d) (R tren ca cua so 2L, giong RTL) ----
nr = numel(r);
if nr < 2*L + 3*SL, res = struct([]); return; end
pr = conj(r(1:end-L)) .* r(1+L:end);
cP = [0; cumsum(pr)];  Pd = cP(1+L:end) - cP(1:end-L);            % d = 1..nr-2L+1
e  = abs(r).^2;  cE = [0; cumsum(e)];  Rd = cE(1+2*L:end) - cE(1:end-2*L);
nd = min(numel(Pd), numel(Rd));  Pd = Pd(1:nd);  Rd = Rd(1:nd);
M  = 2*abs(Pd) ./ max(Rd, eps);
rmin = 4 * prctile_(Rd, 10);
above = (M >= thr) & (Rd >= rmin);

res = struct('seq', {}, 'nfrag', {}, 'len', {}, 'text', {}, 'crc_ok', {}, ...
             'cfo_hz', {}, 'snr_db', {}, 'start', {}, 'bytes', {}, ...
             'H', {}, 'zd', {}, 'pil', {});
% H   : 64 x 1, kenh uoc luong tu LTF (thu tu P.active)
% zd  : 56 x 16, ky hieu du lieu sau can bang (cot = symbol)
% pil : 8 x 16, Y_pilot / H_LTF moi symbol (=1 neu kenh khong doi trong khung)
d = 1;  Zall = [];
while d <= nd
    if ~above(d), d = d + 1; continue; end
    d1 = d; d2 = d; below = 0; [pb, ib] = deal(abs(Pd(d)), d);
    while d < nd && below < CP/2 && d - d1 <= 4*CP + 2*L
        d = d + 1;
        if above(d), d2 = d; below = 0; if abs(Pd(d)) > pb, pb = abs(Pd(d)); ib = d; end
        else, below = below + 1; end
    end
    if d - d1 > 4*CP + 2*L, while d <= nd && above(d), d = d + 1; end, continue; end
    n0  = floor((d1 + d2)/2) + CP/4 + toffs;          % cua so FFT cua preamble
    if n0 + SL*(P.NSYM+2) - 1 > nr, break; end
    phi = angle(Pd(ib));                               % = pi * eps
    nn  = (0:SL*(P.NSYM+2)-1).';
    seg = r(n0 + nn) .* exp(-1j * phi / L * nn);        % bu CFO

    Y   = @(s) fft(seg(s*SL + (1:N)));                % s = 1: LTF, 2..: data
    ia  = mod(P.active, N) + 1;
    Yl  = Y(1);  H = zeros(N,1);  H(ia) = Yl(ia) .* P.ltf(:);
    bits = zeros(P.bits_per_frame, 1);  Zf = [];  pil = zeros(numel(P.pilots), P.NSYM);
    ip  = mod(P.pilots, N) + 1;
    for s = 1:P.NSYM
        Ys = Y(s+1);  Z = Ys .* conj(H);
        zp = Z(mod(P.pilots, N) + 1);                  % pilot
        pil(:, s) = Ys(ip) ./ H(ip);
        C  = zeros(N,1);
        for f = P.data
            ka = find(P.pilots <= f, 1, 'last');  if isempty(ka), ka = 1; end
            ka = min(ka, numel(P.pilots) - 1);
            a  = min(max((f - P.pilots(ka)) / 8, 0), 1);   % bien bang: pilot gan nhat (giong RTL)
            C(mod(f,N)+1) = (1-a)*zp(ka) + a*zp(ka+1);
        end
        zd = Z(mod(P.data, N) + 1) .* conj(C(mod(P.data, N) + 1));
        zd = zd ./ abs(C(mod(P.data,N)+1)) ./ max(abs(H(mod(P.data,N)+1)).^2, eps);
        Zf = [Zf; zd]; %#ok<AGROW>
        b  = [real(zd) < 0, imag(zd) < 0].';
        bits((s-1)*112 + (1:112)) = b(:);
    end
    q  = (sign(real(Zf)) + 1j*sign(imag(Zf))) / sqrt(2);
    snr = 10*log10(mean(abs(q).^2) / mean(abs(Zf*sqrt(1/mean(abs(Zf).^2)) - q).^2));
    k = numel(res) + 1;
    t = ofdm_parse_payload(ofdm_bits_to_payload(bits, P));
    t.cfo_hz = phi / pi * P.fsb / N;
    t.snr_db = snr;
    t.start  = n0;
    t.H      = H(ia);
    t.zd     = reshape(Zf, numel(P.data), P.NSYM);
    t.pil    = pil;
    res(k) = orderfields(t, res);
    Zall = [Zall; Zf]; %#ok<AGROW>
    d = n0 + SL*(P.NSYM+2) - SL;                       % tiep tuc sau khung
end
if doplot && ~isempty(Zall)
    figure; subplot(1,2,1); plot(M); hold on; plot([1 nd], thr*[1 1], 'r--');
    title('Metric Schmidl-Cox'); xlabel('mau 8 kHz');
    subplot(1,2,2); plot(Zall/sqrt(mean(abs(Zall).^2)), '.'); axis equal; grid on;
    title('Chom sao sau can bang');
end
end

function v = getopt(o, f, d), if isfield(o, f), v = o.(f); else, v = d; end, end
function s = sincf(x), s = ones(size(x)); i = x ~= 0; s(i) = sin(pi*x(i))./(pi*x(i)); end
function p = prctile_(v, q), v = sort(v); p = v(max(1, round(q/100*numel(v)))); end
