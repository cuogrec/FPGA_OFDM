function [f, a_dbfs, pnr_db, spec] = tone_estimate(x, fs, band)
% TONE_ESTIMATE  Do tan so, bien do va "do sach" cua mot tone trong dai band.
%   [f, a_dbfs, pnr_db, spec] = tone_estimate(x, fs, [f1 f2])
%   x       : tin hieu thuc (vector), thang -1..1 (1 = full scale cua card am thanh)
%   band    : [f1 f2] Hz, chi tim dinh trong dai nay (tan so tuy y < fs/2)
%   f       : tan so dinh (Hz), noi suy parabol -> chinh xac nho hon 1 bin FFT
%   a_dbfs  : bien do dinh cua tone, dBFS (sin bien do 1.0 = 0 dBFS)
%   pnr_db  : dinh so voi nen nhieu trung binh moi bin FFT (dB); ~ SNR cua tone
%   spec    : struct(f, db) pho trong dai band, de ve
x    = x(:) - mean(x);
N    = numel(x);
w    = 0.5 - 0.5 * cos(2*pi*(0:N-1).' / N);          % Hann
nfft = 2^nextpow2(4 * N);                            % zero-pad x4
X    = fft(x .* w, nfft);
nh   = nfft/2 + 1;
mag  = abs(X(1:nh)) * 2 / sum(w);                    % bien do dinh cua sin
fax  = (0:nh-1).' * fs / nfft;

sel  = find(fax >= band(1) & fax <= band(2));
assert(numel(sel) > 8, 'band qua hep / ngoai (0, fs/2)');
[~, j] = max(mag(sel));  i = sel(j);
if i > 1 && i < nh
    a = log(mag(i-1) + eps); b = log(mag(i) + eps); c = log(mag(i+1) + eps);
    d = 0.5 * (a - c) / (a - 2*b + c);
    f = (i - 1 + d) * fs / nfft;
    amp = exp(b - 0.25 * (a - c) * d);
else
    f = fax(i); amp = mag(i);
end
a_dbfs = 20 * log10(amp + eps);

guard = ceil(4 * nfft / N);                           % bo +-4 bin goc (thuy chinh Hann)
ns    = sel(abs(sel - i) > guard);
if numel(ns) > 8
    nz = median(mag(ns).^2) / log(2);                 % trung vi -> trung binh (chi-binh phuong)
    pnr_db = 10 * log10(amp^2 / nz);
else
    pnr_db = NaN;
end
spec.f  = fax(sel);
spec.db = 20 * log10(mag(sel) + eps);
end
