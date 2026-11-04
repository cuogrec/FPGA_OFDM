function y = ofdm_upconvert(bb, P, fc)
% OFDM_UPCONVERT  baseband 8 kHz -> noi suy x12 (loc sinc-Hamming) -> song mang thuc 96 kHz.
%   Khong can toolbox.
if nargin < 3, fc = P.fc; end
L   = P.dec;  K = 16;                          % 2*K*L+1 = 385 tap
n   = (-K*L:K*L).';
fcut = 3000 / P.fs;                            % bang tin hieu +/-2 kHz
h   = 2*fcut*sinc_(2*fcut*n) .* (0.54 + 0.46*cos(pi*n/(K*L)));
h   = h / sum(h) * L;
up  = zeros(numel(bb)*L, 1);  up(1:L:end) = bb;
up  = conv(up, h);  up = up(K*L+1 : K*L+numel(bb)*L);      % bu tre nhom
t   = (0:numel(up)-1).' / P.fs;
y   = real(up .* exp(2j*pi*fc*t));
end

function s = sinc_(x)
s = ones(size(x)); i = x ~= 0; s(i) = sin(pi*x(i)) ./ (pi*x(i));
end
