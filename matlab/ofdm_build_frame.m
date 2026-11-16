function x = ofdm_build_frame(bits, P)
% OFDM_BUILD_FRAME  1792 bit -> baseband phuc 8 kHz:
%   [CP|S&C preamble] [CP|LTF] [CP|DATA] x 16      (2880 mau, 360 ms)
%   P.ltf_rep = M > 0: chen them [CP|LTF] sau moi M symbol du lieu (M = 4: 3360 mau, 420 ms)
assert(numel(bits) == P.bits_per_frame);
even = P.active(mod(P.active, 2) == 0);
sc   = qpsk(P.sc_bits) * sqrt(2);               % chi subcarrier chan -> 2 nua giong nhau
x    = [sym(even, sc, P); sym(P.active, P.ltf, P)];
q    = reshape(qpsk(bits(:)), numel(P.data), P.NSYM);   % cot = 1 symbol
for s = 1:P.NSYM
    if P.ltf_rep > 0 && s > 1 && mod(s - 1, P.ltf_rep) == 0     % LTF lap (rev 3)
        x = [x; sym(P.active, P.ltf, P)]; %#ok<AGROW>
    end
    x = [x; sym([P.data P.pilots], [q(:,s); ones(numel(P.pilots),1)], P)]; %#ok<AGROW>
end
end

function v = qpsk(b)            % Gray: b0 -> dau I, b1 -> dau Q
b = double(b(:));
v = ((1 - 2*b(1:2:end)) + 1j*(1 - 2*b(2:2:end))) / sqrt(2);
end

function t = sym(f, X, P)
V = zeros(P.N, 1);
V(mod(f, P.N) + 1) = X;
t = ifft(V) * sqrt(P.N);
t = [t(end-P.CP+1:end); t];
end
