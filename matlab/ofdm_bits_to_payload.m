function bytes = ofdm_bits_to_payload(bits, P)
% Nguoc cua ofdm_payload_to_bits: bo lam trang + gom bit LSB truoc thanh byte.
bits  = xor(double(bits(:)) ~= 0, P.whiten);
bytes = uint8(reshape(bits, 8, []).' * (2.^(0:7)).');
end
