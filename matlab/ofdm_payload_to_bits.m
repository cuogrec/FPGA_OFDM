function bits = ofdm_payload_to_bits(bytes, P)
% Byte -> bit (LSB truoc, giong cach FPGA dong goi word 32 bit) -> lam trang (XOR PN9).
bytes = double(bytes(:));
bits  = reshape(mod(floor(bytes * 2.^(-(0:7))), 2).', [], 1);   % LSB first
bits  = xor(bits, P.whiten);
end
