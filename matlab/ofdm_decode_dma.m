function res = ofdm_decode_dma(src)
% OFDM_DECODE_DMA  Giai ket qua FPGA (dma_bits): moi khung 224 byte = 56 word 32 bit
%   little-endian, bit LSB truoc, CHUA bo lam trang.
%   src : ten file nhi phan (dump DDR) hoac vector uint8
P = ofdm_params();
if ischar(src), fid = fopen(src, 'rb'); b = fread(fid, inf, 'uint8=>uint8'); fclose(fid);
else, b = uint8(src(:)); end
nfr = floor(numel(b) / 224);
res = struct([]);
for f = 1:nfr
    raw  = double(b((f-1)*224 + (1:224)));
    bits = reshape(mod(floor(raw * 2.^(-(0:7))), 2).', [], 1);
    r    = ofdm_parse_payload(ofdm_bits_to_payload(bits, P));
    if isempty(res), res = r; else, res(end+1) = r; end %#ok<AGROW>
end
for f = 1:numel(res)
    if res(f).crc_ok, st = 'OK'; else, st = 'LOI'; end
    fprintf('khung %2d: seq %3d  CRC %-4s  "%s"\n', f, res(f).seq, st, res(f).text);
end
end
