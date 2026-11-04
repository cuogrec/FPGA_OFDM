function c = crc16_ccitt(bytes)
% CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, khong dao bit.
c = 65535;
for k = 1:numel(bytes)
    c = bitxor(c, bitshift(double(bytes(k)), 8));
    for i = 1:8
        if bitand(c, 32768), c = bitand(bitxor(bitshift(c,1), 4129), 65535);
        else,                c = bitand(bitshift(c,1), 65535); end
    end
end
end
