function r = ofdm_parse_payload(b)
% Phan tich payload 224 byte (da bo lam trang): header + text + CRC-16.
b = double(b(:)).';
r.seq   = b(1);  r.nfrag = b(2);  r.len = min(b(3), 219);
r.bytes = uint8(b);
r.crc_ok = (crc16_ccitt(b(1:222)) == b(223)*256 + b(224));
r.text  = safe_utf8(b(4:3+r.len));
r.cfo_hz = NaN; r.snr_db = NaN; r.start = NaN;
r = orderfields(r, {'seq','nfrag','len','text','crc_ok','cfo_hz','snr_db','start','bytes'});
end

function t = safe_utf8(u)
try
    t = native2unicode(uint8(u), 'UTF-8');
catch
    u(u > 126 | u < 32) = double('?');  t = char(u);   % khung loi: in ky tu thay the
end
end
