function txt = ofdm_join_text(res)
% Ghep cac khung CRC OK (bo trung lap do lap lai) theo seq thanh van ban.
ok = res([res.crc_ok]);
if isempty(ok), txt = ''; return; end
[~, i] = unique([ok.seq]);  ok = ok(i);
u = [];  for k = 1:numel(ok), u = [u, double(ok(k).bytes(4:3+ok(k).len))]; end %#ok<AGROW>
try, txt = native2unicode(uint8(u), 'UTF-8'); catch, txt = char(u); end
if numel(ok) < ok(1).nfrag, txt = [txt sprintf(' [thieu %d/%d khung]', ok(1).nfrag-numel(ok), ok(1).nfrag)]; end
end
