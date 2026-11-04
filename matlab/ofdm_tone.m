function y = ofdm_tone(P, S)
% Tin hieu kiem tra: 'tone' = sin lien tuc tai S.fc (muc tin hieu, cong huong)
%                    'sweep' = chirp tuyen tinh S.sweep_hz(1) -> (2) (dap ung transducer)
t = (0:round(S.tone_s * P.fs)-1).' / P.fs;
if strcmp(S.mode, 'sweep')
    f0 = S.sweep_hz(1); f1 = S.sweep_hz(2);
    y = S.amp * sin(2*pi*(f0*t + (f1-f0)/(2*S.tone_s)*t.^2));
else
    y = S.amp * sin(2*pi*S.fc*t);
end
r = round(0.01 * P.fs);                                % ramp 10 ms
w = 0.5 - 0.5*cos(pi*(0:r-1).'/r);
y(1:r) = y(1:r) .* w;  y(end-r+1:end) = y(end-r+1:end) .* flipud(w);
end
