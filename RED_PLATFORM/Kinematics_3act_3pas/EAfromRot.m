function EA = EAfromRot(Rot)
% Обратно к rotEA: EA = [th psi phi] в РАДИАНАХ
s = max(-1, min(1, -Rot(3,1)));
th = asin(s);
if abs(abs(s) - 1) > 1e-9
    phi = atan2(Rot(2,1), Rot(1,1));
    psi = atan2(Rot(3,2), Rot(3,3));
else                         % вырожденный случай th = ±90°
    phi = 0;
    psi = atan2(-sign(s)*Rot(1,2), Rot(2,2));
end
EA = [th psi phi];
end
