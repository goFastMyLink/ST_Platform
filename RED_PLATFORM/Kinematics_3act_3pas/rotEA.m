function Rot = rotEA(EA)
% Матрица поворота по углам Эйлера (та же конвенция, что в исходных OZK/PZK):
%   Rot = Rz(phi) * Ry(th) * Rx(psi),   EA = [th psi phi] в РАДИАНАХ
th = EA(1); psi = EA(2); phi = EA(3);
Rz = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
Ry = [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)];
Rx = [1 0 0; 0 cos(psi) -sin(psi); 0 sin(psi) cos(psi)];
Rot = Rz*Ry*Rx;
end
