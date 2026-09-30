function [A, B] = legPoints(p, q, a_deg)
% Координаты всех шарниров в ГСК.
%   q     = [ox oy oz th psi phi] - поза СК платформы (центр плоскости шарниров), углы в рад
%   a_deg - углы серво [a1 a2 a3] в градусах (по порядку p.active); можно не передавать
%   A(:,i) - нижний шарнир ноги i (для активных - конец кривошипа, для пассивных - неподвижная точка)
%   B(:,i) - верхний шарнир ноги i на платформе
Rot = rotEA(q(4:6));
B = Rot*p.Bp + q(1:3)';
A = zeros(3,6);
for i = 1:6
    g = p.A_ang(i)*pi/180;
    R = p.R_A(min(i, numel(p.R_A)));      % R_A - число или [1x6]
    A(:,i) = [R*cos(g); R*sin(g); p.h_A];
end
if nargin > 2
    for k = 1:3
        i = p.active(k);
        [P0,u,v] = crankGeom(p, i);
        a = (a_deg(k) + p.a_off(k))*pi/180;
        A(:,i) = P0 + p.l_kr*(cos(a)*u + sin(a)*v);
    end
end
end
