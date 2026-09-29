function [VZ, EA_deg, ok, q] = fk(p, a_deg, q0)
% ПЗК: углы серво [a1 a2 a3] (град, порядок p.active) -> VZ = [X Y Z] и EA = [th psi phi] (град).
% Неизвестные - сразу поза платформы q = [ox oy oz th psi phi] (6 шт.),
% уравнения - длины 3 активных и 3 пассивных тяг (6 шт.).
% q0 - начальное приближение (по умолчанию нейтральная поза p.q0).
% При движении по траектории передавайте q из предыдущего шага - так решатель
% не перескочит на другую сборку механизма.
if nargin < 3, q0 = p.q0; end
f = @(q) legRes(p, q, a_deg);
[q, ~, flag] = fsolve(f, q0, optimset('Display','off','TolFun',1e-12,'TolX',1e-12));
ok = flag > 0 && max(abs(f(q))) < 1e-6;
[VZ, EA_deg] = poseToVZ(p, q);
end

function r = legRes(p, q, a_deg)
[A, B] = legPoints(p, q, a_deg);
r = zeros(6,1);
for i = 1:6
    L = p.l_pas; if any(p.active == i), L = p.l_act; end
    r(i) = sum((B(:,i) - A(:,i)).^2) - L^2;
end
end
