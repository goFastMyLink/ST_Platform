function [a_deg, VZ, EA_deg, ok] = ik_3dof(p, Z, th_deg, psi_deg, a_ref)
% ОЗК для 3-DOF механизма: задаются только Z точки VZ и два наклона (th, psi).
% X, Y и рыскание phi не задаются - их определяют пассивные тяги.
% Сначала находятся X, Y, phi из условий |B_i - A_i| = l_pas (3 уравнения),
% затем углы серво - аналитически через ik_pose.
if nargin < 5, a_ref = p.a_ref; end
EA2 = [th_deg psi_deg]*pi/180;
f = @(x) pasRes(p, [x(1) x(2) Z], [EA2 x(3)]);
x0 = [p.q0(1) p.q0(2) p.q0(6)];
[x, ~, flag] = fsolve(f, x0, optimset('Display','off','TolFun',1e-12,'TolX',1e-12));
VZ = [x(1) x(2) Z];
EA_deg = [th_deg psi_deg x(3)*180/pi];
[a_deg, ok] = ik_pose(p, VZ, EA_deg, a_ref);
ok = ok && flag > 0 && max(abs(f(x))) < 1e-6;
end

function r = pasRes(p, VZ, EA)
q = VZtoPose(p, VZ, EA*180/pi);
[A, B] = legPoints(p, q);
r = zeros(3,1);
for k = 1:3
    i = p.passive(k);
    r(k) = sum((B(:,i) - A(:,i)).^2) - p.l_pas^2;
end
end
