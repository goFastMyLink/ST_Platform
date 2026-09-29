function [a_deg, ok, res_pas, a_all] = ik_pose(p, VZ, EA_deg, a_ref)
% ОЗК для ПОЛНОЙ позы (VZ = [X Y Z], EA = [th psi phi] в градусах).
% Углы активных приводов ищутся аналитически: d_u*cos(a) + d_v*sin(a) = e.
% У каждого кривошипа ДВА решения (по разные стороны от линии тяги) - оба дают
% одну и ту же позу платформы. Возвращается ближайшее к a_ref; оба - в a_all (3x2).
% Какое брать:
%   - если задано p.branch (+1/-1 для каждого привода) - всегда эта сборка
%     (надёжно, если рабочий диапазон серво не пересекает мёртвую точку);
%   - иначе - ближайшее к a_ref (по умолчанию p.a_ref; на траектории - углы
%     с предыдущего шага).
% Мёртвая точка кривошипа - где оба решения сливаются (кривошип и тяга на одной
% линии в плоскости качания). В ней по позе нельзя понять, с какой стороны кривошип.
%   ok      - true, если все три привода достают до платформы
%   res_pas - невязки длин пассивных тяг, мм. У механизма 3+3 не любая 6D-поза
%             достижима: если |res_pas| заметно больше нуля, такой позы нет.
%             Для 3-DOF задания (Z, th, psi) используйте ik_3dof.
if nargin < 4, a_ref = p.a_ref; end
q = VZtoPose(p, VZ, EA_deg);
[A, B] = legPoints(p, q);
a_deg = nan(1,3); a_all = nan(3,2); ok = true;
for k = 1:3
    i = p.active(k);
    [P0,u,v] = crankGeom(p, i);
    d  = B(:,i) - P0;
    du = d'*u; dv = d'*v;
    e  = (d'*d + p.l_kr^2 - p.l_act^2)/(2*p.l_kr);
    rho = hypot(du, dv);
    if abs(e) > rho
        ok = false; continue;          % тяга не достаёт
    end
    base = atan2(dv, du); dlt = acos(e/rho);
    cand = [base + dlt, base - dlt]*180/pi - p.a_off(k);
    cand = mod(cand - a_ref(k) + 180, 360) - 180 + a_ref(k);   % ближе к a_ref
    a_all(k,:) = cand;
    if isfield(p, 'branch') && ~isempty(p.branch)
        j = 1 + (p.branch(k) < 0);          % +1 -> base+dlt, -1 -> base-dlt
    else
        [~, j] = min(abs(cand - a_ref(k)));
    end
    a_deg(k) = cand(j);
end
res_pas = zeros(1,3);
for k = 1:3
    i = p.passive(k);
    res_pas(k) = norm(B(:,i) - A(:,i)) - p.l_pas;
end
end
