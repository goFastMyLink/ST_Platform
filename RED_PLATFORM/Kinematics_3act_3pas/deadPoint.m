function a_dead = deadPoint(p, k, a_range)
% Ищет угол серво k, при котором кривошип проходит мёртвую точку
% (два решения ОЗК сливаются), при остальных углах = 0. Поиск в a_range, град.
% Возвращает [], если в диапазоне мёртвой точки нет (или ПЗК туда не доходит).
a_dead = [];
a_s = linspace(a_range(1), a_range(2), 241);
g = nan(size(a_s)); q = p.q0;
for n = 1:numel(a_s)
    a = [0 0 0]; a(k) = a_s(n);
    [VZ, EA, ok, q] = fk(p, a, q);
    if ~ok, continue; end
    [~, ~, ~, all2] = ik_pose(p, VZ, EA);
    g(n) = mod(all2(k,1) - all2(k,2) + 180, 360) - 180;   % разность двух решений
end
[~, j] = min(abs(g));
if ~isnan(g(j)) && abs(g(j)) < 2, a_dead = a_s(j); end
end
