% Проверка трёхсерво-платформы (замеренная геометрия, нейтраль p.a_ref)
p = params_3servo();
if ~p.q0_ok
    disp('Нейтральная поза (углы p.a_ref) не собирается: с такими размерами тяги');
    disp('не могут одновременно дотянуться до платформы. Проверьте params_3servo');
    disp('(прежде всего размеры верхней платформы r_B, cB, sB). Остальные тесты пропущены.');
    return;
end
[VZ0, EA0] = poseToVZ(p, p.q0);
fprintf('Нейтраль (серво = %s): VZ = %s, EA = %s\n', mat2str(p.a_ref), mat2str(round(VZ0*1e3)/1e3), mat2str(round(EA0*1e3)/1e3));

disp('--- Мёртвые точки кривошипов в рабочем ходе [-90, 0] (остальные серво = 0) ---');
for k = 1:3
    fprintf('серво %d: a_dead = %s град (пусто - нет в [-90, 0])\n', k, num2str(deadPoint(p, k, [-90 0])));
end

disp('--- ПЗК -> ОЗК на траектории (выбор сборки по предыдущему шагу) ---');
t = linspace(0, 2*pi, 200);
A = p.a_ref + 15*[sin(t); sin(t + 2*pi/3); sin(t + 4*pi/3)]';   % качание вокруг нейтрали
q = p.q0; a_prev = A(1,:); err = 0; nok = 0;
for n = 1:numel(t)
    [VZ, EA, ok, q] = fk(p, A(n,:), q);
    nok = nok + ~ok;
    a_prev = ik_pose(p, VZ, EA, a_prev);
    err = max(err, max(abs(a_prev - A(n,:))));
end
fprintf('несошедшихся ПЗК: %d, макс. ошибка ПЗК->ОЗК: %.2e град\n', nok, err);

disp('--- 3-DOF ОЗК (ok = 0 -> поза недостижима: пассивные тяги не дотягиваются): задаём Z и два наклона, X/Y/рыскание дают пассивные тяги ---');
[a, VZ, EA, ok] = ik_3dof(p, VZ0(3) - 5, 3, -2);
fprintf('Z-5 мм, th=3°, psi=-2° -> серво %s (ok=%d), VZ = %s, EA = %s\n', ...
       mat2str(round(a*1e3)/1e3), ok, mat2str(round(VZ*1e3)/1e3), mat2str(round(EA*1e3)/1e3));
[VZc, EAc] = fk(p, a);
fprintf('проверка ПЗК от этих углов: dVZ = %.1e мм, dEA = %.1e град\n', norm(VZc - VZ), norm(EAc - EA));

% Калибровка 06.10.2026 (по уровню): us0 - импульс при горизонтальном кривошипе (a = 0),
% us_deg = (вертикаль - горизонталь)/(-90). Знак минус: больше импульс -> кривошип вверх (+a в модели - вниз).
% Серво 1 - после ремонта шлицов; серво 3 (-6.51) проверить уровнем.
cal = struct('us0', [937.5 888.7 913.1], 'us_deg', [-7.05 -7.60 -6.51], ...
             'us_min', 500, 'us_max', 2500, 'freq', 50);
[ticks, us] = servoToPCA(a, cal);
fprintf('PCA9685: импульсы %s мкс -> отсчёты %s\n', mat2str(round(us)), mat2str(ticks));
