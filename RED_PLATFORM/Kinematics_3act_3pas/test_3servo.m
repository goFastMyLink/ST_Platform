% Проверка трёхсерво-платформы (пока на заглушках из params_3servo)
p = params_3servo();
[VZ0, EA0] = poseToVZ(p, p.q0);
fprintf('Нейтраль (все серво = 0): VZ = %s, EA = %s\n', mat2str(round(VZ0*1e3)/1e3), mat2str(round(EA0*1e3)/1e3));

disp('--- Мёртвые точки кривошипов (остальные серво = 0) ---');
for k = 1:3
    fprintf('серво %d: a_dead = %s град (пусто - нет в [-60, 60])\n', k, num2str(deadPoint(p, k, [-60 60])));
end

disp('--- ПЗК -> ОЗК на траектории (выбор сборки по предыдущему шагу) ---');
t = linspace(0, 2*pi, 200);
A = 15*[sin(t); sin(t + 2*pi/3); sin(t + 4*pi/3)]';
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

cal = struct('us0', [1500 1500 1500], 'us_deg', [10 10 10], ...   % ЗАГЛУШКИ калибровки
             'us_min', 500, 'us_max', 2500, 'freq', 50);
[ticks, us] = servoToPCA(a, cal);
fprintf('PCA9685: импульсы %s мкс -> отсчёты %s\n', mat2str(round(us)), mat2str(ticks));
