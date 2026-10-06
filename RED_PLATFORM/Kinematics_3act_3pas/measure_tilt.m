% Проверка ОЗК по MPU: платформа встаёт в позы [th psi] на нейтральной высоте,
% после каждой позы MPU меряет крен и тангаж. Сравниваем с заданными углами.
%   >> PORT = "COM5"; measure_tilt
% Основание должно стоять ровно (по уровню) и неподвижно.
if ~exist('PORT', 'var'), PORT = "COM5"; end
p = params_3servo();
[VZ0, ~] = poseToVZ(p, p.q0);
poses = [ 0  0;  8  0; -8  0;  0  8;  0 -8;  0  0];   % [th psi], град
% Обратная связь серво (АЦП -> угол кривошипа), замеры 06.10.2026: АЦП при a = 0 и ед. АЦП на градус.
% Грубо (зависит от напряжения питания серво), но видно, дошёл ли вал до команды.
FB0 = [157 121 160];  FBDEG = [-1.756 -2.144 -1.622];
s = platform_connect(PORT);
M = nan(size(poses,1), 2);
fprintf('  th   psi |  команда a1  a2  a3  |  факт (обр. связь)    |  roll  pitch  (MPU)   |a|\n');
for i = 1:size(poses,1)
    a = ik_3dof(p, VZ0(3), poses(i,1), poses(i,2), p.a_ref);
    platform_send(s, a);
    pause(2);                                   % доехать (60°/с) и успокоиться
    [r, pt, g] = mpu_read(s, 20);
    M(i,:) = [r pt];
    writeline(s, "F"); fb = sscanf(char(readline(s)), 'FB %f %f %f')';
    a_fb = (fb - FB0) ./ FBDEG;
    fprintf('%4.0f %5.0f | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f | %6.2f %6.2f        %5.3f\n', ...
            poses(i,:), a, a_fb, r, pt, g);
end
platform_send(s, p.a_ref);
clear s
D = M - M(1,:);                                 % относительно первой нейтрали (убирает перекос установки MPU)
fprintf('\nОтносительно нейтрали:\n');
for i = 2:size(poses,1)
    fprintf('th=%3.0f psi=%3.0f -> d_roll = %6.2f, d_pitch = %6.2f, |d| = %5.2f\n', ...
            poses(i,:), D(i,:), norm(D(i,:)));
end
fprintf('Возврат в нейтраль: расхождение с первой нейтралью %.2f град\n', norm(D(end,:)));
