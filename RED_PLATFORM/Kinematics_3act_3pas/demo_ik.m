% Управление красной платформой по ОЗК.
% Задаём высоту центра MPU (смещение dZ от нейтрали) и два наклона th, psi;
% ik_3dof находит углы серво (X, Y и рыскание определяют пассивные тяги),
% углы уходят на Arduino (скетч platform_control).
%
% ПЕРЕД ЗАПУСКОМ: залить platform_control.ino, закрыть монитор порта.
% Порт задаётся в командной строке, файл править не нужно:
%   >> PORT = "COM5"; demo_ik
if ~exist('PORT', 'var'), PORT = "COM3"; end
p = params_3servo();
if ~p.q0_ok, error('Нейтральная поза не найдена (p.q0_ok = 0): ПЗК не сошлась при нулевых углах'); end
[VZ0, EA0] = poseToVZ(p, p.q0);
fprintf('Нейтраль: Z MPU = %.1f мм, рыскание = %.1f град\n', VZ0(3), EA0(3));
A_LIM = [-30 20];              % те же пределы, что в скетче

%% 1. Расчёт: набор поз и плавная "волна" наклона
poses = [ 0  0  0;             % [dZ, th, psi]: dZ в мм, углы в град
         -5  0  0;
         -5  4  0;
         -5 -4  0;
         -5  0  4;
         -5  0 -4;
          0  0  0];
A_poses = nan(size(poses,1), 3);
for i = 1:size(poses,1)
    [a, VZ, EA, ok] = ik_3dof(p, VZ0(3) + poses(i,1), poses(i,2), poses(i,3));
    if ok, A_poses(i,:) = a; end
    fprintf('dZ=%4.1f th=%4.1f psi=%4.1f -> a = [%6.2f %6.2f %6.2f]  ok=%d\n', poses(i,:), a, ok);
end

T = 10; dt = 0.05; t = 0:dt:T;          % круговой наклон 4° за 5 с, на 5 мм ниже нейтрали
amp = 4; w = 2*pi/5;
A_wave = nan(numel(t), 3); a_prev = [0 0 0];
for n = 1:numel(t)
    [a, ~, ~, ok] = ik_3dof(p, VZ0(3) - 5, amp*sin(w*t(n)), amp*cos(w*t(n)), a_prev);
    if ~ok, error('Волна: поза t=%.2f недостижима', t(n)); end
    A_wave(n,:) = a; a_prev = a;
end
fprintf('Волна: углы от %.1f до %.1f град\n', min(A_wave(:)), max(A_wave(:)));
if any(A_poses(:) < A_LIM(1) | A_poses(:) > A_LIM(2)) || any(A_wave(:) < A_LIM(1) | A_wave(:) > A_LIM(2))
    error('Углы выходят за пределы скетча [%d, %d] - уменьшите амплитуды', A_LIM);
end

%% 2. Отправка на платформу
s = platform_connect(PORT);
disp('Позы:');
for i = 1:size(A_poses,1)
    if any(isnan(A_poses(i,:))), continue; end
    disp(platform_send(s, A_poses(i,:)));
    pause(1.5);
end
disp('Волна (ровно с нуля, первая точка):');
platform_send(s, A_wave(1,:)); pause(1);
t0 = tic;
for n = 1:numel(t)
    while toc(t0) < t(n), end           % держим шаг dt
    platform_send(s, A_wave(n,:));
end
platform_send(s, [0 0 0]);
clear s                                 % закрыть порт
disp('Готово.');
