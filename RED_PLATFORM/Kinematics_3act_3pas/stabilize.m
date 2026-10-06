% Стабилизация горизонта верхней платформы по MPU (замкнутый контур, ПИ-закон).
% MPU стоит на платформе и меряет её наклон к горизонту. Регулятор подкручивает заданные
% наклоны th, psi (ОЗК ik_3dof) до тех пор, пока MPU не покажет горизонт. Ошибки модели,
% люфты и калибровки серво контур убирает сам.
%
% ЗАПУСК: основание стоит РОВНО (по уровню), платформа не трогается ~2 с после старта:
%   >> PORT = "COM5"; stabilize
% Затем медленно наклоняй основание рукой (до ~8°). Ctrl+C - аварийный стоп.
if ~exist('PORT', 'var'), PORT = "COM5"; end
p = params_3servo();
[VZ0, ~] = poseToVZ(p, p.q0);

% d[roll; pitch]_MPU / d[th; psi], град/град - по measure_tilt 06.10.2026
% (полуразности поз +8/-8, чётная часть ошибки при этом сокращается)
J  = [-0.611 -0.550;
       0.778 -0.393];
Ji = inv(J);
% ПИ-закон: u = uI - KP*Ji*e,  uI накапливает -KI*Ji*e*dt.
%   P-часть реагирует сразу (без накопления) - короче выброс и нет "переката" после ступеньки;
%   I-часть дотягивает остаток до нуля.
% В контуре запаздывание ~0.3-0.5 с (фильтр MPU, ход серво, Serial): KP < 1, KI*задержка << 1.
% Раскачка - уменьшай сначала KP, потом KI. Чистый И-закон: KP = 0.
% Базовый прогон 06.10: KP = 0, KI = 1.5, DEAD = 0.3 -> выброс ~4.5°, успокоение ~1.5 с.
KP    = 0.5;      % доля ошибки, исправляемая сразу
KI    = 1.0;      % 1/с: скорость накопления
DEAD  = 0.15;     % зона нечувствительности ИНТЕГРАТОРА, град (P-часть работает всегда)
LIM   = 8;        % предел |th|, |psi|, град - дальше не хватает хода серво
T     = 60;       % длительность, с

s = platform_connect(PORT);
platform_send(s, p.a_ref);
pause(1.5);
[r0, p0] = mpu_read(s, 30);       % "горизонт" = MPU в нейтрали на ровном основании
fprintf('Горизонт MPU: roll0 = %.2f, pitch0 = %.2f (перекос установки датчика)\n', r0, p0);
input('Наклоняй основание медленно. Enter - старт');

u = [0; 0]; uI = [0; 0]; a_prev = p.a_ref;
L = zeros(0, 8);                  % лог: t, e_roll, e_pitch, th, psi, a1, a2, a3
t0 = tic; nsat = 0; tp = 0;
while toc(t0) < T
    writeline(s, "M");
    x = sscanf(char(readline(s)), 'MPU %f %f %f %f');
    if numel(x) < 3 || x(3) < 0.5, continue; end      % MPU не ответил - пропускаем шаг
    e = [x(1) - r0; x(2) - p0];                        % наклон платформы к горизонту в осях MPU
    tn = toc(t0); dt = min(tn - tp, 0.2); tp = tn;     % реальный шаг цикла, с
    de = Ji * e;                                       % ошибка, пересчитанная в th, psi
    uI_new = uI;
    if norm(e) > DEAD, uI_new = uI - KI * dt * de; end % И-часть: по времени, а не по шагам
    uI_new = max(min(uI_new, LIM), -LIM);
    u_new  = max(min(uI_new - KP * de, LIM), -LIM);    % П + И, насыщение
    if norm(u_new - u) > 0.05                          % заметное изменение - пересчитать ОЗК
        [a, ~, ~, ok] = ik_3dof(p, VZ0(3), u_new(1), u_new(2), a_prev);
        if ok && all(a >= -65 & a <= -2)
            platform_send(s, a); a_prev = a; u = u_new; uI = uI_new;
        else
            nsat = nsat + 1;                           % поза недостижима - держим прежнюю,
        end                                            % интегратор не копим (анти-накопление)
    else
        uI = uI_new;
    end
    L(end+1, :) = [toc(t0), e', u', a_prev];           %#ok<SAGROW>
end
platform_send(s, p.a_ref);
clear s

fprintf('KP = %.2f, KI = %.2f, DEAD = %.2f\n', KP, KI, DEAD);
fprintf('Шагов: %d (%.0f Гц), упирались в предел: %d\n', size(L,1), size(L,1)/T, nsat);
fprintf('Ошибка горизонта: средняя |e| = %.2f°, макс = %.2f°\n', mean(vecnorm(L(:,2:3),2,2)), max(vecnorm(L(:,2:3),2,2)));
figure;
subplot(2,1,1); plot(L(:,1), L(:,2:3)); grid on; legend('e roll', 'e pitch');
ylabel('наклон платформы, град'); title('Отклонение платформы от горизонта (MPU)');
subplot(2,1,2); plot(L(:,1), L(:,4:5)); grid on; legend('th', 'psi');
xlabel('t, с'); ylabel('компенсация, град'); title('Заданный наклон платформы относительно основания');

% Сохранить прогон: лог + настройки, чтобы потом строить графики и сравнивать KI
fn = sprintf('stab_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fn, 'L', 'KP', 'KI', 'DEAD', 'LIM', 'J', 'r0', 'p0', 'nsat');
fprintf('Сохранено: %s  (столбцы L: t, e_roll, e_pitch, th, psi, a1, a2, a3)\n', fn);
