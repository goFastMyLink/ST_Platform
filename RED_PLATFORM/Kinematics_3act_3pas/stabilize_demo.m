function stabilize_demo(PORT)
% Стабилизация горизонта для показа: работает, пока не нажмёшь Ctrl+C.
% По Ctrl+C платформа сама возвращается в нейтраль, порт закрывается,
% строится график и сохраняется лог (как в stabilize.m).
%
%   >> stabilize_demo("COM5")
%
% Основание при старте стоит РОВНО, ~2 с ничего не трогать (запоминается горизонт).
% Регулятор и настройки - те же, что в stabilize.m (ПИ, проверено 06.10.2026).
%
% Как устроена остановка: при Ctrl+C рабочее пространство функции уничтожается
% раньше, чем срабатывает onCleanup, поэтому всё нужное для остановки передаётся
% в demo_stop заранее (порт - это handle-объект, он остаётся жив), а лог
% складывается кусками в глобальную переменную STAB_DEMO (теряется не больше ~1 с).
if nargin < 1, PORT = "COM5"; end
p = params_3servo();
[VZ0, ~] = poseToVZ(p, p.q0);

J  = [-0.611 -0.550;               % d[roll; pitch]_MPU / d[th; psi], measure_tilt 06.10.2026
       0.778 -0.393];
Ji = inv(J);
KP    = 0.4;      % П-часть: доля ошибки, исправляемая сразу
KI    = 1.5;      % И-часть, 1/с
TAU_P = 0.08;     % с: сглаживание ошибки для П-части
DEAD  = 0.15;     % зона нечувствительности интегратора, град
LIM   = 8;        % предел |th|, |psi|, град

global STAB_DEMO
STAB_DEMO = struct('chunks', {{}}, 'nsat', 0, 'r0', NaN, 'p0', NaN, ...
                   'KP', KP, 'KI', KI, 'TAU_P', TAU_P, 'DEAD', DEAD, 'LIM', LIM, 'J', J);

s = platform_connect(PORT);
cleaner = onCleanup(@() demo_stop(s, p.a_ref));    %#ok<NASGU> сработает и при Ctrl+C

platform_send(s, p.a_ref);
pause(1.5);
[r0, p0] = mpu_read(s, 30);        % "горизонт" = MPU в нейтрали на ровном основании
STAB_DEMO.r0 = r0; STAB_DEMO.p0 = p0;
fprintf('Горизонт MPU: roll0 = %.2f, pitch0 = %.2f\n', r0, p0);
input('Enter - старт стабилизации. Остановка - Ctrl+C.');
disp('Стабилизация работает... (Ctrl+C - стоп)');

u = [0; 0]; uI = [0; 0]; ef = [0; 0]; a_prev = p.a_ref;
B = zeros(50, 8); k = 0;           % буфер лога: t, e_roll, e_pitch, th, psi, a1, a2, a3
t0 = tic; tp = 0;
while true
    writeline(s, "M");
    x = sscanf(char(readline(s)), 'MPU %f %f %f %f');
    if numel(x) < 3 || x(3) < 0.5, continue; end      % MPU не ответил - пропускаем шаг
    e  = [x(1) - r0; x(2) - p0];                       % наклон платформы к горизонту (оси MPU)
    tn = toc(t0); dt = min(tn - tp, 0.2); tp = tn;
    de = Ji * e;
    ef = ef + dt / (TAU_P + dt) * (e - ef);
    uI_new = uI;
    if norm(e) > DEAD, uI_new = uI - KI * dt * de; end
    uI_new = max(min(uI_new, LIM), -LIM);
    u_new  = max(min(uI_new - KP * (Ji * ef), LIM), -LIM);
    if norm(u_new - u) > 0.05
        [a, ~, ~, ok] = ik_3dof(p, VZ0(3), u_new(1), u_new(2), a_prev);
        if ok && all(a >= -65 & a <= -2)
            platform_send(s, a); a_prev = a; u = u_new; uI = uI_new;
        else
            STAB_DEMO.nsat = STAB_DEMO.nsat + 1;       % поза недостижима - держим прежнюю
        end
    else
        uI = uI_new;
    end
    k = k + 1; B(k, :) = [tn, e', u', a_prev];
    if k == size(B, 1)                                 % ~1 с набрали - сбросить в глобальный лог
        STAB_DEMO.chunks{end+1} = B; k = 0;
    end
end
end

function demo_stop(s, a_ref)
% Вызывается по Ctrl+C: нейтраль, закрыть порт, график, сохранить лог.
global STAB_DEMO
try
    flush(s);                                          % выбросить недочитанные ответы
    writeline(s, sprintf('A %.2f %.2f %.2f', a_ref));
    pause(0.3);
catch
end
try, delete(s); catch, end                             % закрыть порт
fprintf('\nСтоп. Платформа возвращена в нейтраль, порт закрыт.\n');
if isempty(STAB_DEMO) || isempty(STAB_DEMO.chunks), return; end
D = STAB_DEMO;
L = vertcat(D.chunks{:});
en = vecnorm(L(:, 2:3), 2, 2);
fprintf('Время: %.0f с, шагов: %d (%.0f Гц), упирались в предел: %d\n', ...
        L(end,1), size(L,1), size(L,1)/L(end,1), D.nsat);
fprintf('Ошибка горизонта: средняя |e| = %.2f°, макс = %.2f°\n', mean(en), max(en));
figure;
subplot(2,1,1); plot(L(:,1), L(:,2:3)); grid on; legend('e roll', 'e pitch');
ylabel('наклон платформы, град'); title('Отклонение платформы от горизонта (MPU)');
subplot(2,1,2); plot(L(:,1), L(:,4:5)); grid on; legend('th', 'psi');
xlabel('t, с'); ylabel('компенсация, град'); title('Заданный наклон платформы относительно основания');
drawnow;
KP = D.KP; KI = D.KI; TAU_P = D.TAU_P; DEAD = D.DEAD; LIM = D.LIM; J = D.J; %#ok<NASGU>
r0 = D.r0; p0 = D.p0; nsat = D.nsat;                                       %#ok<NASGU>
fn = sprintf('stab_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fn, 'L', 'KP', 'KI', 'TAU_P', 'DEAD', 'LIM', 'J', 'r0', 'p0', 'nsat');
fprintf('Сохранено: %s\n', fn);
end
