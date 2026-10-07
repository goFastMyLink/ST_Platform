function stabilize_demo(PORT)
% Стабилизация горизонта для показа: работает, пока не нажмёшь Ctrl+C.
% По Ctrl+C платформа сама возвращается в нейтраль, порт закрывается,
% строится график и сохраняется лог (как в stabilize.m).
%
%   >> stabilize_demo("COM5")
%
% Основание при старте стоит РОВНО, ~2 с ничего не трогать (запоминается горизонт).
% Регулятор и настройки - те же, что в stabilize.m (ПИ, проверено 06.10.2026).
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

L = zeros(200000, 8); n = 0;       % лог: t, e_roll, e_pitch, th, psi, a1, a2, a3 (~1 ч при 55 Гц)
nsat = 0; r0 = NaN; p0 = NaN; s = [];
cleaner = onCleanup(@finish);      %#ok<NASGU> вызовется и при Ctrl+C, и при ошибке

s = platform_connect(PORT);
platform_send(s, p.a_ref);
pause(1.5);
[r0, p0] = mpu_read(s, 30);        % "горизонт" = MPU в нейтрали на ровном основании
fprintf('Горизонт MPU: roll0 = %.2f, pitch0 = %.2f\n', r0, p0);
input('Enter - старт стабилизации. Остановка - Ctrl+C.');
disp('Стабилизация работает... (Ctrl+C - стоп)');

u = [0; 0]; uI = [0; 0]; ef = [0; 0]; a_prev = p.a_ref;
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
            nsat = nsat + 1;                           % поза недостижима - держим прежнюю
        end
    else
        uI = uI_new;
    end
    if n < size(L, 1), n = n + 1; L(n, :) = [tn, e', u', a_prev]; end
end

    function finish()
        % Срабатывает при Ctrl+C: нейтраль, закрыть порт, график, сохранить
        if ~isempty(s)
            try
                flush(s);                              % выбросить недочитанные ответы
                writeline(s, sprintf('A %.2f %.2f %.2f', p.a_ref));
                pause(0.3);
            catch
            end
            try, delete(s); catch, end                 % закрыть порт
        end
        fprintf('\nСтоп. Платформа возвращена в нейтраль.\n');
        if n < 2, return; end
        Lg = L(1:n, :);
        en = vecnorm(Lg(:, 2:3), 2, 2);
        fprintf('Время: %.0f с, шагов: %d (%.0f Гц), упирались в предел: %d\n', Lg(end,1), n, n/Lg(end,1), nsat);
        fprintf('Ошибка горизонта: средняя |e| = %.2f°, макс = %.2f°\n', mean(en), max(en));
        figure;
        subplot(2,1,1); plot(Lg(:,1), Lg(:,2:3)); grid on; legend('e roll', 'e pitch');
        ylabel('наклон платформы, град'); title('Отклонение платформы от горизонта (MPU)');
        subplot(2,1,2); plot(Lg(:,1), Lg(:,4:5)); grid on; legend('th', 'psi');
        xlabel('t, с'); ylabel('компенсация, град'); title('Заданный наклон платформы относительно основания');
        drawnow;
        fn = sprintf('stab_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
        L = Lg; save(fn, 'L', 'KP', 'KI', 'TAU_P', 'DEAD', 'LIM', 'J', 'r0', 'p0', 'nsat');
        fprintf('Сохранено: %s\n', fn);
    end
end
