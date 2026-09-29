% Проверка общего кода на жёлтой платформе.
% Эталон (старые PZK_fun/OZK_fun в Octave):
%   [0 0 0]     -> VZ = [0 0 332.645],          EA = [0 0 68.7122]
%   [20 20 20]  -> VZ = [0 0 333.185],          EA = [0 0 68.1257]
%   [10 -15 25] -> VZ = [-4.079 0.908 332.034], EA = [0.975232 -0.299694 69.3037]
p = params_yellow();
disp('--- 1. ПЗК против старого кода, и оба решения ОЗК ---');
for a = {[0 0 0], [20 20 20], [10 -15 25]}
    a = a{1};
    [VZ, EA] = fk(p, a);
    [~, ~, ~, a_all] = ik_pose(p, VZ, EA);
    fprintf('a = %-12s VZ = %-26s EA = %s\n', mat2str(a), mat2str(round(VZ*1e4)/1e4), mat2str(round(EA*1e4)/1e4));
    fprintf('   решения ОЗК: сборка +1 [%s] / сборка -1 [%s]\n', num2str(a_all(:,1)',6), num2str(a_all(:,2)',6));
end

disp('--- 2. Мёртвая точка кривошипа (привод 1, остальные = 0) ---');
fprintf('a_dead = %.2f град\n', deadPoint(p, 1, [-60 60]));

disp('--- 3. Траектория ПЗК -> ОЗК, сборка -1, углы в диапазоне [-30, 10] ---');
t = linspace(0, 2*pi, 200);
A = -10 + 20*[sin(t); sin(t + 2*pi/3); sin(t + 4*pi/3)]';
q = p.q0; err = 0; nok = 0;
for n = 1:numel(t)
    [VZ, EA, ok, q] = fk(p, A(n,:), q);
    nok = nok + ~ok;
    err = max(err, max(abs(ik_pose(p, VZ, EA) - A(n,:))));
end
fprintf('точек: %d, несошедшихся ПЗК: %d, макс. ошибка углов ПЗК->ОЗК: %.2e град\n', numel(t), nok, err);
