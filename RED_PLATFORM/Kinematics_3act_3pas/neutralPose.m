function p = neutralPose(p, guess)
% Находит нейтральную позу при углах серво p.a_ref (если не задано - все 0) и кладёт её в p.q0.
% Сначала решается поза при нулевых углах, затем углы плавно ведутся к p.a_ref,
% чтобы ПЗК не перескочила на другую сборку.
% guess = [oz phi_deg] - грубое начальное приближение высоты и рыскания.
% Если из guess не сошлось, перебираются другие начальные высоты и рыскания,
% берётся решение с платформой НАД основанием.
q0 = [0 0 guess(1) 0 0 guess(2)*pi/180];
[~, ~, ok, q] = fk(p, [0 0 0], q0);
if ~ok
    for z = p.h_A + (0.3:0.2:1.1)*p.l_pas
        for yaw = -90:15:90
            [~, ~, ok, q] = fk(p, [0 0 0], [0 0 z 0 0 yaw*pi/180]);
            if ok && q(3) > p.h_A, break; end
        end
        if ok && q(3) > p.h_A, break; end
    end
end
if ok && isfield(p, 'a_ref') && any(p.a_ref ~= 0)
    n = max(1, ceil(max(abs(p.a_ref))/2));       % шаг ~2°
    for j = 1:n
        [~, ~, ok, q] = fk(p, p.a_ref*j/n, q);
        if ~ok, break; end
    end
    if ~ok, warning('neutralPose: при углах p.a_ref сборка не найдена'); end
end
if ~ok, warning('neutralPose: при нулевых углах серво сборка не найдена - проверьте геометрию'); end
p.q0 = q;
p.q0_ok = ok;
end
