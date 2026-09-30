function p = neutralPose(p, guess)
% Находит нейтральную позу (все углы серво = 0) и кладёт её в p.q0.
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
if ~ok, warning('neutralPose: при нулевых углах серво сборка не найдена - проверьте геометрию'); end
p.q0 = q;
p.q0_ok = ok;
end
