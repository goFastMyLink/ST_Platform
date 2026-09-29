function p = neutralPose(p, guess)
% Находит нейтральную позу (все углы серво = 0) и кладёт её в p.q0.
% guess = [oz phi_deg] - грубое начальное приближение высоты и рыскания.
q0 = [0 0 guess(1) 0 0 guess(2)*pi/180];
[~, ~, ok, q] = fk(p, [0 0 0], q0);
if ~ok, warning('neutralPose: решение не найдено, проверьте геометрию и guess'); end
p.q0 = q;
end
