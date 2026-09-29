function [VZ, EA, A] = workspace(p, a_min, a_max, step)
% Рабочая зона: перебор углов серво в [a_min, a_max] с шагом step (град),
% точки, где ПЗК не сошлась, ОТБРАСЫВАЮТСЯ.
% Для жёлтой: [VZ,EA] = workspace(params_yellow(), -30, 10, 5);
% Цвет точки - рыскание phi: видно паразитный поворот вокруг Z.
if nargin < 4, step = 5; end
g = a_min:step:a_max;
[a1, a2, a3] = ndgrid(g, g, g);
A = [a1(:) a2(:) a3(:)];
N = size(A,1); VZ = nan(N,3); EA = nan(N,3);
q = p.q0;
for n = 1:N
    [v, e, ok, qn] = fk(p, A(n,:), q);
    if ~ok, [v, e, ok, qn] = fk(p, A(n,:)); end     % повтор из нейтрали
    if ok, VZ(n,:) = v; EA(n,:) = e; q = qn; end
end
good = ~isnan(VZ(:,1));
fprintf('workspace: %d из %d точек, ПЗК не сошлась в %d\n', sum(good), N, N - sum(good));
VZ = VZ(good,:); EA = EA(good,:); A = A(good,:);
scatter3(VZ(:,1), VZ(:,2), VZ(:,3), 12, EA(:,3), 'filled');
xlabel('X, мм'); ylabel('Y, мм'); zlabel('Z, мм'); axis equal; grid on;
c = colorbar; ylabel(c, '\phi (рыскание), град');
view(-30, 10);
end
