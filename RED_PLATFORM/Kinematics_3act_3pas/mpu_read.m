function [roll, pitch, g, nres] = mpu_read(s, N)
% Крен и тангаж MPU (град, в осях самого MPU) - среднее по N запросам "M" к platform_control.
% g - модуль ускорения в g (в покое ~1.00); nres - сколько раз MPU пропадал с момента старта.
% Пример: [r, p] = mpu_read(s);
if nargin < 2, N = 10; end
v = nan(N, 3); nres = 0;
for i = 1:N
    writeline(s, "M");
    x = sscanf(char(readline(s)), 'MPU %f %f %f %f');
    if numel(x) >= 3 && x(3) > 0.5, v(i,:) = x(1:3)'; end   % g ~ 0 - MPU выдал нули, не берём
    if numel(x) == 4, nres = x(4); end
    pause(0.02);
end
if nres > 0, warning('mpu_read: MPU пропадал %d раз(а) - проверь питание и провода MPU', nres); end
if all(isnan(v(:,1))), error('mpu_read: MPU не отвечает (ответ ERR mpu или пусто)'); end
m = mean(v, 1, 'omitnan');
roll = m(1); pitch = m(2); g = m(3);
end
