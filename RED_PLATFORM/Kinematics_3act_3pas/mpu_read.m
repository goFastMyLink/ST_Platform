function [roll, pitch, g] = mpu_read(s, N)
% Крен и тангаж MPU (град, в осях самого MPU) - среднее по N запросам "M" к platform_control.
% g - модуль ускорения в g (в покое ~1.00).  Пример: [r, p] = mpu_read(s);
if nargin < 2, N = 10; end
v = nan(N, 3);
for i = 1:N
    writeline(s, "M");
    x = sscanf(char(readline(s)), 'MPU %f %f %f');
    if numel(x) == 3, v(i,:) = x'; end
    pause(0.02);
end
if all(isnan(v(:,1))), error('mpu_read: MPU не отвечает (ответ ERR mpu или пусто)'); end
m = mean(v, 1, 'omitnan');
roll = m(1); pitch = m(2); g = m(3);
end
