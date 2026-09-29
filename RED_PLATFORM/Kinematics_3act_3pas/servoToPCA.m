function [ticks, us] = servoToPCA(a_deg, cal)
% Угол серво (математический, как в fk/ik) -> импульс, мкс -> отсчёт PCA9685 (0..4095).
% cal - калибровка, отдельно для каждого из 3 серво:
%   cal.us0     [1x3] импульс при a = 0, мкс (подбирается на стенде)
%   cal.us_deg  [1x3] мкс на градус со знаком (знак = направление вращения)
%   cal.us_min, cal.us_max - пределы импульса, мкс
%   cal.freq    - частота PWM PCA9685, Гц (обычно 50)
% !!! Числа калибровки FB5118M замерь сам: us_deg = (us2 - us1)/(угол2 - угол1).
us = cal.us0 + cal.us_deg .* a_deg;
if any(us < cal.us_min | us > cal.us_max)
    warning('servoToPCA: угол вне диапазона серво, импульс обрезан');
end
us = min(max(us, cal.us_min), cal.us_max);
ticks = round(us * cal.freq * 4096 / 1e6);
end
