function r = platform_send(s, a_deg)
% Отправить углы кривошипов [a1 a2 a3] (град) на Arduino. Возвращает ответ ("OK ..." / "CLIP ...").
writeline(s, sprintf('A %.2f %.2f %.2f', a_deg));
r = readline(s);
if startsWith(r, "CLIP"), warning('platform_send: углы обрезаны пределами Arduino: %s', r); end
end
