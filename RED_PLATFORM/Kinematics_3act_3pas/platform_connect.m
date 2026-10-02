function s = platform_connect(port)
% Подключение к Arduino со скетчем platform_control.  Пример: s = platform_connect("COM3");
% Номер порта - в Arduino IDE (Инструменты -> Порт). Монитор порта в IDE должен быть ЗАКРЫТ.
s = serialport(port, 115200);
configureTerminator(s, "LF");
s.Timeout = 2;
pause(2);                         % Arduino перезагружается при открытии порта
line = readline(s);               % ждём READY
if ~contains(line, "READY"), warning('platform_connect: ответ "%s" вместо READY', line); end
flush(s);
end
