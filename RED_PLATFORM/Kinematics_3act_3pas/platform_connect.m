function s = platform_connect(port)
% Подключение к Arduino со скетчем platform_control.  Пример: s = platform_connect("COM3");
% Номер порта - в Arduino IDE (Инструменты -> Порт). Монитор порта в IDE должен быть ЗАКРЫТ.
s = serialport(port, 115200);
configureTerminator(s, "LF");
s.Timeout = 5;                    % старт скетча с калибровкой MPU занимает ~1-2 с
pause(2);                         % Arduino перезагружается при открытии порта
line = readline(s);               % ждём READY (вида "READY MPU ok who=0x68")
if ~contains(line, "READY"), warning('platform_connect: ответ "%s" вместо READY', line);
else, disp(line); end
if contains(line, "FAIL"), warning('platform_connect: MPU не отвечает - проверь провода SDA/SCL/VCC/GND'); end
s.Timeout = 2;
flush(s);
end
