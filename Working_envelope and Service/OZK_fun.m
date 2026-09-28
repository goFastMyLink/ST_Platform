function [fin] = OZK_fun(EA, ZET)

to_rad = pi/180; to_deg = 180/pi;

global B6 B2 B4 b6 b2 b4 l_kr l_sh R 

% Параметры платформы

l_kr = 40; l_sh =150; l = 200;
R = 137;
r = 112;
Phi = 30*to_rad;

W = 2*r*sin(pi/3 - Phi/2) + 4*r*sin(Phi/2);
K = r*sin(Phi/2)/( 2*r*sin(Phi/2) + r*sin(pi/3 - Phi/2) );

% Матрицы поворота вокруг оси Z
Rz=[cosd(120) -sind(120) 0;
    sind(120)  cosd(120) 0;
    0         0        1];

Rz15=[cosd(-15) -sind(-15) 0;
      sind(-15)  cosd(-15) 0;
      0          0         1];

Rz_phi = [cos(Phi) -sin(Phi) 0;
          sin(Phi)  cos(Phi) 0;
          0         0        1];


% Координаты основания
B6 = [R*cos(Phi/2); -R*sin(Phi/2); 131];
B1 = Rz_phi * B6;
B2 = Rz*B6;
B3 = Rz*B1;
B4 = Rz*Rz*B6;
B5 = Rz*Rz*B1;
B = [B1 B2 B3 B4 B5 B6];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% входные данные
VZ = [0.0000    0.0000  ZET];        

% Матрица однородного преобразования
th  = EA(1);
psi = EA(2);
phi = EA(3);

Rot = [cos(phi)*cos(th), cos(phi)*sin(psi)*sin(th) - cos(psi)*sin(phi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(th);
       cos(th)*sin(phi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(th), cos(psi)*sin(phi)*sin(th) - cos(phi)*sin(psi);
            -sin(th),                              cos(th)*sin(psi),                              cos(psi)*cos(th)];

T = zeros(4); T(1:3,1:3) = Rot; T(:,4) = [VZ, 1];

P = [1 0 0 0;
     0 1 0 0;
     0 0 1 -21;
     0 0 0 1];

T = T*P;

% Координаты шарниров подвижной платформы при a = [0 0 0]; Z = 0
b1 = [r*cos(Phi/2); -r*sin(Phi/2);0]; 
b2 = Rz_phi*b1;
b3 = Rz*b1;
b4 = Rz*b2;
b5 = Rz*Rz*b1;
b6 = Rz*Rz*b2;
b = [b1 b2 b3 b4 b5 b6];

% Преобразование координат в ГСК
for q=1:6
    e = eye(4);
    e(1:3,4) = b(:,q);
    e = T*e*[0 0 0 1]'; e(4) = [];
    b(:,q) = e;
end



% Поворот на 15° против часовой стрелки
B15 = Rz15\B; b15 = Rz15\b;

B1 = B15(:,1);
B2 = B15(:,2);
B3 = B15(:,3);
B4 = B15(:,4);
B5 = B15(:,5);
B6 = B15(:,6);

b1 = b15(:,1);
b2 = b15(:,2);
b3 = b15(:,3);
b4 = b15(:,4);
b5 = b15(:,5);
b6 = b15(:,6);


%Поиск положения виртуальных точек через решение СНАУ:
% options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
 options = optimoptions('fsolve','Display','none');
fun = @IK;
x0=[137 0 131-l_kr -68.5 118.645 131-l_kr -68.5 -118.645 131-l_kr];
p1 = fsolve(fun,x0,options); %Положение виртуальных точек C6 C2 C4

x0 = [137 -40 131 -33.859 138.645 131 -103.141 -98.645 131];
p2 = fsolve(fun,x0,options); %Положение виртуальных точек C6 C2 C4

x0 = [137 40 131 -103.141 98.645 131 -33.859 -138.645 131];
p3 = fsolve(fun,x0,options); %Положение виртуальных точек C6 C2 C4

x0=[137 0 131+l_kr -68.5 118.645 131+l_kr -68.5 -118.645 131+l_kr];
p4 = fsolve(fun,x0,options); %Положение виртуальных точек C6 C2 C4

p = [p1;p2;p3;p4];

for i = 1:4
    a1 = atan2(p(i,3)-131,p(i,2))*to_deg - 90;
    a2 = atan2(p(i,6)-131, sin(-2*pi/3)*p(i,4) + cos(-2*pi/3)*p(i,5) )*to_deg - 90;
    a3 = atan2(p(i,9)-131, sin(2*pi/3)*p(i,7) + cos(2*pi/3)*p(i,8) )*to_deg - 90;
a(i,:) = [a1 a2 a3];
end

for i = 1:4
    for j = 1:3
        if abs(a(i,j)-round(a(i,j),1)) < 0.005
            fin(j) = round(a(i,j),1);
                if fin(j) < 0
                    fin(j) = fin(j) + 360;
                end
        end
    end
end

end


function F = IK(x)
global B6 B2 B4 b6 b2 b4 l_kr l_sh R 

% 6
F(1) = (x(1)-B6(1))^2 + (x(2)-B6(2))^2 + (x(3)-B6(3))^2 - l_kr^2;
F(2) = (x(1)-b6(1))^2 + (x(2)-b6(2))^2 + (x(3)-b6(3))^2 - l_sh^2;
F(3) =  x(1) - R;

% 2
F(4) = (x(4)-B2(1))^2 + (x(5)-B2(2))^2 + (x(6)-B2(3))^2 - l_kr^2;
F(5) = (x(4)-b2(1))^2 + (x(5)-b2(2))^2 + (x(6)-b2(3))^2 - l_sh^2;
F(6) = -x(4) + sqrt(3)*x(5) - 2*R;

% 4
F(7) = (x(7)-B4(1))^2 + (x(8)-B4(2))^2 + (x(9)-B4(3))^2 - l_kr^2;
F(8) = (x(7)-b4(1))^2 + (x(8)-b4(2))^2 + (x(9)-b4(3))^2 - l_sh^2;
F(9) = -x(7) - sqrt(3)*x(8) - 2*R;

end