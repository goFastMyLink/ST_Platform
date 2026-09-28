tic
clear;clc;
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
VZ = [0             0      333.1849];
EA = [0            0      68.1257]*to_rad;


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

[b1 b2 b3 b4 b5 b6]

% B6 = Rz\B2; b6 = Rz\b2;
% B6 = Rz*B4; b6 = Rz*b4;
% acos(((p(3)-131))/l_kr)
% ans*to_deg
%%
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
% lb = [137 -40 91 -103 98 91 -103 -138 91];
% ub = [137 40 171 -33 138 171 -33 -98 171];
% [xfitted, errorfitted] = lsqnonlin(fun,p,lb,ub)

% a1 = -sign(p(2))*acos(((p(3)-131))/l_kr)*to_deg;
% a2 = -sign(p(5))*acos(((p(6)-131))/l_kr)*to_deg;
% a3 =  sign(p(8))*acos(((p(9)-131))/l_kr)*to_deg;
% a = [a1 a2 a3]
%{
for i = 1:4

if p(i,3) > 131
    a1 = -sign(p(i,2))*acos(((p(i,3)-131))/l_kr)*to_deg;
else 
    a1 = -sign(p(i,2))*(pi/2 + acos(((p(i,3)-131))/l_kr))*to_deg;
end

if p(i,6) > 131
    a2 = sign( p(i,5)-B2(2) )*acos(((p(i,6)-131))/l_kr)*to_deg;
else 
    a2 = sign( p(i,5)-B2(2) )*(pi/2 + acos(((p(i,6)-131))/l_kr))*to_deg;
end

if p(i,9)>131
    a3 = sign( p(i,8)-B4(2) )*acos(((p(i,9)-131))/l_kr)*to_deg;
else 
    a3 = sign( p(8)-B4(2) )*(pi/2 + acos(((p(9)-131))/l_kr))*to_deg;
end

a(i,:) = [a1 a2 a3];

end
%}
%%{
for i = 1:4
    a1 = atan2(p(i,3)-131,p(i,2))*to_deg - 90;
    a2 = atan2(p(i,6)-131, sin(-2*pi/3)*p(i,4) + cos(-2*pi/3)*p(i,5) )*to_deg - 90;
    a3 = atan2(p(i,9)-131, sin(2*pi/3)*p(i,7) + cos(2*pi/3)*p(i,8) )*to_deg - 90;
a(i,:) = [a1 a2 a3];
end


%}

%%
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

disp(fin)
toc