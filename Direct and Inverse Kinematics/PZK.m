tic
to_rad = pi/180; to_deg = 180/pi;

global W K B1 B3 B5 C2 C4 C6 l_sh l

% for i = 1:3
% a(i) = input(['Введите значение угла ', num2str(i),':']);
% end


a = [20 20 20];
for i = 1:length(a)
    if ( a(i) == 90 ) || ( a(i) == 180 ) || ( a(i) == 270 ) || ( a(i) == 360 )
        a(i) = a(i) - 0.0001;
    end
end
a = to_rad*a; % -30 <= a <= 30

% Параметры платформы

l_kr = 40; l_sh = 150; l = 200;
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

% Координаты шарниров на приводных КЦ
C6 = Rz15*[R; -l_kr*sin(a(1)); l_kr*cos(a(1))+131];
C2 = Rz*Rz15 * [R; -l_kr*sin(a(2)); l_kr*cos(a(2))+131];
C4 = Rz*Rz*Rz15 * [R; -l_kr*sin(a(3)); l_kr*cos(a(3))+131];

c = [C2 C4 C6];


%Поиск положения виртуальных точек через решение СНАУ:
options = optimoptions('fsolve','Display','none');
fun = @DK;
x0=[57.505 147.585 311.645 -156.564 -23.992 311.645 99.060 -123.593 311.645];
%x0=[60 150 340 -160 -30 340 100 -130 340];
%x0 = zeros(1,9);
p = fsolve(fun,x0,options); %Положение виртуальных точек W1 W2 W3

Xw1 = p(1); Yw1 = p(2); Zw1 = p(3);
Xw2 = p(4); Yw2 = p(5); Zw2 = p(6);
Xw3 = p(7); Yw3 = p(8); Zw3 = p(9);

W1 = [p(1) p(2) p(3)];
W2 = [p(4) p(5) p(6)];
W3 = [p(7) p(8) p(9)];


Xp = ( Xw1 + Xw2 + Xw3 )/3;
Yp = ( Yw1 + Yw2 + Yw3 )/3;
Zp = ( Zw1 + Zw2 + Zw3 )/3;

P = [Xp Yp Zp];

% Углы Эйлера

A = (Yw2 - Yw1)*(Zw3 - Zw1) - (Yw3 - Yw1)*(Zw2 - Zw1);
B = (Xw3 - Xw1)*(Zw2 - Zw1) - (Xw2 - Xw1)*(Zw3 - Zw1);
C = (Xw2 - Xw1)*(Yw3 - Yw1) - (Xw3 - Xw1)*(Yw2 - Yw1);

cos_az = A/sqrt(A^2 + B^2 + C^2); 
cos_bz = B/sqrt(A^2 + B^2 + C^2);
cos_gz = C/sqrt(A^2 + B^2 + C^2);

cos_ax = (Xw1 - Xp)/sqrt((Xw1-Xp)^2 + (Yw1-Yp)^2 + (Zw1-Zp)^2);
cos_bx = (Yw1 - Yp)/sqrt((Xw1-Xp)^2 + (Yw1-Yp)^2 + (Zw1-Zp)^2);
cos_gx = (Zw1 - Zp)/sqrt((Xw1-Xp)^2 + (Yw1-Yp)^2 + (Zw1-Zp)^2);

cos_ay = cos_bz*cos_gx - cos_gz*cos_bx;
cos_by = cos_gz*cos_ax - cos_az*cos_gx;
cos_gy = cos_az*cos_bx - cos_bz*cos_ax;

T = [cos_ax cos_ay cos_az Xp;
     cos_bx cos_by cos_bz Yp;
     cos_gx cos_gy cos_gz Zp;
     0      0      0      1];
P = [1 0 0 0;
     0 1 0 0;
     0 0 1 21;
     0 0 0 1];

VZ = round(T*P*[0 0 0 1]',4); VZ(4) = [];


Rot = T(1:3,1:3);

%Третья система углов Эйлера
if (Rot(3,1) ~= 1) && (Rot(3,1) ~= -1)
    th=-asind(Rot(3,1));
    phi=atan2d(Rot(2,1),Rot(1,1));
    psi=atan2d(Rot(3,2),Rot(3,3));
elseif Rot(3,1) == 1
    th=asind(Rot(3,1));
    phi=0;
    psi=atan2d(-Rot(1,2),Rot(2,2));
elseif Rot(3,1) == -1
    th=asind(Rot(3,1));
    phi=0;
    psi=atan2d(Rot(1,2),Rot(2,2));
end
EA=[th psi phi];

disp(' ');

disp('   Положение выходного звена:');
disp(['                             X = ',num2str(round(VZ(1),6))]);
disp(['                             Y = ',num2str(round(VZ(2),6))]);
disp(['                             Z = ',num2str(round(VZ(3),6))]);
disp(['' ...
    '   Углы Эйлера:'])
disp(['                             \theta = ',num2str(round(EA(1),6)),'°']);
disp(['                             \psi = ',num2str(round(EA(2),6)),'°']);
disp(['                             \phi = ',num2str(round(EA(3),6)),'°']);
disp(' ')
disp(['VZ = [', num2str(VZ'),'];']);
% VZ = VZ'
disp(['EA = [', num2str(EA),']*to_rad;']);
toc